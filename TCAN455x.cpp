/*
 * Copyright (c) 2020 George Beckstein
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License
 */

#if DEVICE_SPI && FEATURE_EXPERIMENTAL_API

#include "TCAN455x.h"

#include "TCAN4550.h" /** TI driver header file */

#include "platform/mbed_assert.h"
#include "platform/Callback.h"
#include "platform/mbed_wait_api.h"
#include "platform/ScopedLock.h"

#include "mbed_trace.h"

#define TRACE_GROUP "TCAN"

TCAN455x::TCAN455x(PinName mosi, PinName miso, PinName sclk, PinName csn, PinName nint_pin,
        mbed::DigitalOut* rst, mbed::DigitalOut* wake_ctl) : _spi(mosi, miso, sclk, csn, mbed::use_gpio_ssel), _nint(nint_pin, PullUp),
        _rst(rst), _wake_ctl(wake_ctl), _read_errors(0), _write_errors(0)
{
    for(int i = 0; i < TCAN455X_TOTAL_FILTER_COUNT; i++) {
        _filter_handles[i].sid_filter = nullptr;
        _filter_handles[i].xid_filter = nullptr;
    }
}

TCAN455x::~TCAN455x() {
}

void TCAN455x::init(void) {

    mbed::ScopedLock<PlatformMutex> lock(_mutex);

    /* Reset the chip to initial state (this also puts the reset output into the correct state) */
    reset();

    if(_initialized) {
        return;
    }

    // Wake the chip up using hardware pin, if available
    if(_wake_ctl != nullptr) {
        _wake_ctl->write(1);
        wait_us(60);    // Wait > t_WAKE (50us)
        _wake_ctl->write(0);
    }

#ifndef NDEBUG
    TCAN4x5x_Device_ReadDeviceVersion(this);
#endif

    TCAN4x5x_Device_ClearSPIERR(this);                              // Clear any SPI ERR flags that might be set as a result of our pin mux changing during MCU startup

    /* Step one attempt to clear all interrupts */
    TCAN4x5x_Device_Interrupt_Enable dev_ie = {0};                  // Initialize to 0 to all bits are set to 0.
    TCAN4x5x_Device_ConfigureInterruptEnable(this, &dev_ie);        // Disable all non-MCAN related interrupts for simplicity

    TCAN4x5x_Device_Interrupts dev_ir = {0};                        // Setup a new MCAN IR object for easy interrupt checking
    TCAN4x5x_Device_ReadInterrupts(this, &dev_ir);                  // Request that the struct be updated with current DEVICE (not MCAN) interrupt values

    if (dev_ir.PWRON) {                                             // If the Power On interrupt flag is set
        TCAN4x5x_Device_ClearInterrupts(this, &dev_ir);             // Clear it because if it's not cleared within ~4 minutes, it goes to sleep
    }

    /* Configure the CAN bus speeds */
    TCAN4x5x_MCAN_Nominal_Timing_Simple TCANNomTiming = {0};        // 500k arbitration with a 40 MHz crystal ((40E6 / 2) / (32 + 8) = 500E3)
    TCANNomTiming.NominalBitRatePrescaler = 2;
    TCANNomTiming.NominalTqBeforeSamplePoint = 32;
    TCANNomTiming.NominalTqAfterSamplePoint = 8;

    TCAN4x5x_MCAN_Data_Timing_Simple TCANDataTiming = {0};          // 2 Mbps CAN FD with a 40 MHz crystal (40E6 / (15 + 5) = 2E6)
    TCANDataTiming.DataBitRatePrescaler = 1;
    TCANDataTiming.DataTqBeforeSamplePoint = 15;
    TCANDataTiming.DataTqAfterSamplePoint = 5;

    /* Configure the MCAN core settings */
    _cccr_config = {0};                                               // Remember to initialize to 0, or you'll get random garbage!
    _cccr_config.FDOE = 0;                                            // CAN FD mode disable
    _cccr_config.BRSE = 0;                                            // CAN FD Bit rate switch disable

    /* Configure the default CAN packet filtering settings */
    TCAN4x5x_MCAN_Global_Filter_Configuration gfc = {0};
    gfc.RRFE = 1;                                                   // Reject remote frames (TCAN4x5x doesn't support this)
    gfc.RRFS = 1;                                                   // Reject remote frames (TCAN4x5x doesn't support this)
    gfc.ANFE = TCAN4x5x_GFC_ACCEPT_INTO_RXFIFO0;                    // Default behavior if incoming message doesn't match a filter is to accept into RXFIO0 for extended ID messages (29 bit IDs)
    gfc.ANFS = TCAN4x5x_GFC_ACCEPT_INTO_RXFIFO0;                    // Default behavior if incoming message doesn't match a filter is to accept into RXFIO0 for standard ID messages (11 bit IDs)

    /* ************************************************************************
     * In the next configuration block, we will set the MCAN core up to have:
     *   - 3 SID filter elements (default, configurable with MBED_CONF_TCAN455X_SID_FILTER_COUNT)
     *   - 3 XID Filter elements (default, configurable with MBED_CONF_TCAN455X_XID_FILTER_COUNT)
     *   - 5 RX FIFO 0 elements
     *   - RX FIFO 0 is where unfiltered packets go
     *   - RX FIFO 1 is where filtered messages go
     *   - RX Buffer is unused
     *   - No TX Event FIFOs
     *   - 2 Transmit buffers
     */

    // TODO - will need to increase the element size for CAN-FD
    TCAN4x5x_MRAM_Config MRAMConfiguration = {0};
    MRAMConfiguration.SIDNumElements = MBED_CONF_TCAN455X_SID_FILTER_COUNT; // Standard ID number of elements, you MUST have a filter written to MRAM for each element defined
    MRAMConfiguration.XIDNumElements = MBED_CONF_TCAN455X_XID_FILTER_COUNT; // Extended ID number of elements, you MUST have a filter written to MRAM for each element defined
    MRAMConfiguration.Rx0NumElements = MBED_CONF_TCAN455X_RX0_FIFO_SIZE;    // RX0 Number of elements
    MRAMConfiguration.Rx0ElementSize = MRAM_8_Byte_Data;        // RX0 data payload size
    MRAMConfiguration.Rx1NumElements = MBED_CONF_TCAN455X_RX1_FIFO_SIZE;    // RX1 number of elements
    MRAMConfiguration.Rx1ElementSize = MRAM_8_Byte_Data;        // RX1 data payload size
    MRAMConfiguration.RxBufNumElements = 0;                     // RX buffer number of elements
    MRAMConfiguration.RxBufElementSize = MRAM_64_Byte_Data;     // RX buffer data payload size
    MRAMConfiguration.TxEventFIFONumElements = 0;               // TX Event FIFO number of elements
    MRAMConfiguration.TxBufferNumElements = 2;                  // TX buffer number of elements
    MRAMConfiguration.TxBufferElementSize = MRAM_8_Byte_Data;   // TX buffer data payload size


    /* Configure the MCAN core with the settings above, the changes in this block are write protected registers,      *
     * so it makes the most sense to do them all at once, so we only unlock and lock once                             */

    TCAN4x5x_MCAN_EnableProtectedRegisters(this);                       // Start by making protected registers accessible
    TCAN4x5x_MCAN_ConfigureCCCRRegister(this, &_cccr_config);             // Enable FD mode and Bit rate switching
    TCAN4x5x_MCAN_ConfigureGlobalFilter(this, &gfc);                    // Configure the global filter configuration (Default CAN message behavior)
    TCAN4x5x_MCAN_ConfigureNominalTiming_Simple(this, &TCANNomTiming);  // Setup nominal/arbitration bit timing
    TCAN4x5x_MCAN_ConfigureDataTiming_Simple(this, &TCANDataTiming);    // Setup CAN FD timing
    TCAN4x5x_MRAM_Clear(this);                                          // Clear all of MRAM (Writes 0's to all of it)
    TCAN4x5x_MRAM_Configure(this, &MRAMConfiguration);                  // Set up the applicable registers related to MRAM configuration
    TCAN4x5x_MCAN_DisableProtectedRegisters(this);                      // Disable protected write and take device out of INIT mode


    /* Set the interrupts we want to enable for MCAN */
    TCAN4x5x_MCAN_Interrupt_Enable mcan_ie = {0};                   // Remember to initialize to 0, or you'll get random garbage!
    mcan_ie.RF0NE = 1;                                              // RX FIFO 0 new message interrupt enable
    mcan_ie.RF1NE = 1;                                              // RX FIFO 1 new message interrupt enable

    TCAN4x5x_MCAN_ConfigureInterruptEnable(this, &mcan_ie);         // Enable the appropriate registers

    /* Setup standard filters */
    for(int i = 0; i < MBED_CONF_TCAN455X_SID_FILTER_COUNT; i++) {
        _sid_filters[i].tcan_filter_index = i;
        _sid_filters[i].is_in_use = false;
        TCAN4x5x_MCAN_SID_Filter* SID_ID = &_sid_filters[i].filter;
        *SID_ID = {0};
        SID_ID->SFT = TCAN4x5x_SID_SFT_CLASSIC;                      // SFT: Standard filter type. Configured as a classic filter
        SID_ID->SFEC = TCAN4x5x_SID_SFEC_DISABLED;                   // Standard filter element configuration, initially disabled
        SID_ID->SFID1 = 0x055;                                       // SFID1 (Classic mode Filter)
        SID_ID->SFID2 = 0x7FF;                                       // SFID2 (Classic mode Mask)
        TCAN4x5x_MCAN_WriteSIDFilter(this, i, SID_ID);               // Write to the MRAM
    }

    /* Setup extended filters */
    for(int i = 0; i < MBED_CONF_TCAN455X_XID_FILTER_COUNT; i++) {
        _xid_filters[i].tcan_filter_index = i;
        _xid_filters[i].is_in_use = false;
        TCAN4x5x_MCAN_XID_Filter* XID_ID = &_xid_filters[i].filter;
        *XID_ID = {0};
        XID_ID->EFT = TCAN4x5x_XID_EFT_CLASSIC;                  // EFT
        XID_ID->EFEC = TCAN4x5x_XID_EFEC_DISABLED;               // EFEC, initially disabled
        XID_ID->EFID1 = 0x12345678;                              // EFID1 (Classic mode filter)
        XID_ID->EFID2 = 0x1FFFFFFF;                              // EFID2 (Classic mode mask)
        TCAN4x5x_MCAN_WriteXIDFilter(this, i, XID_ID);           // Write to the MRAM
    }

    /* Configure the TCAN4550 Non-CAN-related functions */
    TCAN4x5x_DEV_CONFIG devConfig = {0};                                // Remember to initialize to 0, or you'll get random garbage!
    devConfig.SWE_DIS = 0;                                              // Keep Sleep Wake Error Enabled (it's a disable bit, not an enable)
    devConfig.DEVICE_RESET = 0;                                         // Not requesting a software reset
    devConfig.WD_EN = 0;                                                // Watchdog disabled
    devConfig.nWKRQ_CONFIG = 0;                                         // Mirror INH function (default)
    devConfig.INH_DIS = 0;                                              // INH enabled (default)
    devConfig.GPIO1_GPO_CONFIG = TCAN4x5x_DEV_CONFIG_GPO1_MCAN_INT1;    // MCAN nINT 1 (default)
    devConfig.FAIL_SAFE_EN = 0;                                         // Failsafe disabled (default)
    devConfig.GPIO1_CONFIG = TCAN4x5x_DEV_CONFIG_GPIO1_CONFIG_GPO;      // GPIO set as GPO (Default)
    devConfig.WD_ACTION = TCAN4x5x_DEV_CONFIG_WDT_ACTION_nINT;          // Watchdog set an interrupt (default)
    devConfig.WD_BIT_RESET = 0;                                         // Don't reset the watchdog
    devConfig.nWKRQ_VOLTAGE = 0;                                        // Set nWKRQ to internal voltage rail (default)
    devConfig.GPO2_CONFIG = TCAN4x5x_DEV_CONFIG_GPO2_NO_ACTION;         // GPO2 has no behavior (default)
    devConfig.CLK_REF = 1;                                              // Input crystal is a 40 MHz crystal (default)
    devConfig.WAKE_CONFIG = TCAN4x5x_DEV_CONFIG_WAKE_BOTH_EDGES;        // Wake pin can be triggered by either edge (default)
    TCAN4x5x_Device_Configure(this, &devConfig);                        // Configure the device with the above configuration

    TCAN4x5x_Device_SetMode(this, TCAN4x5x_DEVICE_MODE_NORMAL);         // Set to normal mode, since configuration is done. This line turns on the transceiver

    TCAN4x5x_MCAN_ClearInterruptsAll(this);                             // Resets all MCAN interrupts (does NOT include any SPIERR interrupts)

    // Set up the interrupt input
    this->_nint.fall(mbed::callback(this, &TCAN455x::_tcan_irq_handler));

    _initialized = true;

}


void TCAN455x::sleep(void) {

    /* Lazy initialization */
    initialize_if();

    mbed::ScopedLock<PlatformMutex> lock(_mutex);

    TCAN4x5x_Device_SetMode(this, TCAN4x5x_DEVICE_MODE_SLEEP);
}

int TCAN455x::frequency(int hz) {

    /* Lazy initialization */
    initialize_if();

    mbed::ScopedLock<PlatformMutex> lock(_mutex);

    // Assumes a 40MHz external clock (crystal or otherwise)
    // TODO update this to be compatible with a different clock speed? (Configuration option)

    TCAN4x5x_MCAN_Nominal_Timing_Simple TCANNomTiming = {0};
    if(hz == 1000E3) {
        // 1Mbit arbitration with a 40 MHz crystal ((40E6 / 1) / (32 + 8) = 1000E3)
        TCANNomTiming.NominalBitRatePrescaler = 1;
        TCANNomTiming.NominalTqBeforeSamplePoint = 32;
        TCANNomTiming.NominalTqAfterSamplePoint = 8;
        apply_bitrate_change(TCANNomTiming);
        return 1;
    } else if(hz == 500E3) { // 500Kbps
        // 500Kbit arbitration with a 40 MHz crystal ((40E6 / 2) / (32 + 8) = 500E3)
        TCANNomTiming.NominalBitRatePrescaler = 2;
        TCANNomTiming.NominalTqBeforeSamplePoint = 32;
        TCANNomTiming.NominalTqAfterSamplePoint = 8;
        apply_bitrate_change(TCANNomTiming);
        return 1;
    } else if(hz == 250E3) { // 250Kbps
        // 250Kbit arbitration with a 40 MHz crystal ((40E6 / 4) / (32 + 8) = 250E3)
        TCANNomTiming.NominalBitRatePrescaler = 4;
        TCANNomTiming.NominalTqBeforeSamplePoint = 32;
        TCANNomTiming.NominalTqAfterSamplePoint = 8;
        apply_bitrate_change(TCANNomTiming);
        return 1;
    } else if(hz == 100E3) { // 100Kbps
        // 1Mbit arbitration with a 40 MHz crystal ((40E6 / 10) / (32 + 8) = 100E3)
        TCANNomTiming.NominalBitRatePrescaler = 10;
        TCANNomTiming.NominalTqBeforeSamplePoint = 32;
        TCANNomTiming.NominalTqAfterSamplePoint = 8;
        apply_bitrate_change(TCANNomTiming);
        return 1;
    } else {
        return 0;
    }
}

int TCAN455x::write(mbed::CANMessage msg) {

    /* Reject a message with CANAny as the format */
    if(msg.format == CANAny) {
        return 0;
    }

    /* Lazy initialization */
    initialize_if();

    mbed::ScopedLock<PlatformMutex> lock(_mutex);

    TCAN4x5x_MCAN_TX_Header header = {0};
    header.DLC  = (msg.len & 0xF);
    header.ID   = msg.id;
    header.FDF  = 0;
    header.BRS  = 0;
    header.EFC  = 0;
    header.MM   = 0;
    header.RTR  = 0;
    header.XTD  = (msg.format == CANExtended);
    header.ESI  = 0;

    /* This claims to return "number of bytes read" but I think that's a copy-paste mistake
     * At time of writing (using TCAN45xx driver rev B), the return value is:
     * 0 on failure,
     * 1 << bufIndex (second parameter) on success
     */
    uint32_t retval = TCAN4x5x_MCAN_WriteTXBuffer(this, 0, &header, msg.data);
    if(retval != 1) {
        tr_err("failed to write TX buffer");
        return 0;
    }
    if(TCAN4x5x_MCAN_TransmitBufferContents(this, 0)) {
        return 1;
    } else {
        tr_err("failed to transmit TX buffer contents");
        return 0;
    }
}

int TCAN455x::read(mbed::CANMessage &msg, int handle) {

    /* Lazy initialization */
    initialize_if();

    mbed::ScopedLock<PlatformMutex> lock(_mutex);

    /* Reject out of range handles */
    if(handle < 0 || handle > TCAN455X_TOTAL_FILTER_COUNT) {
        return 0;
    }

    TCAN4x5x_MCAN_Interrupts mcan_ir = {0};
    TCAN4x5x_MCAN_ReadInterrupts(this, &mcan_ir);

    // Clear SPIERR flag if it's set
    TCAN4x5x_Device_Interrupts dev_ir = {0};
    TCAN4x5x_Device_ReadInterrupts(this, &dev_ir);
    if(dev_ir.SPIERR) {
        TCAN4x5x_Device_ClearSPIERR(this);
    }

    TCAN4x5x_MCAN_ClearInterrupts(this, &mcan_ir);

    // Read out all messages from RXFIFO0 (unfiltered messages)
    uint8_t num_bytes = 0;
    tcan455x_message_t tcan_msg;
    do {
        num_bytes = TCAN4x5x_MCAN_ReadNextFIFO(this, RXFIFO0, &tcan_msg.rx_header, tcan_msg.data);
        if(num_bytes) {
            if(!_unfiltered_buffer.full()) {
                _unfiltered_buffer.push(tcan_msg);
            } else {
                tr_warn("unfiltered messages buffer overflow");
                _read_errors++;
            }
        }
    } while(num_bytes);

    // Read out all messages from RXFIFO1 (filtered messages)
    do {
        num_bytes = TCAN4x5x_MCAN_ReadNextFIFO(this, RXFIFO1, &tcan_msg.rx_header, tcan_msg.data);
        if(num_bytes) {
            // Get the associated filter handle
            int filter_index = 0;
            tcan455x_filter_handle_t *filter_handle = get_associated_filter_handle(&tcan_msg.rx_header, &filter_index);
            if(filter_handle) {
                if(!filter_handle->buffer.full()) {
                    filter_handle->buffer.push(tcan_msg);
                } else {
                    tr_warn("buffer overflow on filter handle %d", filter_index);
                    _read_errors++;
                }
            } else {
                tr_warn("received message for invalid filter handle");
            }
        }
    } while(num_bytes);

    // Try to get a message from the specified handle
    if(handle == 0) {
        if(!_unfiltered_buffer.pop(tcan_msg)) {
            return 0;
        }
    } else {
        if(!_filter_handles[handle-1].buffer.pop(tcan_msg)) {
            return 0;
        }
    }

    copy_tcan_rx_header(&msg, &tcan_msg.rx_header);
    memcpy(msg.data, &tcan_msg.data, 8);
    return 1;
}

void TCAN455x::reset(void) {

    mbed::ScopedLock<PlatformMutex> lock(_mutex);

    // Use software reset if there's no dedicated RST output pin
    if(_rst == nullptr) {
        TCAN4x5x_DEV_CONFIG devConfig = {0};    // Remember to initialize to 0, or you'll get random garbage!
        devConfig.DEVICE_RESET = 1;             // Request a software reset

        // Issue the software reset
        TCAN4x5x_Device_Configure(this, &devConfig);
    } else {
        _rst->write(1); // Pull reset high
        wait_us(30);    // Wait t_PulseWidth (30us)
        _rst->write(0); // Pull reset low again
        wait_us(750);   // Wait >700us before communicating
    }

    _initialized = false;

}

void TCAN455x::monitor(bool silent) {

    /* Lazy initialization */
    initialize_if();

    if(silent) {
        // Enter monitoring mode
        _cccr_config.MON = 1;
    } else {
        // Exit monitoring mode
        _cccr_config.MON = 0;
    }

    // Start by making protected registers accessible
    TCAN4x5x_MCAN_EnableProtectedRegisters(this);
    // Change the config
    TCAN4x5x_MCAN_ConfigureCCCRRegister(this, &_cccr_config);
    // Disable access to protected registers
    TCAN4x5x_MCAN_DisableProtectedRegisters(this);
}

int TCAN455x::mode(mbed::interface::can::Mode mode) {

    /* Lazy initialization */
    initialize_if();

    return 0; // TODO
}

int TCAN455x::filter(unsigned int id, unsigned int mask, CANFormat format,
        int handle) {

    /* Lazy initialization */
    initialize_if();

    // Disallow accessing a filter handle beyond what's available
    if(handle < 0 || handle > TCAN455X_TOTAL_FILTER_COUNT) {
        return 0;
    }

    if(handle == 0) {
        // Try to add a filter with an internally-assigned handle
        return add_filter(id, mask, format, handle);
    } else {
        tcan455x_filter_handle_t *filter = &_filter_handles[handle-1];

        // Check if we're accessing an already assigned filter
        if(filter->sid_filter || filter->xid_filter) {
            // If so, remove this filter so we can add a new one
            remove_filter(filter);
        }

        return add_filter(id, mask, format, handle);
    }
}

void TCAN455x::attach(mbed::Callback<void()> func, IrqType type) {
    mbed::ScopedLock<PlatformMutex> lock(_mutex);
    _irq[type] = func;
}

void TCAN455x::apply_bitrate_change(TCAN4x5x_MCAN_Nominal_Timing_Simple timing) {

    // Start by making protected registers accessible
    TCAN4x5x_MCAN_EnableProtectedRegisters(this);
    // Change the config
    TCAN4x5x_MCAN_ConfigureNominalTiming_Simple(this, &timing);
    // Disable access to protected registers
    TCAN4x5x_MCAN_DisableProtectedRegisters(this);

}

void TCAN455x::_tcan_irq_handler(void) {

    tr_debug("tcan irq handler called");

    /**
     * Since we can't access the SPI bus to read the interrupt source
     * during an interrupt, we can only handle one type of IRQ...
     *
     * The RX IRQ is probably the most used so that's the one we provide
     *
     * The RX interrupt is the only one enabled in the TCAN's registers so
     * the nINT pin should only be asserted when an RX interrupt occurs.
     *
     * TODO possible workaround would be if the driver is instantiated with
     * an internally-created SPI bus (ie: not accessible to other application code,
     * dedicated to the TCAN) then we can use a SPI subclass that removes the mutex protection
     *
     */
    if(_irq[mbed::interface::can::IrqType::RxIrq]) {
        _irq[mbed::interface::can::IrqType::RxIrq]();
    }

}

int TCAN455x::add_filter(unsigned int id, unsigned int mask, CANFormat format, int handle) {

    /* If the handle is 0, get one that's free */
    tcan455x_filter_handle_t *filter_handle;
    if(handle == 0) {
        filter_handle = get_available_filter_handle(&handle);
        if(!filter_handle) {
            tr_warn("no filter handles available");
            return 0;
        }
    } else {
        filter_handle = &_filter_handles[handle-1];
    }

    /* Add standard filter */
    if(format == CANStandard || format == CANAny) {
        if(id > 0x7FF) {
            tr_warn("cannot add standard/CANAny filter with ID greater than 0x7FF");
            return 0;
        }

        tcan455x_sid_filter_handle_t *tcan_filter = allocate_sid_filter_handle();
        if(!tcan_filter) {
            tr_warn("could not allocate an sid filter handle");
            return 0;
        }

        filter_handle->sid_filter = tcan_filter;
        filter_handle->sid_filter->filter.SFT = TCAN4x5x_SID_SFT_CLASSIC;
        filter_handle->sid_filter->filter.SFEC = TCAN4x5x_SID_SFEC_STORERX1;
        filter_handle->sid_filter->filter.SFID1 = id;
        filter_handle->sid_filter->filter.SFID2 = mask;
        TCAN4x5x_MCAN_WriteSIDFilter(this, filter_handle->sid_filter->tcan_filter_index, &filter_handle->sid_filter->filter);

    }

    if(format == CANExtended || format == CANAny) {
        if(id > 0x1FFFFFFF) {
            tr_warn("cannot add extended filter with ID greater than 0x1FFFFFFF");
            return 0;
        }

        tcan455x_xid_filter_handle_t *tcan_filter = allocate_xid_filter_handle();
        if(!tcan_filter) {
            /* If this is a CANAny filter and we successfully added an SID filter above, remove it */
            remove_filter(filter_handle);
            tr_warn("could not allocate an xid filter handle");
            return 0;
        }

        filter_handle->xid_filter = tcan_filter;
        filter_handle->xid_filter->filter.EFT = TCAN4x5x_XID_EFT_CLASSIC;
        filter_handle->xid_filter->filter.EFEC = TCAN4x5x_XID_EFEC_STORERX1;
        filter_handle->xid_filter->filter.EFID1 = id;
        filter_handle->xid_filter->filter.EFID2 = mask;
        TCAN4x5x_MCAN_WriteXIDFilter(this, filter_handle->xid_filter->tcan_filter_index, &filter_handle->xid_filter->filter);
    }

    return handle;

}

void TCAN455x::remove_filter(tcan455x_filter_handle_t *handle) {

    /* Disable the filter(s) in the TCAN and decrement counts */
    if(handle->sid_filter) {
        handle->sid_filter->filter.SFEC = TCAN4x5x_SID_SFEC_DISABLED;
        TCAN4x5x_MCAN_WriteSIDFilter(this, handle->sid_filter->tcan_filter_index, &handle->sid_filter->filter);
        free_sid_filter_handle(handle->sid_filter);
        handle->sid_filter = nullptr;
    }

    if(handle->xid_filter) {
        handle->xid_filter->filter.EFEC = TCAN4x5x_XID_EFEC_DISABLED;
        TCAN4x5x_MCAN_WriteXIDFilter(this, handle->xid_filter->tcan_filter_index, &handle->xid_filter->filter);
        free_xid_filter_handle(handle->xid_filter);
        handle->xid_filter = nullptr;
    }

}

TCAN455x::tcan455x_xid_filter_handle_t* TCAN455x::allocate_xid_filter_handle() {
    for(int i = 0; i < MBED_CONF_TCAN455X_XID_FILTER_COUNT; i++) {
        if(!_xid_filters[i].is_in_use) {
            _xid_filters[i].is_in_use = true;
            return &_xid_filters[i];
        }
    }

    /* Could not find one to allocate, return nullptr */
    return nullptr;
}

TCAN455x::tcan455x_sid_filter_handle_t* TCAN455x::allocate_sid_filter_handle() {

    for(int i = 0; i < MBED_CONF_TCAN455X_SID_FILTER_COUNT; i++) {
        if(!_sid_filters[i].is_in_use) {
            _sid_filters[i].is_in_use = true;
            return &_sid_filters[i];
        }
    }

    /* Could not find one to allocate, return nullptr */
    return nullptr;
}

TCAN455x::tcan455x_filter_handle_t* TCAN455x::get_available_filter_handle(int *handle) {

    tcan455x_filter_handle_t* retval = nullptr;
    for(int i = 0; i < TCAN455X_TOTAL_FILTER_COUNT; i++) {
        /* Unassigned filter */
        tcan455x_filter_handle_t* filter_handle = &_filter_handles[i];
        if(!filter_handle->sid_filter && !filter_handle->xid_filter) {
            *handle = i+1;
            retval = filter_handle;
            break;
        }
    }

    return retval;
}

void TCAN455x::copy_tcan_rx_header(CAN_Message* msg, TCAN4x5x_MCAN_RX_Header* header) {
    msg->id = header->ID;
    msg->len = header->DLC;
    msg->format = (header->XTD? CANExtended : CANStandard);
    msg->type = CANData;
}

TCAN455x::tcan455x_filter_handle_t* TCAN455x::get_associated_filter_handle(
        TCAN4x5x_MCAN_RX_Header *rx_header,
        int *index) {
    for(int i = 0; i < TCAN455X_TOTAL_FILTER_COUNT; i++) {
        tcan455x_filter_handle_t *handle = &_filter_handles[i];
        if(rx_header->XTD && handle->xid_filter) {
            if(handle->xid_filter->tcan_filter_index == rx_header->FIDX) {
                *index = i;
                return handle;
            }
        }

        if(!rx_header->XTD && handle->sid_filter) {
            if(handle->sid_filter->tcan_filter_index == rx_header->FIDX) {
                *index = i;
                return handle;
            }
        }
    }

    return nullptr;
}

#endif /** DEVICE_SPI */
