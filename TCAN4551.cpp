/*
 * TCAN4551.cpp
 *
 *  Created on: Feb 10, 2020
 *      Author: gdbeckstein
 */

#include "TCAN4551.h"

#include "TCAN4550.h" /** TI driver header file */

#include "platform/mbed_assert.h"
#include "platform/Callback.h"

TCAN4551::TCAN4551(PinName mosi, PinName miso, PinName sclk, PinName csn,
        PinName nint_pin) : spi(mosi, miso, sclk, csn, mbed::use_gpio_ssel), nint(nint_pin, PullUp),
        irq_handler(NULL), id(0), irq_mask(0), read_errors(0), write_errors(0),
        standard_id_index(0), extended_id_index(0){
    for(int i = 0; i < TCAN4551_TOTAL_FILTER_COUNT; i++) {
        memset(&filtered_buffers[i], 0, sizeof(filtered_buffer_t));
    }
}

TCAN4551::~TCAN4551() {
}

void TCAN4551::init(void) {

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
    cccr_config = {0};                     // Remember to initialize to 0, or you'll get random garbage!
    cccr_config.FDOE = 0;                                            // CAN FD mode disable
    cccr_config.BRSE = 0;                                            // CAN FD Bit rate switch disable

    /* Configure the default CAN packet filtering settings */
    TCAN4x5x_MCAN_Global_Filter_Configuration gfc = {0};
    gfc.RRFE = 1;                                                   // Reject remote frames (TCAN4x5x doesn't support this)
    gfc.RRFS = 1;                                                   // Reject remote frames (TCAN4x5x doesn't support this)
    gfc.ANFE = TCAN4x5x_GFC_ACCEPT_INTO_RXFIFO0;                    // Default behavior if incoming message doesn't match a filter is to accept into RXFIO0 for extended ID messages (29 bit IDs)
    gfc.ANFS = TCAN4x5x_GFC_ACCEPT_INTO_RXFIFO0;                    // Default behavior if incoming message doesn't match a filter is to accept into RXFIO0 for standard ID messages (11 bit IDs)

    /* ************************************************************************
     * In the next configuration block, we will set the MCAN core up to have:
     *   - 3 SID filter elements (default, configurable with MBED_CONF_TCAN4551_SID_FILTER_COUNT)
     *   - 3 XID Filter elements (default, configurable with MBED_CONF_TCAN4551_XID_FILTER_COUNT)
     *   - 5 RX FIFO 0 elements
     *   - RX FIFO 0 is where unfiltered packets go
     *   - RX FIFO 1 is where filtered messages go
     *   - RX Buffer is unused
     *   - No TX Event FIFOs
     *   - 2 Transmit buffers
     */

    // TODO - will need to increase the element size for CAN-FD
    TCAN4x5x_MRAM_Config MRAMConfiguration = {0};
    MRAMConfiguration.SIDNumElements = MBED_CONF_TCAN4551_SID_FILTER_COUNT; // Standard ID number of elements, you MUST have a filter written to MRAM for each element defined
    MRAMConfiguration.XIDNumElements = MBED_CONF_TCAN4551_XID_FILTER_COUNT; // Extended ID number of elements, you MUST have a filter written to MRAM for each element defined
    MRAMConfiguration.Rx0NumElements = 5;                       // RX0 Number of elements
    MRAMConfiguration.Rx0ElementSize = MRAM_8_Byte_Data;        // RX0 data payload size
    MRAMConfiguration.Rx1NumElements = 5;                       // RX1 number of elements
    MRAMConfiguration.Rx1ElementSize = MRAM_8_Byte_Data;        // RX1 data payload size
    MRAMConfiguration.RxBufNumElements = 0;                     // RX buffer number of elements
    MRAMConfiguration.RxBufElementSize = MRAM_64_Byte_Data;     // RX buffer data payload size
    MRAMConfiguration.TxEventFIFONumElements = 0;               // TX Event FIFO number of elements
    MRAMConfiguration.TxBufferNumElements = 2;                  // TX buffer number of elements
    MRAMConfiguration.TxBufferElementSize = MRAM_8_Byte_Data;   // TX buffer data payload size


    /* Configure the MCAN core with the settings above, the changes in this block are write protected registers,      *
     * so it makes the most sense to do them all at once, so we only unlock and lock once                             */

    TCAN4x5x_MCAN_EnableProtectedRegisters(this);                       // Start by making protected registers accessible
    TCAN4x5x_MCAN_ConfigureCCCRRegister(this, &cccr_config);             // Enable FD mode and Bit rate switching
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
    for(int i = 0; i < MBED_CONF_TCAN4551_SID_FILTER_COUNT; i++) {
        TCAN4x5x_MCAN_SID_Filter* SID_ID = &filtered_buffers[i].sid_filter;
        *SID_ID = {0};
        SID_ID->SFT = TCAN4x5x_SID_SFT_CLASSIC;                      // SFT: Standard filter type. Configured as a classic filter
        SID_ID->SFEC = TCAN4x5x_SID_SFEC_DISABLED;                   // Standard filter element configuration, initially disabled
        SID_ID->SFID1 = 0x055;                                       // SFID1 (Classic mode Filter)
        SID_ID->SFID2 = 0x7FF;                                       // SFID2 (Classic mode Mask)
        TCAN4x5x_MCAN_WriteSIDFilter(this, i, SID_ID);               // Write to the MRAM
    }

    /* Setup extended filters */
    for(int i = 0; i < MBED_CONF_TCAN4551_XID_FILTER_COUNT; i++) {
        TCAN4x5x_MCAN_XID_Filter* XID_ID = &filtered_buffers[MBED_CONF_TCAN4551_SID_FILTER_COUNT+i].xid_filter;
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
}

int TCAN4551::frequency(int hz) {

    // Assumes a 40MHz external clock (crystal or otherwise)

    TCAN4x5x_MCAN_Nominal_Timing_Simple TCANNomTiming = {0};
    if(hz == 1000E3) {
        // 1Mbit arbitration with a 40 MHz crystal ((40E6 / 1) / (32 + 8) = 1000E3)
        TCANNomTiming.NominalBitRatePrescaler = 1;
        TCANNomTiming.NominalTqBeforeSamplePoint = 32;
        TCANNomTiming.NominalTqAfterSamplePoint = 8;
        apply_bitrate_change(TCANNomTiming);
        return hz;
    } else if(hz == 500E3) { // 500Kbps
        // 500Kbit arbitration with a 40 MHz crystal ((40E6 / 2) / (32 + 8) = 500E3)
        TCANNomTiming.NominalBitRatePrescaler = 2;
        TCANNomTiming.NominalTqBeforeSamplePoint = 32;
        TCANNomTiming.NominalTqAfterSamplePoint = 8;
        apply_bitrate_change(TCANNomTiming);
        return hz;
    } else if(hz == 250E3) { // 250Kbps
        // 250Kbit arbitration with a 40 MHz crystal ((40E6 / 4) / (32 + 8) = 250E3)
        TCANNomTiming.NominalBitRatePrescaler = 4;
        TCANNomTiming.NominalTqBeforeSamplePoint = 32;
        TCANNomTiming.NominalTqAfterSamplePoint = 8;
        apply_bitrate_change(TCANNomTiming);
        return hz;
    } else if(hz == 100E3) { // 100Kbps
        // 1Mbit arbitration with a 40 MHz crystal ((40E6 / 10) / (32 + 8) = 100E3)
        TCANNomTiming.NominalBitRatePrescaler = 10;
        TCANNomTiming.NominalTqBeforeSamplePoint = 32;
        TCANNomTiming.NominalTqAfterSamplePoint = 8;
        apply_bitrate_change(TCANNomTiming);
        return hz;
    } else {
        return 0;
    }
}

void TCAN4551::attach_irq_handler(can_irq_handler handler, uint32_t id) {

    this->irq_handler = handler;
    this->id = id;

    // Set up the interrupt input
    this->nint.fall(mbed::callback(this, &TCAN4551::_tcan_irq_handler));
}

void TCAN4551::detach_irq_handler(void) {
    this->irq_handler = NULL;
    this->id = 0;

    // Detach the interrupt input handler
    this->nint.fall(nullptr);
}

void TCAN4551::enable_irq(CanIrqType type) {
    // Set the enable bit in the IRQ mask
    irq_mask |= (1 << (unsigned int) type);
}

void TCAN4551::disable_irq(CanIrqType type) {
    irq_mask &= ~(1 << (unsigned int) type);
}

int TCAN4551::write(CAN_Message msg, int cc) {
    TCAN4x5x_MCAN_TX_Header header = {0};
    header.DLC  = (msg.len & 0xF);
    header.ID   = msg.id;
    header.FDF  = 0;
    header.BRS  = 0;
    header.EFC  = 0;
    header.MM   = 0;
    header.RTR  = 0;
    header.XTD  = 0;
    header.ESI  = 0;

    TCAN4x5x_MCAN_WriteTXBuffer(this, 0, &header, msg.data);
    TCAN4x5x_MCAN_TransmitBufferContents(this, 0);

    return 1; // TODO - how to check if it actually succeeded? wait for interrupt?
}

int TCAN4551::read(CAN_Message* msg, int handle) {
    TCAN4x5x_MCAN_Interrupts mcan_ir = {0};
    TCAN4x5x_MCAN_ReadInterrupts(this, &mcan_ir);

    // Clear SPIERR flag if it's set
    TCAN4x5x_Device_Interrupts dev_ir = {0};
    TCAN4x5x_Device_ReadInterrupts(this, &dev_ir);
    if(dev_ir.SPIERR) {
        TCAN4x5x_Device_ClearSPIERR(this);
    }

    TCAN4x5x_MCAN_ClearInterrupts(this, &mcan_ir);

    // See if any of the filtered buffers have a new message for the desired filter
    if(handle != 0) {
        int filter_index = filter_handle_to_index(handle);
        filtered_buffer_t* buffer = filtered_buffers[filter_handle_to_index(handle)];
        if(buffer->new_data_available) {
            // New data has been stored in the local buffers, copy it over
            copy_tcan_rx_header(msg, &buffer->rx_header);
            memcpy(msg->data, buffer->data, buffer->rx_header.DLC);
            buffer->new_data_available = false;
            return 1;
        } else {
            // See if the FIFO1 has a new filtered message for us
            uint8_t num_bytes = 0;
            num_bytes = TCAN4x5x_MCAN_ReadNextFIFO(this, RXFIFO1, &buffer->rx_header, buffer->data);
            if(num_bytes == 0) {
                return 0; // No filtered messages received
            }

            // Make sure this new filtered message matches the given filter
            uint32_t filter_id = (buffer->rx_header.XTD?
                                  buffer->xid_filter.EFID1 : buffer->sid_filter.SFID1);
            if(filter_id == buffer->rx_header.ID) {
                // Filter matches, copy it over and return it
                copy_tcan_rx_header(msg, &buffer->rx_header);
                memcpy(msg->data, buffer->data, num_bytes);
            } else {
                // Otherwise, we stored it in the temporary buffer so flag it
                buffer->new_data_available = true;
            }
        }
    } else {
        // Attempt to read FIFO0 (unfiltered messages)

        TCAN4x5x_MCAN_RX_Header msg_header = {0};
        uint8_t num_bytes = 0;

        num_bytes = TCAN4x5x_MCAN_ReadNextFIFO(this, RXFIFO0, &msg_header, msg->data);

        if(num_bytes == 0) {
            return 0; // No message received
        } else {
            // Copy over the header information
            copy_tcan_rx_header(msg, &msg_header);
            return 1;
        }
    }
}

int TCAN4551::mode(CanMode mode) {
    return 0; // TODO
}

int TCAN4551::filter(unsigned int id, unsigned int mask, CANFormat format,
        int handle) {

    if(format == CANStandard) {

        TCAN4x5x_MCAN_SID_Filter* handle_ptr;
        int filter_index = filter_handle_to_index(handle);

        // Disallow accessing a filter handle beyond what's available
        if(filter_index < 0 || filter_index >= MBED_CONF_TCAN4551_SID_FILTER_COUNT) {
            return 0;
        }

        // If the user didn't provide a valid handle
        if(filter_index == -1) {
            filter_index = alloc_sid_handle_index();   // Try to allocate one that's available
            if(filter_index == -1) {               // No filters left to allocate :(
                return 0;
            }
        }

        // Get a pointer to the filter struct
        handle_ptr = &filtered_buffers[filter_index].sid_filter;

        // Configure the filter and write it to the controller
        handle_ptr->SFT = TCAN4x5x_SID_SFT_CLASSIC;
        handle_ptr->SFEC = TCAN4x5x_SID_SFEC_STORERX0;
        handle_ptr->SFID1 = id;
        handle_ptr->SFID2 = mask;
        TCAN4x5x_MCAN_WriteSIDFilter(this, filter_index, handle_ptr);
        return filter_index_to_handle(filter_index);

    } else
    if(format == CANExtended) {

        TCAN4x5x_MCAN_XID_Filter* handle_ptr;
        int filter_index = filter_handle_to_index(handle);

        // Disallow accessing a filter handle beyond what's available
        if(filter_index < MBED_CONF_TCAN4551_SID_FILTER_COUNT || filter_index >= TCAN4551_TOTAL_FILTER_COUNT) {
            return 0;
        }

        // If the user didn't provide a valid handle
        if(filter_index == -1) {
            filter_index = alloc_xid_handle_index();   // Try to allocate one that's available
            if(filter_index == -1) {               // No filters left to allocate :(
                return 0;
            }
        }

        // Get a pointer to the filter struct
        handle_ptr = &filtered_buffers[filter_index].xid_filter;

        // Configure the filter and write it to the controller
        handle_ptr->EFT = TCAN4x5x_XID_EFT_CLASSIC;
        handle_ptr->EFEC = TCAN4x5x_XID_EFEC_STORERX0;
        handle_ptr->EFID1 = id;
        handle_ptr->EFID2 = mask;
        TCAN4x5x_MCAN_WriteXIDFilter(this, filter_index, handle_ptr);
        return filter_index_to_handle(filter_index);
    }
    else {
        // Invalid input
        return 0;
    }

}

void TCAN4551::reset(void) {
    // TODO - add ability to configure a hardware reset pin?
    TCAN4x5x_DEV_CONFIG devConfig = {0};    // Remember to initialize to 0, or you'll get random garbage!
    devConfig.DEVICE_RESET = 1;             // Request a software reset

    // Issue the software reset
    TCAN4x5x_Device_Configure(this, &devConfig);

}

void TCAN4551::monitor(bool silent) {
    if(silent) {
        // Enter monitoring mode
        cccr_config.MON = 1;
    } else {
        // Exit monitoring mode
        cccr_config.MON = 0;
    }

    // Start by making protected registers accessible
    TCAN4x5x_MCAN_EnableProtectedRegisters(this);
    // Change the config
    TCAN4x5x_MCAN_ConfigureCCCRRegister(this, &cccr_config);
    // Disable access to protected registers
    TCAN4x5x_MCAN_DisableProtectedRegisters(this);
}

void TCAN4551::apply_bitrate_change(TCAN4x5x_MCAN_Nominal_Timing_Simple timing) {

    // Start by making protected registers accessible
    TCAN4x5x_MCAN_EnableProtectedRegisters(this);
    // Change the config
    TCAN4x5x_MCAN_ConfigureNominalTiming_Simple(this, &timing);
    // Disable access to protected registers
    TCAN4x5x_MCAN_DisableProtectedRegisters(this);

}

void TCAN4551::_tcan_irq_handler(void) {
    // Call the application handler if we have it
    if(this->irq_handler != NULL) {
//        this->irq_handler(this->id); // TODO - add interrupt type info
    }
}

void TCAN4551::copy_tcan_rx_header(CAN_Message* msg, TCAN4x5x_MCAN_RX_Header* header) {
    msg->id = header->ID;
    msg->len = header->DLC;
    msg->format = (header->XTD? CANExtended : CANStandard);
    msg->type = CANData;
}


