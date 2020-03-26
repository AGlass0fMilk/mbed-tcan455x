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

static TCAN4x5x_MCAN_SID_Filter standard_ids[MBED_CONF_TCAN4551_SID_FILTER_COUNT] = {0};
static int standard_id_index = 0;

static TCAN4x5x_MCAN_XID_Filter extended_ids[MBED_CONF_TCAN4551_XID_FILTER_COUNT] = {0};
static int extended_id_index = 0;

inline int filter_handle_to_index(int handle) {
    return handle-1;
}

inline int filter_index_to_handle(int index) {
    return index+1;
}

/*
 *  Internal function to allocate a standard ID filter handle
 *  @retval Index of allocated handle in static array, -1 if out of memory
 */
int alloc_sid_handle(void) {
    if(standard_id_index < MBED_CONF_TCAN4551_SID_FILTER_COUNT) {
        return standard_id_index++;
    } else {
        return -1;
    }
}

/*
 *  Internal function to allocate an extended ID filter handle
 *  @retval Index of allocated handle in static array, -1 if out of memory
 */int alloc_xid_handle(void) {
    if(extended_id_index < MBED_CONF_TCAN4551_XID_FILTER_COUNT) {
        return extended_id_index++;
    } else {
        return -1;
    }
}

TCAN4551::TCAN4551(PinName mosi, PinName miso, PinName sclk, PinName csn,
        PinName nint_pin) : spi(mosi, miso, sclk, csn, mbed::use_gpio_ssel), nint(nint_pin, PullUp),
        irq_handler(NULL), id(0), irq_mask(0), read_errors(0), write_errors(0) {
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
     *   - 3 SID filter elements
     *   - 3 XID Filter elements
     *   - 5 RX FIFO 0 elements
     *   - RX FIFO 0 supports data payloads up to 64 bytes
     *   - RX FIFO 1 and RX Buffer will not have any elements, but we still set their data payload sizes, even though it's not required
     *   - No TX Event FIFOs
     *   - 2 Transmit buffers supporting up to 64 bytes of data payload
     */
    TCAN4x5x_MRAM_Config MRAMConfiguration = {0};
    MRAMConfiguration.SIDNumElements = MBED_CONF_TCAN4551_SID_FILTER_COUNT; // Standard ID number of elements, you MUST have a filter written to MRAM for each element defined
    MRAMConfiguration.XIDNumElements = MBED_CONF_TCAN4551_XID_FILTER_COUNT; // Extended ID number of elements, you MUST have a filter written to MRAM for each element defined
    MRAMConfiguration.Rx0NumElements = 5;                       // RX0 Number of elements
    MRAMConfiguration.Rx0ElementSize = MRAM_64_Byte_Data;       // RX0 data payload size
    MRAMConfiguration.Rx1NumElements = 0;                       // RX1 number of elements
    MRAMConfiguration.Rx1ElementSize = MRAM_64_Byte_Data;       // RX1 data payload size
    MRAMConfiguration.RxBufNumElements = 0;                     // RX buffer number of elements
    MRAMConfiguration.RxBufElementSize = MRAM_64_Byte_Data;     // RX buffer data payload size
    MRAMConfiguration.TxEventFIFONumElements = 0;               // TX Event FIFO number of elements
    MRAMConfiguration.TxBufferNumElements = 2;                  // TX buffer number of elements
    MRAMConfiguration.TxBufferElementSize = MRAM_64_Byte_Data;  // TX buffer data payload size


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

    TCAN4x5x_MCAN_ConfigureInterruptEnable(this, &mcan_ie);         // Enable the appropriate registers


    /* Setup standard filters */
    for(int i = 0; i < MBED_CONF_TCAN4551_SID_FILTER_COUNT; i++) {
        TCAN4x5x_MCAN_SID_Filter* SID_ID = &standard_ids[i];
        *SID_ID = {0};
        SID_ID->SFT = TCAN4x5x_SID_SFT_CLASSIC;                      // SFT: Standard filter type. Configured as a classic filter
        SID_ID->SFEC = TCAN4x5x_SID_SFEC_DISABLED;                   // Standard filter element configuration, initially disabled
        SID_ID->SFID1 = 0x055;                                       // SFID1 (Classic mode Filter)
        SID_ID->SFID2 = 0x7FF;                                       // SFID2 (Classic mode Mask)
        TCAN4x5x_MCAN_WriteSIDFilter(this, i, SID_ID);               // Write to the MRAM
    }

    /* Setup extended filters */
    for(int i = 0; i < MBED_CONF_TCAN4551_XID_FILTER_COUNT; i++) {
        TCAN4x5x_MCAN_XID_Filter* XID_ID = &extended_ids[i];
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
    return 0;// TODO
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

    if(mcan_ir.RF0N) {
        TCAN4x5x_MCAN_RX_Header msg_header = {0};
        uint8_t num_bytes = 0;

        TCAN4x5x_MCAN_ClearInterrupts(this, &mcan_ir);

        num_bytes = TCAN4x5x_MCAN_ReadNextFIFO(this, RXFIFO0, &msg_header, msg->data);

        if(handle != 0 && handle != msg_header.ID) {
            return 0; // Received message was filtered out
        } else {
            return 1;
        }

    } else {
        return 0; // No message arrived
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
            filter_index = alloc_sid_handle();   // Try to allocate one that's available
            if(handle_ptr == -1) {               // No filters left to allocate :(
                return 0;
            }
        }

        // Get a pointer to the filter struct
        handle_ptr = &standard_ids[filter_index];

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
        if(filter_index < 0 || filter_index >= MBED_CONF_TCAN4551_XID_FILTER_COUNT) {
            return 0;
        }

        // If the user didn't provide a valid handle
        if(filter_index == -1) {
            filter_index = alloc_xid_handle();   // Try to allocate one that's available
            if(handle_ptr == -1) {               // No filters left to allocate :(
                return 0;
            }
        }

        // Get a pointer to the filter struct
        handle_ptr = &extended_ids[filter_index];

        // Configure the filter and write it to the controller
        handle_ptr->EFT = TCAN4x5x_XID_EFT_CLASSIC
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
    // TODO
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

void TCAN4551::_tcan_irq_handler(void) {
    // Call the application handler if we have it
    if(this->irq_handler != NULL) {
//        this->irq_handler(this->id); // TODO - add interrupt type info
    }
}


