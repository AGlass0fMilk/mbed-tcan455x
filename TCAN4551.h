/*
 * TCAN4551.h
 *
 *  Created on: Feb 10, 2020
 *      Author: gdbeckstein
 */

#ifndef MBED_TCAN4551_TCAN4551_H_
#define MBED_TCAN4551_TCAN4551_H_

#include "drivers/CAN.h"
#include "drivers/SPI.h"
#include "drivers/InterruptIn.h"

#include "TCAN4x5x_Data_Structs.h"

#define TCAN4551_TOTAL_FILTER_COUNT (MBED_CONF_TCAN4551_SID_FILTER_COUNT+MBED_CONF_TCAN4551_XID_FILTER_COUNT)

/**
 * TCAN4551 driver
 */
class TCAN4551
{

protected:

    /**
     * Filtered buffer control structure
     *
     * Since filtered messages are stored in a FIFO, we
     * must store them in a buffer if the application
     * reads the FIFO looking for messages from one filter
     * and subsequently dequeues messages from another filter
     */
    typedef struct filtered_buffer_t {
        TCAN4x5x_MCAN_RX_Header rx_header;
        bool new_data_available;
        union {
            TCAN4x5x_MCAN_SID_Filter sid_filter;
            TCAN4x5x_MCAN_XID_Filter xid_filter;
        };
        uint8_t data[8]; // TODO - update this to be configurable in size
    } filtered_buffer_t;


public:

    TCAN4551(PinName mosi, PinName miso, PinName sclk, PinName csn, PinName nint_pin);

    virtual ~TCAN4551();

    /**
     * Gets the TCAN4551 driver associated with the given CAN object
     * @param[in] can_handle Handle of Mbed CAN object
     * @retval tcan Corresponding TCAN driver object
     */
//    static TCAN4551& get_tcan_handle(mbed::CAN& can_handle);

    virtual void init(void);

    /**
     * Sets the frequency of the CAN transmission
     *
     * @note this sets the base frequency during bus arbitration if
     * flexible data rate is enabled.
     *
     * @param[in] hz Frequency of bus
     */
    int frequency(int hz);

    /**
     * Attaches an interrupt handler
     * @param[in] interrupt handler
     * @param[in] id to use
     */
    void attach_irq_handler(can_irq_handler handler, uint32_t id);

    /**
     * Detaches interrupt handler
     */
    void detach_irq_handler(void);

    /**
     * Enables the given CanIrqType
     *
     * @param[in] irq_type Type of irq to enable
     */
    void enable_irq(CanIrqType type);

    /**
     * Disables the given CanIrqType
     *
     * @param[in] irq_type Type of irq to disable
     */
    void disable_irq(CanIrqType type);

    /**
     * Write a CANMessage to the TCAN4551
     */
    virtual int write(CAN_Message msg, int cc);

    /**
     * Read a CANMessage from the TCAN4551
     */
    virtual int read(CAN_Message* msg, int handle = 0);

    /**
     * Sets the CAN controller to the desired mode
     * @param[in] mode Mode to enter
     * @retval result 0 if mode change failed or unsupported
     *                1 if mode change was successful
     */
    virtual int mode(CanMode mode);

    /** Filter out incoming messages
     *
     *  @param id the id to filter on
     *  @param mask the mask applied to the id
     *  @param format format to filter on (Default CANAny)
     *  @param handle message filter handle (Optional)
     *
     *  @returns
     *    0 if filter change failed or unsupported,
     *    new filter handle if successful
     */
    virtual int filter(unsigned int id, unsigned int mask, CANFormat format = CANAny, int handle = 0);

    /**
     * Resets the TCAN4551
     *
     * @note init must be called again after calling reset!
     */
    void reset(void);

    /**
     * Puts or removes the TCAN interface into silent monitoring mode
     * @param silent boolean indicating whether to go into silent mode or not
     */
    void monitor(bool silent);



#if MBED_CONF_TCAN4551_ENABLE_FD && DEVICE_CANFD

    /**
     * Sets the packet size of the transmission
     *
     * @param[in] size Size of transmitted packets
     *
     * @note this is limited by TODO [configuration parameter name]
     * and may be up to 8 bytes with classical CAN and up to 64 bytes
     * with CAN-FD
     */
    void set_packet_size(int size);

    /**
     * Sets the frequency of the data bits
     * @param[in] hz Data frame frequency
     */
    void data_frequency(int hz);

    /**
     * Enable bit rate switching during data frame
     */
    void enable_brs(void);

    /**
     * Disable bit rate switching during data frame
     */
    void disable_brs(void);

#endif

    unsigned char get_read_errors() const {
        return read_errors;
    }

    unsigned char get_write_errors() const {
        return write_errors;
    }

    mbed::SPI& get_spi_handle() {
        return spi;
    }

protected:

    /**
     * Internal function to apply bit rate changes
     * @param[in] bit rate struct to send to TCAN4x5x
     */
    void apply_bitrate_change(TCAN4x5x_MCAN_Nominal_Timing_Simple timing);

    /**
     * Internal interrupt handler
     */
    void _tcan_irq_handler(void);

    /**
     * 0 is not a valid handle, so map the index to a different number (add 1)
     */
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
    int alloc_sid_handle_index(void) {
        if(standard_id_index < MBED_CONF_TCAN4551_SID_FILTER_COUNT) {
            return standard_id_index++;
        } else {
            return -1;
        }
    }

    /*
     *  Internal function to allocate an extended ID filter handle
     *  @retval Index of allocated handle in static array, -1 if out of memory
     */int alloc_xid_handle_index(void) {
        if(extended_id_index < MBED_CONF_TCAN4551_XID_FILTER_COUNT) {
            return (MBED_CONF_TCAN4551_SID_FILTER_COUNT + extended_id_index++);
        } else {
            return -1;
        }
    }

     /**
      * Copies a TCAN4551-format header over to Mbed's CAN_Message format
      * @param[in] msg Mbed CAN_Message struct destination
      * @param[in] header TCAN4551 rx header struct source
      */
     static void copy_tcan_rx_header(CAN_Message* msg, TCAN4x5x_MCAN_RX_Header* header);

protected:

    mbed::SPI spi;                  /** SPI interface to TCAN4551 */
    mbed::InterruptIn nint;         /** nINT interrupt input pin */

    can_irq_handler irq_handler;    /** IRQ handler function */
    uint32_t id;                    /** ID given by C++ API to can_irq_init */

    uint32_t irq_mask;              /** Bitflags for enabled IRQs */

    unsigned char read_errors;      /** Number of read errors */
    unsigned char write_errors;     /** Number of write errors */

    TCAN4x5x_MCAN_CCCR_Config cccr_config; /** TCAN configuration */

    /**
     * Array of filtered buffer control blocks
     * The SID filters come first (starting at index 0)
     * and the XID filters come after that (starting at MBED_CONF_TCAN4551_SID_FILTER_COUNT)
     */
    filtered_buffer_t filtered_buffers[TCAN4551_TOTAL_FILTER_COUNT];
    int standard_id_index;
    int extended_id_index;

};

#endif /* MBED_TCAN4551_TCAN4551_H_ */
