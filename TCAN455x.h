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

#ifndef MBED_TCAN455X_TCAN455X_H_
#define MBED_TCAN455X_TCAN455X_H_

#include "drivers/CAN.h"
#include "drivers/SPI.h"
#include "drivers/InterruptIn.h"
#include "drivers/DigitalOut.h"

#include "TCAN4x5x_Data_Structs.h"

#define TCAN455X_TOTAL_FILTER_COUNT (MBED_CONF_TCAN455X_SID_FILTER_COUNT+MBED_CONF_TCAN455X_XID_FILTER_COUNT)

/**
 * TCAN455x driver
 */
class TCAN455x
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

    /**
     * Create a TCAN455x interface
     * @param[in] mosi MOSI pin name to use for SPI
     * @param[in] miso MISO pin name to use for SPI
     * @param[in] sclk SCLK pin name to use for SPI
     * @param[in] csn Chip select pin name to use for SPI
     * @param[in] nint_pin Interrupt in pin name to use
     * @param[in] rst (Optional) Hardware reset control, if used (active high)
     * @param[in] wake_ctl (Optional) Wake control output (active high). Pulls WAKE pin low through an external transistor
     */
    TCAN455x(PinName mosi, PinName miso, PinName sclk, PinName csn, PinName nint_pin,
            mbed::DigitalOut* rst = nullptr, mbed::DigitalOut* wake_ctl = nullptr);

    virtual ~TCAN455x();

    /**
     * Gets the TCAN455x driver associated with the given CAN object
     * @param[in] can_handle Handle of Mbed CAN object
     * @retval tcan Corresponding TCAN driver object
     */
//    static TCAN455x& get_tcan_handle(mbed::CAN& can_handle);

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
     * Write a CANMessage to the TCAN455x
     */
    virtual int write(CAN_Message msg, int cc);

    /**
     * Read a CANMessage from the TCAN455x
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
     * Resets the TCAN455x
     *
     * @note init must be called again after calling reset!
     */
    void reset(void);

    /**
     * Puts or removes the TCAN interface into silent monitoring mode
     * @param silent boolean indicating whether to go into silent mode or not
     */
    void monitor(bool silent);



#if MBED_CONF_TCAN455X_ENABLE_FD && DEVICE_CANFD

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
        if(standard_id_index < MBED_CONF_TCAN455X_SID_FILTER_COUNT) {
            return standard_id_index++;
        } else {
            return -1;
        }
    }

    /*
     *  Internal function to allocate an extended ID filter handle
     *  @retval Index of allocated handle in static array, -1 if out of memory
     */int alloc_xid_handle_index(void) {
        if(extended_id_index < MBED_CONF_TCAN455X_XID_FILTER_COUNT) {
            return (MBED_CONF_TCAN455X_SID_FILTER_COUNT + extended_id_index++);
        } else {
            return -1;
        }
    }

     /**
      * Copies a TCAN455x-format header over to Mbed's CAN_Message format
      * @param[in] msg Mbed CAN_Message struct destination
      * @param[in] header TCAN455x rx header struct source
      */
     static void copy_tcan_rx_header(CAN_Message* msg, TCAN4x5x_MCAN_RX_Header* header);

protected:

    mbed::SPI spi;                  /** SPI interface to TCAN455x */
    mbed::InterruptIn nint;         /** nINT interrupt input pin */
    mbed::DigitalOut* _rst;         /** RST output */
    mbed::DigitalOut* _wake_ctl;   /** Wake control output */

    can_irq_handler irq_handler;    /** IRQ handler function */
    uint32_t id;                    /** ID given by C++ API to can_irq_init */

    uint32_t irq_mask;              /** Bitflags for enabled IRQs */

    unsigned char read_errors;      /** Number of read errors */
    unsigned char write_errors;     /** Number of write errors */

    TCAN4x5x_MCAN_CCCR_Config cccr_config; /** TCAN configuration */

    /**
     * Array of filtered buffer control blocks
     * The SID filters come first (starting at index 0)
     * and the XID filters come after that (starting at MBED_CONF_TCAN455X_SID_FILTER_COUNT)
     */
    filtered_buffer_t filtered_buffers[TCAN455X_TOTAL_FILTER_COUNT];
    int standard_id_index;
    int extended_id_index;

};

#endif /* MBED_TCAN455X_TCAN455X_H_ */
