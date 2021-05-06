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

#if DEVICE_SPI && FEATURE_EXPERIMENTAL_API

#include "drivers/interfaces/InterfaceCAN.h"
#include "drivers/SPI.h"
#include "drivers/InterruptIn.h"
#include "drivers/DigitalOut.h"

#include "platform/NonCopyable.h"
#include "platform/PlatformMutex.h"

#include "TCAN4x5x_Data_Structs.h"

#define TCAN455X_TOTAL_FILTER_COUNT (MBED_CONF_TCAN455X_SID_FILTER_COUNT+MBED_CONF_TCAN455X_XID_FILTER_COUNT)

/**
 * TCAN455x driver
 */
class TCAN455x final : public mbed::interface::CAN, private mbed::NonCopyable<TCAN4551>
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
     * Initialize the chip. It is okay to call this multiple times.
     *
     * This must be called at startup to put the TCAN455x into a known state.
     * If it is not called the TCAN may go to sleep because it hasn't been configured properly.
     *
     * If the MCU is powered by the TCAN's internal LDO, this will shut down the MCU
     *
     * This is lazily called by all methods that access the TCAN over SPI
     *
     * @note calling reset will deinitialize the chip
     */
    void init(void);

    /**
     * Enter sleep mode
     *
     * @note The SPI interface of the TCAN455x is shut down in sleep mode.
     * You will have to wake the TCAN with an external signal (eg: reset, see datasheet for others)
     *
     * @note An incoming CAN frame will wake the TCAN automatically
     *
     * @note If the MCU is powered by the TCAN's internal LDO, this will
     * power down the TCAN and the MCU!
     */
    void sleep(void);

    /** interface::CAN overrides */

    /**
     * Sets the frequency of the CAN transmission
     *
     * @note this sets the base frequency during bus arbitration if
     * flexible data rate is enabled.
     *
     * @param[in] hz Frequency of bus
     */
    int frequency(int hz) override;

    int write(mbed::CANMessage msg) override;

    int read(mbed::CANMessage &msg, int handle = 0) override;

    void reset() override;

    void monitor(bool silent) override;

    int mode(mbed::interface::can::Mode mode) override;

    int filter(unsigned int id, unsigned int mask, CANFormat format = CANAny, int handle = 0) override;

    unsigned char rderror() override {
        return _read_errors;
    }

    unsigned char tderror() override {
        return _write_errors;
    }

    /** Attach a function to call whenever a CAN frame received interrupt is
     *  generated.
     *
     *  This function locks the deep sleep while a callback is attached
     *
     *  @param func A pointer to a void function, or 0 to set as none
     *  @param type Which CAN interrupt to attach the member function to (CAN::RxIrq for message received, CAN::TxIrq for transmitted or aborted, CAN::EwIrq for error warning, CAN::DoIrq for data overrun, CAN::WuIrq for wake-up, CAN::EpIrq for error passive, CAN::AlIrq for arbitration lost, CAN::BeIrq for bus error)
     *
     *  @note For the TCAN455x, the only one implemented is the RX IRQ
     */
    void attach(mbed::Callback<void()> func, IrqType type = IrqType::RxIrq) override;




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

protected:

    inline void initialize_if() {
        if(!_initialized) {
            init();
        }
    }

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

     /**
      * Used by the underlying TI driver
      */
     friend mbed::SPI &get_spi_handle(TCAN4551 *ptr);

protected:

    mbed::SPI _spi;                 /** SPI interface to TCAN4551 */
    mbed::InterruptIn _nint;        /** nINT interrupt input pin */
    mbed::DigitalOut* _rst;         /** RST output */
    mbed::DigitalOut* _wake_ctl;    /** Wake control output */

    unsigned char _read_errors;     /** Number of read errors */
    unsigned char _write_errors;    /** Number of write errors */

    TCAN4x5x_MCAN_CCCR_Config _cccr_config; /** TCAN configuration */

    /**
     * Array of filtered buffer control blocks
     * The SID filters come first (starting at index 0)
     * and the XID filters come after that (starting at MBED_CONF_TCAN455X_SID_FILTER_COUNT)
     */
    filtered_buffer_t _filtered_buffers[TCAN455X_TOTAL_FILTER_COUNT];
    int _standard_id_index;
    int _extended_id_index;

    mbed::Callback<void()>    _irq[IrqType::IrqCnt];

    bool _initialized = false;

    PlatformMutex _mutex;

};

#endif /* DEVICE_SPI && FEATURE_EXPERIMENTAL_API */

#endif /* MBED_TCAN455X_TCAN455X_H_ */
