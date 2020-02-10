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

/**
 * TCAN4551 driver
 */
class TCAN4551
{

public:

    TCAN4551(PinName mosi, PinName miso, PinName sclk, PinName csn, PinName nint_pin);

    ~TCAN4551();

    /**
     * Gets the TCAN4551 driver associated with the given CAN object
     * @param[in] can_handle Handle of Mbed CAN object
     * @retval tcan Corresponding TCAN driver object
     */
    static TCAN4551& get_tcan_handle(mbed::CAN& can_handle);

    void init(void);

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
    int write(CAN_Message msg, int cc);

    /**
     * Read a CANMessage from the TCAN4551
     */
    int read(CAN_Message* msg, int handle = 0);

    /**
     * Sets the CAN controller to the desired mode
     * @param[in] mode Mode to enter
     * @retval result 0 if mode change failed or unsupported
     *                1 if mode change was successful
     */
    int mode(CanMode mode);

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
    int filter(unsigned int id, unsigned int mask, CANFormat format = CANAny, int handle = 0);

    /**
     * Resets the TCAN4551
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

protected:

    /**
     * Internal interrupt handler
     */
    void _tcan_irq_handler(void);

protected:

    mbed::SPI spi;                  /** SPI interface to TCAN4551 */
    mbed::InterruptIn nint;         /** nINT interrupt input pin */

    can_irq_handler irq_handler;    /** IRQ handler function */
    uint32_t id;                    /** ID given by C++ API to can_irq_init */

    uint32_t irq_mask;              /** Bitflags for enabled IRQs */

    unsigned char read_errors;      /** Number of read errors */
    unsigned char write_errors;     /** Number of write errors */

};

#endif /* MBED_TCAN4551_TCAN4551_H_ */
