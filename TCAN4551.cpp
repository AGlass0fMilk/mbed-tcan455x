/*
 * TCAN4551.cpp
 *
 *  Created on: Feb 10, 2020
 *      Author: gdbeckstein
 */

#include "TCAN4551.h"

#include "platform/Callback.h"

TCAN4551::TCAN4551(PinName mosi, PinName miso, PinName sclk, PinName csn,
        PinName nint_pin) : spi(mosi, miso, sclk, csn), nint(nint_pin, PullUp),
        irq_handler(NULL), id(0), irq_mask(0), read_errors(0), write_errors(0) {
}

TCAN4551::~TCAN4551() {
    // TODO
}

TCAN4551& TCAN4551::get_tcan_handle(mbed::CAN& can_handle) {
    // TODO - remove this?
}

void TCAN4551::init(void) {
    // TODO
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
    return 0; // TODO
}

int TCAN4551::read(CAN_Message* msg, int handle) {
    return 0; // TODO
}

int TCAN4551::mode(CanMode mode) {
    return 0; // TODO
}

int TCAN4551::filter(unsigned int id, unsigned int mask, CANFormat format,
        int handle) {
    return 0; // TODO
}

void TCAN4551::reset(void) {
    // TODO
}

void TCAN4551::monitor(bool silent) {
    // TODO
}

void TCAN4551::_tcan_irq_handler(void) {
    // Call the application handler if we have it
    if(this->irq_handler != NULL) {
//        this->irq_handler(this->id); // TODO - add interrupt type info
    }
}
