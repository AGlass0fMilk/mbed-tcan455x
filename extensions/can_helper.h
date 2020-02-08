/**
 * ep-oc-mcu
 * Embedded Planet Open Core for Microcontrollers
 * 
 * Built with ARM Mbed-OS
 *
 * Copyright (c) 2019-2020 Embedded Planet, Inc.
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
 * limitations under the License.
 *
 */
#ifndef CAN_HELPER_H_
#define CAN_HELPER_H_

#ifdef __cplusplus
extern "C" {
#endif


typedef struct {

    /**
     * The rd and td pins are used to map a CAN object to a set of SPI pins.
     * The corresponding rd/td pins should be used to construct the CAN driver
     * by the application. They should be pins actually used by the TCAN SPI
     * interface. They may not be NC.
     *
     * This is what uniquely identifies each separate TCAN interface
     */
    PinName rd;
    PinName td;

    /**
     * These are the pins actually used to interface to the TCAN
     * controller/transceiver IC.
     */
    PinName mosi;
    PinName miso;
    PinName sclk;
    PinName csn;
    PinName nint;

} PinMapTCAN4551;

/**
 * This symbol is defined by default if tcan4551.multiple-tcan-enable is DISABLED
 *
 * If you are using multiple tcan interfaces, you must define this map in a
 * separate C file yourself.
 *
 * See default_can_pins.c for more information
 */
extern const PinMapTCAN4551 PinMap_TCAN4551[];


/**
 * Map CAN instance rd and td pins to TCAN instance and pins for SPI
 *
 * @retval index corresponding instance index in PinMap_TCAN4551 array
 */
int pin_instance_tcan(PinName rd, PinName td);


#ifdef __cplusplus
}
#endif


#endif /* CAN_HELPER_H_ */
