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

/**
 * The user must define this pin map if they want to use multiple TCAN interfaces
 *
 * They must define default pins for mosi, miso, sclk, and nint if they are
 * using the default TCAN instance
 */
#if !MBED_CONF_TCAN4551_MULTIPLE_TCAN_ENABLE

#include "can_helper.h"

#include "platform/mbed_assert.h"

/**
 * Ensure default pins are properly configured if the application is
 * using the default TCAN pin mapping.
 */
MBED_STATIC_ASSERT((MBED_CONF_TCAN4551_DEFAULT_TCAN_MOSI != NC),
        "Default TCAN MOSI pin cannot be NC if the default pin mapping is used!");

MBED_STATIC_ASSERT(MBED_CONF_TCAN4551_DEFAULT_TCAN_MISO != NC,
        "Default TCAN MISO pin cannot be NC if the default pin mapping is used!");

MBED_STATIC_ASSERT(MBED_CONF_TCAN4551_DEFAULT_TCAN_SCLK != NC,
        "Default TCAN SCLK pin cannot be NC if the default pin mapping is used!");

MBED_STATIC_ASSERT(MBED_CONF_TCAN4551_DEFAULT_TCAN_NINT != NC,
        "Default TCAN MOSI pin cannot be NC if the default pin mapping is used!");

/**
 * Redefine a map similar to this if you're using multiple TCAN interfaces
 *
 * The last line should always end with a PinMap that is all "NC"
 */
const PinMapTCAN4551 PinMap_TCAN4551[] = {
        { .rd   = MBED_CONF_TCAN4551_DEFAULT_TCAN_MISO,
          .td   = MBED_CONF_TCAN4551_DEFAULT_TCAN_MOSI,
          .mosi = MBED_CONF_TCAN4551_DEFAULT_TCAN_MOSI,
          .miso = MBED_CONF_TCAN4551_DEFAULT_TCAN_MISO,
          .sclk = MBED_CONF_TCAN4551_DEFAULT_TCAN_SCLK,
          .csn  = MBED_CONF_TCAN4551_DEFAULT_TCAN_CSN,
          .nint = MBED_CONF_TCAN4551_DEFAULT_TCAN_NINT
        },
        { .rd = NC, .td = NC, .mosi = NC, .miso = NC, .sclk = NC, .csn = NC, .nint = NC }
};

#endif /** MBED_CONF_TCAN4551_MULTIPLE_TCAN_ENABLE */


