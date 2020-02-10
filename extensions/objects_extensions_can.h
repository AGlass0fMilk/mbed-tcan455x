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
#ifndef OBJECTS_EXTENSIONS_CAN_H_
#define OBJECTS_EXTENSIONS_CAN_H_

#include "cmsis.h"
#include "PortNames.h"
#include "PeripheralNames.h"
#include "PinNames.h"

#if defined(DEVICE_CAN)
#include "can_api.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if MBED_CONF_HAL_ENABLE_OBJECTS_EXTENSIONS && defined(DEVICE_CAN)

struct can_s {
    void* instance_ptr;     /** Pointer to TCAN4551 object */
};

#endif /** MBED_CONF_HAL_ENABLE_OBJECTS_EXTENSIONS */

#ifdef __cplusplus
}
#endif

#endif /* OBJECTS_EXTENSIONS_CAN_H_ */
