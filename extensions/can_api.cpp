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

#include "hal/can_api.h"

#include "platform/mbed_assert.h"

#include "tcan_helper.h"
#include "TCAN4551.h"

#if !DEVICE_SPI
#error TCAN driver requires the target device to have a SPI peripheral available!
#endif

#if !DEVICE_INTERRUPTIN
#error TCAN driver requires the target device to support interrupt inputs!
#endif

#define UNUSED(x) (void)(x)

#if DEVICE_CAN

/**
 * These are C functions declared in can_api.h
 */
extern "C" {

    void can_init(can_t *obj, PinName rd, PinName td) {

        // Make sure the pointer field is empty
        MBED_ASSERT(obj->instance_ptr == NULL);

        // Get the corresponding SPI pins
        const PinMapTCAN4551* pins = &PinMap_TCAN4551[pin_instance_tcan(rd, td)];

        // Make sure the rd/td are valid
        MBED_ASSERT((pins->rd != NC) && (pins->td != NC));

        mbed::DigitalOut* rst = nullptr;
        mbed::DigitalOut* wake_ctl = nullptr;

#if (MBED_CONF_TCAN4551_NUMBER_OF_TCANS == 1)
        if(MBED_CONF_TCAN4551_DEFAULT_TCAN_RST != NC) {
            rst = new mbed::DigitalOut(MBED_CONF_TCAN4551_DEFAULT_TCAN_RST, 0); // TODO where should this get deleted? Use SharedPtr?
        }
#endif

#if (MBED_CONF_TCAN4551_DEFAULT_TCAN_WAKE_CTL == NC)
#warning TCAN4551 wake control pin not configured. The MCU will be unable to autonomously wake the TCAN from sleep mode.
#endif
        if(MBED_CONF_TCAN4551_DEFAULT_TCAN_WAKE_CTL != NC) {
            wake_ctl = new mbed::DigitalOut(MBED_CONF_TCAN4551_DEFAULT_TCAN_WAKE_CTL, 0);
        }


        // Create the TCAN driver object
        TCAN4551* tcan = new TCAN4551(pins->mosi,
                pins->miso, pins->sclk, pins->csn, pins->nint, rst, wake_ctl);

        obj->instance_ptr = tcan;

        tcan->init();
    }

    void can_init_direct(can_t *obj, const can_pinmap_t *pinmap) {
        can_init(obj, pinmap->rd_pin, pinmap->td_pin);
    }

    void can_init_freq(can_t *obj, PinName rd, PinName td, int hz) {
        can_init(obj, rd, td);
        can_frequency(obj, hz);
    }

    void can_init_freq_direct(can_t *obj, const can_pinmap_t *pinmap, int hz) {
        can_init(obj, pinmap->rd_pin, pinmap->td_pin);
        can_frequency(obj, hz);
    }

    void can_free(can_t *obj) {
        MBED_ASSERT(obj->instance_ptr != NULL);
        delete (TCAN4551*)obj->instance_ptr;
        obj->instance_ptr = NULL;
    }

    int can_frequency(can_t *obj, int hz) {
        TCAN4551* tcan = (TCAN4551*) obj->instance_ptr;
        return tcan->frequency(hz);
    }

    void can_irq_init(can_t *obj, can_irq_handler handler, uint32_t id) {
        TCAN4551* tcan = (TCAN4551*) obj->instance_ptr;
        tcan->attach_irq_handler(handler, id);
    }

    void can_irq_free(can_t *obj) {
        TCAN4551* tcan = (TCAN4551*) obj->instance_ptr;
        tcan->detach_irq_handler();
    }

    void can_irq_set(can_t *obj, CanIrqType irq, uint32_t enable) {
        TCAN4551* tcan = (TCAN4551*) obj->instance_ptr;
        if(enable) {
            tcan->enable_irq(irq);
        } else {
            tcan->disable_irq(irq);
        }
    }

    int can_write(can_t *obj, CAN_Message msg, int cc) {
        TCAN4551* tcan = (TCAN4551*) obj->instance_ptr;
        return tcan->write(msg, cc);
    }

    int can_read(can_t *obj, CAN_Message *msg, int handle) {
        TCAN4551* tcan = (TCAN4551*) obj->instance_ptr;
        return tcan->read(msg, handle);
    }

    int can_mode(can_t *obj, CanMode mode) {
        TCAN4551* tcan = (TCAN4551*) obj->instance_ptr;
        return tcan->mode(mode);
    }

    int can_filter(can_t *obj, uint32_t id, uint32_t mask,
            CANFormat format, int32_t handle) {
        TCAN4551* tcan = (TCAN4551*) obj->instance_ptr;
        return tcan->filter(id, mask, format, handle);
    }

    void can_reset(can_t *obj) {
        TCAN4551* tcan = (TCAN4551*) obj->instance_ptr;
        tcan->reset();
    }

    unsigned char can_rderror(can_t *obj) {
        TCAN4551* tcan = (TCAN4551*) obj->instance_ptr;
        return tcan->get_read_errors();
    }

    unsigned char can_tderror(can_t *obj) {
        TCAN4551* tcan = (TCAN4551*) obj->instance_ptr;
        return tcan->get_write_errors();
    }

    void can_monitor(can_t *obj, int silent) {
        TCAN4551* tcan = (TCAN4551*) obj->instance_ptr;
        tcan->monitor(silent);
    }

    /**
     * Generates pinmaps for rd and td from TCAN pinmaps
     * @param[out] rd_pinmap Pointer to rd_pinmap
     * @param[out] td_pinmap Pointer to td_pinmap
     */
    void get_pinmaps(PinMap* rd_pinmap_ptr, PinMap* td_pinmap_ptr) {

        // Dynamically generate the supported rd pinmap from tcan pin map
        static bool pinmaps_inited = false;
        static PinMap rd_pinmaps[MBED_CONF_TCAN4551_NUMBER_OF_TCANS+1];
        static PinMap td_pinmaps[MBED_CONF_TCAN4551_NUMBER_OF_TCANS+1];

        if(!pinmaps_inited) {
            for(int i = 0; i < MBED_CONF_TCAN4551_NUMBER_OF_TCANS; i++) {
                PinMap* rd_map = &rd_pinmaps[i];
                rd_map->function = 0;
                rd_map->peripheral = 0;
                rd_map->pin = PinMap_TCAN4551[i].rd;

                PinMap* td_map = &td_pinmaps[i];
                td_map->function = 0;
                td_map->peripheral = 0;
                td_map->pin = PinMap_TCAN4551[i].td;
            }

            // Set the last ones to {NC, NC, 0}
            rd_pinmaps[MBED_CONF_TCAN4551_NUMBER_OF_TCANS] = {NC, NC, 0};
            td_pinmaps[MBED_CONF_TCAN4551_NUMBER_OF_TCANS] = {NC, NC, 0};
            pinmaps_inited = true;
        }
        rd_pinmap_ptr = rd_pinmaps;
        td_pinmap_ptr = td_pinmaps;

        // Suppress "unused" warnings...
        UNUSED(rd_pinmap_ptr);
        UNUSED(td_pinmap_ptr);
    }

    /** Get the pins that support CAN RD
     *
     * Return a PinMap array of pins that support CAN RD. The
     * array is terminated with {NC, NC, 0}.
     *
     * @return PinMap array
     */
    const PinMap *can_rd_pinmap(void) {
        PinMap *rd_pinmap = NULL, *td_pinmap = NULL;
        get_pinmaps(rd_pinmap, td_pinmap);
        return rd_pinmap;
    }

    /** Get the pins that support CAN TD
     *
     * Return a PinMap array of pins that support CAN TD. The
     * array is terminated with {NC, NC, 0}.
     *
     * @return PinMap array
     */
    const PinMap *can_td_pinmap(void) {
        PinMap *rd_pinmap = NULL, *td_pinmap = NULL;
        get_pinmaps(rd_pinmap, td_pinmap);
        return td_pinmap;
    }

}

#endif
