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

#include "tcan_helper.h"
#include <stddef.h>

int pin_instance_tcan(PinName rd, PinName td) {

    /* Search static pin map for corresponding TCAN instance */
    size_t index;
    for(index = 0;
        !((PinMap_TCAN4551[index].rd == NC) &&
          (PinMap_TCAN4551[index].td == NC));
        index++) {

        // Stop if we've found a match
        if((PinMap_TCAN4551[index].rd == rd) &&
           (PinMap_TCAN4551[index].td == td)) {
            break;
        }

        // If that never happens, we'll end up with the last index (all NC)

    }

    return index;

}

