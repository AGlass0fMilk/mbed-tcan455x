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

#include "utest/utest.h"
#include "unity/unity.h"
#include "greentea-client/test_env.h"
#include "utest/utest_default_handlers.h"

#include "TCAN455x.h"

#include "platform/mbed_wait_api.h"

// TARGET_NRF52840_DK
#define TCAN455X_MOSI_PIN_1 P0_29
#define TCAN455X_MISO_PIN_1 P0_28
#define TCAN455X_SCLK_PIN_1 P0_30
#define TCAN455X_CS_PIN_1   P0_4
#define TCAN455X_NINT_PIN_1 P0_3
#define TCAN455X_RST_PIN_1  P0_31

#define TCAN455X_MOSI_PIN_2 P0_26
#define TCAN455X_MISO_PIN_2 P0_2
#define TCAN455X_SCLK_PIN_2 P0_27
#define TCAN455X_CS_PIN_2   P1_15
#define TCAN455X_NINT_PIN_2 P1_14
#define TCAN455X_RST_PIN_2  P1_13

/**
 * TODO test CAN-FD (once that's developed)
 */

mbed::DigitalOut can1_rst(TCAN455X_RST_PIN_1, 0);
mbed::DigitalOut can2_rst(TCAN455X_RST_PIN_2, 0);

#define TCAN455X_INSTANCE_1 TCAN455x can1(TCAN455X_MOSI_PIN_1, TCAN455X_MISO_PIN_1, TCAN455X_SCLK_PIN_1, TCAN455X_CS_PIN_1, TCAN455X_NINT_PIN_1, &can1_rst)
#define TCAN455X_INSTANCE_2 TCAN455x can2(TCAN455X_MOSI_PIN_2, TCAN455X_MISO_PIN_2, TCAN455X_SCLK_PIN_2, TCAN455X_CS_PIN_2, TCAN455X_NINT_PIN_2, &can2_rst)

using namespace utest::v1;

void test_unfiltered_messages() {
    TCAN455X_INSTANCE_1;
    TCAN455X_INSTANCE_2;
    int result = 0;
    mbed::CANMessage tx_message;
    mbed::CANMessage rx_message;
    tx_message.id = 0x123;
    tx_message.format = CANStandard;
    tx_message.len = 8;
    tx_message.data[0] = 1;

    /* Send unfiltered message with SID */

    /* Read 1 message (should not receive anything since nothing sent) */
    result = can1.read(rx_message, 0);
    TEST_ASSERT_EQUAL(0, result);

    /* Send 1 message */
    result = can2.write(tx_message);
    TEST_ASSERT_EQUAL(1, result);

    /* Read 1 message */
    result = can1.read(rx_message, 0);
    TEST_ASSERT_EQUAL(1, result);
    TEST_ASSERT_EQUAL(CANStandard, rx_message.format);
    TEST_ASSERT_EQUAL(tx_message.id, rx_message.id);

    /* Try to read another message (should fail) */
    result = can1.read(rx_message, 0);
    TEST_ASSERT_EQUAL(0, result);

    /* Send unfiltered message with XID */
    tx_message.format = CANExtended;

    /* Read 1 message (should not receive anything since nothing sent) */
    result = can1.read(rx_message, 0);
    TEST_ASSERT_EQUAL(0, result);

    /* Send 1 message with matching ID */
    result = can2.write(tx_message);
    TEST_ASSERT_EQUAL(1, result);

    /* Read 1 message from filter */
    result = can1.read(rx_message, 0);
    TEST_ASSERT_EQUAL(1, result);
    TEST_ASSERT_EQUAL(CANExtended, rx_message.format);
    TEST_ASSERT_EQUAL(tx_message.id, rx_message.id);

    /* Try to read another message (should fail) */
    result = can1.read(rx_message, 0);
    TEST_ASSERT_EQUAL(0, result);

}

void test_fail_write_can_any() {
    /* Try to write a CANAny message, it should fail */
    TCAN455X_INSTANCE_1;
    TCAN455X_INSTANCE_2;
    int result = 0;
    mbed::CANMessage tx_message;
    tx_message.id = 0x124;
    tx_message.format = CANAny;
    tx_message.len = 8;
    tx_message.data[0] = 1;

    /* Send 1 message with non-matching ID */
    result = can2.write(tx_message);
    TEST_ASSERT_EQUAL(0, result);
}

void test_invalid_sid_filter() {
    TCAN455X_INSTANCE_1;
    TCAN455X_INSTANCE_2;
    int handle;

    /* Test failure to add standard filter with large ID */
    handle = can1.filter(0xFFF, 0x7FF, CANStandard);
    TEST_ASSERT_EQUAL(0, handle);

    /* Test failure to add standard filter with invalid handle */
    handle = can1.filter(0x1234, 0x7FF, CANStandard, TCAN455X_TOTAL_FILTER_COUNT+1);
    TEST_ASSERT_EQUAL(0, handle);
}

void test_too_many_sid_filters() {
    TCAN455X_INSTANCE_1;
    TCAN455X_INSTANCE_2;
    int handle;

    /* Add too many SID filters */
    for(int i = 0; i < MBED_CONF_TCAN455X_SID_FILTER_COUNT+1; i++) {
        handle = can1.filter(i, i, CANStandard);
        if(i < MBED_CONF_TCAN455X_SID_FILTER_COUNT) {
            TEST_ASSERT_NOT_EQUAL(0, handle);
        }
    }

    TEST_ASSERT_EQUAL(0, handle);
}

void test_sid_filter() {
    TCAN455X_INSTANCE_1;
    TCAN455X_INSTANCE_2;
    int handle;
    int result = 0;
    mbed::CANMessage tx_message;
    mbed::CANMessage rx_message;
    tx_message.id = 0x124;
    tx_message.format = CANStandard;
    tx_message.len = 8;
    tx_message.data[0] = 1;

    handle = can1.filter(0x123, 0x7FF, CANStandard);

    /* Send 1 message with non-matching ID */
    result = can2.write(tx_message);
    TEST_ASSERT_EQUAL(1, result);

    /* Read 1 message from filter (should not receive anything since ID does not match) */
    result = can1.read(rx_message, handle);
    TEST_ASSERT_EQUAL(0, result);

    /* Send 1 message with matching ID */
    tx_message.id = 0x123;
    result = can2.write(tx_message);
    TEST_ASSERT_EQUAL(1, result);

    /* Read 1 message from filter */
    result = can1.read(rx_message, handle);
    TEST_ASSERT_EQUAL(1, result);
    TEST_ASSERT_EQUAL(tx_message.id, rx_message.id);

    /* Try to read another message (should fail) */
    result = can1.read(rx_message, handle);
    TEST_ASSERT_EQUAL(0, result);

}

void test_reassigned_sid_filter() {
    TCAN455X_INSTANCE_1;
    TCAN455X_INSTANCE_2;
    int handle;
    int result = 0;
    mbed::CANMessage tx_message;
    mbed::CANMessage rx_message;
    tx_message.id = 0x123;
    tx_message.format = CANStandard;
    tx_message.len = 8;
    tx_message.data[0] = 1;

    handle = can1.filter(0x123, 0x7FF, CANStandard);

    /* Send 1 message with matching ID */
    result = can2.write(tx_message);
    TEST_ASSERT_EQUAL(1, result);

    /* Read 1 message from filter */
    result = can1.read(rx_message, handle);
    TEST_ASSERT_EQUAL(1, result);

    /* Try to read another message (should fail) */
    result = can1.read(rx_message, handle);
    TEST_ASSERT_EQUAL(0, result);

    /* Reassigned SID handle with new ID */
    handle = can1.filter(0x124, 0x7FF, CANStandard, handle);
    TEST_ASSERT_NOT_EQUAL(0, handle);

    /* Send 1 message with matching ID */
    tx_message.id = 0x124;
    result = can2.write(tx_message);
    TEST_ASSERT_EQUAL(1, result);

    /* Read 1 message from filter */
    result = can1.read(rx_message, handle);
    TEST_ASSERT_EQUAL(1, result);
    TEST_ASSERT_EQUAL(tx_message.id, rx_message.id);

    /* Try to read another message (should fail) */
    result = can1.read(rx_message, handle);
    TEST_ASSERT_EQUAL(0, result);
}

//void test_sid_filter_overflow() {
//    TCAN455X_INSTANCE_1;
//    TCAN455X_INSTANCE_2;
//    int handle;
//    int result = 0;
//    unsigned char rderror = 0;
//    mbed::CANMessage tx_message;
//    mbed::CANMessage rx_message;
//    tx_message.id = 0x123;
//    tx_message.format = CANStandard;
//    tx_message.len = 8;
//    tx_message.data[0] = 1;
//
//    handle = can1.filter(0x123, 0x7FF, CANStandard);
//
//    /* Cause overflow */
//    for(int i = 0; i < MBED_CONF_TCAN455X_RX1_FIFO_SIZE+1; i++) {
//        can2.write(tx_message);
//        wait_us(1000);
//    }
//
//    /* Read packets, expect only RX1_FIFO_SIZE */
//    for(int i = 0; i < MBED_CONF_TCAN455X_RX1_FIFO_SIZE+1; i++ ) {
//        result = can1.read(rx_message, handle);
//        if(i < MBED_CONF_TCAN455X_RX1_FIFO_SIZE) {
//            TEST_ASSERT_EQUAL(1, result);
//        } else {
//            TEST_ASSERT_EQUAL(0, result);
//        }
//    }
//
//    rderror = can1.rderror();
//    TEST_ASSERT_EQUAL(1, rderror);
//
//}

void test_multiple_sid_filters()
{
    TCAN455X_INSTANCE_1;
    TCAN455X_INSTANCE_2;
    int handle1, handle2, handle3;
    int result = 0;
    mbed::CANMessage tx_message;
    mbed::CANMessage rx_message;
    tx_message.id = 0x123;
    tx_message.format = CANStandard;
    tx_message.len = 8;
    tx_message.data[0] = 1;

    handle1 = can1.filter(0x123, 0x7FF, CANStandard);
    handle2 = can1.filter(0x124, 0x7FF, CANStandard);
    handle3 = can1.filter(0x125, 0x7FF, CANStandard);

    /* Send one to each of the above filters */
    result = can2.write(tx_message);
    TEST_ASSERT_EQUAL(1, result);
    tx_message.id = 0x124;
    result = can2.write(tx_message);
    TEST_ASSERT_EQUAL(1, result);
    tx_message.id = 0x125;
    result = can2.write(tx_message);
    TEST_ASSERT_EQUAL(1, result);

    /* Read each filter, should get 1 per filter */
    result = can1.read(rx_message, handle1);
    TEST_ASSERT_EQUAL(1, result);
    TEST_ASSERT_EQUAL(0x123, rx_message.id);
    result = can1.read(rx_message, handle2);
    TEST_ASSERT_EQUAL(1, result);
    TEST_ASSERT_EQUAL(0x124, rx_message.id);
    result = can1.read(rx_message, handle3);
    TEST_ASSERT_EQUAL(1, result);
    TEST_ASSERT_EQUAL(0x125, rx_message.id);

    /* Try to read another message (should fail) */
    result = can1.read(rx_message, handle1);
    TEST_ASSERT_EQUAL(0, result);
    result = can1.read(rx_message, handle2);
    TEST_ASSERT_EQUAL(0, result);
    result = can1.read(rx_message, handle3);
    TEST_ASSERT_EQUAL(0, result);
}

void test_invalid_xid_filter()
{
    TCAN455X_INSTANCE_1;
    TCAN455X_INSTANCE_2;
    int handle;

    /* Test failure to add standard filter with large ID */
    handle = can1.filter(0xFFFFFFFF, 0x1FFFFFFF, CANExtended);
    TEST_ASSERT_EQUAL(0, handle);

    /* Test failure to add standard filter with invalid handle */
    handle = can1.filter(0x1234, 0x1FFFFFFF, CANExtended, TCAN455X_TOTAL_FILTER_COUNT+1);
    TEST_ASSERT_EQUAL(0, handle);
}

void test_too_many_xid_filters()
{
    TCAN455X_INSTANCE_1;
    TCAN455X_INSTANCE_2;
    int handle;

    /* Add too many XID filters */
    for(int i = 0; i < MBED_CONF_TCAN455X_XID_FILTER_COUNT+1; i++) {
        handle = can1.filter(i, i, CANExtended);
        if(i < MBED_CONF_TCAN455X_XID_FILTER_COUNT) {
            TEST_ASSERT_NOT_EQUAL(0, handle);
        }
    }

    TEST_ASSERT_EQUAL(0, handle);
}

void test_xid_filter()
{
    TCAN455X_INSTANCE_1;
    TCAN455X_INSTANCE_2;
    int handle;
    int result = 0;
    mbed::CANMessage tx_message;
    mbed::CANMessage rx_message;
    tx_message.id = 0x1235;
    tx_message.format = CANExtended;
    tx_message.len = 8;
    tx_message.data[0] = 1;

    handle = can1.filter(0x1234, 0x7FF, CANExtended);

    /* Send 1 message with non-matching ID */
    result = can2.write(tx_message);
    TEST_ASSERT_EQUAL(1, result);

    /* Read 1 message from filter (should not receive anything since ID does not match) */
    result = can1.read(rx_message, handle);
    TEST_ASSERT_EQUAL(0, result);

    /* Send 1 message with matching ID */
    tx_message.id = 0x1234;
    result = can2.write(tx_message);
    TEST_ASSERT_EQUAL(1, result);

    /* Read 1 message from filter */
    result = can1.read(rx_message, handle);
    TEST_ASSERT_EQUAL(1, result);
    TEST_ASSERT_EQUAL(tx_message.id, rx_message.id);

    /* Try to read another message (should fail) */
    result = can1.read(rx_message, handle);
    TEST_ASSERT_EQUAL(0, result);
}

void test_reassigned_xid_filter() {
    TCAN455X_INSTANCE_1;
    TCAN455X_INSTANCE_2;
    int handle;
    int result = 0;
    mbed::CANMessage tx_message;
    mbed::CANMessage rx_message;
    tx_message.id = 0x123;
    tx_message.format = CANExtended;
    tx_message.len = 8;
    tx_message.data[0] = 1;

    handle = can1.filter(0x123, 0x7FF, CANExtended);

    /* Send 1 message with matching ID */
    result = can2.write(tx_message);
    TEST_ASSERT_EQUAL(1, result);

    /* Read 1 message from filter */
    result = can1.read(rx_message, handle);
    TEST_ASSERT_EQUAL(1, result);

    /* Try to read another message (should fail) */
    result = can1.read(rx_message, handle);
    TEST_ASSERT_EQUAL(0, result);

    /* Reassigned SID handle with new ID */
    handle = can1.filter(0x124, 0x7FF, CANExtended, handle);
    TEST_ASSERT_NOT_EQUAL(0, handle);

    /* Send 1 message with matching ID */
    tx_message.id = 0x124;
    result = can2.write(tx_message);
    TEST_ASSERT_EQUAL(1, result);

    /* Read 1 message from filter */
    result = can1.read(rx_message, handle);
    TEST_ASSERT_EQUAL(1, result);
    TEST_ASSERT_EQUAL(tx_message.id, rx_message.id);

    /* Try to read another message (should fail) */
    result = can1.read(rx_message, handle);
    TEST_ASSERT_EQUAL(0, result);
}

void test_multiple_xid_filters()
{
    TCAN455X_INSTANCE_1;
    TCAN455X_INSTANCE_2;
    int handle1, handle2, handle3;
    int result = 0;
    mbed::CANMessage tx_message;
    mbed::CANMessage rx_message;
    tx_message.id = 0x1234;
    tx_message.format = CANExtended;
    tx_message.len = 8;
    tx_message.data[0] = 1;

    handle1 = can1.filter(0x1234, 0x7FF, CANExtended);
    handle2 = can1.filter(0x1235, 0x7FF, CANExtended);
    handle3 = can1.filter(0x1236, 0x7FF, CANExtended);

    /* Send one to each of the above filters */
    result = can2.write(tx_message);
    TEST_ASSERT_EQUAL(1, result);
    tx_message.id = 0x1235;
    result = can2.write(tx_message);
    TEST_ASSERT_EQUAL(1, result);
    tx_message.id = 0x1236;
    result = can2.write(tx_message);
    TEST_ASSERT_EQUAL(1, result);

    /* Read each filter, should get 1 per filter */
    result = can1.read(rx_message, handle1);
    TEST_ASSERT_EQUAL(1, result);
    TEST_ASSERT_EQUAL(0x1234, rx_message.id);
    result = can1.read(rx_message, handle2);
    TEST_ASSERT_EQUAL(1, result);
    TEST_ASSERT_EQUAL(0x1235, rx_message.id);
    result = can1.read(rx_message, handle3);
    TEST_ASSERT_EQUAL(1, result);
    TEST_ASSERT_EQUAL(0x1236, rx_message.id);

    /* Try to read another message (should fail) */
    result = can1.read(rx_message, handle1);
    TEST_ASSERT_EQUAL(0, result);
    result = can1.read(rx_message, handle2);
    TEST_ASSERT_EQUAL(0, result);
    result = can1.read(rx_message, handle3);
    TEST_ASSERT_EQUAL(0, result);
}

void test_too_many_total_filters() {
    TCAN455X_INSTANCE_1;
    TCAN455X_INSTANCE_2;
    int handle;

    /* Add maximum SID filters */
    for(int i = 0; i < MBED_CONF_TCAN455X_SID_FILTER_COUNT; i++) {
        handle = can1.filter(i, i, CANStandard);
        if(i < MBED_CONF_TCAN455X_SID_FILTER_COUNT) {
            TEST_ASSERT_NOT_EQUAL(0, handle);
        }
    }

    /* Add too many XID filters */
    for(int i = 0; i < MBED_CONF_TCAN455X_XID_FILTER_COUNT+1; i++) {
        handle = can1.filter(i, i, CANExtended);
        if(i < MBED_CONF_TCAN455X_XID_FILTER_COUNT) {
            TEST_ASSERT_NOT_EQUAL(0, handle);
        }
    }

    TEST_ASSERT_EQUAL(0, handle);
}

void test_invalid_any_filter() {
    TCAN455X_INSTANCE_1;
    TCAN455X_INSTANCE_2;
    int handle;

    /* Test failure to add standard filter with large ID */
    handle = can1.filter(0x1234, 0x7FF, CANAny);
    TEST_ASSERT_EQUAL(0, handle);

    /* Test failure to add standard filter with invalid handle */
    handle = can1.filter(0x123, 0x7FF, CANAny, TCAN455X_TOTAL_FILTER_COUNT+1);
    TEST_ASSERT_EQUAL(0, handle);
}

void test_too_many_mixed_filters() {
    /**
     * The TCAN455x does not support a "CANAny" filter, so to accomplish it we
     * set up both an SID and XID filter. We can have up to 6 total filters, but if
     * one of those is a CANAny filter we won't be able to allocate enough XID or SID filters
     * to fill all 6 filter slots.
     *
     * This test makes sure that using mixed filters along with CANAny is disallowed if
     * the total available XID or SID filters is exhausted, even if the total number of filter handles
     * is not.
     */

    /* Allocate 1 SID filter */
    TCAN455X_INSTANCE_1;
    TCAN455X_INSTANCE_2;
    int handle = can1.filter(0x123, 0x7FF, CANStandard);
    TEST_ASSERT_NOT_EQUAL(0, handle);

    /* Allocate 2 XID filters */
    handle = can1.filter(0x124, 0x7FF, CANExtended);
    TEST_ASSERT_NOT_EQUAL(0, handle);
    handle = can1.filter(0x125, 0x7FF, CANExtended);
    TEST_ASSERT_NOT_EQUAL(0, handle);


    /* Allocate 1 "CANAny" filter */
    handle = can1.filter(0x126, 0x7FF, CANAny);
    TEST_ASSERT_NOT_EQUAL(0, handle);

    /* Now try to allocate another "CANAny" filter, this should fail since we're out of XID filters*/
    // TODO this should also somehow test the path that deinitalizes the SID filter in the "half initialized" CANAny filter
    handle = can1.filter(0x127, 0x7FF, CANAny);
    TEST_ASSERT_EQUAL(0, handle);

    /* Now, we should still be able to register another SID filter since we've only used two of those (1 explicitly, 1 in CANAny) */
    handle = can1.filter(0x128, 0x7FF, CANStandard);
    TEST_ASSERT_NOT_EQUAL(0, handle);
}

void test_any_filter() {
    TCAN455X_INSTANCE_1;
    TCAN455X_INSTANCE_2;
    int handle;
    int result = 0;
    mbed::CANMessage tx_message;
    mbed::CANMessage rx_message;
    tx_message.id = 0x124;
    tx_message.format = CANStandard;
    tx_message.len = 8;
    tx_message.data[0] = 1;

    handle = can1.filter(0x123, 0x7FF, CANAny);

    /* Send 1 message with non-matching SID */
    result = can2.write(tx_message);
    TEST_ASSERT_EQUAL(1, result);

    /* Read 1 message from filter (should not receive anything since ID does not match) */
    result = can1.read(rx_message, handle);
    TEST_ASSERT_EQUAL(0, result);

    /* Send 1 message with non-matching XID */
    tx_message.format = CANExtended;
    result = can2.write(tx_message);
    TEST_ASSERT_EQUAL(1, result);

    /* Read 1 message from filter (should not receive anything since ID does not match) */
    result = can1.read(rx_message, handle);
    TEST_ASSERT_EQUAL(0, result);

    /* Send 1 message with matching SID */
    tx_message.id = 0x123;
    tx_message.format = CANStandard;
    result = can2.write(tx_message);
    TEST_ASSERT_EQUAL(1, result);

    /* Read 1 message from filter */
    result = can1.read(rx_message, handle);
    TEST_ASSERT_EQUAL(1, result);
    TEST_ASSERT_EQUAL(CANStandard, rx_message.format);
    TEST_ASSERT_EQUAL(tx_message.id, rx_message.id);

    /* Send 1 message with matching XID */
    tx_message.id = 0x123;
    tx_message.format = CANExtended;
    result = can2.write(tx_message);
    TEST_ASSERT_EQUAL(1, result);

    /* Read 1 message from filter */
    result = can1.read(rx_message, handle);
    TEST_ASSERT_EQUAL(1, result);
    TEST_ASSERT_EQUAL(CANExtended, rx_message.format);
    TEST_ASSERT_EQUAL(tx_message.id, rx_message.id);

    /* Try to read another message (should fail) */
    result = can1.read(rx_message, handle);
    TEST_ASSERT_EQUAL(0, result);

    /* Send matching SID and XID messages and test to make sure the filter receives both */
    tx_message.id = 0x123;
    tx_message.format = CANStandard;
    result = can2.write(tx_message);
    TEST_ASSERT_EQUAL(1, result);

    tx_message.id = 0x123;
    tx_message.format = CANExtended;
    result = can2.write(tx_message);
    TEST_ASSERT_EQUAL(1, result);

    /* Read first message from filter */
    result = can1.read(rx_message, handle);
    TEST_ASSERT_EQUAL(1, result);
    TEST_ASSERT_EQUAL(CANStandard, rx_message.format);
    TEST_ASSERT_EQUAL(tx_message.id, rx_message.id);

    /* Read second message from filter */
    result = can1.read(rx_message, handle);
    TEST_ASSERT_EQUAL(1, result);
    TEST_ASSERT_EQUAL(CANExtended, rx_message.format);
    TEST_ASSERT_EQUAL(tx_message.id, rx_message.id);

}

void test_mixed_filters() {

    /* Make a bunch of mixed filters and test to make sure data is routed correctly */
    /* Allocate 1 SID filter */
    TCAN455X_INSTANCE_1;
    TCAN455X_INSTANCE_2;
    int result = 0;
    mbed::CANMessage tx_message;
    mbed::CANMessage rx_message;
    tx_message.id = 0x124;
    tx_message.format = CANStandard;
    tx_message.len = 8;
    tx_message.data[0] = 1;

    int handle1 = can1.filter(0x123, 0x7FF, CANStandard);
    TEST_ASSERT_NOT_EQUAL(0, handle1);

    /* Allocate 2 XID filters */
    int handle2 = can1.filter(0x124, 0x7FF, CANExtended);
    TEST_ASSERT_NOT_EQUAL(0, handle2);
    int handle3 = can1.filter(0x125, 0x7FF, CANExtended);
    TEST_ASSERT_NOT_EQUAL(0, handle3);


    /* Allocate 1 "CANAny" filter */
    int handle4 = can1.filter(0x126, 0x7FF, CANAny);
    TEST_ASSERT_NOT_EQUAL(0, handle4);

    /* Now, we should still be able to register another SID filter since we've only used two of those (1 explicitly, 1 in CANAny) */
    int handle5 = can1.filter(0x127, 0x7FF, CANStandard);
    TEST_ASSERT_NOT_EQUAL(0, handle5);

    /* Write to first filter */
    tx_message.id = 0x123;
    tx_message.format = CANStandard;
    result = can2.write(tx_message);
    TEST_ASSERT_EQUAL(1, result);

    /* Read from first filter */
    result = can1.read(rx_message, handle1);
    TEST_ASSERT_EQUAL(1, result);
    TEST_ASSERT_EQUAL(CANStandard, rx_message.format);
    TEST_ASSERT_EQUAL(tx_message.id, rx_message.id);

    /* Write to second, third, and fourth filter */
    tx_message.id = 0x124;
    tx_message.format = CANExtended;
    result = can2.write(tx_message);
    TEST_ASSERT_EQUAL(1, result);
    tx_message.id = 0x125;
    tx_message.format = CANExtended;
    result = can2.write(tx_message);
    TEST_ASSERT_EQUAL(1, result);
    tx_message.id = 0x126;
    tx_message.format = CANExtended;
    result = can2.write(tx_message);
    TEST_ASSERT_EQUAL(1, result);
    tx_message.id = 0x126;
    tx_message.format = CANStandard;
    result = can2.write(tx_message);
    TEST_ASSERT_EQUAL(1, result);

    /* Read from second filter */
    result = can1.read(rx_message, handle2);
    TEST_ASSERT_EQUAL(1, result);
    TEST_ASSERT_EQUAL(CANExtended, rx_message.format);
    TEST_ASSERT_EQUAL(0x124, rx_message.id);

    /* Read from third filter */
    result = can1.read(rx_message, handle3);
    TEST_ASSERT_EQUAL(1, result);
    TEST_ASSERT_EQUAL(CANExtended, rx_message.format);
    TEST_ASSERT_EQUAL(0x125, rx_message.id);

    /* Read from fourth filter */
    result = can1.read(rx_message, handle4);
    TEST_ASSERT_EQUAL(1, result);
    TEST_ASSERT_EQUAL(CANExtended, rx_message.format);
    TEST_ASSERT_EQUAL(0x126, rx_message.id);
    result = can1.read(rx_message, handle4);
    TEST_ASSERT_EQUAL(1, result);
    TEST_ASSERT_EQUAL(CANStandard, rx_message.format);
    TEST_ASSERT_EQUAL(0x126, rx_message.id);

    /* Write to fifth filter */
    tx_message.id = 0x127;
    tx_message.format = CANStandard;
    result = can2.write(tx_message);
    TEST_ASSERT_EQUAL(1, result);

    /* Read from fifth filter */
    result = can1.read(rx_message, handle5);
    TEST_ASSERT_EQUAL(1, result);
    TEST_ASSERT_EQUAL(CANStandard, rx_message.format);
    TEST_ASSERT_EQUAL(0x127, rx_message.id);

}

void test_reassigned_mixed_filters() {
    /* Make a bunch of mixed filters and test to make sure data is routed correctly */
    /* Allocate 1 SID filter */
    TCAN455X_INSTANCE_1;
    TCAN455X_INSTANCE_2;
    int result = 0;
    mbed::CANMessage tx_message;
    mbed::CANMessage rx_message;
    tx_message.id = 0x124;
    tx_message.format = CANStandard;
    tx_message.len = 8;
    tx_message.data[0] = 1;

    int handle1 = can1.filter(0x123, 0x7FF, CANStandard);
    TEST_ASSERT_NOT_EQUAL(0, handle1);

    /* Allocate 2 XID filters */
    int handle2 = can1.filter(0x124, 0x7FF, CANExtended);
    TEST_ASSERT_NOT_EQUAL(0, handle2);
    int handle3 = can1.filter(0x125, 0x7FF, CANExtended);
    TEST_ASSERT_NOT_EQUAL(0, handle3);


    /* Allocate 1 "CANAny" filter */
    int handle4 = can1.filter(0x126, 0x7FF, CANAny);
    TEST_ASSERT_NOT_EQUAL(0, handle4);

    /* Now, we should still be able to register another SID filter since we've only used two of those (1 explicitly, 1 in CANAny) */
    int handle5 = can1.filter(0x127, 0x7FF, CANStandard);
    TEST_ASSERT_NOT_EQUAL(0, handle5);

    /* Write to first filter */
    tx_message.id = 0x123;
    tx_message.format = CANStandard;
    result = can2.write(tx_message);
    TEST_ASSERT_EQUAL(1, result);

    /* Read from first filter */
    result = can1.read(rx_message, handle1);
    TEST_ASSERT_EQUAL(1, result);
    TEST_ASSERT_EQUAL(CANStandard, rx_message.format);
    TEST_ASSERT_EQUAL(tx_message.id, rx_message.id);

    /* Write to second, third, and fourth filter */
    tx_message.id = 0x124;
    tx_message.format = CANExtended;
    result = can2.write(tx_message);
    TEST_ASSERT_EQUAL(1, result);
    tx_message.id = 0x125;
    tx_message.format = CANExtended;
    result = can2.write(tx_message);
    TEST_ASSERT_EQUAL(1, result);
    tx_message.id = 0x126;
    tx_message.format = CANExtended;
    result = can2.write(tx_message);
    TEST_ASSERT_EQUAL(1, result);
    tx_message.id = 0x126;
    tx_message.format = CANStandard;
    result = can2.write(tx_message);
    TEST_ASSERT_EQUAL(1, result);

    /* Read from second filter */
    result = can1.read(rx_message, handle2);
    TEST_ASSERT_EQUAL(1, result);
    TEST_ASSERT_EQUAL(CANExtended, rx_message.format);
    TEST_ASSERT_EQUAL(0x124, rx_message.id);

    /* Read from third filter */
    result = can1.read(rx_message, handle3);
    TEST_ASSERT_EQUAL(1, result);
    TEST_ASSERT_EQUAL(CANExtended, rx_message.format);
    TEST_ASSERT_EQUAL(0x125, rx_message.id);

    /* Read from fourth filter */
    result = can1.read(rx_message, handle4);
    TEST_ASSERT_EQUAL(1, result);
    TEST_ASSERT_EQUAL(CANExtended, rx_message.format);
    TEST_ASSERT_EQUAL(0x126, rx_message.id);
    result = can1.read(rx_message, handle4);
    TEST_ASSERT_EQUAL(1, result);
    TEST_ASSERT_EQUAL(CANStandard, rx_message.format);
    TEST_ASSERT_EQUAL(0x126, rx_message.id);

    /* Reassign the fourth filter */
    handle4 = can1.filter(0x128, 0x7FF, CANExtended, handle4);
    TEST_ASSERT_NOT_EQUAL(0, handle4);

    /* Write to reassigned fourth filter with non-matching SID  (because it is an XID filter) */
    tx_message.id = 0x128;
    tx_message.format = CANStandard;
    result = can2.write(tx_message);
    TEST_ASSERT_EQUAL(1, result);

    /* Should not read back (SID sent, filtering on XID) */
    result = can1.read(rx_message, handle4);
    TEST_ASSERT_EQUAL(0, result);

    /* Write to reassigned fourth filter with matching XID */
    tx_message.id = 0x128;
    tx_message.format = CANExtended;
    result = can2.write(tx_message);
    TEST_ASSERT_EQUAL(1, result);

    /* Should read back */
    result = can1.read(rx_message, handle4);
    TEST_ASSERT_EQUAL(1, result);
    TEST_ASSERT_EQUAL(CANExtended, rx_message.format);
    TEST_ASSERT_EQUAL(0x128, rx_message.id);

    /* Assign another SID filter */
    int handle6 = can1.filter(0x129, 0x7FF, CANStandard);
    TEST_ASSERT_NOT_EQUAL(0, handle5);

    /* Write to sixth filter */
    tx_message.id = 0x129;
    tx_message.format = CANStandard;
    result = can2.write(tx_message);
    TEST_ASSERT_EQUAL(1, result);

    /* Should read back */
    result = can1.read(rx_message, handle6);
    TEST_ASSERT_EQUAL(1, result);
    TEST_ASSERT_EQUAL(CANStandard, rx_message.format);
    TEST_ASSERT_EQUAL(0x129, rx_message.id);

    /* Write to fifth filter */
    tx_message.id = 0x127;
    tx_message.format = CANStandard;
    result = can2.write(tx_message);
    TEST_ASSERT_EQUAL(1, result);

    /* Read from fifth filter */
    result = can1.read(rx_message, handle5);
    TEST_ASSERT_EQUAL(1, result);
    TEST_ASSERT_EQUAL(CANStandard, rx_message.format);
    TEST_ASSERT_EQUAL(0x127, rx_message.id);
}

void test_loopback_frequency_mismatch() {
    /* This should test the ability to set frequency by introducing an intentional mismatch during loopback */
    TCAN455X_INSTANCE_1;
    TCAN455X_INSTANCE_2;
    int result = 0;
    mbed::CANMessage tx_message;
    mbed::CANMessage rx_message;
    tx_message.id = 0x123;
    tx_message.format = CANStandard;
    tx_message.len = 8;
    tx_message.data[0] = 1;

    /* Mismatch the frequencies */
    result = can1.frequency(100E3);
    TEST_ASSERT_EQUAL(1, result);
    result = can2.frequency(500E3);
    TEST_ASSERT_EQUAL(1, result);


    /* Send unfiltered message with SID */

    /* Send 1 message */
    result = can2.write(tx_message);
    TEST_ASSERT_EQUAL(1, result);

    /* Try to read message (should fail) */
    result = can1.read(rx_message, 0);
    TEST_ASSERT_EQUAL(0, result);

    /* Fix frequency mismatch */
    result = can1.frequency(500E3);
    TEST_ASSERT_EQUAL(1, result);

    /* Send 1 message */
    result = can2.write(tx_message);
    TEST_ASSERT_EQUAL(1, result);

    /* Read 1 message */
    result = can1.read(rx_message, 0);
    TEST_ASSERT_EQUAL(1, result);
    TEST_ASSERT_EQUAL(CANStandard, rx_message.format);
    TEST_ASSERT_EQUAL(tx_message.id, rx_message.id);
}

void stress_test() {

}

Case cases[] = {
        Case("test unfiltered messages", test_unfiltered_messages),
        Case("test fail write CANAny", test_fail_write_can_any),
        Case("test invalid sid filter", test_invalid_sid_filter),
        Case("test too many sid filters", test_too_many_sid_filters),
        Case("test sid filter", test_sid_filter),
//        Case("test sid filter overflow", test_sid_filter_overflow),
        Case("test reassigned sid filter", test_reassigned_sid_filter),
        Case("test multiple sid filters", test_multiple_sid_filters),
        Case("test invalid xid filter", test_invalid_xid_filter),
        Case("test too many xid filters", test_too_many_xid_filters),
        Case("test xid filter", test_xid_filter),
//        Case("test xid filter overflow", test_xid_filter_overflow),
        Case("test reassigned xid filter", test_reassigned_xid_filter),
        Case("test multiple xid filters", test_multiple_xid_filters),
        Case("test too many total filters", test_too_many_total_filters),
        Case("test invalid any filter", test_invalid_any_filter),
        Case("test too many mixed filters", test_too_many_mixed_filters),
        Case("test any filter", test_any_filter),
//        Case("test any filter overflow", test_any_filter_overflow),
        Case("test mixed filters", test_mixed_filters),
        Case("test reassigned mixed filters", test_reassigned_mixed_filters),
        Case("test loopback frequency mismatch", test_loopback_frequency_mismatch),
        Case("stress test", stress_test),
};

utest::v1::status_t greentea_test_setup(const size_t number_of_cases)
{
    GREENTEA_SETUP(60, "default_auto");
    return greentea_test_setup_handler(number_of_cases);
}

Specification specification(greentea_test_setup, cases, greentea_test_teardown_handler);

int main()
{
    Harness::run(specification);
}
