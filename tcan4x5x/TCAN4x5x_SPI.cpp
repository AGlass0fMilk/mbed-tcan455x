/*
 * TCAN4x5x_SPI.cpp
 * Description: This file is responsible for abstracting the lower-level microcontroller SPI read and write functions
 *
 * Created on: Feb 10, 2020
 * Author: gdbeckstein
 *
 */

#if DEVICE_SPI && FEATURE_EXPERIMENTAL_API

#include "TCAN4x5x_SPI.h"

#include "TCAN4551.h"

mbed::SPI &get_spi_handle(TCAN4551 *ptr) {
    return ptr->_spi;
}

extern "C" {

    /*
     * @brief Single word write
     *
     * @param address A 16-bit address of the destination register
     * @param data A 32-bit word of data to write to the destination register
     */
    void
    tcan_spi_write_32(tcan_handle_t handle, uint16_t address, uint32_t data)
    {
        tcan_spi_write_burst_start(handle, address, 1);
        tcan_spi_write_burst_write(handle, data);
        tcan_spi_write_burst_end(handle);
    }


    /*
     * @brief Single word read
     *
     * @param handle Handle of the TCAN instance
     * @param address A 16-bit address of the source register
     *
     * @return Returns 32-bit word of data from source register
     */
    uint32_t
    tcan_spi_read_32(tcan_handle_t handle, uint16_t address)
    {
        uint32_t returnData;

        tcan_spi_read_burst_start(handle, address, 1);
        returnData = tcan_spi_read_burst_read(handle);
        tcan_spi_read_burst_end(handle);

        return returnData;
    }

    /**
     * Initiates a multi-write operation
     *
     * The slave select line will be asserted until corresponding "..._end" is executed
     *
     * @param[in] handle Handle of the TCAN instance
     * @param[in] address Register address to start writing at
     * @param[in] words Number of 32-bit words to write. 0 = 256 words
     */
    void tcan_spi_write_burst_start(tcan_handle_t handle, uint16_t address, uint8_t words) {

        uint32_t header = 0;
        header |= (words << 24);
        header |= ((address & 0xFF) << 16);
        header |= (address & 0xFF00);
        header |= AHB_WRITE_OPCODE;

        mbed::SPI& spi = get_spi_handle((TCAN4551*)handle);
        spi.select();

        spi.write((const char*) &header, 4, NULL, 0);
    }

    /*
     * @brief Burst write
     *
     * Writes a number of words to the SPI bus
     *
     * @param[in] handle Handle of the TCAN instance
     * @param[in] data Data-word to write
     */
    void
    tcan_spi_write_burst_write(tcan_handle_t handle, uint32_t data)
    {
        // TODO note that 0 = 256 words
        uint8_t buffer[4] = {0};
        mbed::SPI& spi = get_spi_handle((TCAN4551*)handle);
        buffer[0] = ((data & 0xFF000000) >> 24);
        buffer[1] = ((data & 0x00FF0000) >> 16);
        buffer[2] |= ((data & 0x0000FF00) >> 8);
        buffer[3] |= ((data & 0x000000FF));
        spi.write((const char*) buffer, 4, NULL, 0);
    }

    /**
     * Ends a multi-write operation
     *
     * @param[in] handle Handle of the TCAN instance
     */
    void tcan_spi_write_burst_end(tcan_handle_t handle) {
        mbed::SPI& spi = get_spi_handle((TCAN4551*)handle);
        spi.deselect();
    }

    /**
     * Initiates a multi-read operation
     *
     * The slave select line will be asserted until corresponding "..._end" is executed
     *
     * @param[in] handle Handle of the TCAN instance
     * @param[in] address Starting register address to read from
     * @param[in] words Number of words to read
     */
    void tcan_spi_read_burst_start(tcan_handle_t handle, uint16_t address, uint8_t words) {
        uint32_t header = 0;
        header |= (words << 24);
        header |= ((address & 0xFF) << 16);
        header |= (address & 0xFF00);
        header |= AHB_READ_OPCODE;

        mbed::SPI& spi = get_spi_handle((TCAN4551*)handle);
        spi.select();

        spi.write((const char*) &header, 4, NULL, 0);
    }

    /*
     * @brief Burst read start
     *
     * The SPI transaction contains 3 parts: the header (start), the payload, and the end of data (end)
     * This function where each word of data is read from the TCAN4x5x
     *
     * @param handle Handle of the TCAN instance     */
    uint32_t
    tcan_spi_read_burst_read(tcan_handle_t handle)
    {
        // todo note that 0 = 256 words
        uint8_t buffer[4] = {0};
        uint32_t returnValue = 0;

        mbed::SPI& spi = get_spi_handle((TCAN4551*)handle);
        spi.write(NULL, 0, (char*) buffer, 4);

        returnValue = (((uint32_t)buffer[0]) << 24)
                | (((uint32_t)buffer[1] << 16))
                | (((uint32_t)buffer[2]) << 8)
                | buffer[3];

        return returnValue;
    }

    /**
     * Ends a multi-read operation
     *
     * @param[in] handle Handle of the TCAN instance
     */
    void tcan_spi_read_burst_end(tcan_handle_t handle) {
        mbed::SPI& spi = get_spi_handle((TCAN4551*)handle);
        spi.deselect();
    }
}

#endif
