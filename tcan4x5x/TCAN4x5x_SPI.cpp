/*
 * TCAN4x5x_SPI.c
 * Description: This file is responsible for abstracting the lower-level microcontroller SPI read and write functions
 *
 * Created on: Feb 10, 2020
 * Author: gdbeckstein
 *
 */


#include "TCAN4x5x_SPI.h"

#include "TCAN4551.h"

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
        mbed::SPI& spi = ((TCAN4551*)handle)->get_spi_handle();
        spi.select();

        spi.write((const char*) &address, 2, NULL, 0);
        spi.write((const char*) &words, 1, NULL, 0);
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
        mbed::SPI& spi = ((TCAN4551*)handle)->get_spi_handle();
        spi.write((const char*) &data, 4, NULL, 0);
    }

    /**
     * Ends a multi-write operation
     *
     * @param[in] handle Handle of the TCAN instance
     */
    void tcan_spi_write_burst_end(tcan_handle_t handle) {
        mbed::SPI& spi = ((TCAN4551*)handle)->get_spi_handle();
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
        mbed::SPI& spi = ((TCAN4551*)handle)->get_spi_handle();
        spi.select();

        spi.write((const char*) &address, 2, NULL, 0);
        spi.write((const char*) &words, 1, NULL, 0);
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
        uint32_t returnValue = 0;

        mbed::SPI& spi = ((TCAN4551*)handle)->get_spi_handle();
        spi.write(NULL, 0, (char*) &returnValue, 4);

        return returnValue;
    }

    /**
     * Ends a multi-read operation
     *
     * @param[in] handle Handle of the TCAN instance
     */
    void tcan_spi_read_burst_end(tcan_handle_t handle) {
        mbed::SPI& spi = ((TCAN4551*)handle)->get_spi_handle();
        spi.deselect();
    }
}
