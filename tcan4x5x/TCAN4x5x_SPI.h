/*
 * TCAN4x5x_SPI.h
 * Version 1.1
 * Description: This file is responsible for abstracting the lower-level microcontroller SPI read and write functions
 *
 * Created on: Feb 10, 2020
 * Author: gdbeckstein
 */

/**
 * TODO - optimize transfers... very wasteful to do burst transfers as it stands
 *
 */

#ifndef TCAN4X5X_SPI_H_
#define TCAN4X5X_SPI_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Used by C driver to distinguish between multiple TCAN instances
typedef void* tcan_handle_t;

//------------------------------------------------------------------------
// AHB Access Op Codes
//------------------------------------------------------------------------
#define AHB_WRITE_OPCODE                            0x61
#define AHB_READ_OPCODE                             0x41


//------------------------------------------------------------------------
//							Write Functions
//------------------------------------------------------------------------

void tcan_spi_write_32(tcan_handle_t handle, uint16_t address, uint32_t data);
void tcan_spi_write_burst_start(tcan_handle_t handle, uint16_t address, uint8_t words);
void tcan_spi_write_burst_write(tcan_handle_t handle, uint32_t data);
void tcan_spi_write_burst_end(tcan_handle_t handle);


//--------------------------------------------------------------------------
//							Read Functions
//--------------------------------------------------------------------------
uint32_t tcan_spi_read_32(tcan_handle_t handle, uint16_t address);
void tcan_spi_read_burst_start(tcan_handle_t handle, uint16_t address, uint8_t words);
uint32_t tcan_spi_read_burst_read(tcan_handle_t handle);
void tcan_spi_read_burst_end(tcan_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif
