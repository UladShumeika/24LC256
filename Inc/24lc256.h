/**
  ******************************************************************************
  * @file    24lc256.h
  * @author  Ulad Shumeika
  * @version v1.0
  * @date    09 June 2023
  * @brief   Header file of EEPROM 24LC256 driver.
  *
  ******************************************************************************
  */

#ifndef __24lc256_h
#define __24lc256_h

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "ush_stm32f4xx_gpio.h"
#include "ush_stm32f4xx_dma.h"

//---------------------------------------------------------------------------
// Definitions
//---------------------------------------------------------------------------
#define PRJ_24LC256_I2C_USED						 (I2C2)
#define PRJ_24LC256_I2C_CLOCK_SPEED					 (100000U)

#define PRJ_24LC256_MSG_SIZE					 	 (93U)

#define PRJ_24LC256_WP_ENABLED						 (0U)

#if(PRJ_24LC256_WP_ENABLED == 1U)
	#define PRJ_24LC256_WP_PORT						 GPIOC
	#define PRJ_24LC256_WP_PIN						 GPIO_PIN_14
#endif

//---------------------------------------------------------------------------
// Types
//---------------------------------------------------------------------------

/*!
 * @brief dma handlers' pointers structure definition.
 * @note  This structure is needed to store pointers to dma handlers.
 */
typedef struct
{
	prj_dma_handler_t* p_dma_tx;		/*!< A pointer to dma tx handler */

	prj_dma_handler_t* p_dma_rx;		/*!< A pointer to dma rx handler */

} prj_24lc256_dma_handlers_t;

/*!
 * @brief Memory system parameters structure definition.
 */
typedef struct
{
	uint8_t buffer_index;				/*!< Index of the status and parameter buffers */

	uint8_t record_number;				/*!< Memory record number */

	uint16_t actual_data_address;		/*!< Actual data address */

	uint16_t next_record_address;		/*!< Next record address */

	uint8_t reserved1;					/*!< For structure size alignment */

	uint8_t reserved2;					/*!< For structure size alignment */

} prj_24lc256_system_t;

//---------------------------------------------------------------------------
// API
//---------------------------------------------------------------------------

/*!
 * @brief Initialize 24lc256 memory.
 *
 * This function is used to capture status and parameter buffers.
 *
 * @param[in] dev_address	A target device address.
 *
 * @return @ref PRJ_STATUS_OK if initialization was successful.
 * @return @ref PRJ_STATUS_ERROR if the data was not read from memory.
 * @return @ref PRJ_STATUS_TIMEOUT if a timeout is detected on any flag.
 */
uint32_t prj_eeprom_24lc256_init(uint8_t dev_address);

/*!
 * @brief Set dma handlers' pointers in 24LC256 structure.
 *
 * This function is used to get and to safe dma handlers' pointers and
 * to link dma handlers and i2c transmission structures.
 *
 * @param[in] p_dma_tx		A pointer to dma tx handler.
 * @param[in] p_dma_rx		A pointer to dma rx handler.
 *
 * @return None.
 */
void prj_eeprom_24lc256_dma_handlers_set(prj_dma_handler_t* p_dma_tx, prj_dma_handler_t* p_dma_rx);

/*!
 * @brief Check device availability on the bus.
 *
 * This function is used to check the specified device availability several times.
 * The number of checks is defined in @ref PRJ_24LC256_TRIALS.
 *
 * @param[in] dev_address	A target device address.
 *
 * @return @ref PRJ_STATUS_OK if the device is available.
 * @return @ref PRJ_STATUS_ERROR if a null pointer to the peripheral is passed
 * 		   or the device is not detected.
 * @return @ref PRJ_STATUS_TIMEOUT if the timeout has passed.
 */
uint32_t prj_eeprom_24lc256_connect_test(uint8_t dev_address);

/*!
 * @brief Erase the memory.
 *
 * This function is used to erase all memory.
 *
 * @param[in] dev_address	A target device address.
 *
 * @return @ref PRJ_STATUS_OK if memory clearing was successful.
 * @return @ref PRJ_STATUS_ERROR if the device is not detected.
 * @return @ref PRJ_STATUS_TIMEOUT if a timeout is detected on any flag.
 */
uint32_t prj_eeprom_24lc256_erase_memory(uint8_t dev_address);

/*!
 * @brief Read the data in the static data space.
 *
 * This function is used to read data in the static space.
 *
 * @param[in]  dev_address	A target device address.
 * @param[out] data			A pointer to the array in which to store the data.
 * @param[in]  data_size	Size of the array.
 *
 * @return @ref PRJ_STATUS_OK if memory reading was successful.
 * @return @ref PRJ_STATUS_ERROR if the device is not detected or the pointer is NULL or
 * 		   array size is larger than static data space.
 * @return @ref PRJ_STATUS_TIMEOUT if a timeout is detected on any flag.
 */
uint32_t prj_eeprom_24lc256_read_static(uint8_t dev_address, void* data, uint8_t data_size);

/*!
 * @brief Write the data in the static data space.
 *
 * This function is used to write data in the static space.
 *
 * @param[in] dev_address	A target device address.
 * @param[in] data			A pointer to the array in which to store the data.
 * @param[in] data_size	 	Size of the array.
 *
 * @return @ref PRJ_STATUS_OK if memory writing was successful.
 * @return @ref PRJ_STATUS_ERROR if the device is not detected or the pointer is NULL or
 * 		   array size is larger than static data space.
 * @return @ref PRJ_STATUS_TIMEOUT if a timeout is detected on any flag.
 */
uint32_t prj_eeprom_24lc256_write_static(uint8_t dev_address, void* data, uint8_t data_size);

/*!
 * @brief Write the data in the dynamic data space.
 *
 * This function is used to write data in the dynamic space.
 *
 * @param[in] dev_address	A target device address.
 * @param[in] data			A pointer to the array in which to store the data.
 * @param[in] data_size	 	Size of the array.
 *
 * @return @ref PRJ_STATUS_OK if memory writing was successful.
 * @return @ref PRJ_STATUS_ERROR if the device is not detected or the pointer is NULL or
 * 		   array size is larger than static data space.
 * @return @ref PRJ_STATUS_TIMEOUT if a timeout is detected on any flag.
 */
uint32_t prj_eeprom_24lc256_write_dynamic(uint8_t dev_address, uint8_t* data, uint16_t data_size);

/*!
 * @brief Read the data in the dynamic data space.
 *
 * This function is used to write data in the dynamic space.
 *
 * @param[in] dev_address	A target device address.
 * @param[out] data			A pointer to the array in which will store the data.
 * @param[in] data_size	 	Size of the array.
 *
 * @return @ref PRJ_STATUS_OK if memory writing was successful.
 * @return @ref PRJ_STATUS_ERROR if the device is not detected or the pointer is NULL or
 * 		   array size is larger than static data space.
 * @return @ref PRJ_STATUS_TIMEOUT if a timeout is detected on any flag.
 */
uint32_t prj_eeprom_24lc256_read_dynamic(uint8_t dev_address, uint8_t* data, uint16_t data_size);

#endif /* __24lc256_h */
