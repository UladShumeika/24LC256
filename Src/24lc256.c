/**
  ******************************************************************************
  * @file    24lc256.c
  * @author  Ulad Shumeika
  * @version v1.0
  * @date    09 June 2023
  * @brief	 This file contains the implementation of functions for working with
  * 		 eeprom 24lc256.
  *
  ******************************************************************************
  */

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "24lc256.h"
#include "ush_stm32f4xx_conf.h"
#include <stddef.h>
#include <string.h>

//---------------------------------------------------------------------------
// Definitions
//---------------------------------------------------------------------------
#define PRJ_24LC256_PAGE_SIZE								(64U)
#define PRJ_24LC256_MAX_MEM_SIZE							(32768U)
#define PRJ_24LC256_MAX_MEM_ADDRESS							(PRJ_24LC256_MAX_MEM_SIZE - 1U)

/* Used to store data that is written only once and
 * does not change during the operation of the device (64 bytes) */
#define PRJ_24LC256_STATIC_DATA_SPACE_SIZE					(PRJ_24LC256_STATIC_DATA_SPACE_END - \
															 PRJ_24LC256_STATIC_DATA_SPACE_BEGIN + 1U)
#define PRJ_24LC256_STATIC_DATA_SPACE_BEGIN					(0x0000U)
#define PRJ_24LC256_STATIC_DATA_SPACE_END					(0x003FU)

/* The status buffer that stores a pointer to the index of the parameter buffer
 * that already stores a pointer to the actual data (32 bytes) */
#define PRJ_24LC256_DINAMIC_DATA_STATUS_SPACE_SIZE_BYTES	(PRJ_24LC256_DINAMIC_DATA_STATUS_SPACE_END - \
															 PRJ_24LC256_DINAMIC_DATA_STATUS_SPACE_BEGIN + 1U)
#define PRJ_24LC256_DINAMIC_DATA_STATUS_SPACE_SIZE			(PRJ_24LC256_DINAMIC_DATA_STATUS_SPACE_SIZE_BYTES / sizeof(uint16_t))
#define PRJ_24LC256_DINAMIC_DATA_STATUS_SPACE_BEGIN			(0x0040U)
#define PRJ_24LC256_DINAMIC_DATA_STATUS_SPACE_END			(0x005FU)

/* The parameter buffer that stores a pointer to the actual data (32 bytes) */
#define PRJ_24LC256_DINAMIC_DATA_PARAMETER_SPACE_SIZE_BYTES	(PRJ_24LC256_DINAMIC_DATA_PARAMETER_SPACE_END - \
															 PRJ_24LC256_DINAMIC_DATA_PARAMETER_SPACE_BEGIN + 1U)
#define PRJ_24LC256_DINAMIC_DATA_PARAMETER_SPACE_SIZE		(PRJ_24LC256_DINAMIC_DATA_PARAMETER_SPACE_SIZE_BYTES / sizeof(uint16_t))
#define PRJ_24LC256_DINAMIC_DATA_PARAMETER_SPACE_BEGIN		(0x0060U)
#define PRJ_24LC256_DINAMIC_DATA_PARAMETER_SPACE_END		(0x007FU)

/* Used to store the actual data */
#define PRJ_24LC256_DINAMIC_DATA_SPACE_SIZE					(PRJ_24LC256_MAX_MEM_ADDRESS - \
															 PRJ_24LC256_DINAMIC_DATA_SPACE_BEGIN + 1U)
#define PRJ_24LC256_DINAMIC_DATA_SPACE_BEGIN				(0x0080U)
#define PRJ_24LC256_DINAMIC_DATA_SPACE_END					(PRJ_24LC256_MAX_MEM_ADDRESS)

#define PRJ_24LC256_MAX_NUM_RECORDS							(200U)  /* Used for status buffer */
#define PRJ_24LC256_TRIALS									(3U)
#define PRJ_24LC256_DELAY									(10U)

//---------------------------------------------------------------------------
// Types
//---------------------------------------------------------------------------
static prj_24lc256_dma_handlers_t m_dma_handlers = {0};
static prj_i2c_transmission_t m_i2c_tx = {0};
static prj_i2c_transmission_t m_i2c_rx = {0};

static prj_24lc256_system_t m_system_param = {0};

//---------------------------------------------------------------------------
// Variables
//---------------------------------------------------------------------------
static uint16_t m_status_buffer[PRJ_24LC256_DINAMIC_DATA_STATUS_SPACE_SIZE] = {0};
static uint16_t m_parameter_buffer[PRJ_24LC256_DINAMIC_DATA_PARAMETER_SPACE_SIZE] = {0};
static uint8_t m_page_buffer[PRJ_24LC256_PAGE_SIZE] = {0};
static uint8_t m_transition_buffer[PRJ_24LC256_MSG_SIZE] = {0};

//---------------------------------------------------------------------------
// Static functions declaration
//---------------------------------------------------------------------------
static void eeprom_24lc256_status_buffer_index_get(uint16_t* data, uint8_t data_size, prj_24lc256_system_t* system);
static uint8_t eeprom_24lc256_free_space_current_page(prj_24lc256_system_t* system);
static uint32_t eeprom_24lc256_update_system_parameters(prj_24lc256_system_t* system_param);

#if(PRJ_24LC256_WP_ENABLED == 1U)
	static void eeprom_24lc256_write_protection(uint32_t state);
#endif

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
uint32_t prj_eeprom_24lc256_init(uint8_t dev_address)
{
	uint32_t status = PRJ_STATUS_OK;
	uint16_t remainder = 0U;

	#if(PRJ_24LC256_WP_ENABLED == 1U)
		eeprom_24lc256_write_protection(PRJ_STATE_ENABLE);
	#endif

	/* Fill in the general parameters */
	m_i2c_rx.p_i2c				= PRJ_24LC256_I2C_USED;
	m_i2c_rx.dev_address		= dev_address;
	m_i2c_rx.mem_address_size	= PRJ_I2C_MEM_ADDRESS_SIZE_16BIT;

	/* Capture status buffer from memory */
	m_i2c_rx.mem_address		= PRJ_24LC256_DINAMIC_DATA_STATUS_SPACE_BEGIN;
	m_i2c_rx.p_data				= (uint8_t*)m_status_buffer;
	m_i2c_rx.data_size 			= sizeof(m_status_buffer);
	status = prj_i2c_read_dma(&m_i2c_rx);

	/* Capture parameter buffer from memory */
	m_i2c_rx.mem_address		= PRJ_24LC256_DINAMIC_DATA_PARAMETER_SPACE_BEGIN;
	m_i2c_rx.p_data				= (uint8_t*)m_parameter_buffer;
	m_i2c_rx.data_size 			= sizeof(m_parameter_buffer);
	status = prj_i2c_read_dma(&m_i2c_rx);

	if(status == PRJ_STATUS_OK)
	{
		/* If this condition is met, then system buffers havn't been initialized */
		if(m_status_buffer[0] > PRJ_24LC256_MAX_NUM_RECORDS)
		{
			#if(PRJ_24LC256_WP_ENABLED == 1U)
				eprom_24lc256_write_protection(PRJ_STATE_DISABLE);
			#endif

			/* Initialize the status buffer */
			memset(m_status_buffer, 0x00U, sizeof(m_status_buffer));

			/* Initialize the parameter buffer */
			for(uint8_t i = 0; i < (sizeof(m_parameter_buffer) / sizeof(m_parameter_buffer[0])); i++)
			{
				m_parameter_buffer[i] = PRJ_24LC256_DINAMIC_DATA_SPACE_BEGIN;
			}

			/* Fill in the general parameters */
			m_i2c_tx.p_i2c				= PRJ_24LC256_I2C_USED;
			m_i2c_tx.dev_address		= dev_address;
			m_i2c_tx.mem_address_size	= PRJ_I2C_MEM_ADDRESS_SIZE_16BIT;

			/* Write the status buffer to the memory */
			m_i2c_tx.mem_address		= PRJ_24LC256_DINAMIC_DATA_STATUS_SPACE_BEGIN;
			m_i2c_tx.p_data				= (uint8_t*)m_status_buffer;
			m_i2c_tx.data_size 			= sizeof(m_status_buffer);
			status = prj_i2c_write_dma(&m_i2c_tx);

			/* Write the parameter buffer to the memory */
			m_i2c_tx.mem_address		= PRJ_24LC256_DINAMIC_DATA_PARAMETER_SPACE_BEGIN;
			m_i2c_tx.p_data				= (uint8_t*)m_parameter_buffer;
			m_i2c_tx.data_size 			= sizeof(m_parameter_buffer);
			status = prj_i2c_write_dma(&m_i2c_tx);

			#if(PRJ_24LC256_WP_ENABLED == 1U)
				eeprom_24lc256_write_protection(PRJ_STATE_ENABLE);
			#endif
		}
		else
		{
			; /* DO NOTHING */
		}
	}
	else
	{
		/* DO NOTHING */
	}

	if(status == PRJ_STATUS_OK)
	{
		/* Search for the index of the status buffer that contains the maximum value */
		eeprom_24lc256_status_buffer_index_get(m_status_buffer,
											   PRJ_24LC256_DINAMIC_DATA_STATUS_SPACE_SIZE,
											   &m_system_param);

		/* Set the actual record address */
		m_system_param.actual_data_address = m_parameter_buffer[m_system_param.buffer_index];

		/* Calculate the next record address */
		if(m_system_param.record_number == 0U)
		{
			m_system_param.next_record_address = PRJ_24LC256_DINAMIC_DATA_SPACE_BEGIN;
		}
		else
		{
			m_system_param.next_record_address = m_system_param.actual_data_address + PRJ_24LC256_MSG_SIZE;

			/* Calculate the address of the next record if there was a transition from the end
			 * to the beginning of the memory area */
			if(m_system_param.next_record_address >= PRJ_24LC256_MAX_MEM_SIZE)
			{
				 remainder = (m_system_param.next_record_address - PRJ_24LC256_MAX_MEM_SIZE);
				 m_system_param.next_record_address = remainder + PRJ_24LC256_DINAMIC_DATA_SPACE_BEGIN;
			}
			else
			{
				; /* DO NOTHING */
			}
		}
	}
	else
	{
		; /* DO NOTHING */
	}

	return status;
}

/*!
 * @brief Set dma handlers' pointers in 24LC256 structure.
 *
 * This function is used to get and to safe dma handlers' pointers and to link dma handlers and i2c transmission structures.
 *
 * @param[in] p_dma_tx		A ponter to dma tx handler.
 * @param[in] p_dma_rx		A ponter to dma rx handler.
 *
 * @return None.
 */
void prj_eeprom_24lc256_dma_handlers_set(prj_dma_handler_t* p_dma_tx, prj_dma_handler_t* p_dma_rx)
{
	/* Set DMA handlers' pointers */
	m_dma_handlers.p_dma_tx = p_dma_tx;
	m_dma_handlers.p_dma_rx = p_dma_rx;

	/* Link DMA tx handler and i2c tx transmission */
	m_i2c_tx.p_dma										= m_dma_handlers.p_dma_tx;
	m_dma_handlers.p_dma_tx->p_controls_peripherals 	= (void*)&m_i2c_tx;

	/* Link DMA rx handler and i2c rx transmission */
	m_i2c_rx.p_dma										= m_dma_handlers.p_dma_rx;
	m_dma_handlers.p_dma_rx->p_controls_peripherals 	= (void*)&m_i2c_rx;
}

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
uint32_t prj_eeprom_24lc256_connect_test(uint8_t dev_address)
{
	uint32_t status = PRJ_STATUS_OK;

#if(PRJ_24LC256_WP_ENABLED == 1U)
	eeprom_24lc256_write_protection(PRJ_STATE_ENABLE);
#endif

	status = prj_i2c_is_device_ready(PRJ_24LC256_I2C_USED, dev_address, PRJ_24LC256_TRIALS);

	return status;
}

/*!
 * @brief Erase the memory
 *
 * This function is used to erase all memory.
 *
 * @param[in] dev_address	A target device address.
 *
 * @return @ref PRJ_STATUS_OK if memory clearing was successful.
 * @return @ref PRJ_STATUS_ERROR if the device is not detected.
 * @return @ref PRJ_STATUS_TIMEOUT if a timeout is detected on any flag.
 */
uint32_t prj_eeprom_24lc256_erase_memory(uint8_t dev_address)
{
	uint32_t status = PRJ_STATUS_OK;
	uint16_t mem_address = 0x0000U;

	/* Fill in the page buffer */
	memset(m_page_buffer, 0xFFU, PRJ_24LC256_PAGE_SIZE);

	/* Fill in the i2c tx structure */
	m_i2c_tx.p_i2c				= PRJ_24LC256_I2C_USED;
	m_i2c_tx.dev_address		= dev_address;
	m_i2c_tx.mem_address_size	= PRJ_I2C_MEM_ADDRESS_SIZE_16BIT;
	m_i2c_tx.p_data				= m_page_buffer;
	m_i2c_tx.data_size 			= PRJ_24LC256_PAGE_SIZE;

#if(PRJ_24LC256_WP_ENABLED == 1U)
	eeprom_24lc256_write_protection(PRJ_STATE_DISABLE);
#endif

	/* Erase every memory page */
	while((mem_address < PRJ_24LC256_MAX_MEM_ADDRESS) && (status == PRJ_STATUS_OK))
	{
		m_i2c_tx.mem_address = mem_address;
		mem_address = mem_address + PRJ_24LC256_PAGE_SIZE;

		status = prj_i2c_write_dma(&m_i2c_tx);

		MISC_timeoutDelay(PRJ_24LC256_DELAY);
	}

#if(PRJ_24LC256_WP_ENABLED == 1U)
	eeprom_24lc256_write_protection(PRJ_STATE_ENABLE);
#endif

	return status;
}

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
uint32_t prj_eeprom_24lc256_read_static(uint8_t dev_address, void* data, uint8_t data_size)
{
	uint32_t status = PRJ_STATUS_OK;

	/* Check the pointer and data size */
	if((data == NULL) || (data_size >= PRJ_24LC256_STATIC_DATA_SPACE_SIZE))
	{
		status = PRJ_STATUS_ERROR;
	}
	else
	{
		/* DO NOTHING */
	}

#if(PRJ_24LC256_WP_ENABLED == 1U)
	eeprom_24lc256_write_protection(PRJ_STATE_ENABLE);
#endif

	if(status == PRJ_STATUS_OK)
	{
		/* Fill i2c rx structure */
		m_i2c_rx.p_i2c				= PRJ_24LC256_I2C_USED;
		m_i2c_rx.dev_address		= dev_address;
		m_i2c_rx.mem_address		= PRJ_24LC256_STATIC_DATA_SPACE_BEGIN;
		m_i2c_rx.mem_address_size	= PRJ_I2C_MEM_ADDRESS_SIZE_16BIT;
		m_i2c_rx.p_data				= (uint8_t*)data;
		m_i2c_rx.data_size 			= data_size;
		status = prj_i2c_read_dma(&m_i2c_rx);
	}
	else
	{
		/* DO NOTHING */
	}

	return status;
}

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
uint32_t prj_eeprom_24lc256_write_static(uint8_t dev_address, void* data, uint8_t data_size)
{
	uint32_t status = PRJ_STATUS_OK;

	/* Check the pointer and data size */
	if((data == NULL) || (data_size >= PRJ_24LC256_STATIC_DATA_SPACE_SIZE))
	{
		status = PRJ_STATUS_ERROR;
	}
	else
	{
		/* DO NOTHING */
	}

#if(PRJ_24LC256_WP_ENABLED == 1U)
	eeprom_24lc256_write_protection(PRJ_STATE_DISABLE);
#endif

	if(status == PRJ_STATUS_OK)
	{
		/* Fill i2c tx structure */
		m_i2c_tx.p_i2c				= PRJ_24LC256_I2C_USED;
		m_i2c_tx.dev_address		= dev_address;
		m_i2c_tx.mem_address		= PRJ_24LC256_STATIC_DATA_SPACE_BEGIN;
		m_i2c_tx.mem_address_size	= PRJ_I2C_MEM_ADDRESS_SIZE_16BIT;
		m_i2c_tx.p_data				= (uint8_t*)data;
		m_i2c_tx.data_size 			= data_size;
		status = prj_i2c_write_dma(&m_i2c_tx);

		MISC_timeoutDelay(PRJ_24LC256_DELAY);
	}
	else
	{
		/* DO NOTHING */
	}

#if(PRJ_24LC256_WP_ENABLED == 1U)
	eeprom_24lc256_write_protection(PRJ_STATE_ENABLE);
#endif

	return status;
}

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
uint32_t prj_eeprom_24lc256_write_dynamic(uint8_t dev_address, uint8_t* data, uint16_t data_size)
{
	uint32_t status = PRJ_STATUS_OK;
	uint16_t remainig_data = 0U;
	uint16_t index_transition_buffer = 0U;
	uint8_t temp_size_data;
 	uint8_t free_space = 0U;

	/* Check the pointer and data size */
	if((data == NULL) || (data_size > PRJ_24LC256_DINAMIC_DATA_SPACE_SIZE) || (data_size != PRJ_24LC256_MSG_SIZE))
	{
		status = PRJ_STATUS_ERROR;
	}
	else
	{
		/* DO NOTHING */
	}

	if(status == PRJ_STATUS_OK)
	{

		#if(PRJ_24LC256_WP_ENABLED == 1U)
			eeprom_24lc256_write_protection(PRJ_STATE_DISABLE);
		#endif

		/* Calculate free space in current page and the next record address */
		free_space = eeprom_24lc256_free_space_current_page(&m_system_param);

		/* Fill in the general parameters */
		m_i2c_tx.p_i2c				= PRJ_24LC256_I2C_USED;
		m_i2c_tx.dev_address		= dev_address;
		m_i2c_tx.mem_address_size	= PRJ_I2C_MEM_ADDRESS_SIZE_16BIT;

		if(data_size <= free_space)
		{
			/* Fill i2c tx structure */
			m_i2c_tx.mem_address		= m_system_param.next_record_address;
			m_i2c_tx.p_data				= data;
			m_i2c_tx.data_size 			= data_size;
			status = prj_i2c_write_dma(&m_i2c_tx);

			MISC_timeoutDelay(PRJ_24LC256_DELAY);
		}
		else
		{
			/* Copy input data in the transition buffer */
			memcpy(m_transition_buffer, data, data_size);

			/* Set remaining data, transition buffer index and temp data size */
			remainig_data = data_size;
			index_transition_buffer = 0U;
			temp_size_data = free_space;

			/* Send data */
			while((remainig_data > 0U) && (status == PRJ_STATUS_OK))
			{
				/* Fill i2c tx structure */
				m_i2c_tx.mem_address		= m_system_param.next_record_address;
				m_i2c_tx.p_data				= &m_transition_buffer[index_transition_buffer];
				m_i2c_tx.data_size 			= temp_size_data;
				status = prj_i2c_write_dma(&m_i2c_tx);

				/* Calculate remaining data and transition buffer index */
				remainig_data -= temp_size_data;
				index_transition_buffer += temp_size_data;

				/* Calculate next record address */
				m_system_param.next_record_address = ((m_system_param.next_record_address + temp_size_data) % PRJ_24LC256_MAX_MEM_SIZE);

				if(m_system_param.next_record_address < PRJ_24LC256_DINAMIC_DATA_SPACE_BEGIN)
				{
					m_system_param.next_record_address = PRJ_24LC256_DINAMIC_DATA_SPACE_BEGIN;
				}
				else
				{
					; /* DO NOTHING */
				}

				/* Calculate free space */
				free_space = eeprom_24lc256_free_space_current_page(&m_system_param);

				/* Calculate temp data size */
				temp_size_data = (free_space >= remainig_data) ? remainig_data : free_space;

				MISC_timeoutDelay(PRJ_24LC256_DELAY);
			}
		}

		status = eeprom_24lc256_update_system_parameters(&m_system_param);
	}
	else
	{
		; /* DO NOTHING */
	}

	#if(PRJ_24LC256_WP_ENABLED == 1U)
		eeprom_24lc256_write_protection(PRJ_STATE_ENABLE);
	#endif

	return status;
}

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
uint32_t prj_eeprom_24lc256_read_dynamic(uint8_t dev_address, uint8_t* data, uint16_t data_size)
{
	uint32_t status = PRJ_STATUS_OK;
	uint16_t pointer_actual_data = 0U;
	uint16_t first_part_data_size = 0U;
	uint16_t second_part_data_size = 0U;

	/* Check the pointer and data size */
	if((data == NULL) || (data_size > PRJ_24LC256_DINAMIC_DATA_SPACE_SIZE) || (data_size != PRJ_24LC256_MSG_SIZE))
	{
		status = PRJ_STATUS_ERROR;
	}
	else
	{
		; /* DO NOTHING */
	}

	if(status == PRJ_STATUS_OK)
	{
		/* Read the actual data address */
		pointer_actual_data = m_system_param.actual_data_address;

		/* Fill in the general parameters */
		m_i2c_rx.p_i2c				= PRJ_24LC256_I2C_USED;
		m_i2c_rx.dev_address		= dev_address;
		m_i2c_rx.mem_address_size	= PRJ_I2C_MEM_ADDRESS_SIZE_16BIT;

		/* Check the actual data pointer */
		if((pointer_actual_data >= PRJ_24LC256_DINAMIC_DATA_SPACE_BEGIN) && (pointer_actual_data <= PRJ_24LC256_DINAMIC_DATA_SPACE_END))
		{
			/* Check data size */
			if((pointer_actual_data + data_size) <= PRJ_24LC256_DINAMIC_DATA_SPACE_END)
			{
				/* Fill i2c rx structure and read the memory */
				m_i2c_rx.mem_address		= pointer_actual_data;
				m_i2c_rx.p_data				= data;
				m_i2c_rx.data_size 			= data_size;
				status = prj_i2c_read_dma(&m_i2c_rx);
			}
			else
			{
				/* Calculate the first part data size */
				first_part_data_size = PRJ_24LC256_MAX_MEM_SIZE - pointer_actual_data;

				/* Fill i2c rx structure and read the memory */
				m_i2c_rx.mem_address		= pointer_actual_data;
				m_i2c_rx.p_data				= data;
				m_i2c_rx.data_size 			= first_part_data_size;
				status = prj_i2c_read_dma(&m_i2c_rx);

				/* Calculate the second part data size */
				second_part_data_size = data_size - first_part_data_size;

				/* Fill i2c rx structure and read the memory */
				m_i2c_rx.mem_address		= PRJ_24LC256_DINAMIC_DATA_SPACE_BEGIN;
				m_i2c_rx.p_data				= (data + first_part_data_size);
				m_i2c_rx.data_size 			= second_part_data_size;
				status = prj_i2c_read_dma(&m_i2c_rx);
			}
		}
		else
		{
			status = PRJ_STATUS_ERROR;
		}
	}
	else
	{
		; /* DO NOTHING */
	}

	return status;
}

//---------------------------------------------------------------------------
// STATIC
//---------------------------------------------------------------------------

/*!
 * @brief Search for the index in the status buffer.
 *
 * This function is used to search for the index of the status buffer that contains the maximum value
 * and to safe system parameters.
 *
 * @param[in] 	data		A pointer to the status buffer.
 * @param[in]	data_size	The status buffer size.
 * @param[out]	system		A pointer to the structure in which the system parameters will be stored.
 *
 * @return None.
 */
static void eeprom_24lc256_status_buffer_index_get(uint16_t* data, uint8_t data_size, prj_24lc256_system_t* system)
{
	uint8_t index = 0U;
	uint16_t max_value = 0U;

	if((data != NULL) && (system != NULL))
	{
		for(uint8_t i = 0U; i < data_size; i++)
		{
			if(max_value < data[i])
			{
				max_value = data[i];
				index = i;
			}
			else
			{
				/* DO NOTHING */
			}
		}

		/* Safe data */
		system->buffer_index = index;
		system->record_number = max_value;
	}
	else
	{
		/* DO NOTHING */
	}
}

/*!
 * @brief Calculate the current page's free space.
 *
 * This function is used to calculate free space of the current page.
 *
 * @param[in] system_param  system structure that contains information about
 * 							the address of the current data location and the number of records made.
 *
 * @return the current page's free space.
 */
static uint8_t eeprom_24lc256_free_space_current_page(prj_24lc256_system_t* system_param)
{
	uint8_t free_space_current_page = 0U;
	uint16_t next_page_address = 0U;
	uint16_t current_page = 0U;

	/* from 0 */
	current_page = system_param->next_record_address / PRJ_24LC256_PAGE_SIZE;

	if(system_param->record_number == 0U)
	{
		free_space_current_page = PRJ_24LC256_PAGE_SIZE;
	}
	else
	{
		next_page_address = (current_page + 1) * PRJ_24LC256_PAGE_SIZE;
		free_space_current_page = (uint8_t)(next_page_address - system_param->next_record_address);
	}

	return free_space_current_page;
}

/*!
 * @brief Update system parameters
 *
 * @param[in] system_param 	system structure that contains information about
 * 							the address of the current data location and the number of records made.
 *
 * @return @ref PRJ_STATUS_OK if the data is updated successfully.
 * @return @ref PRJ_STATUS_ERROR if a pointer is not passed either to the structure itself
 * 		   or to the DMA peripheral or the size of the transmitted data is 0.
 * @return @ref PRJ_STATUS_TIMEOUT if a timeout is detected on any flag.
 */
static uint32_t eeprom_24lc256_update_system_parameters(prj_24lc256_system_t* system_param)
{
	uint32_t status = PRJ_STATUS_OK;
	uint16_t first_part_message = 0U;
	uint16_t second_part_message = 0U;

	/* Calculate the new actual data address */
	system_param->actual_data_address = system_param->next_record_address - PRJ_24LC256_MSG_SIZE;

	/* If there was a transition from the end to the beginning of the buffer, correctly calculate
	 * the address of the next record */
	if(system_param->actual_data_address < PRJ_24LC256_DINAMIC_DATA_SPACE_BEGIN)
	{
		second_part_message = system_param->next_record_address % PRJ_24LC256_DINAMIC_DATA_SPACE_BEGIN;
		first_part_message = PRJ_24LC256_MSG_SIZE - second_part_message;
		system_param->actual_data_address = PRJ_24LC256_MAX_MEM_SIZE - first_part_message;
	}
	else
	{
		; /* DO NOTHING */
	}

	/* Increase buffer index */
	system_param->buffer_index++;
	if(system_param->buffer_index >= PRJ_24LC256_DINAMIC_DATA_STATUS_SPACE_SIZE)
	{
		system_param->buffer_index = 0U;
	}
	else
	{
		; /* DO NOTHING */
	}

	/* Write to parameter buffer */
	m_parameter_buffer[system_param->buffer_index] = system_param->actual_data_address;

	/* Update record number and write to status buffer */
	system_param->record_number++;
	if(system_param->record_number > PRJ_24LC256_MAX_NUM_RECORDS)
	{
		/* Start counting from 1 and reset the status buffer */
		system_param->record_number = 1U;
		memset(m_status_buffer, 0x00U, sizeof(m_status_buffer));
	}
	else
	{
		; /* DO NOTHING */
	}

	m_status_buffer[system_param->buffer_index] = system_param->record_number;

	/* Fill i2c tx structure end send parameter buffer */
	m_i2c_tx.mem_address		= PRJ_24LC256_DINAMIC_DATA_PARAMETER_SPACE_BEGIN;
	m_i2c_tx.p_data				= (uint8_t*)m_parameter_buffer;
	m_i2c_tx.data_size 			= sizeof(m_parameter_buffer);
	status = prj_i2c_write_dma(&m_i2c_tx);
	MISC_timeoutDelay(PRJ_24LC256_DELAY);

	/* Fill i2c tx structure end send status buffer */
	m_i2c_tx.mem_address		= PRJ_24LC256_DINAMIC_DATA_STATUS_SPACE_BEGIN;
	m_i2c_tx.p_data				= (uint8_t*)m_status_buffer;
	m_i2c_tx.data_size 			= sizeof(m_status_buffer);
	status = prj_i2c_write_dma(&m_i2c_tx);
	MISC_timeoutDelay(PRJ_24LC256_DELAY);

	return status;
}

#if(PRJ_24LC256_WP_ENABLED == 1U)
/*!
 * @brief Enable or disable eeprom write protection.
 *
 * @param state[in] 	Enable or disable state.
 *
 * @return None.
 */
static void eeprom_24lc256_write_protection(uint32_t state)
{
	if(state == PRJ_STATE_ENABLE)
	{
		GPIO_writeBits(PRJ_24LC256_WP_PORT, PRJ_24LC256_WP_PIN, GPIO_PIN_SET);
	}
	else
	{
		GPIO_writeBits(PRJ_24LC256_WP_PORT, PRJ_24LC256_WP_PIN, GPIO_PIN_RESET);
	}
}
#endif
