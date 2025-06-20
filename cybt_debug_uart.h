/*******************************************************************************
* \file cybt_debug_uart.h

*
* \brief
* Provides API to access Debug transport.
*
********************************************************************************
* \copyright
* Copyright 2024-2025 Cypress Semiconductor Corporation
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
*******************************************************************************/
#ifndef CYBT_DEBUG_UART_H
#define CYBT_DEBUG_UART_H

#include "wiced_bt_dev.h"

#if !(defined(STACK_INSIDE_BT_CTRLR) && (STACK_INSIDE_BT_CTRLR == 1))
#include "cybt_result.h"
#else
/** BT result code */
typedef enum
{
    CYBT_SUCCESS = 0,
    CYBT_ERR_BADARG = 0xB1,
    CYBT_ERR_OUT_OF_MEMORY,
    CYBT_ERR_TIMEOUT,
    CYBT_ERR_HCI_INIT_FAILED,
    CYBT_ERR_HCI_UNSUPPORTED_IF,
    CYBT_ERR_HCI_UNSUPPORTED_BAUDRATE,
    CYBT_ERR_HCI_NOT_INITIALIZE,
    CYBT_ERR_HCI_WRITE_FAILED,
    CYBT_ERR_HCI_READ_FAILED,
    CYBT_ERR_HCI_GET_TX_MUTEX_FAILED,
    CYBT_ERR_HCI_GET_RX_MUTEX_FAILED,
    CYBT_ERR_HCI_SET_BAUDRATE_FAILED,
    CYBT_ERR_HCI_SET_FLOW_CTRL_FAILED,
    CYBT_ERR_INIT_MEMPOOL_FAILED,
    CYBT_ERR_INIT_QUEUE_FAILED,
    CYBT_ERR_CREATE_TASK_FAILED,
    CYBT_ERR_SEND_QUEUE_FAILED,
    CYBT_ERR_MEMPOOL_NOT_INITIALIZE,
    CYBT_ERR_QUEUE_ALMOST_FULL,
    CYBT_ERR_QUEUE_FULL,
    CYBT_ERR_GPIO_POWER_INIT_FAILED,
    CYBT_ERR_GPIO_DEV_WAKE_INIT_FAILED,
    CYBT_ERR_GPIO_HOST_WAKE_INIT_FAILED,
    CYBT_ERR_GENERIC
} cybt_result_t;
#endif

#if defined(STACK_INSIDE_BT_CTRLR) && (STACK_INSIDE_BT_CTRLR == 1)
#define BTSTACK_PORTING_SECTION_BEGIN
#define BTSTACK_PORTING_SECTION_END
#endif

 /*To use printf() when airoc-hci-transport is included and initialized in your application */
#ifndef ENABLE_AIROC_HCI_TRANSPORT_PRINTF
#define ENABLE_AIROC_HCI_TRANSPORT_PRINTF 1
#endif

/**
 *  @addtogroup    debug_uart_cfg   Debug UART Configuration
 *
 * The BLESS UART-specific configurations, including hardware pin assignment.
 * This debug UART is used for communication between the PSOC and host connected via USB cable.
 *  @{
 */

#ifdef ENHANCED_WICED_HCI
    #define WICED_HCI_PREAMBLE_LEN  4                            // Sync Train (3) + Hdr CRC (1)
    #define WICED_HCI_HDR_LEN       (WICED_HCI_PREAMBLE_LEN + 4) // Add opcode (2), length (2)
    #define WICED_HCI_OVERHEAD      (WICED_HCI_HDR_LEN + 2)      // Add Packet CRC (2)
#else
    #define WICED_HCI_PREAMBLE_LEN  1                            // Sync Train (1 byte, 0x19)
    #define WICED_HCI_HDR_LEN       (WICED_HCI_PREAMBLE_LEN + 4) // Add opcode (2) and length (2)
    #define WICED_HCI_OVERHEAD      WICED_HCI_HDR_LEN            // No additional overhead
#endif


/**
 * Received data handler callback type
 *
 * @param[in] p_data   : received data pointer
 * @param[in] data_len : data length
 *
 */
typedef uint32_t (*cybt_debug_uart_data_handler_t)( uint8_t* p_data, uint32_t data_len );

/** Debug Uart Configuration */
#if defined (CY_USING_HAL)
typedef struct
{
    cyhal_gpio_t         uart_tx_pin;  /**< Uart TXD pin */
    cyhal_gpio_t         uart_rx_pin;  /**< Uart RXD pin */
    cyhal_gpio_t         uart_rts_pin;  /**< Uart RTS pin */
    cyhal_gpio_t         uart_cts_pin;  /**< Uart CTS pin */
    uint32_t             baud_rate;     /**< Uart baud rate */
    bool                 flow_control;  /**< flow control status */
} cybt_debug_uart_config_t;
#else
typedef struct
{
    uint32_t             baud_rate;     /**< Uart baud rate */
    bool                 flow_control;  /**< flow control status */
} cybt_debug_uart_config_t;
#endif

/**
 * Initialize Debug UART.
 * This debug UART is used for communication between the PSOC and host connected via USB cable.
 *
 * @param[in] config       : uart configuration
 * @param[in] p_data_handler  : received data handler callback pointer
 *
 * @returns  CYBT_SUCCESS if success else error reason.
 *
 * @note : Debug UART Must be initialized to send traces over Debug UART.
 */
cybt_result_t cybt_debug_uart_init(cybt_debug_uart_config_t* config, cybt_debug_uart_data_handler_t p_data_handler);

/**
 * De-initialize Debug UART.
 *
 * @note : Debug UART Must be initialized to send traces over Debug UART.
 */
void cybt_debug_uart_deinit();

/**
 * Determines if the UART peripheral is currently in use for TX
 *
 * @return TX channel active status (active=true)
 */
bool cybt_debug_uart_is_tx_active();

/**
 * Determines if the UART peripheral is currently in use for RX
 *
 * @return RX channel active status (active=true)
 */
bool cybt_debug_uart_is_rx_active();

/**
 * Sends traces over Debug UART
 *
 * @param[in] length: Length of the data
 * @param[in] p_data: data pointer
 *
 * @returns  CYBT_SUCCESS if success else error reason.
 *
 */
cybt_result_t cybt_debug_uart_send_trace (uint16_t length, uint8_t* p_data);

/**
 * Sends HCI traces over Debug UART
 *
 * @param[in] type  : Trace data type (refer wiced_bt_hci_trace_type_t in wiced_bt_dev.h)
 * @param[in] length: Length of the data
 * @param[in] p_data: data pointer
 *
 * @returns  CYBT_SUCCESS if success else error reason.
 *
 */
cybt_result_t cybt_debug_uart_send_hci_trace (uint8_t type, uint16_t length, uint8_t* p_data);

/**
 * Sends data over Debug UART
 *
 * @param[in] opcode  : Opcode
 * @param[in] length: Length of the data
 * @param[in] p_data: data pointer
 *
 * @returns  CYBT_SUCCESS if success else error reason.
 *
 * @note This can be used from register callback of wiced_bt_dev_register_hci_trace function.
 */
cybt_result_t cybt_debug_uart_send_data (uint16_t opcode, uint16_t data_size, uint8_t *p_data);

/**
 * Sends coredump HCI traces over Debug UART
 *
 * @param[in] length: Length of the data
 * @param[in] p_data: data pointer
 *
 * @returns  CYBT_SUCCESS if success else error reason.
 */
cybt_result_t cybt_send_coredump_hci_trace (uint16_t data_size, uint8_t *p_data);

/**
* Sends a WICED HCI packet already in a WICED buffer over Debug UART
*
* @param[in] p_pkt: pointer to the WICED HCI packet in a wiced_bt_buffer
* @param[in] op_code: the WICED HCI command/event code
* @param[in] pay_len: the application payload length
*
* @returns  CYBT_SUCCESS if the transmission is started, else error reason.
*
* @note the buffer should be obtained using wiced_bt_get_buffer(). The first
* WICED_HCI_HDR_LEN bytes of the buffer should be left unused by the application,
* as they will be filled in with the WICED HCI header.
*
* If this function returns error, the buffer is not freed. After transmission,
* the the buffer is freed using a call to wiced_bt_free_buffer().
*/
cybt_result_t cybt_debug_uart_send_wiced_hci_buf (void *p_buf, uint16_t op_code, uint16_t pay_len);

/**@} */

#endif //CYBT_DEBUG_UART_H