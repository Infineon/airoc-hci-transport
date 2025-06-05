#if !defined (CY_USING_HAL)
#include "cyabs_rtos.h"
#include "cybsp_types.h"
#include "cybt_debug_uart.h"
#include "wiced_memory.h"
#include "cycfg.h"
#include "cybsp_bt_config.h"
#include "mtb_hal.h"

#if !(defined(STACK_INSIDE_BT_CTRLR) && (STACK_INSIDE_BT_CTRLR == 1))
#include "cybt_platform_task.h"
#include "cybt_platform_config.h"
#include "cybt_platform_interface.h"

#if (CY_CFG_PWR_SYS_IDLE_MODE == CY_CFG_PWR_MODE_DEEPSLEEP)
#include "mtb_syspm_callbacks.h"
#define DEBUG_UART_SYSPM_SKIP_MODE         (0U)
#define DEBUG_UART_SYSPM_CALLBACK_ORDER    (1U)
#endif

#define CYBSP_DEBUG_UART_IRQ_SOURCE   (CYBSP_DEBUG_UART_IRQ)

cy_stc_scb_uart_context_t  CYBSP_DEBUG_UART_airoc_context;  /** UART context */
mtb_async_transfer_context_t CYBSP_DEBUG_UART_async_tx_context;

extern void host_stack_mutex_lock(void * p_lock_context);
extern void host_stack_mutex_unlock(void * p_lock_context);
#else
cy_mutex_t   bt_stack_mutex;
#endif

#ifndef CYHAL_UART_DMA_ENABLED
#define CYHAL_UART_DMA_ENABLED FALSE
#endif

#ifdef ENHANCED_WICED_HCI
    #define WICED_PREAMBLE_BYTE1        0x7E
    #define WICED_PREAMBLE_BYTE2        0xA5
    #define WICED_PREAMBLE_BYTE3        0x5A
    enum
    {
        WAIT_PREAMBLE_1 = 0,
        WAIT_PREAMBLE_2,
        WAIT_PREAMBLE_3,
        WAIT_OP_CODE_AND_LEN,
        WAIT_PAYLOAD
    };

	static uint16_t calc_packet_crc (uint8_t *p_data, int length);
	static void cybt_build_enhanced_wiced_hci_pkt (uint8_t *p_buf, uint8_t type, uint16_t  opcode, uint16_t length, uint8_t* p_data);
	static void cybt_debug_enhanced_rx_task(void);
#else
    #define HCI_WICED_PKT               0x19

    enum
    {
        HEADER_PHASE = 0,
        DATA_PHASE
    };
#endif // #ifdef ENHANCED_WICED_HCI

#ifndef DISABLE_TX_TASK
#define BT_TASK_NAME_DEBUG_UART_TX       "TX_CYBT_DEBUG_UART_Task"
#ifndef DEBUG_UART_TX_TASK_STACK_SIZE
#define DEBUG_UART_TX_TASK_STACK_SIZE    (0x1700)
#endif
#ifndef DEBUG_UART_TX_TASK_QUEUE_COUNT
#define DEBUG_UART_TX_TASK_QUEUE_COUNT   (50)
#endif
#define DEBUG_UART_TX_QUEUE_ITEM_SIZE    (sizeof(void *))
#define DEBUG_UART_TX_TASK_QUEUE         cybt_debug_uart_tx_queue
#define DEBUG_UART_TX_TASK_PRIORITY      (CY_RTOS_PRIORITY_ABOVENORMAL)

#ifndef DEBUG_UART_MEMORY_SIZE
#define DEBUG_UART_MEMORY_SIZE           (6144)
#endif

wiced_bt_heap_t *debug_task_heap = NULL;
#endif

#define BT_TASK_NAME_DEBUG_UART_RX       "RX_CYBT_DEBUG_UART_Task"

#ifndef DEBUG_UART_RX_TASK_STACK_SIZE
#define DEBUG_UART_RX_TASK_STACK_SIZE    (0x1700)
#endif
#define DEBUG_UART_RX_TASK_PRIORITY     (CY_RTOS_PRIORITY_ABOVENORMAL)

#define WICED_HDR_SZ 5

#define MAX_RX_DATA_LEN                 1000

#define HCI_CONTROL_GROUP_DEVICE       0x00
#define HCI_CONTROL_EVENT_WICED_TRACE  ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x02 )    /* WICED trace packet */
#define HCI_CONTROL_EVENT_HCI_TRACE    ( ( HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x03 )    /* Bluetooth protocol trace */
#define INVALID_TYPE                    0xFF
#define GENERIC_TYPE                    0x00

#undef CYBT_TRACE_BUFFER_SIZE
#define  CYBT_TRACE_BUFFER_SIZE    (256)

#undef INITIAL_TRACE_LEVEL_STACK
#define INITIAL_TRACE_LEVEL_STACK         (CYBT_TRACE_LEVEL_MAX)

typedef struct
{
    bool            inited;
    mtb_hal_uart_t    CYBSP_DEBUG_UART_hal_obj;
#ifndef DISABLE_TX_TASK
    cy_semaphore_t  tx_complete;
    cy_semaphore_t  tx_ready;
    cy_mutex_t      tx_heap_lock;
#endif
    cy_semaphore_t  rx_complete;
    cy_mutex_t      tx_atomic;
} hci_uart_cb_t;

typedef struct
{
    uint16_t length;
    uint8_t *data;
}recv_data_t;

hci_uart_cb_t cy_trans_uart;
#ifndef DISABLE_TX_TASK
cy_queue_t  cybt_debug_uart_tx_queue;
cy_thread_t cybt_debug_uart_tx_task;
#endif
cy_thread_t cybt_debug_uart_rx_task;

/*
 * Global variable declarations
 */
uint8_t  wiced_rx_cmd[MAX_RX_DATA_LEN + WICED_HCI_OVERHEAD]; //RX command pool.


static bool trans_setup = false;
static cybt_debug_uart_data_handler_t  p_app_rx_cmd_handler =  NULL;
volatile bool rx_done = false;

static bool cybt_debug_uart_setup = false;

#ifdef DISABLE_TX_TASK

static cybt_result_t cybt_trans_blocking_write (uint8_t type, uint16_t opcode, uint16_t data_size, uint8_t *p_data);

#else

static cybt_result_t cybt_enqueue_tx_data (uint8_t type, uint16_t  op,uint16_t length, uint8_t* p_data);

#if defined(STACK_INSIDE_BT_CTRLR) && (STACK_INSIDE_BT_CTRLR == 1)
void cybt_platform_disable_irq(void)
{
    __disable_irq();
}

void cybt_platform_enable_irq(void)
{
    __enable_irq();
}

void *cybt_platform_malloc(uint32_t req_size)
{
    return malloc((size_t) req_size);
}

void host_stack_mutex_lock(void * p_lock_context)
{
    cy_rtos_get_mutex(&bt_stack_mutex, CY_RTOS_NEVER_TIMEOUT);
}

void host_stack_mutex_unlock(void * p_lock_context)
{
    cy_rtos_set_mutex(&bt_stack_mutex);
}
#endif

/* DEBUG UART deepsleep callback parameters  */
#if (CY_CFG_PWR_SYS_IDLE_MODE == CY_CFG_PWR_MODE_DEEPSLEEP)
/* Context reference structure for Debug UART */
static mtb_syspm_uart_deepsleep_context_t debug_uart_syspm_ds_context =
{
    .uart_context       = &CYBSP_DEBUG_UART_airoc_context,
    .async_context      = &CYBSP_DEBUG_UART_async_tx_context,
    .tx_pin =
    {
        .port           = CYBSP_DEBUG_UART_TX_PORT,
        .pinNum         = CYBSP_DEBUG_UART_TX_PIN,
        .hsiom          = CYBSP_DEBUG_UART_TX_HSIOM
    },
    .rts_pin =
    {
        .port           = CYBSP_DEBUG_UART_RTS_PORT,
        .pinNum         = CYBSP_DEBUG_UART_RTS_PIN,
        .hsiom          = CYBSP_DEBUG_UART_RTS_HSIOM
    }
};

/* SysPm callback parameter structure for Debug UART */
static cy_stc_syspm_callback_params_t debug_uart_syspm_cb_params =
{
    .context            = &debug_uart_syspm_ds_context,
    .base               = CYBSP_DEBUG_UART_HW
};

/* SysPm callback structure for BT UART */
static cy_stc_syspm_callback_t debug_uart_syspm_cb =
{
    .callback           = &mtb_syspm_scb_uart_deepsleep_callback,
    .skipMode           = DEBUG_UART_SYSPM_SKIP_MODE,
    .type               = CY_SYSPM_DEEPSLEEP,
    .callbackParams     = &debug_uart_syspm_cb_params,
    .prevItm            = NULL,
    .nextItm            = NULL,
    .order              = DEBUG_UART_SYSPM_CALLBACK_ORDER
};
#endif

BTSTACK_PORTING_SECTION_BEGIN
static void debug_heap_mutex_lock(void * p_lock_context)
{
    cy_rtos_get_mutex(p_lock_context, CY_RTOS_NEVER_TIMEOUT);
}
BTSTACK_PORTING_SECTION_END

BTSTACK_PORTING_SECTION_BEGIN
static void debug_heap_mutex_unlock(void * p_lock_context)
{
    cy_rtos_set_mutex(p_lock_context);
}
BTSTACK_PORTING_SECTION_END

BTSTACK_PORTING_SECTION_BEGIN
static void *cybt_platform_debug_task_mempool_alloc(uint32_t req_size)
{
    void *p_mem_block;

    if (NULL == debug_task_heap)
    {
        return NULL;
    }

    p_mem_block = (void *) wiced_bt_get_buffer_from_heap(debug_task_heap, req_size);

    return p_mem_block;
}
BTSTACK_PORTING_SECTION_END

BTSTACK_PORTING_SECTION_BEGIN
static void *cybt_memcpy (void *dest, const void *src, size_t len)
{
    uint8_t *pDest = (uint8_t*)dest;
    const uint8_t *pSrc = (const uint8_t*)src;
    while(len--)
    {
        *pDest++ = *pSrc++;
    }
    return dest;
}
BTSTACK_PORTING_SECTION_END

BTSTACK_PORTING_SECTION_BEGIN
static void cybt_platform_debug_task_mempool_free(void *p_mem_block)
{
    wiced_bt_free_buffer((wiced_bt_buffer_t *) p_mem_block);
}
BTSTACK_PORTING_SECTION_END

BTSTACK_PORTING_SECTION_BEGIN
static cybt_result_t cybt_enqueue_tx_data (uint8_t type, uint16_t  opcode, uint16_t length, uint8_t* p_data)
{
    uint8_t     *p_buf = NULL, *p_buf_start = NULL;
    cy_rslt_t   result = CYBT_ERR_GENERIC;
    size_t      count = 0;

    if (trans_setup == false)
        return CYBT_ERR_GENERIC;

    result = cy_rtos_get_semaphore(&cy_trans_uart.tx_ready, CY_RTOS_NEVER_TIMEOUT, false);

    if (CY_RSLT_SUCCESS != result)
        return CYBT_ERR_GENERIC;

    result = cy_rtos_count_queue(&DEBUG_UART_TX_TASK_QUEUE, &count);
    if ( (result != CY_RSLT_SUCCESS) || (count == DEBUG_UART_TX_TASK_QUEUE_COUNT))
    {
        cy_rtos_set_semaphore(&cy_trans_uart.tx_ready, false);
        return CYBT_ERR_QUEUE_FULL;
    }

    p_buf = cybt_platform_debug_task_mempool_alloc(length + WICED_HCI_OVERHEAD + 1);
    if (p_buf == NULL)
    {
        cy_rtos_set_semaphore(&cy_trans_uart.tx_ready, false);
        return CYBT_ERR_OUT_OF_MEMORY;
    }
    p_buf_start = p_buf;

#ifdef ENHANCED_WICED_HCI
    cybt_build_enhanced_wiced_hci_pkt (p_buf, type, opcode, length, p_data);
#else
    *p_buf++ = HCI_WICED_PKT;

    if ( (type != INVALID_TYPE) || (opcode == HCI_CONTROL_EVENT_HCI_TRACE) )
    {
        *p_buf++ = (uint8_t)(HCI_CONTROL_EVENT_HCI_TRACE);
        *p_buf++ = (uint8_t)(HCI_CONTROL_EVENT_HCI_TRACE >> 8);
        *p_buf++ = (uint8_t)(length + 1);
        *p_buf++ = (uint8_t)((length + 1) >> 8);
        *p_buf++ = type;
    }
    else
    {
        *p_buf++ = (uint8_t)(opcode);
        *p_buf++ = (uint8_t)(opcode >> 8);
        *p_buf++ = (uint8_t)(length);
        *p_buf++ = (uint8_t)(length >> 8);
    }

    cybt_memcpy (p_buf, p_data, length);
#endif // ifdef ENHANCED_WICED_HCI

    result = cy_rtos_put_queue (&DEBUG_UART_TX_TASK_QUEUE, (void *) &p_buf_start, 0, false);

    if (CY_RSLT_SUCCESS != result)
        cybt_platform_debug_task_mempool_free((void *)p_buf_start);

    cy_rtos_set_semaphore(&cy_trans_uart.tx_ready, false);
    return CYBT_SUCCESS;
}
BTSTACK_PORTING_SECTION_END

BTSTACK_PORTING_SECTION_BEGIN
cybt_result_t cybt_debug_uart_send_wiced_hci_buf (void *p_buf, uint16_t op_code, uint16_t pay_len)
{
    cy_rslt_t   result = CYBT_ERR_GENERIC;
    size_t      count = 0;
    uint8_t     *pb = (uint8_t *)p_buf;
    uint8_t     *pb_start = (uint8_t *)p_buf;

    if (trans_setup == false)
        return CYBT_ERR_GENERIC;

    result = cy_rtos_get_semaphore(&cy_trans_uart.tx_ready, CY_RTOS_NEVER_TIMEOUT, false);

    if (CY_RSLT_SUCCESS != result)
        return CYBT_ERR_GENERIC;

    result = cy_rtos_count_queue(&DEBUG_UART_TX_TASK_QUEUE, &count);
    if ( (result != CY_RSLT_SUCCESS) || (count == DEBUG_UART_TX_TASK_QUEUE_COUNT))
    {
        cy_rtos_set_semaphore (&cy_trans_uart.tx_ready, false);
        return CYBT_ERR_QUEUE_FULL;
    }

#ifdef ENHANCED_WICED_HCI
    uint8_t     hdr_crc;
    uint16_t    pkt_crc;

    *pb++ = WICED_PREAMBLE_BYTE1;
    *pb++ = WICED_PREAMBLE_BYTE2;
    *pb++ = WICED_PREAMBLE_BYTE3;

    hdr_crc  = (uint8_t)(op_code) + (uint8_t)(op_code >> 8) + (uint8_t)(pay_len) + (uint8_t)(pay_len >> 8);
    *pb++ = hdr_crc;

    *pb++ = (uint8_t)(op_code);
    *pb++ = (uint8_t)(op_code >> 8);
    *pb++ = (uint8_t)(pay_len);
    *pb++ = (uint8_t)(pay_len >> 8);


    pb += pay_len;

    pkt_crc = calc_packet_crc (pb_start, (uint16_t)(pb - pb_start));
    *pb++ = (uint8_t)(pkt_crc);
    *pb++ = (uint8_t)(pkt_crc >> 8);
#else
    *pb++ = HCI_WICED_PKT;
    *pb++ = (uint8_t)(op_code);
    *pb++ = (uint8_t)(op_code >> 8);
    *pb++ = (uint8_t)(pay_len);
    *pb++ = (uint8_t)(pay_len >> 8);
#endif // ifdef ENHANCED_WICED_HCI

    result = cy_rtos_put_queue (&DEBUG_UART_TX_TASK_QUEUE, (void *)&pb_start, 0, false);

    cy_rtos_set_semaphore(&cy_trans_uart.tx_ready, false);
    return (cybt_result_t)result;
}
BTSTACK_PORTING_SECTION_END

BTSTACK_PORTING_SECTION_BEGIN
#if defined(STACK_INSIDE_BT_CTRLR) && (STACK_INSIDE_BT_CTRLR == 1)
static void cybt_debug_tx_task(cy_thread_arg_t arg)
#else
static void cybt_debug_tx_task(void *arg)
#endif
{
    cy_rslt_t   result;
    uint8_t     *p_data;
    size_t      data_len;

    while (1)
    {
        p_data = NULL;
        result = cy_rtos_get_queue(&DEBUG_UART_TX_TASK_QUEUE,
                                   (void *)&p_data,
                                   CY_RTOS_NEVER_TIMEOUT,
                                   false
                                   );

        if (CY_RSLT_SUCCESS != result || NULL == p_data)
            continue;

        // The WICED HCI header with the length follows the preamble
        data_len = ((p_data[WICED_HCI_PREAMBLE_LEN + 3] << 8) | p_data[WICED_HCI_PREAMBLE_LEN + 2]);

        result = mtb_hal_uart_write_async (&cy_trans_uart.CYBSP_DEBUG_UART_hal_obj, p_data, data_len + WICED_HCI_OVERHEAD);

        if (CY_RSLT_SUCCESS == result)
            cy_rtos_get_semaphore (&cy_trans_uart.tx_complete, CY_RTOS_NEVER_TIMEOUT, false);

        cybt_platform_debug_task_mempool_free (p_data);
    }
}
BTSTACK_PORTING_SECTION_END

#endif

BTSTACK_PORTING_SECTION_BEGIN
#if defined(STACK_INSIDE_BT_CTRLR) && (STACK_INSIDE_BT_CTRLR == 1)
static void cybt_debug_rx_task(cy_thread_arg_t arg)
#else
static void cybt_debug_rx_task(void *arg)
#endif
{
#ifdef ENHANCED_WICED_HCI
    cybt_debug_enhanced_rx_task();
#else
    cy_rslt_t result;
    volatile uint32_t numAvailable = 0;
    volatile size_t expectedlength = 0;
    volatile uint32_t head=0,phase=HEADER_PHASE,data_counter=0;

    while(1)
    {
        result = cy_rtos_get_semaphore(&cy_trans_uart.rx_complete, CY_RTOS_NEVER_TIMEOUT, false);

        if (result != CY_RSLT_SUCCESS)
            continue;

        numAvailable = 0;
        expectedlength = ( phase == DATA_PHASE ) ? ( data_counter ) : ( WICED_HDR_SZ );
        if (!rx_done)
        {
            if (CYHAL_UART_DMA_ENABLED == TRUE)
            {
				mtb_hal_uart_enable_event(&cy_trans_uart.CYBSP_DEBUG_UART_hal_obj, MTB_HAL_UART_IRQ_RX_DONE, true);
				mtb_hal_uart_read_async(&cy_trans_uart.CYBSP_DEBUG_UART_hal_obj, wiced_rx_cmd + head, expectedlength);
                continue;
            }
            else
            {
                numAvailable = mtb_hal_uart_readable(&cy_trans_uart.CYBSP_DEBUG_UART_hal_obj);
                if (numAvailable >= expectedlength)
                {
			mtb_hal_uart_read(&cy_trans_uart.CYBSP_DEBUG_UART_hal_obj, wiced_rx_cmd + head, (size_t *)&expectedlength);
                    numAvailable -= expectedlength;
                }
                else
                {
					mtb_hal_uart_enable_event(&cy_trans_uart.CYBSP_DEBUG_UART_hal_obj, MTB_HAL_UART_IRQ_RX_DONE, true);
					mtb_hal_uart_read_async(&cy_trans_uart.CYBSP_DEBUG_UART_hal_obj, wiced_rx_cmd + head, expectedlength);
                    continue;
                }
            }
        }

        if (numAvailable == 0)
        {
            rx_done = false;
            mtb_hal_uart_enable_event(&cy_trans_uart.CYBSP_DEBUG_UART_hal_obj, MTB_HAL_UART_IRQ_RX_NOT_EMPTY, true);
        }

        switch (phase)
        {
            case HEADER_PHASE:
                if(wiced_rx_cmd[0] != HCI_WICED_PKT)
                {
                    head=0x0;
                    break;
                }
                data_counter = ( wiced_rx_cmd[3] | (uint32_t)(wiced_rx_cmd[4])<<8);
                head  = WICED_HDR_SZ;
                phase = DATA_PHASE;
                break;
            case DATA_PHASE:
                data_counter -= expectedlength;
                head         += expectedlength;
                break;
        }

        if ( (data_counter == 0) && (head != 0) && (p_app_rx_cmd_handler != NULL) )
        {
            phase = HEADER_PHASE;
            host_stack_mutex_lock(NULL);
            p_app_rx_cmd_handler(wiced_rx_cmd+1, head-1);
            host_stack_mutex_unlock(NULL);
            head = 0;
        }

        if (numAvailable != 0)
        {
            // re-enter the loop if data is available
            cy_rtos_set_semaphore(&cy_trans_uart.rx_complete, true);
            continue;
        }
    }
#endif  // #ifdef ENHANCED_WICED_HCI
}
BTSTACK_PORTING_SECTION_END

cybt_result_t cybt_init_debug_trans_task(void)
{
    cy_rslt_t result;

    if(true == trans_setup)
        return CYBT_SUCCESS;

#ifndef DISABLE_TX_TASK
    {
        void *p_heap_mem = NULL;

        cy_rtos_init_mutex(&cy_trans_uart.tx_heap_lock);

        wiced_bt_lock_t lock = {
            .p_lock_context = &cy_trans_uart.tx_heap_lock,
            .pf_lock_func = debug_heap_mutex_lock,
            .pf_unlock_func = debug_heap_mutex_unlock
        };

        p_heap_mem = (wiced_bt_heap_t *)cybt_platform_malloc(DEBUG_UART_MEMORY_SIZE);
        if (p_heap_mem == NULL)
            return CYBT_ERR_OUT_OF_MEMORY;

        debug_task_heap = wiced_bt_create_heap("CYBT_DEBUG_TASK_POOL",
                                               p_heap_mem,
                                               DEBUG_UART_MEMORY_SIZE,
                                               &lock,
                                               FALSE
                                               );

        result = cy_rtos_init_queue(&DEBUG_UART_TX_TASK_QUEUE,
                                    DEBUG_UART_TX_TASK_QUEUE_COUNT,
                                    DEBUG_UART_TX_QUEUE_ITEM_SIZE
                                    );
        if (result != CY_RSLT_SUCCESS)
            return CYBT_ERR_INIT_QUEUE_FAILED;

        result = cy_rtos_create_thread(&cybt_debug_uart_tx_task,
                                       cybt_debug_tx_task,
                                       BT_TASK_NAME_DEBUG_UART_TX,
                                       NULL,
                                       DEBUG_UART_TX_TASK_STACK_SIZE,
                                       DEBUG_UART_TX_TASK_PRIORITY,
                                       (cy_thread_arg_t) NULL
                                       );
        if (result != CY_RSLT_SUCCESS)
            return CYBT_ERR_CREATE_TASK_FAILED;
    }
#endif

    result = cy_rtos_create_thread(&cybt_debug_uart_rx_task,
                                   cybt_debug_rx_task,
                                   BT_TASK_NAME_DEBUG_UART_RX,
                                   NULL,
                                   DEBUG_UART_RX_TASK_STACK_SIZE,
                                   DEBUG_UART_RX_TASK_PRIORITY,
                                   (cy_thread_arg_t) NULL
                                   );

    if (result != CY_RSLT_SUCCESS)
        return CYBT_ERR_CREATE_TASK_FAILED;

    trans_setup = true;
    return CYBT_SUCCESS;
}

#ifndef DISABLE_TX_TASK
static void cybt_uart_tx_irq(void)
{
    cy_rtos_set_semaphore(&cy_trans_uart.tx_complete, true);
}
#endif

BTSTACK_PORTING_SECTION_BEGIN
static void cybt_uart_irq_handler_(void *handler_arg, mtb_hal_uart_event_t event)
{
    switch(event)
    {
        case MTB_HAL_UART_IRQ_RX_NOT_EMPTY:
		mtb_hal_uart_enable_event(&cy_trans_uart.CYBSP_DEBUG_UART_hal_obj, MTB_HAL_UART_IRQ_RX_NOT_EMPTY, false);
            cy_rtos_set_semaphore(&cy_trans_uart.rx_complete, true);
            break;
        case MTB_HAL_UART_IRQ_RX_DONE:
            rx_done = true;
            mtb_hal_uart_enable_event(&cy_trans_uart.CYBSP_DEBUG_UART_hal_obj, MTB_HAL_UART_IRQ_RX_DONE, false);
            cy_rtos_set_semaphore(&cy_trans_uart.rx_complete, true);
            break;
#ifndef DISABLE_TX_TASK
        case MTB_HAL_UART_IRQ_TX_TRANSMIT_IN_FIFO:
		mtb_hal_uart_enable_event(  &cy_trans_uart.CYBSP_DEBUG_UART_hal_obj,
		                                        MTB_HAL_UART_IRQ_TX_EMPTY,
		                                        true);
		break;
        case MTB_HAL_UART_IRQ_TX_EMPTY:
			mtb_hal_uart_enable_event(  &cy_trans_uart.CYBSP_DEBUG_UART_hal_obj,
										MTB_HAL_UART_IRQ_TX_EMPTY,
										false);
            cybt_uart_tx_irq();
            break;
#endif
        default:
            break;
    }
}
BTSTACK_PORTING_SECTION_END

void mw_debug_uart_process_interrupt(mtb_hal_uart_t* uart)
{
    mtb_hal_uart_process_interrupt(uart);
}

void cybsb_debug_uart_interrupt_handler(void)
{
    mw_debug_uart_process_interrupt(&cy_trans_uart.CYBSP_DEBUG_UART_hal_obj);
}

cybt_result_t cybt_debug_uart_init(cybt_debug_uart_config_t *config, cybt_debug_uart_data_handler_t p_data_handler)
{
	cy_rslt_t result;
	uint32_t actual_baud_rate;

    uint16_t enable_irq_event = (MTB_HAL_UART_IRQ_TX_TRANSMIT_IN_FIFO | MTB_HAL_UART_IRQ_RX_NOT_EMPTY);

    if(!cybt_debug_uart_setup)
    {
        memset(&cy_trans_uart, 0, sizeof(hci_uart_cb_t));
    }
    else
    {
        memset(&cy_trans_uart.CYBSP_DEBUG_UART_hal_obj, 0, sizeof(mtb_hal_uart_t));
    }

    if (!config)
	{
		return CYBT_ERR_BADARG;
	}

	/* Initialize the SCB UART */
	result = (cy_rslt_t)Cy_SCB_UART_Init(CYBSP_DEBUG_UART_HW,
										&CYBSP_DEBUG_UART_config,
										&CYBSP_DEBUG_UART_airoc_context);
	if(CY_RSLT_SUCCESS != result)
	{
		return  CYBT_ERR_HCI_INIT_FAILED;
	}

	/* Enable the SCB UART */
	Cy_SCB_UART_Enable(CYBSP_DEBUG_UART_HW);

	/*Enable the interrupt*/
	cy_stc_sysint_t intr_cfg_1 = {.intrSrc = CYBSP_DEBUG_UART_IRQ_SOURCE, .intrPriority = 7u};
	Cy_SysInt_Init(&intr_cfg_1, cybsb_debug_uart_interrupt_handler);
	NVIC_EnableIRQ(CYBSP_DEBUG_UART_IRQ);

	/*Initialize the HAL NEXT object*/
	result = mtb_hal_uart_setup(&cy_trans_uart.CYBSP_DEBUG_UART_hal_obj,
								&CYBSP_DEBUG_UART_hal_config,
								&CYBSP_DEBUG_UART_airoc_context, NULL);
	if(CY_RSLT_SUCCESS != result)
	{
		return  CYBT_ERR_HCI_INIT_FAILED;
	}

	result = mtb_hal_uart_config_async(&cy_trans_uart.CYBSP_DEBUG_UART_hal_obj,&CYBSP_DEBUG_UART_async_tx_context);
	if(CY_RSLT_SUCCESS != result)
	{
		return  CYBT_ERR_HCI_INIT_FAILED;
	}

    result = mtb_hal_uart_set_baud(&cy_trans_uart.CYBSP_DEBUG_UART_hal_obj,
								config->baud_rate,
									&actual_baud_rate
								   );
    if(CY_RSLT_SUCCESS != result)
	{
		return  CYBT_ERR_HCI_SET_BAUDRATE_FAILED;
	}

	if (config->flow_control)
	{
		result = mtb_hal_uart_enable_cts_flow_control(&cy_trans_uart.CYBSP_DEBUG_UART_hal_obj, true);
		if(CY_RSLT_SUCCESS != result)
		{
			return  CYBT_ERR_HCI_SET_FLOW_CTRL_FAILED;
		}
	}

    if(!cybt_debug_uart_setup)
    {
#ifndef DISABLE_TX_TASK
        cy_rtos_init_semaphore(&cy_trans_uart.tx_complete, 1, 0);
        cy_rtos_init_semaphore(&cy_trans_uart.tx_ready, 1, 1);
#endif
        cy_rtos_init_semaphore(&cy_trans_uart.rx_complete, 1, 0);

        cy_rtos_init_mutex(&cy_trans_uart.tx_atomic);
    }

    mtb_hal_uart_register_callback(&cy_trans_uart.CYBSP_DEBUG_UART_hal_obj, cybt_uart_irq_handler_, NULL);
    mtb_hal_uart_enable_event(&cy_trans_uart.CYBSP_DEBUG_UART_hal_obj, (mtb_hal_uart_event_t)enable_irq_event, true);

#if (CY_CFG_PWR_SYS_IDLE_MODE == CY_CFG_PWR_MODE_DEEPSLEEP)
    /* UART SysPm callback registration for DEBUG UART */
    Cy_SysPm_RegisterCallback(&debug_uart_syspm_cb);
#endif /* (CY_CFG_PWR_SYS_IDLE_MODE == CY_CFG_PWR_MODE_DEEPSLEEP) */

    cy_trans_uart.inited = true;
    p_app_rx_cmd_handler = p_data_handler;

    if(!cybt_debug_uart_setup)
    {
        cybt_debug_uart_setup = true;
        cybt_init_debug_trans_task();
    }

    return CYBT_SUCCESS;
}

void cybt_debug_uart_deinit()
{
    if(cybt_debug_uart_setup)
    {
        cy_trans_uart.inited = false;
        cy_rtos_set_semaphore(&cy_trans_uart.tx_complete, true);
    }
}

cybt_result_t cybt_debug_uart_send_trace(uint16_t length, uint8_t* p_data)
{
#ifdef DISABLE_TX_TASK
    return cybt_trans_blocking_write(INVALID_TYPE, HCI_CONTROL_EVENT_WICED_TRACE, length, p_data);
#else
    return cybt_enqueue_tx_data(INVALID_TYPE, HCI_CONTROL_EVENT_WICED_TRACE, length, p_data);
#endif
}

BTSTACK_PORTING_SECTION_BEGIN
cybt_result_t cybt_debug_uart_send_data (uint16_t opcode, uint16_t data_size, uint8_t *p_data)
{
#ifdef DISABLE_TX_TASK
    return cybt_trans_blocking_write(INVALID_TYPE,(uint16_t)opcode, data_size, p_data);
#else
    return cybt_enqueue_tx_data(INVALID_TYPE, (uint16_t)opcode, data_size, p_data);
#endif
}
BTSTACK_PORTING_SECTION_END

BTSTACK_PORTING_SECTION_BEGIN
cybt_result_t cybt_debug_uart_send_hci_trace (uint8_t type, uint16_t data_size, uint8_t *p_data)
{
#ifdef DISABLE_TX_TASK
    return cybt_trans_blocking_write((uint16_t)type, HCI_CONTROL_EVENT_HCI_TRACE, data_size, p_data);
#else
    return cybt_enqueue_tx_data (type, HCI_CONTROL_EVENT_HCI_TRACE, data_size, p_data);
#endif
}
BTSTACK_PORTING_SECTION_END

cybt_result_t cybt_send_coredump_hci_trace (uint16_t data_size, uint8_t *p_data)
{
#ifdef DISABLE_TX_TASK
    return cybt_trans_blocking_write(GENERIC_TYPE, HCI_CONTROL_EVENT_HCI_TRACE, data_size, p_data);
#else
    return cybt_enqueue_tx_data(GENERIC_TYPE, HCI_CONTROL_EVENT_HCI_TRACE, data_size, p_data);
#endif
}

#ifdef DISABLE_TX_TASK

BTSTACK_PORTING_SECTION_BEGIN
cybt_result_t cybt_trans_blocking_write (uint8_t type, uint16_t op, uint16_t data_size, uint8_t *p_data)
{
    cybt_result_t result = CYBT_ERR_GENERIC;
    cy_rslt_t status = CY_RSLT_SUCCESS;
    size_t index = 0;
    uint8_t opcode = (uint8_t)(op&0xff);
    uint8_t group_code = (uint8_t)((op >> 8)&0xff);
    uint8_t data[2000];
    int nChars = 0;
    char *ptr = NULL;

    if (cy_trans_uart.inited == false)
        return CYBT_ERR_GENERIC;

    status = cy_rtos_get_mutex(&cy_trans_uart.tx_atomic, CY_RTOS_NEVER_TIMEOUT);

    if(CY_RSLT_SUCCESS != status)
    {
        return result;
    }

    data[index++] = HCI_WICED_PKT;

    if ( (type != 0xFF) || ((group_code == 0x00) && (opcode == 0x03)) )
    {
        uint16_t new_size = (data_size+1);
        data[index++] = 0x03;
        data[index++] = 0x00;
        data[index++] = (uint8_t)(new_size&0xff);
        data[index++] = (uint8_t)((new_size >> 8)&0xff);
        data[index++] = type;
    }
    else
    {
        data[index++] = opcode;
        data[index++] = group_code;
        data[index++] = (uint8_t)(data_size&0xff);
        data[index++] = (uint8_t)((data_size >> 8)&0xff);
    }
    memcpy(&data[index], p_data, data_size);
    index += data_size;

    /*! NB TODO
    * Remove local data var. Print individual bytes. */
    ptr = (char *)&data[0];
    for (/* Empty */; nChars < index; ++nChars)
    {
	mtb_hal_uart_put(&cy_trans_uart.CYBSP_DEBUG_UART_hal_obj, *ptr);
        ++ptr;
    }

    cy_rtos_set_mutex(&cy_trans_uart.tx_atomic);
    return result;
}
BTSTACK_PORTING_SECTION_END
#endif // DISABLE_TX_TASK

#if defined(ENABLE_AIROC_HCI_TRANSPORT_PRINTF) && (ENABLE_AIROC_HCI_TRANSPORT_PRINTF==1)
#if defined (__ICCARM__)
#ifndef PRINTF_BUF_SIZE_IAR
#define PRINTF_BUF_SIZE_IAR 128
#endif

static char printf_buf_iar[PRINTF_BUF_SIZE_IAR];
static int char_count = 0;

int __write(int fd, const char* ptr, int len)
#else
int _write(int fd, const char* ptr, int len)
#endif
{
#if defined (__ICCARM__)
    printf_buf_iar[char_count] = *ptr;
    char_count++;

    if ((char_count == PRINTF_BUF_SIZE_IAR) || (*ptr == '\n') || (*ptr == '\r'))
    {
        if(cybt_debug_uart_send_trace(char_count,(uint8_t* )&printf_buf_iar) == CYBT_SUCCESS)
        {
            char_count = 0;
            return 1;
        }
        char_count = 0;
        return 0;
    }
    return 1;
#else
    if(cybt_debug_uart_send_trace(len,(uint8_t* )ptr) == CYBT_SUCCESS)
    {
       return len;
    }
    return 0;
#endif
}
#endif //ENABLE_AIROC_HCI_TRANSPORT_PRINTF

#ifdef ENHANCED_WICED_HCI
static const uint16_t ccitt_lkup_table[] =
{
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

BTSTACK_PORTING_SECTION_BEGIN
static uint16_t calc_packet_crc (uint8_t *p_data, int length)
{
    uint16_t crc = 0xFFFF;
    int      i;

    for (i = 0; i < length; ++i)
    {
        crc = (crc << 8) ^ ccitt_lkup_table[((crc >> 8) ^ p_data[i]) & 0xFF];
    }
    return crc;
}
BTSTACK_PORTING_SECTION_END

BTSTACK_PORTING_SECTION_BEGIN
static void cybt_build_enhanced_wiced_hci_pkt (uint8_t *p_buf, uint8_t type, uint16_t  opcode, uint16_t length, uint8_t *p_data)
{
    uint8_t     *p_buf_start = p_buf;
    uint8_t     hdr_crc;
    uint16_t    pkt_crc;

    // Enhanced WICED HCI has a 3-byte preamble
    *p_buf++ = WICED_PREAMBLE_BYTE1;
    *p_buf++ = WICED_PREAMBLE_BYTE2;
    *p_buf++ = WICED_PREAMBLE_BYTE3;

    if ( (type != INVALID_TYPE) || (opcode == HCI_CONTROL_EVENT_HCI_TRACE) )
    {
        hdr_crc  = (uint8_t)(HCI_CONTROL_EVENT_HCI_TRACE) + (uint8_t)(HCI_CONTROL_EVENT_HCI_TRACE >> 8) + (uint8_t)(length + 1) + (uint8_t)((length + 1) >> 8);
        *p_buf++ = hdr_crc;

        *p_buf++ = (uint8_t)(HCI_CONTROL_EVENT_HCI_TRACE);
        *p_buf++ = (uint8_t)(HCI_CONTROL_EVENT_HCI_TRACE >> 8);
        *p_buf++ = (uint8_t)(length + 1);
        *p_buf++ = (uint8_t)((length + 1) >> 8);
        *p_buf++ = type;
    }
    else
    {
        hdr_crc  = (uint8_t)(opcode) + (uint8_t)(opcode >> 8) + (uint8_t)(length) + (uint8_t)(length >> 8);
        *p_buf++ = hdr_crc;

        *p_buf++ = (uint8_t)(opcode);
        *p_buf++ = (uint8_t)(opcode >> 8);
        *p_buf++ = (uint8_t)(length);
        *p_buf++ = (uint8_t)(length >> 8);
    }

    cybt_memcpy (p_buf, p_data, length);
    p_buf += length;

    pkt_crc = calc_packet_crc (p_buf_start, (uint16_t)(p_buf - p_buf_start));
    *p_buf++ = (uint8_t)(pkt_crc);
    *p_buf++ = (uint8_t)(pkt_crc >> 8);
}
BTSTACK_PORTING_SECTION_END

// Enable tracing for debugging only
#ifdef ENH_WICED_HCI_DEBUG
    extern void BTU_trace_error(const char *p_fmt, ...);
    #define ENH_RX_TRACE_DEBUG BTU_trace_error
#else
    #define ENH_RX_TRACE_DEBUG(...)
#endif

static cy_timer_t rx_timer;

static wiced_bool_t bRxTimedOut = WICED_FALSE;
static void rx_timer_callback(cy_timer_callback_arg_t arg)
{
    bRxTimedOut = WICED_TRUE;
    cy_rtos_set_semaphore(&cy_trans_uart.rx_complete, true);
}

BTSTACK_PORTING_SECTION_BEGIN
static void cybt_debug_enhanced_rx_task(void)
{
    cy_rslt_t result;
    volatile uint32_t numAvailable = 0;
    volatile size_t   expectedlength = 0;
    volatile uint32_t offset = 0;
    volatile uint32_t phase = WAIT_PREAMBLE_1;
    volatile uint32_t phase_len = 1;
    uint32_t          payload_len = 0;
    uint8_t           hdr_crc;
    uint16_t          pkt_crc;

    cy_rtos_init_timer(&rx_timer, CY_TIMER_TYPE_ONCE, rx_timer_callback, (cy_timer_callback_arg_t)NULL);

    while (1)
    {
        result = cy_rtos_get_semaphore(&cy_trans_uart.rx_complete, CY_RTOS_NEVER_TIMEOUT, false);

        if (bRxTimedOut)
        {
            ENH_RX_TRACE_DEBUG ("[%s] Rx TIMEOUT  phase: %d phase_len: %d  offset: %d", __FUNCTION__, phase, phase_len, offset);

            mtb_hal_uart_read_abort (&cy_trans_uart.CYBSP_DEBUG_UART_hal_obj);
            bRxTimedOut = WICED_FALSE;
            phase       = WAIT_PREAMBLE_1;
            rx_done     = false;
            wiced_rx_cmd[0] = 0;
        }

        if (phase == WAIT_PREAMBLE_1)
        {
            phase_len       = 1;
            offset          = 0;
        }

        if (result != CY_RSLT_SUCCESS)
            continue;

        numAvailable   = 0;
        expectedlength = phase_len;

        if (!rx_done)
        {
            if (CYHAL_UART_DMA_ENABLED == TRUE)
            {
				mtb_hal_uart_enable_event(&cy_trans_uart.CYBSP_DEBUG_UART_hal_obj, MTB_HAL_UART_IRQ_RX_DONE, true);
				mtb_hal_uart_read_async(&cy_trans_uart.CYBSP_DEBUG_UART_hal_obj, wiced_rx_cmd + offset, expectedlength);
                continue;
            }
            else
            {
                numAvailable = mtb_hal_uart_readable(&cy_trans_uart.CYBSP_DEBUG_UART_hal_obj);
                if (numAvailable >= expectedlength)
                {
			mtb_hal_uart_read(&cy_trans_uart.CYBSP_DEBUG_UART_hal_obj, wiced_rx_cmd + offset, (size_t *)&expectedlength);
                    numAvailable -= expectedlength;
                }
                else
                {
					mtb_hal_uart_enable_event(&cy_trans_uart.CYBSP_DEBUG_UART_hal_obj, MTB_HAL_UART_IRQ_RX_DONE, true);
					mtb_hal_uart_read_async(&cy_trans_uart.CYBSP_DEBUG_UART_hal_obj, wiced_rx_cmd + offset, expectedlength);
                    continue;
                }
            }
        }

        if (numAvailable == 0)
        {
            rx_done = false;
            mtb_hal_uart_enable_event(&cy_trans_uart.CYBSP_DEBUG_UART_hal_obj, MTB_HAL_UART_IRQ_RX_NOT_EMPTY, true);
        }

        switch (phase)
        {
        case WAIT_PREAMBLE_1:
            if (wiced_rx_cmd[0] == WICED_PREAMBLE_BYTE1)
            {
                phase = WAIT_PREAMBLE_2;
                offset = 1;
            }
            break;

        case WAIT_PREAMBLE_2:
            if (wiced_rx_cmd[1] == WICED_PREAMBLE_BYTE2)
            {
                phase  = WAIT_PREAMBLE_3;
                offset = 2;
            }
            else if (wiced_rx_cmd[1] == WICED_PREAMBLE_BYTE1)
            {
                offset = 1;
                ENH_RX_TRACE_DEBUG ("[%s] State WAIT_PREAMBLE_2 got WICED_PREAMBLE_BYTE1", __FUNCTION__);
            }
            else
            {
                phase  = WAIT_PREAMBLE_1;
                ENH_RX_TRACE_DEBUG ("[%s] State WAIT_PREAMBLE_2 got unexpected char: 0x%02x", __FUNCTION__, wiced_rx_cmd[1]);
            }
            break;

        case WAIT_PREAMBLE_3:
            if (wiced_rx_cmd[2] == WICED_PREAMBLE_BYTE3)
            {
                phase     = WAIT_OP_CODE_AND_LEN;
                offset    = 3;
                phase_len = 5;

                result = cy_rtos_start_timer(&rx_timer, 100);
            }
            else if (wiced_rx_cmd[2] == WICED_PREAMBLE_BYTE1)
            {
                ENH_RX_TRACE_DEBUG ("[%s] State WAIT_PREAMBLE_3 got WICED_PREAMBLE_BYTE1", __FUNCTION__);
                phase  = WAIT_PREAMBLE_2;
                offset = 1;
            }
            else
            {
                ENH_RX_TRACE_DEBUG ("[%s] State WAIT_PREAMBLE_3 got unexpected char: 0x%02x", __FUNCTION__, wiced_rx_cmd[1]);
                phase  = WAIT_PREAMBLE_1;
            }
            break;

        case WAIT_OP_CODE_AND_LEN:
            // Before opcode and len is a 1-byte CRC
            hdr_crc = wiced_rx_cmd[4] + wiced_rx_cmd[5] + wiced_rx_cmd[6] + wiced_rx_cmd[7];
            if (hdr_crc != wiced_rx_cmd[3])
            {
                ENH_RX_TRACE_DEBUG ("[%s] Bad hdr CRC Exp: 0x%02x  Got: 0x%02x", __FUNCTION__, hdr_crc, wiced_rx_cmd[3]);
                phase = WAIT_PREAMBLE_1;
            }
            else
            {
                payload_len = (wiced_rx_cmd[6] | (uint32_t)(wiced_rx_cmd[7]) << 8);
                if (payload_len > MAX_RX_DATA_LEN)
                {
                    ENH_RX_TRACE_DEBUG ("[%s] Invalid payload length: %d", __FUNCTION__, payload_len);
                    phase = WAIT_PREAMBLE_1;
                }
                else
                {
                    phase     = WAIT_PAYLOAD;
                    phase_len = payload_len + 2;
                    offset    = 8;
                }
            }
            break;

        case WAIT_PAYLOAD:
            cy_rtos_stop_timer (&rx_timer);

            phase_len -= expectedlength;
            offset    += expectedlength;

            if ( (phase_len == 0) && (p_app_rx_cmd_handler != NULL) )
            {
                // Verify the CRC
                pkt_crc = calc_packet_crc (wiced_rx_cmd, offset - 2);
                if ( (wiced_rx_cmd[offset - 2] == (uint8_t)pkt_crc)
                 &&  (wiced_rx_cmd[offset - 1] == (uint8_t)(pkt_crc >> 8)) )
                {
                    host_stack_mutex_lock(NULL);
                    p_app_rx_cmd_handler(&wiced_rx_cmd[4], payload_len + 4);
                    host_stack_mutex_unlock(NULL);
                }
                else
                {
                    ENH_RX_TRACE_DEBUG ("[%s] Bad packet CRC Exp: 0x%04x  Got: 0x%04x", __FUNCTION__,
                        pkt_crc, wiced_rx_cmd[offset - 1] | wiced_rx_cmd[offset - 1] << 8);
                }

                phase = WAIT_PREAMBLE_1;
            }
            break;
        }

        // Re-enter the loop if data is available
        if (numAvailable != 0)
        {
            cy_rtos_set_semaphore(&cy_trans_uart.rx_complete, true);
        }
    }
}
BTSTACK_PORTING_SECTION_END

#endif  // ENHANCED_WICED_HCI
#endif //!defined (CY_USING_HAL)
