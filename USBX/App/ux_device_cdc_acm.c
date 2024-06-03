/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ux_device_cdc_acm.c
  * @author  MCD Application Team
  * @brief   USBX Device applicative file
  ******************************************************************************
    * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "ux_device_cdc_acm.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "ux_api.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define APP_RX_DATA_SIZE   2048
#define APP_TX_DATA_SIZE   2048

/* Rx/TX flag */
#define RX_NEW_RECEIVED_DATA      0x01
#define TX_NEW_TRANSMITTED_DATA   0x02

/* Data length for vcp */
#define VCP_WORDLENGTH8  8
#define VCP_WORDLENGTH9  9

/* the minimum baudrate */
#define MIN_BAUDRATE     9600

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

UX_SLAVE_CLASS_CDC_ACM  *cdc_acm;

/* Data to send over USB CDC are stored in this buffer   */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

uint32_t UserTxBufPtrIn;
uint32_t UserTxBufPtrOut;

UART_HandleTypeDef *uart_handler;
//extern TX_EVENT_FLAGS_GROUP EventFlag;

UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_PARAMETER CDC_VCP_LineCoding =
{
  115200, /* baud rate */
  0x00,   /* stop bits-1 */
  0x00,   /* parity - none */
  0x08    /* nb. of bits 8 */
};

#define APP_CDC_ACM_READ_STATE_TX_START  (UX_STATE_APP_STEP + 0)
#define APP_CDC_ACM_READ_STATE_TX_WAIT   (UX_STATE_APP_STEP + 1)

static UINT write_state = UX_STATE_RESET;
static UINT read_state = UX_STATE_RESET;

volatile ULONG EventFlag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
extern VOID USBX_APP_UART_Init(UART_HandleTypeDef **huart);
static void USBD_CDC_VCP_Config(UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_PARAMETER *);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  USBD_CDC_ACM_Activate
  *         This function is called when insertion of a CDC ACM device.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_Activate(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_Activate */

  /* Save the CDC instance */
  cdc_acm = (UX_SLAVE_CLASS_CDC_ACM*) cdc_acm_instance;

  /* Configure the UART peripheral */
  USBX_APP_UART_Init(&uart_handler);

  /* Get default UART parameters */
  CDC_VCP_LineCoding.ux_slave_class_cdc_acm_parameter_baudrate = uart_handler->Init.BaudRate;

  /* Set the UART data type : only 8bits and 9bits are supported */
  switch (uart_handler->Init.WordLength)
  {
    case UART_WORDLENGTH_8B:
    {
      /* Set UART data bit to 8 */
      CDC_VCP_LineCoding.ux_slave_class_cdc_acm_parameter_data_bit = VCP_WORDLENGTH8;
      break;
    }

    case UART_WORDLENGTH_9B:
    {
      /* Set UART data bit to 9 */
      CDC_VCP_LineCoding.ux_slave_class_cdc_acm_parameter_data_bit = VCP_WORDLENGTH9;
      break;
    }

    default :
    {
      /* By default set UART data bit to 8 */
      CDC_VCP_LineCoding.ux_slave_class_cdc_acm_parameter_data_bit = VCP_WORDLENGTH8;
      break;
    }
  }

  /* Get UART Parity */
  CDC_VCP_LineCoding.ux_slave_class_cdc_acm_parameter_parity = uart_handler->Init.Parity;

  /* Get UART StopBits */
  CDC_VCP_LineCoding.ux_slave_class_cdc_acm_parameter_stop_bit = uart_handler->Init.StopBits;

  /* Set device class_cdc_acm with default parameters */
  if (ux_device_class_cdc_acm_ioctl(cdc_acm, UX_SLAVE_CLASS_CDC_ACM_IOCTL_SET_LINE_CODING,
                                    &CDC_VCP_LineCoding) != UX_SUCCESS)
  {
    Error_Handler();
  }

  /* Receive an amount of data in interrupt mode */
  if (HAL_UART_Receive_IT(uart_handler, (uint8_t *)UserTxBufferFS, 1) != HAL_OK)
  {
    /* Transfer error in reception process */
    Error_Handler();
  }

  /* USER CODE END USBD_CDC_ACM_Activate */

  return;
}

/**
  * @brief  USBD_CDC_ACM_Deactivate
  *         This function is called when extraction of a CDC ACM device.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_Deactivate(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_Deactivate */
  UX_PARAMETER_NOT_USED(cdc_acm_instance);

  /* Reset the cdc acm instance */
  cdc_acm = UX_NULL;

  /* DeInitialize the UART peripheral */
  HAL_UART_DeInit(uart_handler);

  /* USER CODE END USBD_CDC_ACM_Deactivate */

  return;
}

/**
  * @brief  USBD_CDC_ACM_ParameterChange
  *         This function is invoked to manage the CDC ACM class requests.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_ParameterChange(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_ParameterChange */
  UX_PARAMETER_NOT_USED(cdc_acm_instance);

  ULONG request;
  UX_SLAVE_TRANSFER *transfer_request;
  UX_SLAVE_DEVICE *device;

  /* Get the pointer to the device.  */
  device = &_ux_system_slave -> ux_system_slave_device;

  /* Get the pointer to the transfer request associated with the control endpoint. */
  transfer_request = &device -> ux_slave_device_control_endpoint.ux_slave_endpoint_transfer_request;

  request = *(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_REQUEST);

  switch (request)
  {
    case UX_SLAVE_CLASS_CDC_ACM_SET_LINE_CODING :

      /* Get the Line Coding parameters */
      if (ux_device_class_cdc_acm_ioctl(cdc_acm, UX_SLAVE_CLASS_CDC_ACM_IOCTL_GET_LINE_CODING,
                                        &CDC_VCP_LineCoding) != UX_SUCCESS)
      {
        Error_Handler();
      }

      /* Check if baudrate < 9600) then set it to 9600 */
      if (CDC_VCP_LineCoding.ux_slave_class_cdc_acm_parameter_baudrate < MIN_BAUDRATE)
      {
        CDC_VCP_LineCoding.ux_slave_class_cdc_acm_parameter_baudrate = MIN_BAUDRATE;

        /* Set the new configuration of ComPort */
        USBD_CDC_VCP_Config(&CDC_VCP_LineCoding);
      }
      else
      {
        /* Set the new configuration of ComPort */
        USBD_CDC_VCP_Config(&CDC_VCP_LineCoding);
      }

      break;

    case UX_SLAVE_CLASS_CDC_ACM_GET_LINE_CODING :

      /* Set the Line Coding parameters */
      if (ux_device_class_cdc_acm_ioctl(cdc_acm, UX_SLAVE_CLASS_CDC_ACM_IOCTL_SET_LINE_CODING,
                                        &CDC_VCP_LineCoding) != UX_SUCCESS)
      {
        Error_Handler();
      }

      break;

    case UX_SLAVE_CLASS_CDC_ACM_SET_CONTROL_LINE_STATE :
    default :
      break;
  }

  /* USER CODE END USBD_CDC_ACM_ParameterChange */

  return;
}

/* USER CODE BEGIN 1 */

/**
  * @brief  Function implementing usbx_cdc_acm_thread_entry.
  * @param  thread_input: Not used
  * @retval none
  */
VOID CDC_ACM_Read_Task(VOID)
{
  UX_SLAVE_DEVICE *device;
  UX_SLAVE_INTERFACE *data_interface;
  UX_SLAVE_CLASS_CDC_ACM *cdc_acm;
  UINT  status;
  ULONG read_length;
  static ULONG actual_length;

  /* Get device */
  device = &_ux_system_slave->ux_system_slave_device;

  /* Check if device is configured */
  if (device->ux_slave_device_state != UX_DEVICE_CONFIGURED)
  {
    read_state = UX_STATE_RESET;
    return;
  }

  /* Get Data interface (interface 1) */
  data_interface = device->ux_slave_device_first_interface->ux_slave_interface_next_interface;
  cdc_acm =  data_interface->ux_slave_interface_class_instance;
  read_length = (_ux_system_slave->ux_system_slave_speed == UX_HIGH_SPEED_DEVICE) ? 512 : 64;

  /* Run state machine.  */
  switch(read_state)
  {
    case UX_STATE_RESET:
      read_state = UX_STATE_WAIT;
      /* Fall through.  */
    case UX_STATE_WAIT:
      status = ux_device_class_cdc_acm_read_run(cdc_acm,
                                                (UCHAR *)UserRxBufferFS, read_length,
                                                &actual_length);
      /* Error.  */
      if (status <= UX_STATE_ERROR)
      {
        /* Reset state.  */
        read_state = UX_STATE_RESET;
        return;
      }
      if (status == UX_STATE_NEXT)
      {
        if (actual_length != 0)
        {
          read_state = APP_CDC_ACM_READ_STATE_TX_START;
        }
        else
        {
          read_state = UX_STATE_RESET;
        }
        return;
      }
      /* Wait.  */
      return;
    case APP_CDC_ACM_READ_STATE_TX_START:
      /* Send the data via UART */
      status = HAL_UART_Transmit_DMA(uart_handler, (uint8_t *)UserRxBufferFS,
                                      actual_length);
      if (status == HAL_BUSY)
      {
        /* Keep trying.  */
        return;
      }
      if (status != HAL_OK)
      {
        /* Transfer error in reception process */
        Error_Handler();
      }
      /* DMA started.  */
      read_state = APP_CDC_ACM_READ_STATE_TX_WAIT;
      /* Fall through.  */
    case APP_CDC_ACM_READ_STATE_TX_WAIT:
      if (EventFlag & TX_NEW_TRANSMITTED_DATA)
      {
        EventFlag &= ~TX_NEW_TRANSMITTED_DATA;
      }
      read_state = UX_STATE_WAIT;
      return;
    default:
      return;
  }
}



/**
  * @brief  Function implementing usbx_cdc_acm_write_thread_entry.
  * @param  thread_input: Not used
  * @retval none
  */
VOID CDC_ACM_Write_Task(VOID)
{
  UX_SLAVE_DEVICE    *device;
  UX_SLAVE_INTERFACE *data_interface;
  UX_SLAVE_CLASS_CDC_ACM *cdc_acm;
  ULONG actual_length;
  ULONG buffptr;
  ULONG buffsize;
  UINT ux_status = UX_SUCCESS;

  /* Get device */
  device = &_ux_system_slave->ux_system_slave_device;

  /* Check if device is configured */
  if (device->ux_slave_device_state != UX_DEVICE_CONFIGURED)
  {
    read_state = UX_STATE_RESET;
    return;
  }

  /* Get Data interface */
  data_interface = device->ux_slave_device_first_interface->ux_slave_interface_next_interface;
  cdc_acm =  data_interface->ux_slave_interface_class_instance;

  switch(write_state)
  {
    case UX_STATE_RESET:
      if (EventFlag & RX_NEW_RECEIVED_DATA)
      {
        EventFlag &= ~RX_NEW_RECEIVED_DATA;
        /* Check if there is a new data to send */
        if (UserTxBufPtrOut != UserTxBufPtrIn)
        {
          /* Check buffer overflow and Rollback */
          if (UserTxBufPtrOut > UserTxBufPtrIn)
          {
            buffsize = APP_RX_DATA_SIZE - UserTxBufPtrOut;
          }
          else
          {
            /* Calculate data size */
            buffsize = UserTxBufPtrIn - UserTxBufPtrOut;
          }

          /* Copy UserTxBufPtrOut in buffptr */
          buffptr = UserTxBufPtrOut;

          /* Send data over the class cdc_acm_write */
          ux_status = ux_device_class_cdc_acm_write_run(cdc_acm,
                                                        (UCHAR *)(&UserTxBufferFS[buffptr]),
                                                        buffsize, &actual_length);
          if (ux_status != UX_STATE_WAIT)
          {
            /* Reset state.  */
            read_state = UX_STATE_RESET;
            return;
          }
          write_state = UX_STATE_WAIT;
          return;
        }
        return;
      }
      return;

    case UX_STATE_WAIT:
      /* Continue to run state machine.  */
      ux_status = ux_device_class_cdc_acm_write_run(cdc_acm, UX_NULL, 0, &actual_length);
      /* Check if there is  fatal error.  */
      if (ux_status < UX_STATE_IDLE)
      {
        /* Reset state.  */
        read_state = UX_STATE_RESET;
        return;
      }
      /* Check if dataset is transmitted */
      if (ux_status <= UX_STATE_NEXT)
      {
        /* Increment the UserTxBufPtrOut pointer */
        UserTxBufPtrOut += actual_length;

        /* Rollback UserTxBufPtrOut if it equal to APP_TX_DATA_SIZE */
        if (UserTxBufPtrOut == APP_TX_DATA_SIZE)
        {
          UserTxBufPtrOut = 0;
        }
        write_state = UX_STATE_RESET;
      }
      /* Keep waiting.  */
      return;
    default:
      return;
  }
}

/**
  * @brief  Tx Transfer completed callback.
  * @param  huart UART handle.
  * @retval none
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	  EventFlag |= TX_NEW_TRANSMITTED_DATA;
}

/**
  * @brief  Rx Transfer completed callback
  * @param  huart: UART handle
  * @retval none
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	  EventFlag |= RX_NEW_RECEIVED_DATA;

	    /* Increment the UserTxBufPtrIn pointer */
	  UserTxBufPtrIn++;

	  /* Rollback the UserTxBufPtrIn if it equal to APP_TX_DATA_SIZE */
	  if (UserTxBufPtrIn == APP_TX_DATA_SIZE)
	  {
	    UserTxBufPtrIn = 0;
	  }

	  /* Start another reception: provide the buffer pointer with offset and the buffer size */
	  if (HAL_UART_Receive_IT(uart_handler, (uint8_t *)UserTxBufferFS + UserTxBufPtrIn, 1) != HAL_OK)
	  {
	    /* Transfer error in reception process */
	    Error_Handler();
	  }
}

/**
  * @brief  UART error callbacks
            Transfer error occurred in reception and/or transmission process.
  * @param  UartHandle: UART handle
  * @retval none
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
  Error_Handler();
}

/**
  * @brief  USBD_CDC_VCP_Config
            Configure the COM Port with the parameters received from host.
  * @param  UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_PARAMETER: linecoding struct.
  * @param  CDC_VCP_LineCoding: CDC VCP line coding.
  * @retval none
  */
static VOID USBD_CDC_VCP_Config(UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_PARAMETER
                                *CDC_VCP_LineCoding)
{
  /* Deinitialization UART */
  if (HAL_UART_DeInit(uart_handler) != HAL_OK)
  {
    /* Deinitialization Error */
    Error_Handler();
  }

  /* Check stop bit parameter */
  switch (CDC_VCP_LineCoding->ux_slave_class_cdc_acm_parameter_stop_bit)
  {
    case 0:

      /* Set the UART Stop bit to 1 */
      uart_handler->Init.StopBits = UART_STOPBITS_1;

      break;

    case 2:

      /* Set the UART Stop bit to 2 */
      uart_handler->Init.StopBits = UART_STOPBITS_2;

      break;

    default :

      /* By default set the UART Stop bit to 1 */
      uart_handler->Init.StopBits = UART_STOPBITS_1;

      break;
  }

  /* Check parity parameter */
  switch (CDC_VCP_LineCoding->ux_slave_class_cdc_acm_parameter_parity)
  {
    case 0:

      /* Set the UART parity bit to none */
      uart_handler->Init.Parity = UART_PARITY_NONE;

      break;

    case 1:

      /* Set the UART parity bit to ODD */
      uart_handler->Init.Parity = UART_PARITY_ODD;

      break;

    case 2:

      /* Set the UART parity bit to even */
      uart_handler->Init.Parity = UART_PARITY_EVEN;

      break;

    default :

      /* By default set the UART parity bit to none */
      uart_handler->Init.Parity = UART_PARITY_NONE;

      break;
  }

  /* Set the UART data type : only 8bits and 9bits is supported */
  switch (CDC_VCP_LineCoding->ux_slave_class_cdc_acm_parameter_data_bit)
  {
    case 0x07:

      /* With this configuration a parity (Even or Odd) must be set */
      uart_handler->Init.WordLength = UART_WORDLENGTH_8B;

      break;

    case 0x08:

      if (uart_handler->Init.Parity == UART_PARITY_NONE)
      {
        uart_handler->Init.WordLength = UART_WORDLENGTH_8B;
      }
      else
      {
        uart_handler->Init.WordLength = UART_WORDLENGTH_9B;
      }

      break;

    default :

      uart_handler->Init.WordLength = UART_WORDLENGTH_8B;

      break;
  }

  /* Get the UART baudrate from vcp */
  uart_handler->Init.BaudRate = CDC_VCP_LineCoding->ux_slave_class_cdc_acm_parameter_baudrate;

  /* Set the UART Hw flow control to none */
  uart_handler->Init.HwFlowCtl = UART_HWCONTROL_NONE;

  /* Set the UART mode */
  uart_handler->Init.Mode = UART_MODE_TX_RX;

  /* Set the UART sampling */
  uart_handler->Init.OverSampling = UART_OVERSAMPLING_16;

  /* Initialization UART */
  if (HAL_UART_Init(uart_handler) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Start reception: provide the buffer pointer with offset and the buffer size */
  HAL_UART_Receive_IT(uart_handler, (uint8_t *)(UserTxBufferFS + UserTxBufPtrIn), 1);
}

/* USER CODE END 1 */
