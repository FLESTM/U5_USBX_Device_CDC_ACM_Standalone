# U5_USBX_Device_CDC_ACM_Standalone
Code example USBX device CDC ACM standalone generated with CubeMX

CDC_ACM_Read_Task: used to Read the received data from Virtual COM Port.
CDC_ACM_Write_Task: used to send the received data over UART.
During enumeration phase, three communication pipes "endpoints" are declared in the CDC class implementation :

1 x Bulk IN endpoint for receiving data from STM32 device to PC host: When data are received over UART they are saved in the buffer "UserTxBufferFS". Periodically, in a usbx_cdc_acm_write_thread_entry the state of the buffer "UserTxBufferFS" is checked. If there are available data, they are transmitted in response to IN token otherwise it is NAKed.

1 x Bulk OUT endpoint for transmitting data from PC host to STM32 device: When data are received through this endpoint they are saved in the buffer "UserRxBufferFS" then they are transmitted over UART using DMA mode and in meanwhile the OUT endpoint is NAKed. Once the transmission is over, the OUT endpoint is prepared to receive next packet in HAL_UART_RxCpltCallback().

1 x Interrupt IN endpoint for setting and getting serial-port parameters: When control setup is received, the corresponding request is executed in ux_app_parameters_change().

In this application, two requests are implemented:

- Set line: Set the bit rate, number of Stop bits, parity, and number of data bits
- Get line: Get the bit rate, number of Stop bits, parity, and number of data bits
The other requests (send break, control line state) are not implemented.

Notes
Receiving data over UART is handled by interrupt while transmitting is handled by DMA allowing hence the application to receive data at the same time it is transmitting another data (full- duplex feature).
The user has to check the list of the COM ports in Device Manager to find out the COM port number that have been assigned (by OS) to the VCP interface.

Expected success behavior
When plugged to PC host, the STM32U575ZI must be properly enumerated as an USB Serial device and an STlink Com port. During the enumeration phase, the device must provide host with the requested descriptors (Device descriptor, configuration descriptor, string descriptors). Those descriptors are used by host driver to identify the device capabilities. Once STM32U575ZI USB device successfully completed the enumeration phase, Open two hyperterminals (USB com port and UART com port(USB STLink VCP)) to send/receive data to/from host from/to device.

Error behaviors
Host PC shows that USB device does not operate as designed (CDC Device enumeration failed, PC and Device can not communicate over VCP ports).

STM32U575ZI Set-up

Connect the STM32U575ZI board CN15 to the PC through "MICRO-USB" to "Standard A" cable.

For VCP the configuration is dynamic for example it can be :

BaudRate = 115200 baud
Word Length = 8 Bits
Stop Bit = 1
Parity = None
Flow control = None

By default the USART1 communication between the target MCU and ST-LINK MCU is enabled. It's configuration is as following:

BaudRate = 115200 baud
Word Length = 8 Bits
Stop Bit = 1
Parity = None
Flow control = None


