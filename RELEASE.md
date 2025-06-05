# Airoc-hci-transport
The airoc-hci-transport is a utility library, offering a comprehensive solution for extracting debug traces and facilitating communication with external host application.

### What's Included?
The release of airoc-hci-transport library includes the following:
- Bluetooth protocol (HCI) traces over [BTSPY](https://github.com/Infineon/btsdk-utils) tool
- printf() support over a BTSpy tool
- Facilitating communication with external host application
- Support for GCC, IAR, and ARM toolchains

### What Changed?

#### v1.2.0
* Added Hardware Abstraction Layer support for the PSE8XXGP family of chips.
* Added new APIs cybt_debug_uart_is_tx_active() and cybt_debug_uart_is_rx_active() to check UART state

#### v1.1.0
* Changes to enable application to override default values of stack size of TX and RX tasks and queue count of TX task as per its need

#### v1.0.0
* Initial release of airoc-hci-transport

 © Infineon Technologies, 2024.