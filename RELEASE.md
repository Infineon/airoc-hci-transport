# Airoc-hci-transport
The airoc-hci-transport is a utility library, offering a comprehensive solution for extracting debug traces and facilitating communication with external host application.

### What's Included?
The release of airoc-hci-transport library includes the following:
- Bluetooth protocol (HCI) traces over [BTSPY](https://github.com/Infineon/btsdk-utils) tool
- printf() support over a BTSpy tool
- Facilitating communication with external host application
- Support for GCC, IAR, and ARM toolchains

### What Changed?

#### v1.3.0
* TX transmissions are gated by CTS; data is dropped if the host is not ready, preventing deep sleep blocking.
* Added Host Wake and Device Wake support for non-PSE8XXGP platform.

#### v1.2.1
* Updated README.md to point that airoc-hci-transport asset can't be used along with [retarget-io](https://github.com/Infineon/retarget-io) [release-v1.8.0](https://github.com/Infineon/retarget-io/tree/release-v1.8.0) and later as _write function in retarget-io was changed from weak to strong symbol.

#### v1.2.0
* Added Hardware Abstraction Layer support for the PSE8XXGP family of chips.
* Added new APIs cybt_debug_uart_is_tx_active() and cybt_debug_uart_is_rx_active() to check UART state.

#### v1.1.0
* Changes to enable application to override default values of stack size of TX and RX tasks and queue count of TX task as per its need.

#### v1.0.0
* Initial release of airoc-hci-transport.

 © Infineon Technologies, 2024.
