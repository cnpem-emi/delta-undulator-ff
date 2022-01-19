# mq-serial
POSIX IPC Message Queue/Serial Communication middleman

## Building
`make`

## Implementation details
The queue `cmd_ff` receives commands, `reply_ff` is populated by command replies. Priority `0` is reserved for `eth-bridge`, while priority `1` (higher priority) is reserved for `feed-forward`.

Priority `0` returns a full response, priority `1` returns a single byte with status:
* `\xff`: OK
* `\xee`: Reading failure
* `\x00`: No reading

## Maximum performance

It is recommended to set the latency timer of `ttyUSB0` to 0, [as the default value incurs 16ms of delay](https://granitedevices.com/wiki/FTDI_Linux_USB_latency). In order to do that, run `echo 0 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer`. Other ways of reducing latency include setting an event character (safe) or increasing the kernel tick timer, which may result in severe system instability.

Currently, when returning only a byte determining the status of the operation, the maximum round trip time (from message queue'd to reply received and parsed) is roughly 200us, while it doubles to 400us when receiving the full serial reply.
`
