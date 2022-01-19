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

### Performance details
Benchmarked by sending 100k messages, using two CPUs on a Beaglebone AI. Single core performance is equiparable (with a very load-light core).

#### Semi Write Only
(Response is considered but not transmitted in full to sender, only the status)

|Run      |Time  |
|---------|------|
|1        |260 us|
|2        |259 us|
|3        |260 us|
|4        |260 us|

Average roundtrip time: 260 us

#### Read/Write
(Response transmitted in full back to sender. Any message with an invalid checksum is considered an error)

|Run      |Time  |Comm. Errors|
|---------|------|------------|
|1        |544 us|0.06%       |
|2        |574 us|0.12%       |
|3        |570 us|0.10%       |
|4        |581 us|0.08%       |

Average roundtrip time: 567 us
Average comm. error percentage: 0.09%

Obs.: this is near (but not equal to) full speed. A single `printf` was added to every iteration to slightly throttle the program in order to increase stability.
