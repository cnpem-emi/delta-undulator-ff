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
