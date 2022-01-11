import posix_ipc

reply_queue = posix_ipc.MessageQueue("/reply_ff")
cmd_queue = posix_ipc.MessageQueue("/cmd_ff")

cmd_queue.send(b"\x01\x10\x00\x01\x03\xeb")
reply, _ = reply_queue.receive()
print(reply)
