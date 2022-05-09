# Delta Undulator - Feed Forward

Software/PRU firmware made for interpreting IOC commands, performing corrections, updating lookup tables and more

## Building

Make sure the `libevent-dev` package is installed and run `make redis` if hiredis isn't installed

```
make
```

## Running

Just run `./interfaceFF`

### Errors

- #5: Redis array has a member count that is not a multiple of 5

## Notes

Runs better on a PREEMPT patched kernel, with CPU 1 isolated and IRQ balancing disabled.
