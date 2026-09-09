# WR application fabric

`WhiteRabbitCore.sink` and `.source` are byte-wide LiteX stream endpoints in
`sys`, with `first`, `last` and `error`. Data transfers on `valid & ready`.
The producer must hold all fields while stalled. An error is reported on the
last byte; a consumer must discard the complete packet when that flag is set.
Applications that cannot undo partial packet processing should buffer a whole
packet before committing it.

The adapters connect these streams to the WR fabric in `wr_sys`. WR fabric is
pipelined Wishbone: `STB & ~STALL` accepts a request, while `ACK`, `ERR` or `RTY`
completes a previously accepted request. The transmit adapter keeps `CYC`
asserted until every response has returned. It supports consecutive accepted
words and up to 16 outstanding requests by default. An error or retry aborts
the Wishbone cycle and drains the rest of the application packet. The slave
must discard aborted requests when `CYC` falls; responses must be low before
the next packet begins. There is no automatic retry or response timeout.

The receive adapter stalls before accepting data it cannot buffer. It queues
an explicit end-of-frame token, including when `CYC` falls with a full FIFO.
One word of lookahead preserves trailing error status and single-byte final
words. OOB metadata is consumed without being exposed as packet payload.
Both byte-select encodings for a single byte are supported. Empty frames
produce no application packet.

The FIFO, partial-word converter and protocol state machines share a reset
derived from both clock domains, with asynchronous assertion and locally
synchronized release. Resetting either side discards buffered packets; users
must also reset their downstream packet assembly on a WR reset.

## Diagnostics

Each adapter exposes coherent, read-only 32-bit counters in `sys`:

| CSR suffix | Meaning |
| --- | --- |
| `packets` | Completed or aborted fabric cycles |
| `errors` | Packets with an input error, malformed receive request, or bus error/retry |
| `stalls` | Fabric clock cycles with a request waiting on `STALL` |
| `overflow` | Receive requests changed or withdrawn while stalled |

Counters wrap and reset with the adapter. Receive overflow marks the affected
packet as erroneous; a master that respects `STALL` does not overflow the
adapter. The transmit overflow counter remains zero because its stream input
supports backpressure. These counters cover the adapters, not losses inside
the WR endpoint or a downstream application.

The NIC PHY preserves the error flag into LiteEth. A LiteScope capture of
the WR bus and protocol state machines must use `wr_sys`; the application
stream uses `sys` and requires a separate capture or an explicit CDC probe.

## Validation

`pytest -q test/test_wr_fabric.py` exercises asynchronous loopback with random
backpressure, odd/even lengths, delayed pipelined responses, trailing status,
bus errors/retries, a reset during a partial packet, and a master violating
`STALL`. Physical packet throughput and WR link behavior require a connected
WR peer and are separate from CPU/console smoke tests.
