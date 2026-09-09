# Application time, events and triggers

`WhiteRabbitCore.time` exposes a common record in the native WR reference
clock domain (`wr`, 62.5 MHz): `seconds` (40-bit TAI seconds), `cycles` (28-bit
ticks, 16 ns each), `time_valid`, `link_up` and `state`. It uses the WR core's
timecode directly; it does not introduce another free-running counter. The
existing `TimeGenerator` remains available for applications that interpolate
time in another clock domain.

By default, time is usable only when the core marks it valid and synchronized
link status is high. State 0 means invalid; state 1 means valid with link.
`allow_time_holdover=True` permits state 2 while the core still marks time
valid after link loss. This opt-in policy does not guarantee oscillator
holdover accuracy. These states do not independently certify servo lock.

## Coherent host snapshots

```sh
python3 -m litex_wr_nic.wr --csr-csv test/csr.csv read-time
```

The `wr_time_capture` write requests a snapshot in `wr`. `wr_time_busy` is
set until the complete record reaches `sys`; `wr_time_done` then becomes 1.
All fields remain stable until the next completed capture. Repeated capture
writes while busy are ignored. This prevents torn seconds/cycles reads at a
second boundary. Reset of the time domain invalidates the snapshot. A stopped
reference clock causes the host command to time out instead of claiming a
fresh timestamp.

## Timestamping events

```python
from litex_wr_nic.gateware.wr_time import WREventTimestamp

self.events = WREventTimestamp(self.wr_core.time, depth=16)
self.comb += [
    self.events.event.eq(event_pulse_wr),
    self.events.tag.eq(event_tag_wr),
    self.events.source.connect(application_timestamp_sink),
]
```

`event` and `tag` must be synchronous to `wr`. The output is a `sys` stream
containing the time record and an 8-bit tag, with `first=last=1` per event.
Invalid time is delivered with `time_valid=0`. A full FIFO drops the event
and increments `dropped` in `wr`; the corresponding CSR is transferred
coherently to `sys`. The output supports backpressure. `cd_out` and tag width
are configurable. An asynchronous input requires a synchronizer or capture
circuit; its latency and uncertainty are part of the timestamp error budget.

## Scheduling a trigger

```python
from litex_wr_nic.gateware.wr_time import WRTimeTrigger

self.trigger = WRTimeTrigger(self.wr_core.time)
# Feed trigger.sink with a future {seconds, cycles} in the wr domain.
# If commands originate in sys, use a stream.ClockDomainCrossing first.
self.comb += scheduled_commands_wr.connect(self.trigger.sink)
```

One command can be pending. `pulse` is asserted for the matching native WR
tick. A past target, invalid cycle count or forward step past the target
increments `missed`; invalid time or a backward step cancels the pending
command and increments `cancelled`. No late pulse is emitted. Commands are
accepted only when `sink.ready` is high. The counters and pulse are in `wr`.

Realtime outputs require an active reference clock. Link-state propagation
takes synchronizer cycles; a stopped clock can also stretch a tick. Board
output logic must apply its clock-health and output-enable policy. Pin-level
delays, pulse shaping and sub-tick interpolation belong to that logic; this
interface does not claim calibrated PPS/trigger accuracy.

Simulations cover second-boundary snapshots, reset invalidation, link/holdover
policy, event FIFO backpressure/overflow and trigger cancellation. Qualification
of physical timing and servo recovery requires a WR reference peer.
