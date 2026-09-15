# Batch 2 — Drivers

The four modules that touch hardware directly. Each owns one peripheral and hides
everything about it: nowhere else in the firmware knows a shunt value, an SPI frame
format, which timer channel drives which phase, or that the receive path is a ring
buffer.

| File | Lines | Code | Verdict |
|---|---|---|---|
| `drivers/encoder.{h,c}` | 728 | 232 | logic exhaustively correct; **two serious context bugs** |
| `drivers/gate_driver.{h,c}` | 544 | 152 | correct, including the dead-time sign; one wrong comment |
| `drivers/sensors.{h,c}` | 426 | 106 | correct; one fragile contract, one missing `volatile` |
| `drivers/USB_Comm.{h,c}` | 370 | 98 | ring buffer reasoning is sound; dead code left in comments |

The headline for this batch is not any single line. It is **execution context** — which
code runs inside the 32 kHz ADC interrupt, and what is safe there. Two of the findings
below are only visible by tracing call graphs across four files, which is exactly the
kind of thing the per-file comments cannot say.

---

## The context rule this batch turns on

Established from the configuration, not assumed:

| Interrupt | Priority | Source |
|---|---|---|
| `ADC_IRQn` (the control loop) | **0** — highest | `stm32f4xx_hal_msp.c:138` |
| SysTick (`HAL_GetTick`) | **15** — lowest | `stm32f4xx_hal_conf.h:151` |

SysTick cannot preempt the ADC interrupt. **`HAL_GetTick()` is frozen for the whole
duration of `HAL_ADCEx_InjectedConvCpltCallback`.** Anything inside that callback which
waits on a tick-based timeout waits forever, and because the ADC interrupt is priority
0, a hang there means nothing else on the processor ever runs again.

`encoder.c` knows this. It says so at L256–260, as the justification for
`exchange_frame_direct` using a spin count instead:

> A count is used rather than a millisecond timeout because this runs inside the
> control interrupt, and `HAL_GetTick()` cannot advance there […] A tick-based timeout
> would therefore never expire, turning any stall into a permanent hang.

That reasoning is correct. Findings E2.1 and E2.2 are both cases of code reaching the
same interrupt by a different path, where the rule was not applied.

---

## `App/drivers/encoder.{h,c}`

### Purpose

Driver for the AS5147 magnetic rotary sensor (U8) on SPI1, and the owner of
`encoder_t` — the raw count, the calibrated zero offset, the direction flag, and the
mechanical angle derived from all three.

### Place in the architecture

Includes only `<stdint.h>` and `main.h`. Runs in **both** contexts, which is the whole
problem:

```
ISR   (ADC, prio 0, 32 kHz)  loop.c → motor_update → encoder_capture
                                    → encoder_read_angle_pipelined
                                    → exchange_frame_direct     [spin timeout  — right]

ISR   (same callback)        loop.c → telemetry_capture → encoder_read_angle
                                    → encoder_read_register
                                    → exchange_frame  ×2        [tick timeout  — WRONG]

main                         commands.c → encoder_read_angle / encoder_read_diagnostics
                                    → exchange_frame  ×2        [tick timeout  — fine here]

main                         main.c → encoder_init, motor.c / loop.c → encoder_prime_pipeline
```

### The frame format, and why the pipeline exists

Every exchange is 16 bits: bit 15 parity, bit 14 read/error, bits 13:0 payload. The
sensor answers a command **in the following frame**, not the current one. So a naive
register read costs two chip-select cycles — one to post the request, one to clock the
answer out.

`encoder_read_angle_pipelined` exploits that: it sends the *next* ANGLECOM request and
receives the *previous* one's answer in a single exchange, giving one useful answer per
frame instead of one per two. That halves a 32 kHz cost from ~7 µs to ~3.5 µs, in a
31.25 µs period. `encoder_prime_pipeline` exists solely to discard the first,
meaningless reply so the steady state is aligned.

### Line by line — the parts that carry weight

**L44–71 `apply_even_parity`** — parity by folding, not counting:

```c
uint16_t folded = frame & 0x7FFFu;
folded ^= (uint16_t)(folded >> 8);
folded ^= (uint16_t)(folded >> 4);
folded ^= (uint16_t)(folded >> 2);
folded ^= (uint16_t)(folded >> 1);
```

Each xor combines pairs of bits, halving the span: after the first, bit *n* holds the
parity of itself and bit *n+8*; after the fourth, bit 0 holds the parity of all fifteen.
Four operations against fifteen iterations of shift-mask-add-compare — the comment
notes the loop it replaced cost around two microseconds in an unoptimised build, inside
an interrupt with thirty-one to spend in total.

The `& 0x7FFFu` is load-bearing: bit 15 is the bit being *decided*, so it must not
count towards its own value.

**L77–87 `check_even_parity`** — the same fold over all sixteen bits, because here the
parity bit is part of what is being checked.

**Verified exhaustively.** Over all 32,768 payloads, `apply_even_parity` leaves the
payload untouched and produces an even total bit count every time. Over all 65,536
frames, `check_even_parity` agrees with an independent bit count. Every framed payload
passes its own check, and all 14,176 tested single-bit flips are rejected.

**L103–151 `exchange_frame`** — the HAL path. Chip select low, `HAL_SPI_TransmitReceive`
with `Size = 1`, chip select high, then forty `__NOP()`s.

The `Size` parameter counts **data frames, not bytes** — SPI1 is configured for 16-bit
frames, so `1` means one 16-bit word, and passing `2` would read past the end of two
local `uint16_t`s. The `(uint8_t *)` casts are what HAL's prototype demands; the
peripheral still moves 16 bits because that is how the data size was configured.

The NOPs enforce the sensor's 350 ns minimum chip-select-high time. At 96 MHz, forty
NOPs plus loop overhead is comfortably over 400 ns. `__NOP()` is a volatile intrinsic,
so the compiler cannot delete them.

**L261 `SPI_SPIN_LIMIT 2000`** and **L274–278 `ENCODER_CS_LOW/HIGH`** — the direct path's
two departures from HAL, both justified in place. `BSRR` is written rather than
`HAL_GPIO_WritePin` because a single store to that register sets or clears a pin with
no read-modify-write, so no interrupt can land in the middle; and because the driver
call is argument-checked overhead measured in hundreds of nanoseconds against a
transfer budget of four microseconds.

**L296–345 `exchange_frame_direct`** — the register-level exchange. Wait `TXE`, write
`DR`, wait `RXNE`, read `DR`. No busy-flag wait and no post-CS padding, and the comment
justifies both: `RXNE` only sets once the final bit has been shifted in, and the next
frame is a whole control period away — ninety times the sensor's requirement.

**L407–421 `compute_mechanical_angle`** —

```c
uint32_t from_zero = ((uint32_t)raw - (uint32_t)e->offset_counts)
                    & (ENCODER_COUNTS_PER_REVOLUTION - 1u);
```

The mask does the modulo, which is only possible because the encoder scale is a power
of two. Doing it with a conditional (`if (raw < offset) from_zero += 16384`) would be
equivalent and slower; doing it with `%` would call a division routine. Promoting to
`uint32_t` before subtracting is what makes the underflow well defined rather than
relying on `uint16_t` wrap.

Reverse direction reflects about the origin — `16384 - from_zero`, masked again so that
the zero case maps to zero rather than to 16384.

**Verified** over 163,930 combinations of offset × direction × raw: always in range,
wrap correct in both directions, and reading exactly 0 at the calibrated offset either
way round.

**L423–436 `encoder_capture`** — on a failed read it returns 0 and **keeps the previous
angles**. Substituting zero would look like the rotor jumping to the origin, and a loop
acting on that applies a large correction to something that never happened. This is the
right call and it is made consistently (`motor_update` does the same).

### Contracts

- `encoder_prime_pipeline()` must be called before the first `encoder_read_angle_pipelined()`
  of a run, or the first answer belongs to nothing.
- The pipeline assumes **exactly one exchange per control period, from one context**.
  Both E2.2 and E2.3 are violations of this.
- A failed read must leave the previous angle in place, never zero.

### Concerns

**E2.1 — HIGH. A tick-based timeout runs inside the control interrupt, where ticks are
frozen. A stalled SPI clock hangs the processor permanently.**

`telemetry_capture()` is called from inside `HAL_ADCEx_InjectedConvCpltCallback`
(`loop.c` L110). When the encoder telemetry channel is enabled it calls
`encoder_read_angle()` (`telemetry.c` L131–133), which is `encoder_read_register` →
two `exchange_frame` calls → `HAL_SPI_TransmitReceive(..., ENCODER_SPI_TIMEOUT_MS)`.

HAL's timeout is (`stm32f4xx_hal_spi.c` L855):

```c
if ((((HAL_GetTick() - tickstart) >= Timeout) && (Timeout != HAL_MAX_DELAY)) || (Timeout == 0U))
```

With `Timeout = 1` and `HAL_GetTick()` frozen, `HAL_GetTick() - tickstart` is 0 forever.
The condition never becomes true. The ISR spins inside HAL's flag wait indefinitely,
and since `ADC_IRQn` is priority 0, no other interrupt and no main-loop code ever runs
again — the board is dead until reset, with no fault, no telemetry and no console.

`encoder.c`'s own header comment at L22–28 shows this path was noticed and
misdiagnosed: it shortened the timeout from 2 ms to 1 ms "because this is now also
called from the control interrupt", reasoning about the *cost* of a timeout while
L256–260 of the same file explains that the timeout cannot happen at all there.

It requires a stalled SPI to trigger, so it has not fired on the bench. It is a
latent hang with no recovery path.

**E2.2 — HIGH. The SPI peripheral is shared between the ISR and main context with no
mutual exclusion, and the ISR path bypasses the only lock that exists.**

`HAL_SPI_TransmitReceive` protects itself: it checks `hspi->State` and then takes
`__HAL_LOCK(hspi)`, returning `HAL_BUSY` if either says the peripheral is in use. So
two HAL callers fail cleanly rather than corrupting each other.

`exchange_frame_direct` participates in neither. It never sets `hspi->State`, never
takes the lock, and writes `DR` directly.

So when `commands.c` runs `angle` or `encdiag` from the main loop — a HAL transfer in
progress — and the ADC interrupt fires at priority 0 partway through, the ISR drops
chip select, writes its own frame to `DR`, consumes the `RXNE` that belonged to the
main-context transfer, and raises chip select again. Both sides then continue on wrong
data. HAL has no way to notice.

The worst case is quiet rather than loud: `encoder_read_register` for `DIAAGC` and the
pipelined `ANGLECOM` read produce frames that are **indistinguishable** once received —
both are 14 bits of payload with valid parity and a clear error bit. The control loop
can therefore take a field-magnitude reading as a rotor angle, and nothing in the
firmware can tell. With a current loop closed, that is a torque transient in an
arbitrary direction.

This has been happening all session, every time `angle` or `encdiag` was typed with
the loop running.

**E2.3 — MEDIUM. A timed-out direct exchange desynchronises the pipeline permanently.**

If `exchange_frame_direct` gives up at the `RXNE` wait (L320–325) it raises chip select
and returns 0, but the transfer it already started by writing `DR` completes anyway a
few microseconds later, leaving `RXNE` set with an unread frame. The *next* call waits
on `TXE`, writes `DR`, then finds `RXNE` already set and reads the **stale** frame.

From then on every reading is one frame old, permanently — and because the pipelined
read is already one frame delayed by design, there is no way to detect the difference
from the value alone. There is no overrun (`OVR`) check and no resynchronisation path.
The fix is to drain `DR` and clear `OVR` on entry, or to re-prime the pipeline after
any failure.

**E2.4 — LOW. `encoder_get_*` / `encoder_set_*` (L182–202) are in a different style from
the rest of the file** — tabs instead of four spaces, `{` on the declaration line,
`encoder_t* e` instead of `encoder_t *e`. These are the remains of the original
hand-written draft. Cosmetic, but this is a refactor whose stated purpose is
readability, and the seam is visible.

**E2.5 — LOW. `pipelined_command` is a non-`volatile` file static touched from both
contexts.** `ensure_pipelined_command()` does an unsynchronised check-then-set. Benign
in practice: the store is a single aligned 16-bit write, and both contexts compute the
same constant. Worth `volatile` for consistency with how every other cross-context
field in this codebase is declared.

---

## `App/drivers/gate_driver.{h,c}`

### Purpose

Drives the three IR2104 half-bridge drivers and the six FDMS86300DC MOSFETs: PWM duty
per phase, the active-low shutdown pins, the bootstrap charging sequence, and the
dead-time compensation that turns a voltage demand into a duty.

### Place in the architecture

Includes only `<stdint.h>` and `main.h`. Called from main context (`estimate`,
`calibration`, `commands`, the `_start` functions) and from the ISR
(`gate_driver_set_duty` / `gate_driver_apply_voltage` via `motor_apply_dq`).

### Line by line — the parts that carry weight

**L19–27, the phase table** — timer channel, shutdown port and shutdown pin per phase,
in one place. The comment states the payoff precisely: no function below contains a
`switch` on the phase number. Porting to a board with the bridge on a different timer
means editing three rows, not auditing every function.

The designated-initialiser form (`[GATE_DRIVER_PHASE_A] = {...}`) rather than
positional is what makes the table order-independent — reordering the `#define`s cannot
silently mis-map a phase.

**L69–78 `duty_to_compare_value`** — see G2.1. `__HAL_TIM_GET_AUTORELOAD(&htim3)` is
read from the peripheral rather than hard-coded, so changing the PWM frequency in
CubeMX cannot silently break the duty scale. That part is right and worth keeping.

**L101–106 `write_shutdown_pin`** — the **only** place in the module that knows SD is
active low. That is the entire justification for the function existing rather than the
two call sites writing the pin themselves.

**L112–132 `gate_driver_init`** — ordering is load-bearing and the comment says why:
every shutdown pin is driven low **before** `HAL_TIM_PWM_Start`. Starting the timer
first would mean the bridge could begin switching before any duty had been set. With
the pins low the drivers ignore the edges entirely, which is what makes this safe to
call with a motor and bus voltage connected.

**L153–178 `gate_driver_enable_phase`** — forces duty to zero, raises SD, blocks 5 ms,
restores the duty. The block is deliberate: making it asynchronous would let the caller
raise duty before the bootstrap capacitor charged, which is exactly the failure the
function exists to prevent. Turning the high side on with an empty bootstrap capacitor
leaves the FET part-way on and destroys it.

`HAL_Delay` here is a tick-based wait, so by the rule above it would hang in the ISR.
**Checked:** all six call sites (`calibration.c:267`, `estimate.c:120/145/146`,
`openloop.c:317`, `currentloop.c:153`, `commands.c:285/970/971`) are inside
`_start()` / `_run()` functions invoked from main context. The installed loop functions
are `openloop_control_step` and `currentloop_step`, and neither calls it. Safe today,
but it is the same trap as E2.1 and nothing enforces it.

**L230–325 `gate_driver_apply_voltage`** — the interesting function.

The resting point is half duty, because a bridge cannot drive a phase below the
negative rail; at half on all three phases the terminals sit at the same potential and
no current flows. The demand is expressed as a fraction of the **measured** bus rather
than a nominal figure, so the applied voltage stays correct as the supply sags — which
matters on this board, where the bus reading moves by two volts under load.

**The dead-time correction sign is correct.** Walked through from the physics rather
than trusted:

- Positive current (into the terminal) must keep flowing during the dead time, so it
  forward-biases the **low-side** body diode and pulls the terminal to −0.7 V. The
  phase therefore spent *less* time high than commanded, the applied voltage is *low*,
  and the correction must be **upward**. `scaled` is positive, `correction` is
  positive, duty increases. ✓
- Negative current forward-biases the **high-side** diode, holding the terminal above
  the rail, so the phase spent *longer* high than commanded and the correction is
  **downward**. ✓

**L303–322, the bound, is the most important thing in the file** and its comment earns
its length. The correction takes its sign from the measured current and the voltage it
admits sustains that current — a feedback loop. On this board the full correction is 24
parts per thousand of the bus, about 290 mV, which across a 54 mΩ path is over five
amps. Without the bound, the correction alone drives more current than any sensible
command and then holds it there: the motor draws heavily, produces no useful torque,
and which direction it latched in is arbitrary. Bounding the correction by
`|commanded|` breaks the loop — it can still restore voltage the dead time removed, but
it cannot create voltage nobody asked for, so with nothing commanded nothing happens.

**L319–324, the signed clamp before the cast** — also necessary, and for a reason the
comment states: `duty` is signed here and `gate_driver_set_duty`'s parameter is not, so
an unclamped negative value would wrap to a huge positive one on the cast rather than
saturating to zero. `set_duty`'s own clamp would then produce `GATE_DRIVER_DUTY_MAXIMUM`
— full duty — from what should have been zero.

### Concerns

**G2.1 — LOW (numerically), but the comment describes the wrong counting mode.**

`duty_to_compare_value` says:

> The timer counts from 0 to ARR and the output is high while the count is below the
> compare register, so compare value divided by (ARR + 1) is the fraction of each
> period spent with the high side on.

TIM3 is configured `TIM_COUNTERMODE_CENTERALIGNED1` with `Period = 1499`, `Prescaler = 0`
(`main.c` L168–170). The counter goes 0 → 1499 → 0, so a period is 2 × ARR = 2998 counts
(31.229 µs at 96 MHz — which is where `GATE_DRIVER_PERIOD_NS = 31229` comes from, so the
header has it right). In centre-aligned mode the high time is 2 × CCR out of 2 × ARR, so
the fraction is **CCR / ARR**, not CCR / (ARR + 1).

The code computes `duty × (ARR + 1) / 1000` = `duty × 1.5`, so the delivered duty is high
by a factor 1500/1499 — 0.067%. At the 900 ceiling that is 90.06% instead of 90.00%.
Numerically irrelevant.

The comment is the finding, not the arithmetic. This is the file where the duty scale
underpins every voltage computation in the firmware — `eres`, `eind`, `ical` and the
current loop all work backwards from duty to volts — and a reader checking that chain
against an edge-aligned mental model will not reproduce the numbers. The worked overflow
example ("at most 1000 * 3125") is wrong too; it is 1000 × 1500.

**G2.2 — LOW. `self` is dereferenced unguarded in three functions.**
`gate_driver_set_duty` (L147), `gate_driver_get_duty` and `gate_driver_is_enabled`
assume `gate_driver_init` has run. `gate_driver_disable_all` explicitly guards with
`if (self != NULL)` and documents why — it is the fault handler's escape hatch and must
survive being called before init, e.g. when a CubeMX peripheral init fails during
startup and calls `Error_Handler`. That reasoning applies with equal force to
`set_duty`: `Error_Handler` is reachable from any of the `MX_*_Init` calls, and any
future fault path that zeroes duties before disabling would fault instead.

---

## `App/drivers/sensors.{h,c}`

### Purpose

Everything the ADC measures — bus voltage, temperature, and the two phase currents —
with the shunt value, the divider ratio and the ADC group membership all hidden here.

### Place in the architecture

Two ADC groups with genuinely different timing requirements, deliberately in one module:

- **Injected group** — phase currents, triggered by TIM3_CH4, read by
  `sensors_capture_currents()` from the control ISR. Must be sampled at an exact instant
  in the PWM cycle.
- **Regular group** — bus and temperature, converting continuously into a DMA circular
  buffer. Reading one is a memory access with no waiting.

The injected group automatically preempts the regular group, which is the whole reason
both exist: a current sample is never delayed by a slow housekeeping conversion.

### Line by line — the parts that carry weight

**L56–70 `HAL_ADC_Start_DMA(&hadc1, (uint32_t *)(void *)self->regular_results, 2)`** —
this looks like a buffer overflow and is not. The buffer is `volatile uint16_t[2]` and
the parameter is `uint32_t *`, which reads as though the DMA will write two 32-bit words
into a 32-bit array. **Checked the configuration:** `PeriphDataAlignment` and
`MemDataAlignment` are both `DMA_xDATAALIGN_HALFWORD`, mode `DMA_CIRCULAR`, with
`NbrOfConversion = 2` — so exactly two half-words are written and `uint16_t[2]` is the
right size.

The existing comment explains the `volatile` discard but says nothing about the width,
which is the half that looks alarming. Worth adding.

**L86–96 `counts_to_milliamps`** —

```c
int32_t counts_from_zero = (int32_t)raw_counts - (int32_t)zero_reference;
```

Both operands cast to signed **before** subtracting. Left unsigned, a reading below the
reference would wrap to an enormous positive number instead of going negative — a
current of −100 mA would read as roughly +82 amps and trip every protection limit at
once.

**L119–124** — the direction multiply added in this session, applied after the offset
subtraction. Order matters: direction is a property of the sensor's polarity, the offset
is a property of its zero, and correcting polarity before removing the zero would negate
the offset too.

**L99–110, the calibration branch** — during zero calibration, raw values are
accumulated and nothing is published, because the references that conversion needs are
exactly what is being measured. A clean way to avoid a chicken-and-egg problem in the
ISR.

### Concerns

**S2.1 — MEDIUM (fragile contract, not currently a bug).**
`sensors_end_current_calibration` divides by the **constant** `CURRENT_ZERO_CALIBRATION_SAMPLES`
(64), not by `calibration_samples_taken`:

```c
self->zero_counts_a = (uint16_t)(self->calibration_total_a / CURRENT_ZERO_CALIBRATION_SAMPLES);
```

Called after only 32 samples, the zero reference comes out **half** its true value —
about 1024 counts low, which at 40.28 mA per count is a phantom ~41 A offset baked into
every subsequent current reading, in both the control loop and every protection check.

`sensors_current_calibration_done()` exists to prevent this but nothing *enforces* it.
**Checked the only caller:** `loop.c:collect_zero_references` waits correctly, has a
`ZERO_CALIBRATION_TIMEOUT_MS` bound, and calls `sensors_cancel_current_calibration()`
rather than `_end_` on timeout. It runs in main context so its `HAL_GetTick()` works.

So the bug cannot fire today. It is one careless second caller away from firing, and
dividing by `calibration_samples_taken` with a zero guard would make it impossible
instead of merely unlikely.

**S2.2 — LOW. `direction_a` / `direction_b` are not `volatile`.** They are written from
main context (`sensors_set_direction`, via `estimate_current_direction`) and read inside
the ISR (`sensors_capture_currents`). Every other cross-context field in `sensors_t` —
`current_a_ma`, `calibrating_currents`, the accumulators — is `volatile`. These two are
not. Harmless in practice (`int8_t` stores are atomic, and each ISR pass reads them
once), but it is an inconsistency in exactly the place where the convention matters.

---

## `App/drivers/USB_Comm.{h,c}`

### Purpose

The USB CDC transport: a receive ring buffer filled from interrupt context and drained
by the main loop, and a blocking transmit path.

### Place in the architecture

```
USB interrupt  →  usbd_cdc_if.c  →  usb_rx_enqueue        (producer)
main loop      →  protocol.c     →  usb_rx_dequeue        (consumer)
main loop      →  protocol_text.c / protocol_binary.c → usb_tx_bytes / usb_tx_string
main loop      →  telemetry.c    →  usb_tx_is_ready
```

**Checked:** no transmit path is reachable from the control ISR, so `usb_tx_bytes`'s
20 ms `HAL_GetTick()` wait is safe where it is used. `telemetry.c` comments at L222 that
it deliberately avoids `usb_tx_bytes` in the capture path for exactly this reason.

### Line by line — the ring buffer

The concurrency argument at L39–64 is the most carefully reasoned comment in the
codebase and it is correct. The structure is single-producer / single-consumer:

- the interrupt is the **only** writer of `rx_queue_end_index` and only ever *reads*
  `rx_queue_start_index`;
- the main loop is the **only** writer of `rx_queue_start_index` and only ever *reads*
  `rx_queue_end_index`.

With one writer per variable and 32-bit aligned accesses that a Cortex-M4 cannot split,
neither side can observe a half-written value, so no interrupt disabling is needed.
Reading a stale index is harmless in the one direction it can happen: the main loop
thinks the queue is a byte shorter than it is and picks the byte up next pass.

The comment closes by noting that the "one caller each" rule in the header is therefore
**not a style preference** — add a second caller of either function and the whole
argument collapses. That is exactly right and worth preserving verbatim.

**L78–81 `advance_index`** — `(index + 1) & (RX_QUEUE_SIZE - 1)`. Valid only because
`RX_QUEUE_SIZE` is 512, a power of two; a modulo would call a division routine. The
dependency is documented in the comment, which matters because changing the size to
something like 500 would break it silently.

**L94–105, the sacrificed slot** — the queue reports full one slot early, because
`start == end` already means empty and there would otherwise be no way to distinguish
the two states. Standard, and the comment explains it rather than assuming the reader
knows.

**L107–114, the store ordering** — the byte is written to the slot *before* the index
advances, so the consumer cannot see a slot that has not been filled. Correct, and on
Cortex-M4 with `volatile` accesses the compiler will not reorder them. Strictly, a
`__DMB()` would be the architecturally portable guarantee; on this core's memory model
it makes no difference.

**L146–155 `usb_tx_is_idle`** — reads `TxState` through `hUsbDeviceFS.pClassData`, with
a `NULL` check for the window before the class is initialised.

**L173–180 `usb_tx_bytes`'s wait** — `(HAL_GetTick() - wait_start_tick) > TX_TIMEOUT_MS`,
subtraction rather than comparison against a sum, so it stays correct across the 49-day
tick wrap. The same idiom is used in `main.c`'s heartbeat and `loop.c`'s calibration
wait; it is consistent throughout.

### Concerns

**U2.1 — LOW. Dead code left in comments, and it references a function that no longer
exists.** `usb_tx_formatted` at L199–227 is commented out and calls
`usb_transmit_bytes` — a name that was renamed to `usb_tx_bytes`. `<stdarg.h>` and
`<stdio.h>` are included at L15–16 solely for it; `<stdio.h>` in particular is worth not
including for nothing. Either restore the function with the correct name or delete it
and the two includes.

**U2.2 — LOW. `usb_tx_bytes` silently truncates.** `count > TX_BUFFER_SIZE` clamps to
256 and then returns success. A caller sending more loses the tail with no indication.
The buffer sizes make this unreachable today (the largest framed packet is well under
256), but "returns 1" and "sent your data" are not the same statement.

---

## Methodology correction to batch 1

Batch 1 stated that dead-code claims were verified by searching "the whole tree". They
were not — the search covered `App/` and `Core/Src/` only, and **missed `USB_DEVICE/`
and `Middlewares/`**.

This surfaced here: `usb_rx_enqueue` appears in batch 1's list of functions "never
called outside their own `.c`", and it is not dead at all — it is called from
`USB_DEVICE/App/usbd_cdc_if.c`, which is how every received byte enters the firmware.

**All nine of batch 1's dead-code claims were re-run against the full tree** including
`USB_DEVICE/` and `Middlewares/`. All nine still hold at zero call sites:
`units_mrad_to_angle`, `units_counts_rate_to_mrads`, `units_mrads_to_rpm`, `led1`,
`led2_toggle`, `rs485_set_transmit`, `filter_init`, `filter_reset`, `filter_process`.

So no batch 1 conclusion changes. The method statement was wrong and is corrected here;
every subsequent batch searches `App Core/Src USB_DEVICE Middlewares`.

---

## Verification method

- `apply_even_parity` and `check_even_parity` lifted verbatim and run over **all 65,536
  frames**, checked against an independent bit count; every framed payload round-trips;
  all 14,176 tested single-bit flips rejected.
- `compute_mechanical_angle` lifted verbatim and run over **163,930** offset × direction
  × raw combinations, checked against an independently written expression, including the
  property that it reads exactly 0 at the calibrated offset in both directions.
- Interrupt priorities read from `stm32f4xx_hal_msp.c` and `stm32f4xx_hal_conf.h`.
- HAL's timeout condition, state check and `__HAL_LOCK` read from
  `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c` in this repository.
- TIM3 counter mode, period and prescaler, and the ADC/DMA data alignment, read from
  `Core/Src/main.c` and `Core/Src/stm32f4xx_hal_msp.c`.
- Every call site of `gate_driver_enable_phase` traced to its enclosing function to
  establish that no `HAL_Delay` is reachable from the ISR.
- Cross-compile clean before and after; no source file modified.

---

## Findings carried forward

| ID | Severity | Summary |
|---|---|---|
| E2.1 | **High** | `telemetry_capture` → `encoder_read_angle` uses a tick-based SPI timeout inside the ADC ISR, where ticks are frozen — a stalled SPI hangs the board permanently with no recovery |
| E2.2 | **High** | SPI shared between ISR and main context; `exchange_frame_direct` bypasses HAL's state check and lock. `angle`/`encdiag` while the loop runs can feed the control loop a diagnostics register as a rotor angle |
| E2.3 | Medium | A timed-out direct exchange leaves `RXNE` set; the pipeline is then permanently one frame stale, with no `OVR` check or resync |
| S2.1 | Medium | `sensors_end_current_calibration` divides by the constant 64, not the samples actually taken — an early call bakes a ~41 A phantom offset into every reading |
| G2.1 | Low | `duty_to_compare_value`'s comment describes edge-aligned counting; TIM3 is centre-aligned. Duty high by 0.067%; the worked overflow example is also wrong |
| G2.2 | Low | `self` dereferenced unguarded in `set_duty`/`get_duty`/`is_enabled` while `disable_all` guards and documents why |
| S2.2 | Low | `direction_a`/`direction_b` not `volatile` where every other cross-context field in the struct is |
| U2.1 | Low | Commented-out `usb_tx_formatted` calls a renamed function; `<stdarg.h>`/`<stdio.h>` included only for it |
| U2.2 | Low | `usb_tx_bytes` truncates above 256 bytes and still reports success |
| E2.4 | Low | `encoder_get_*`/`encoder_set_*` in a different brace/indent style from the rest of the file |
| E2.5 | Low | `pipelined_command` non-`volatile` despite being touched from both contexts |
