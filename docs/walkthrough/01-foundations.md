# Batch 1 — Foundations

The six files nothing else depends on. Every one of them is a leaf in the include
graph: they pull in nothing from the project, so they can be read without knowing
anything else about the firmware, and everything above them can be read knowing only
what is here.

| File | Lines | Code | Verdict |
|---|---|---|---|
| `board/units.h` | 142 | 25 | one latent overflow bug, three dead functions |
| `board/apex_board.h` | 140 | 18 | stale header comment, architectural smell, three dead helpers |
| `comms/crc16.{h,c}` | 143 | 26 | correct — check value verified |
| `comms/cobs.{h,c}` | 293 | 96 | correct, but deviates from canonical COBS in one case |
| `control/pid.{h,c}` | 211 | 45 | correct; one unguarded division, two efficiency notes |
| `control/filter.{h,c}` | 112 | 18 | **entirely dead code** |

Everything asserted below was checked by running it, not by reading it. The harness is
described at the end.

---

## `App/board/units.h`

### Purpose

Defines the units every value crossing the console is expressed in, and the integer
conversions between those and the raw quantities inside the firmware. It exists so
that neither a human at a terminal nor the desktop application has to know that the
encoder is 14-bit or that duty is per-thousand.

### Place in the architecture

A leaf. Includes only `<stdint.h>`. Every function is `static inline`, so there is no
`units.c` and no link-time dependency — including the header is the whole cost.

Consumers: `commands.c` (5 call sites) and `calibration.c` (via the include). Nothing
else.

### Line by line

**L57 `#include <stdint.h>`** — for `int32_t`/`uint16_t`. The fixed-width types are
load-bearing here rather than stylistic: every conversion below reasons about exactly
where its intermediate product overflows, and that reasoning is only valid if the
width is pinned. `int` would be 16-bit on some targets and the overflow analysis in
the comments would silently become wrong on a port.

**L60 `#define UNITS_MRAD_PER_TURN 6283`** — 2π × 1000 = 6283.185…, truncated. The
0.003% error is far below the encoder's resolution (16384 counts per turn is 0.38
mrad), so it cannot be observed. It must be an integer because every conversion below
is integer arithmetic; a float here would force floats through the whole chain for no
gain.

**L63 `#define UNITS_COUNTS_PER_TURN 16384`** — the AS5147's 14 bits. Note this is a
power of two, which matters in `encoder.c` where the same constant lets angle wrap be
done with a mask instead of a modulo. Here it is only a divisor.

**L67 `#define UNITS_ANGLE_SCALE 65536`** — the internal angle representation: a
`uint16_t` spanning one turn. The choice is deliberate and is the reason the control
path has no angle-wrapping code at all: adding to a `uint16_t` wraps at exactly one
revolution as a property of the type. Any other scale (say 3600 for tenths of a
degree) would need an explicit `if (angle >= SCALE) angle -= SCALE` at every addition,
which is both slower and a place to forget.

**L75–81 `units_counts_to_mrad`** —

```c
return ((int32_t)counts * UNITS_MRAD_PER_TURN) / UNITS_COUNTS_PER_TURN;
```

Multiply-then-divide, not divide-then-multiply. Dividing first would give
`counts/16384`, which is 0 for every input below one full turn — the entire result.
The cast to `int32_t` happens before the multiply: without it, `counts * 6283` is
computed in `int` and, while that is 32-bit here, the promotion rules are what make it
so, and stating the width removes the question. Maximum product is 16383 × 6283 =
102,935,589, about 5% of `INT32_MAX`. Safe with a wide margin.

**L89–92 `units_angle_to_mrad`** — same shape. Maximum product 65535 × 6283 =
411,756,405, about 19% of `INT32_MAX`. Safe.

**L100–111 `units_mrad_to_angle`** —

```c
int32_t within_turn = mrad % UNITS_MRAD_PER_TURN;
if (within_turn < 0) { within_turn += UNITS_MRAD_PER_TURN; }
```

The modulo first, so a caller passing several turns' worth gets the equivalent angle
rather than an overflow on the subsequent multiply. The sign correction is required
because C's `%` takes the sign of the dividend: `-100 % 6283` is `-100`, not `6183`.
Without the correction, a negative input would produce a negative `uint16_t` cast,
which wraps to near 65535 — an angle almost a full turn away from the right one. This
is the kind of error that produces a motor that runs correctly in one direction and
violently wrong in the other.

**L119–126 `units_counts_rate_to_mrads`** — see Concerns. The multiply-first ordering
is right for precision and wrong for range, and the comment's justification does not
hold.

**L135–140 `units_mrads_to_rpm`** — `(mrads * 60) / 6283`. Multiply first again.
Overflow at `INT32_MAX/60` = 35.8 million mrad/s, which is 5700 revolutions per
second. Not reachable. Safe.

### Contracts

- Angles inside the control path are `uint16_t` spanning one turn, and wrap by type
  overflow. Nothing may introduce a different angle scale without breaking that.
- All conversions truncate. A round trip `mrad → angle → mrad` can lose up to 1 mrad.
  Nothing currently round-trips, but a velocity loop differentiating a converted angle
  would see that truncation as noise.

### Concerns

**C1.1 — `units_counts_rate_to_mrads` overflows and wraps negative at 1252 rpm.
(Latent; currently unreachable.)**

The product `counts_per_second * 6283` exceeds `INT32_MAX` above 341,792 counts/s.
Measured:

```
units_counts_rate_to_mrads(400000) = -108750      (true answer 153394)
```

341,792 counts/s is 20.9 rev/s — **1252 rpm**. The comment claims this is "about 20
revolutions per second faster than this motor will ever turn", which misreads its own
number: 20.9 rev/s is the total, not the headroom, and 1252 rpm is ordinary for a
7-pole-pair BLDC on a 12 V bus.

It cannot bite today because the function has **zero callers**. It will bite the
moment the velocity loop is written, which is exactly the thing not yet built. The
failure mode is a velocity that reads correctly up to 1252 rpm and then reports a
large negative number — which a velocity PI would respond to by commanding full
torque in the wrong direction.

Fix: compute through `int64_t`, or exploit `16384 = 2^14` and shift after a widened
multiply. Either is one line.

**C1.2 — three of the five conversions are dead.** `units_mrad_to_angle`,
`units_counts_rate_to_mrads` and `units_mrads_to_rpm` have zero call sites anywhere in
the tree, `commands.c` included. All three are plausibly wanted by the unbuilt
velocity and position loops, so the recommendation is to keep them and fix C1.1, not
to delete them — but they are unexercised, and C1.1 is what unexercised code looks
like.

---

## `App/board/apex_board.h`

### Purpose

Two unrelated jobs: an aggregate `#include` of the entire project, and three pairs of
GPIO helpers for the LEDs and the RS-485 direction pin.

### Place in the architecture

Included by exactly two files: `commands.c` and `protection.c`. Includes `main.h`
(for the CubeMX pin macros) and then all 21 project headers.

### Line by line

**L37 `#include "main.h"`** — must come first. The `LED1_Pin` / `LED1_GPIO_Port`
macros used below are generated by CubeMX into `Core/Inc/main.h` from the User Labels
in the `.ioc`. This is the mechanism that keeps the `.ioc` as the single source of
truth for the pin map: nothing in this file names a port or a pin number, so
relabelling a pin and regenerating propagates automatically.

**L41–67** — the aggregate include, ordered by layer. See Concerns.

**L84–88 `led1`** —

```c
HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin,
                  (on != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
```

The `(on != 0u) ?` rather than a cast: `GPIO_PinState` is an enum with values 0 and 1,
and `on` is any non-zero truth value. Casting `on` directly would pass e.g. 2 into a
HAL function whose parameter is documented as taking only those two enumerators. The
ternary normalises it. `static inline` rather than a function in a `.c` so this
compiles to the two or three instructions `HAL_GPIO_WritePin` inlines to, with no call
overhead and no `apex_board.c` to link.

**L95–99 `led2`**, **L105–108 `led1_toggle`**, **L113–116 `led2_toggle`** — same
shape. `HAL_GPIO_TogglePin` exists because read-modify-write on `ODR` is not atomic
against an interrupt that touches another pin on the same port; the HAL function uses
`BSRR`, which is.

**L134–138 `rs485_set_transmit`** — the DE and RE pins of the transceiver are tied
together on this board. The comment explains why that works: RE is active low and DE
active high, so one level transmits and the other receives, and no level leaves both
active. Driving one pin is therefore sufficient and no dead-time between the two is
possible.

### Concerns

**C1.3 — the file's own header comment describes helpers that are not in it.**

> WHY THE POLARITY HELPERS EXIST — Two signals on this board are ACTIVE LOW […] so the
> polarity is written down once, here, and nothing else is allowed to know it.

Nothing in this file is active low. Both LEDs are active high (as the LED section
itself states two paragraphs later), and the RS-485 pin is described as high-to-
transmit. The active-low signals on this board are the three IR2104 shutdown pins, and
they are handled in `gate_driver.c`, not here. The comment appears to be left over
from an earlier arrangement. It is actively misleading: it tells a reader that this
file is where bridge polarity lives.

**C1.4 — `rs485_set_transmit` has zero callers, so RS-485 transmit cannot work.**

The pin resets low, which selects receive. Nothing ever raises it. If RS-485 is
intended as a transport, the direction control is missing; if it is not yet
implemented, the helper is correct but premature. Worth an explicit decision rather
than being left ambiguous — this is the kind of gap that is found on a bench with a
scope after an hour of wondering why nothing transmits.

**C1.5 — `led1` and `led2_toggle` are dead.** `led2` is used by `protection.c` (3
sites) and `led1_toggle` by `main.c`'s heartbeat. The other two have no callers. Low
severity — they are the symmetric completion of an obvious API.

**C1.6 — the aggregate include works against the stated modularity goal.**

`apex_board.h` pulls in all 21 project headers, including `commands.h` and
`estimate.h`. Any file that includes it acquires the entire project's namespace, which
means the compiler can no longer tell you that a module reached across a layer it
should not have. It also creates a latent include cycle: `apex_board.h` includes
`commands.h`, and `commands.c` includes `apex_board.h`. The include guards make that
harmless today, but it is harmless by accident rather than by design.

Only two files use it, and both could name their dependencies directly — `protection.c`
needs four headers, not twenty-one. The board-specific GPIO helpers are worth having in
a board header; the aggregate include is worth deleting.

---

## `App/comms/crc16.{h,c}`

### Purpose

CRC-16/CCITT-FALSE over a decoded packet's contents, so corruption that COBS framing
cannot detect is caught before the payload is acted on.

### Place in the architecture

A leaf; includes only `<stdint.h>` and `<stddef.h>`. Called by `packet.c` (framing)
and `commands.c` (the `selftest` command, which checks the check value on the target
itself).

### Line by line — `crc16.h`

**L61 `#define CRC16_CHECK_VALUE 0x29B1u`** — the single most useful line in the file.
"CRC16" names dozens of mutually incompatible algorithms; two implementations
disagreeing on any one parameter reject every packet the other sends, and the symptom
is indistinguishable from a wiring fault. Publishing the check value means the desktop
application can prove agreement before any real traffic is attempted.

**Verified:** `crc16_compute("123456789", 9)` returns `0x29B1`. The header's claim is
true, so the constant can be trusted as the interop anchor.

**L65 `#define CRC16_INITIAL_VALUE 0xFFFFu`** — exposed, not private, because
`crc16_update` takes a starting value and a caller computing across several buffers
needs somewhere to begin.

### Line by line — `crc16.c`

**L8 `#define CRC16_POLYNOMIAL 0x1021u`** — x¹⁶ + x¹² + x⁵ + 1 with the x¹⁶ term
omitted. It has to be omitted: that term is the bit that shifts out of a 16-bit
register, so it is implicit in the register width and there is nowhere to store it.

**L16–18** —

```c
if ((data == NULL) || (length == 0u)) { return crc; }
```

Returns the running value unchanged rather than zero or the initial value. That is the
mathematically correct identity for "add no bytes", and it is what makes
`crc16_update` safe to call in a loop over a scatter list where some entries are
empty. **Verified:** `crc16_update(0x1234, NULL, 5)` returns `0x1234`.

**L26 `crc ^= (uint16_t)((uint16_t)data[i] << 8);`** — the byte enters at the *top* of
the register. This is what "input not reflected" means in the specification: the most
significant bit of each byte is processed first, so the byte must align with the high
half. A reflected variant would xor into the low half and shift right instead. The
inner `(uint16_t)` cast on the shift is not redundant: `data[i]` promotes to `int`, so
`data[i] << 8` is an `int` expression, and on a 16-bit `int` target that shift would
overflow into the sign bit. Casting makes the width explicit.

**L34–38** —

```c
if ((crc & 0x8000u) != 0u) {
    crc = (uint16_t)((uint16_t)(crc << 1) ^ CRC16_POLYNOMIAL);
} else {
    crc = (uint16_t)(crc << 1);
}
```

Polynomial long division in GF(2), where subtraction is xor. Test the bit about to
fall off the top; shift; if it was set, subtract the polynomial. There is no shorter
formulation of this that is also readable — the common branchless
`crc = (crc << 1) ^ (poly & -(crc >> 15))` is the same operation with the conditional
hidden inside a sign-extension trick, and buys nothing here.

**L45–49 `crc16_compute`** — delegates to `crc16_update` from the initial value. The
one-line body is the whole point: this variant has no final xor and no output
reflection, so the running value *is* the result, and stating that in a comment is
cheaper than a reader checking the specification.

### Concerns

None. The implementation matches its documented specification, the check value is
correct, and the bit-at-a-time choice is justified (packets are small and the CRC runs
outside the control interrupt, so a 512-byte table would buy nothing worth the flash).

---

## `App/comms/cobs.{h,c}`

### Purpose

Consistent Overhead Byte Stuffing: rewrite a block so it contains no zero bytes, so
that a zero can be used unambiguously as a frame delimiter.

### Place in the architecture

A leaf. Called by `packet.c` for real framing and by `commands.c` for `selftest`.

### Line by line — `cobs.h`

**L67–68 `COBS_ENCODED_MAX`** —

```c
#define COBS_ENCODED_MAX(input_length) \
    ((input_length) + (((input_length) + 253u) / 254u) + 1u)
```

One overhead byte to start, plus one more for every 254 bytes that contain no zero.
`(n + 253)/254` is integer ceiling division. The `+ 1` covers the leading length byte
for input that is shorter than one full run. **Verified** as a true upper bound across
all 65 test cases, including the exact-multiple boundaries where it is tightest.

The comment that it excludes the trailing delimiter is load-bearing: a caller sizing a
buffer from this macro and then appending a zero overruns by one.

### Line by line — `cobs.c`

**L9 `#define COBS_MAXIMUM_RUN 254u`** — a length byte counts the bytes up to *and
including* the next link, so the largest value, 255, covers 254 data bytes. This is
where COBS's one-byte-per-254 worst case comes from.

**L19–23** — `length_byte_position = 0`, `output_position = 1`. The length byte for a
run cannot be written until the run ends, so its slot is reserved and filled in later.
This is why the encoder cannot be a pure streaming transform over the output: it must
write backwards by up to 254 bytes.

**L28 `uint8_t run_length = 1u;`** — starts at 1 because the length byte counts
itself. A run with no data bytes is written as `1`.

**L38 `final_length_byte_pending`** — exists for exactly one case, which the comment
states: input ending precisely as a maximum-length run completes. See C1.7.

**L40–48** — `NULL` guards, then `output_capacity < 1`. The second check is separate
because even empty input writes one byte, so a zero-capacity buffer fails for empty
input too.

**L52–63, the zero branch** —

```c
output[length_byte_position] = run_length;
if (output_position >= output_capacity) { return 0u; }
length_byte_position = output_position;
output_position++;
```

The write precedes the bounds check, which looks wrong and is not: `length_byte_position`
was validated when it was *reserved* — either at L19 (where `output_capacity >= 1` was
checked) or at L61/L89 (each guarded by the check immediately above). The invariant is
that a reserved slot is always in range. The zero byte itself is never stored; its
position is what the length encodes, which is the whole trick.

**L67–72, the non-zero branch** — bounds check, copy, advance, count.

**L76–95, the run-full branch** — when `run_length` reaches 255 there is no zero to
point at, so a link is inserted. `run_length` is `uint8_t` and the comparison is
against 255, so it can never wrap. The `(i + 1u) < input_length` test is the
optimisation described in C1.7.

**L101–103** — close the final run unless the loop already did and deliberately
declined to open another.

**L131–133, decode** — a length byte of zero is rejected. It cannot occur in valid
encoded data (it would point at itself), so it means either corruption or that a
delimiter was mistakenly included in the input. Rejecting here rather than letting the
CRC catch it means the CRC is never computed over nonsense.

**L139–157** — copy `run_length - 1` data bytes, rejecting three things: a run
pointing past the end of the input, a zero inside the data (encoding removed all of
them), and output overflow.

**L165–173** —

```c
if ((run_length < (COBS_MAXIMUM_RUN + 1u)) && (input_position < input_length))
```

Restore the zero that ended this run — unless the run was maximum-length (it ended
because it was full, not because of a zero), or the run ended at the end of the input
(encoding closed the final run without a following zero). Both exceptions are
necessary; dropping either corrupts the output by one trailing zero.

### Contracts

- The caller owns the delimiter. `cobs_encode` does not append the trailing zero and
  `cobs_decode` must not be given it. `packet.c` is where that framing lives.
- Input and output must not overlap. Neither function checks this; both write forward
  while reading forward, and the encoder additionally writes backwards to a reserved
  slot, so an in-place call would corrupt silently.
- Zero return means failure. Unambiguous because a successful encode always writes at
  least one byte.

### Verification

65 cases — lengths 0, 1, 2, 3, 253, 254, 255, 256, 507, 508, 509, 760, 1000 crossed
with all-zeros, no-zeros, alternating, zero-only-at-the-end, and random content —
encoded, compared against an independently written reference encoder, checked for zero
bytes in the output, checked against `COBS_ENCODED_MAX`, and round-tripped. Malformed
input (zero length byte, run past end, zero inside a run) and an undersized output
buffer were each confirmed rejected.

All round trips are lossless. All malformed inputs are rejected. One difference from
the reference, in C1.7.

### Concerns

**C1.7 — the encoder is not byte-identical to canonical COBS. It is interoperable, but
the desktop application's author needs to know.**

When the input ends exactly as a maximum-length run completes — 254 non-zero bytes,
508, 762 — this encoder emits one byte fewer than the canonical algorithm:

```
254 non-zero bytes:  ours 255 bytes,  canonical 256
508 non-zero bytes:  ours 510 bytes,  canonical 511
```

Canonical COBS appends a trailing `0x01` describing an empty final group. This encoder
suppresses it (the `(i + 1u) < input_length` test at L85).

This is deliberate and documented in the code. It is also safe in both directions,
which I checked by hand-tracing both decoders against both encodings:

- a canonical decoder reads our 255-byte frame, copies 254 bytes, appends no zero
  because the code was `0xFF`, and finds the input exhausted — correct;
- our decoder reads a canonical 256-byte frame, and the trailing `0x01` group
  contributes no data bytes and no zero because `input_position` has reached
  `input_length` — correct.

So nothing breaks. But if the datalogger is written against Python's `cobs` package or
any other standard implementation and the two are compared byte-for-byte — in a unit
test, or when debugging a capture — they will differ on exactly these lengths, and the
cause will not be obvious. Either document it at the protocol boundary or drop the
optimisation; one byte per 254 is not worth a confusing afternoon.

---

## `App/control/pid.{h,c}`

### Purpose

The regulator every stage of the cascade is built from. Currently two instances (Id
and Iq in `currentloop.c`); the velocity, position and impedance stages will each add
one.

### Place in the architecture

A leaf — `pid.h` includes nothing at all, not even `<stdint.h>`, because it uses only
`float`. Called by `currentloop.c`.

### Line by line — `pid.h`

**L49–61, `pid_t`** — kp, ki, kd, `sample_time_s`, the two output limits, the
integrator, and `previous_measurement`. Two fields deserve note.

`integrator` is a *separate addressable field*, and the header's long opening comment
explains why this module exists rather than wrapping CMSIS-DSP's
`arm_pid_instance_f32`: CMSIS folds the integral into the coefficients of a recursive
difference equation, so there is no single value to clamp or freeze when the output
saturates. A cascaded FOC loop saturates by design — a current loop pinned at the
phase limit, a velocity loop pinned at the current limit below it — so anti-windup is
not optional, and the representation has to support it. This is a case where the
justification for not using the library is sound and worth having written down.

`previous_measurement`, not `previous_error`, because the derivative is taken on the
measurement. A setpoint step would otherwise appear as an infinite rate of change and
produce a kick that has nothing to do with the plant.

### Line by line — `pid.c`

**L3–18 `pid_init`** — plain assignment of all six parameters, then `pid_reset`.
Calling `pid_reset` rather than zeroing the two state fields inline means there is one
definition of "what reset means", so the two cannot drift apart.

**L20–24 `pid_reset`** — zeroes the integrator and the derivative history, leaving
gains and limits alone. The split matters: retuning gains mid-flight should not dump
the integrator, and re-enabling a stage should not change its tuning.

**L26–31 `pid_set_gains`**, **L33–37 `pid_set_output_limits`** — the two halves of
that split, as separate entry points.

**L41 `float error = setpoint - measurement;`**

**L43 `float proportional = p->kp * error;`**

**L47–48** —

```c
float derivative = -p->kd * (measurement - p->previous_measurement)
                   / p->sample_time_s;
```

The leading minus is the derivative-on-measurement form: `d(error)/dt` is
`-d(measurement)/dt` when the setpoint is constant, so the sign is carried here rather
than by differencing in the other order. Writing `(previous - measurement)` would be
equivalent and marginally cheaper, but reads as a typo.

**L49** — `previous_measurement` updated immediately, before any early return could
skip it. There is no early return today, but placing the update next to its read is
what keeps that true.

**L54–55 `integrator_candidate`** — the integrator is *not* committed yet. This is the
mechanism of the anti-windup: the candidate is computed, the output evaluated with it,
and only then is it decided whether keeping it is allowed.

**L57** — `output = proportional + integrator_candidate + derivative`.

**L64–76, conditional integration** —

```c
if (output > p->output_max) {
    output = p->output_max;
    if (integrator_candidate < p->integrator) { p->integrator = integrator_candidate; }
} else if (output < p->output_min) {
    output = p->output_min;
    if (integrator_candidate > p->integrator) { p->integrator = integrator_candidate; }
} else {
    p->integrator = integrator_candidate;
}
```

The logic is right. Saturated high, the candidate is kept only if it is moving *down*
— away from the limit; saturated low, only if moving up; unsaturated, always kept. The
integrator can therefore always unwind but never wind further into a limit it has
already hit, which is the behaviour that makes recovery from saturation prompt instead
of producing a large overshoot.

The alternative — back-calculation, feeding the clamping error back through a
tracking gain — is more tunable but needs another gain to choose. Conditional
integration needs none and is the right default.

### Contracts

- `pid_update` must be called at exactly the rate given as `sample_time_s`. Nothing
  enforces this; the header says so and `currentloop.c` must honour it.
- `pid_reset` must be called when a stage is enabled after being idle. See C1.9.

### Concerns

**C1.8 — `sample_time_s` is divided by with no guard.** A `pid_t` that reached
`pid_update` with `sample_time_s == 0` — from a miscomputed rate, or a struct that was
never `pid_init`-ed — divides by zero. On Cortex-M4 with the FPU that yields `inf`
rather than a trap, `inf` propagates through `output`, and the clamp at L64 catches it
(`inf > output_max` is true) so the output is finite. So the immediate blast radius is
small. But `p->integrator` can be left holding `inf` or `NaN`, and `NaN` fails every
comparison, so the anti-windup branches all take the `else` path and the integrator
never recovers. Worth a guard in `pid_init`, or storing the reciprocal.

**C1.9 — nothing outside `pid.c` ever calls `pid_reset`.** The header documents
calling it whenever a stage is enabled after being idle, and gives the reason.
`currentloop.c` never does. Whether that is a real defect depends on whether
`currentloop_start` re-initialises — checked in batch 5, and carried forward as an open
question.

**C1.10 — efficiency, low priority.** `pid_update` runs at 32 kHz and performs one
float division per call. Storing `1.0f / sample_time_s` and `ki * sample_time_s` at
init would replace it with a multiply and remove another. On an M4F a single-precision
divide is ~14 cycles against 1 for a multiply; with two PIDs that is ~26 cycles per
control period out of ~2600 available. Real but not pressing — worth doing when a
third and fourth stage are added, not before.

**C1.11 — no `NULL` guards.** Every other module in this codebase checks. `pid.c` does
not. All current callers pass `&`-of-static, so it cannot fire; the inconsistency is
the finding, not the risk.

---

## `App/control/filter.{h,c}`

### Purpose

A single biquad IIR section wrapping CMSIS-DSP's transposed direct form II, intended
for smoothing a measurement before it feeds a `pid_t`.

### Place in the architecture

A leaf. **Nothing calls it.** See C1.12.

### Line by line — `filter.h`

**L36 `#include "arm_math.h"`** — the only project header in batch 1 with an external
dependency, and the reason `filter_t` cannot be declared without it:
`arm_biquad_cascade_df2T_instance_f32` is embedded by value.

**L43–47, `filter_t`** — the CMSIS instance, `state[2]`, and `coeffs[5]`. The state
array is 2 floats because transposed direct form II keeps two state variables per
stage (as against direct form I's four), which is the form's main attraction along
with better numerical behaviour at single precision. Owning `coeffs` rather than
pointing at the caller's array means the array passed to `filter_init` need not
outlive the call — CMSIS stores the pointer, so without the copy a caller passing a
local would leave a dangling pointer inside the instance.

**L12–24, the coefficient-sign comment** — CMSIS's `a1`/`a2` are the *negative* of what
MATLAB and scipy hand you. This is the single most useful thing in the file and the
most common way to turn a low-pass filter into an oscillator. Keep it wherever this
code ends up.

### Line by line — `filter.c`

**L5–7** — copy five coefficients. A `memcpy` would do; the loop avoids including
`<string.h>` for one call and compiles to the same thing at `-O1`.

**L10** — `arm_biquad_cascade_df2T_init_f32(&f->instance, 1u, f->coeffs, f->state)`.
The `1u` is the stage count: one biquad, not a cascade.

**L12 `filter_reset(f);`** — **redundant.** Read from the CMSIS source in this repo:

```c
void arm_biquad_cascade_df2T_init_f32(...)
{
  S->numStages = numStages;
  S->pCoeffs = pCoeffs;
  memset(pState, 0, (2U * (uint32_t) numStages) * sizeof(float32_t));
  S->pState = pState;
}
```

`init` already zeroes the state buffer. Harmless, and arguably defensible as
not-depending-on-library-internals, but it is a line that does nothing.

**L15–19 `filter_reset`** — zeroes both state variables explicitly rather than calling
`memset`, for the same reason as the coefficient loop.

**L21–28 `filter_process`** — wraps the CMSIS call with block size 1, because one new
measurement arrives per control period. This is the least efficient way to use a
function designed to amortise setup across a block, and it is the only correct way for
a real-time loop that cannot batch.

### Concerns

**C1.12 — the entire module is dead code.** `filter_init`, `filter_reset` and
`filter_process` have zero call sites outside `filter.c`. Nothing in the firmware
filters anything.

This is worth more than a "delete it" note, because there is a measurement problem in
this firmware that a filter is the obvious answer to. The bench work established that
the PWM ripple on this motor is comparable to the DC current — roughly 5.5 A
peak-to-peak on a 3 A bias inside `eind` — and that the current-versus-duty
relationship is non-linear in a way consistent with the current being sampled
somewhere other than the midpoint of that ripple. A biquad on the current reading does
not fix a sampling-phase error (the aliasing has already happened by the time the
sample is taken), so this module is *not* the fix for that. But a velocity estimate
differentiated from a 14-bit encoder at 32 kHz absolutely will need it, and that is
the loop not yet written.

Recommendation: keep, and treat as pre-positioned for the velocity loop rather than
dead. Revisit if the velocity loop lands without using it.

**C1.13 — `filter_reset` inside `filter_init` is a no-op** (above). One line.

---

## Verification method

Everything asserted above was executed, not inferred.

`crc16.c` and `cobs.c` were compiled natively against a test harness
(`scratchpad/b1/t.c`) which:

- computes the published check value and compares against `CRC16_CHECK_VALUE`;
- exercises the `NULL`/zero-length identity of `crc16_update`;
- encodes 65 inputs spanning the 254-byte run boundary and five content patterns,
  comparing against a reference encoder written from the COBS specification rather
  than from `cobs.c`;
- asserts no zero byte appears in any encoded output, and that every output fits
  within `COBS_ENCODED_MAX`;
- round-trips every case;
- confirms rejection of a zero length byte, a run pointing past the end, a zero inside
  a run, and an undersized output buffer;
- evaluates `units_counts_rate_to_mrads` at the overflow point.

Dead-code claims were made by searching the whole tree — `App/` and `Core/Src/`,
`commands.c` included — for each symbol, not by inference from the include graph.

The CMSIS claim in C1.13 was read from
`Middlewares/ARM/CMSIS-DSP/Source/FilteringFunctions/arm_biquad_cascade_df2T_init_f32.c`
in this repository, not from documentation.

The cross-compile (`/tmp/build_verify/build.sh`, 61 sources, `-Wall -Wextra`, linked
against `STM32F411RETX_FLASH.ld`) was clean before and after this batch; no source file
was modified.

---

## Findings carried forward

| ID | Severity | Summary |
|---|---|---|
| C1.1 | **High** (latent) | `units_counts_rate_to_mrads` wraps negative above 1252 rpm; blocks the velocity loop |
| C1.4 | Medium | `rs485_set_transmit` never called — RS-485 transmit cannot work |
| C1.3 | Medium | `apex_board.h` header comment describes helpers the file does not contain |
| C1.6 | Medium | `apex_board.h` aggregate include defeats layering, latent include cycle |
| C1.8 | Medium | `pid_update` divides by `sample_time_s` unguarded; `NaN` would stick in the integrator |
| C1.9 | Open question | `pid_reset` never called externally — resolve against `currentloop.c` in batch 5 |
| C1.7 | Low (interop) | COBS encoder deviates from canonical by one byte at multiples of 254 |
| C1.2 | Low | three `units_*` conversions dead — keep, pending the velocity loop |
| C1.12 | Low | `filter.{c,h}` entirely dead — keep, pending the velocity loop |
| C1.5 | Low | `led1`, `led2_toggle` dead |
| C1.10 | Low | `pid_update` does an avoidable float divide at 32 kHz |
| C1.11 | Low | `pid.c` has no `NULL` guards where every other module does |
| C1.13 | Trivial | `filter_reset` inside `filter_init` is a no-op |
