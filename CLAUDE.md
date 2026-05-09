# Wall Follower — Project Notes

## Hardware
- MCU: ATmega328P (Arduino Nano), F_CPU = 16MHz
- 3x HC-SR04 ultrasonic sensors (Left, Front, Right) — all on PORTC
- 2x DC motors with L298N (or equivalent)
- 2x Encoders (quadrature)
- 2x IR sensors for turn detection

## Pin Assignments
```
US Left  TRIG → PC2 (A2)    US Left  ECHO → PC3 (A3)
US Right TRIG → PC4 (A4)    US Right ECHO → PC5 (A5)
US Front TRIG → PC0 (A0)    US Front ECHO → PC1 (A1)
Motor L  IN1  → PB0 (D8)    Motor L  IN2  → PB1 (D9)
Motor R  IN3  → PB2 (D10)   Motor R  IN4  → PB3 (D11)
PWM Left ENA  → PD5 (D5)    PWM Right ENB → PD6 (D6)
Enc A Left    → PD7 (D7)    PCINT23
Enc B Left    → PD2 (D2)    direction sense
Enc A Right   → PB5 (D13)   PCINT5
Enc B Right   → PB4 (D12)   PCINT4
IR Left       → PD3 (D3)
IR Right      → PD4 (D4)
UART TX       → PD1 (D1)    9600 baud
```

## Timer Allocation
| Timer  | Mode           | Prescaler | Used for                              |
|--------|----------------|-----------|---------------------------------------|
| Timer0 | Fast PWM       | 64        | Motor PWM — OC0A (PD6), OC0B (PD5)   |
| Timer1 | CTC, OCR1A=1999| 8        | 1ms tick (timer.h) + ultrasonic timing|
| Timer2 | —              | —         | Unused / free                         |

Timer1 gives 0.5µs/tick. `us_now()` in `app/main.c` combines `timer_get_ms()` + `TCNT1`
to produce a microsecond timestamp that correctly handles the CTC reset boundary.

## Known Bug — `timer_delay_us()` in `drivers/timer/timer.c`
`timer_delay_us()` uses unsigned subtraction on TCNT1, which assumes overflow at 65535.
In CTC mode TCNT1 resets at 1999, so delays that cross that boundary exit early.
**Not used in `app/main.c`** so no current impact, but do not call it.

---

## Changes Made This Session

### 1. Fixed Ultrasonic Driver Filenames and Includes
- Renamed `drivers/Ultrasonic/Utrasonic.h` → `Ultrasonic.h` (typo fix)
- Fixed `drivers/Ultrasonic/main.c` include: `"Ultra.h"` → `"Ultrasonic.h"`
- Fixed `drivers/Ultrasonic/Ultrasonic.c` include: `"ultrasonic.h"` → `"Ultrasonic.h"`

### 2. Rewrote `drivers/Ultrasonic/Ultrasonic.c` — Timer1 TCNT1 Polling
Replaced the old busy-wait (`count++; _delay_us(1)`) approach with hardware-timed
polling using TCNT1 (Timer1, prescaler 8, 0.5µs/tick):
- Rising edge: poll ECHO pin HIGH, snapshot TCNT1 → t_start
- Falling edge: poll ECHO pin LOW, snapshot TCNT1 → t_end
- `distance = (t_end - t_start) / 116`

Reduced `ECHO_MAX_TICKS` from 60000 (30ms) to 6000 (~3ms) since walls are always
under 50cm. Timeout returns 0 = open space.

### 3. Multi-Sensor Support in `drivers/Ultrasonic/Ultrasonic.c`
Rewrote driver to accept port/pin parameters instead of hardcoded pins.
Uses AVR register layout (`PORT - 1 = DDR`, `PORT - 2 = PIN`) to derive
DDR and PIN from the PORT pointer.

New API:
```c
void     Ultrasonic_Init(volatile uint8_t *trig_port, uint8_t trig_pin,
                         volatile uint8_t *echo_port, uint8_t echo_pin);
uint16_t Ultrasonic_ReadDistance(volatile uint8_t *trig_port, uint8_t trig_pin,
                                 volatile uint8_t *echo_port, uint8_t echo_pin);
```

Note: this standalone driver is NOT used by `app/main.c` which has its own
inline ultrasonic implementation. The driver is available for isolated testing.

### 4. Deleted `drivers/Ultrasonic/main.c`
Test file removed — `app/main.c` is the real entry point.

### 5. Replaced `us_read_median3` with `us_read_once` in `app/main.c`
`us_read_median3` called `us_read_once` 3 times per sensor, blocking the CPU
for ~21ms per loop iteration (3 sensors × 3 reads × ~3ms echo at 50cm).
Also, the 10ms inter-read wait was insufficient for HC-SR04's required 60ms
cycle time, risking echo contamination between the 3 consecutive reads.

Replaced with single `us_read_once` per sensor per loop iteration:
- Main loop sensor reads: `us_read_median3` → `us_read_once` (left + right)
- Realign state: `us_read_median3` → `us_read_once` (left + right)
- Warm-up calibration: kept `us_read_median3` (one-time, not time-critical)

Result: loop blocking time ~21ms → ~7ms. PID updates ~3× faster.
The existing jump filter (`JUMP_LIMIT = 6`) handles noise spikes that
median filtering was previously catching.

---

## Next Step — PCINT Non-Blocking Ultrasonic Reads

### Problem
Even with single reads, `us_read_once()` blocks the CPU for ~3ms per sensor
(~9ms total for 3 sensors). During this window:
- No PID correction runs
- At 1m/s the robot travels 9mm blind per loop iteration
- Gets worse at higher speeds

### Solution — PCINT Interrupts on ECHO Pins
Attach Pin Change Interrupts (PCINT) to all 3 ECHO pins (PC1, PC3, PC5).

**How it works:**
1. Main loop fires the trigger pulse for a sensor and immediately moves on
2. PCINT ISR fires on ECHO rising edge → records `us_now()` as t_start
3. PCINT ISR fires on ECHO falling edge → computes distance, stores result
4. Main loop reads the latest completed distance — no blocking, no waiting

**Result:**
- PID runs at full CPU speed, completely decoupled from sensor timing
- Sensor updates arrive asynchronously in the background
- No blind windows at any robot speed

**Implementation outline:**
```c
// Volatile result storage — written by ISR, read by main loop
volatile uint16_t g_dist_left  = 0;
volatile uint16_t g_dist_front = 0;
volatile uint16_t g_dist_right = 0;

// PCINT1 ISR handles PC1 (front echo), PC3 (left echo), PC5 (right echo)
ISR(PCINT1_vect) {
    // Check which pin changed and whether rising or falling
    // On rising: record us_now() as t_start for that sensor
    // On falling: distance = (us_now() - t_start) / 58, store in g_dist_*
}
```

**Trade-off:** More complex ISR, need to track which sensor is mid-measurement,
trigger pulses must be staggered to avoid cross-talk between sensors.
