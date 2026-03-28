# STM32 Watch NFC — Firmware Documentation

## Overview

A custom smartwatch firmware for the **STM32L031G6U6** (ARM Cortex-M0+, 32KB Flash, 8KB RAM). The watch has two display modes, gesture/button wakeup, automatic brightness, a buzzer, and battery monitoring. The "NFC" in the project name suggests planned NFC functionality, but **no NFC code exists in the firmware yet**.

Generated with STM32CubeIDE (HAL-based project). User code lives inside the `/* USER CODE BEGIN/END */` comment blocks.

---

## File Structure

```
Core/
├── Inc/
│   ├── main.h              # GPIO pin defines, exported function prototypes
│   ├── Din_LED.h           # WS2812B LED driver types and API
│   ├── BMA400.h            # BMA400 accelerometer driver API
│   ├── stm32l0xx_hal_conf.h
│   └── stm32l0xx_it.h
└── Src/
    ├── main.c              # All application logic (main loop + callbacks)
    ├── Din_LED.c           # WS2812B GRB bit-pattern encoding
    ├── BMA400.c            # BMA400 I2C driver (orientation change interrupt)
    ├── stm32l0xx_hal_msp.c # Peripheral MSP init/deinit (clock, DMA, GPIO)
    ├── stm32l0xx_it.c      # ISR handlers (EXTI, DMA, SysTick)
    ├── system_stm32l0xx.c  # System clock initialization
    ├── syscalls.c          # Standard library syscalls
    └── sysmem.c            # Heap memory
```

---

## Clock Configuration

- **System clock**: HSI (16 MHz) → PLL ×4 ÷2 = **32 MHz**
- **RTC clock**: **LSE** (32.768 kHz external crystal) — accurate timekeeping in Stop mode

---

## GPIO Pinout

| Pin  | Name        | Direction       | Function                                      |
|------|-------------|-----------------|-----------------------------------------------|
| PA1  | Button_LB   | Input, EXTI↑    | Left-bottom button                            |
| PA2  | Row1        | Output          | Charlieplex row 1                             |
| PA3  | Row2        | Output          | Charlieplex row 2                             |
| PA4  | Row3        | Output          | Charlieplex row 3                             |
| PA5  | Row4        | Output          | Charlieplex row 4                             |
| PA6  | Row5        | Output          | Charlieplex row 5                             |
| PA7  | Row6        | Output          | Charlieplex row 6                             |
| PA8  | Button_LT   | Input, EXTI↑↓   | Left-top button                               |
| PA9  | EN_5V       | Output          | Enables 5 V rail for the WS2812B LED ring     |
| PA10 | Button_R    | Input, EXTI↑    | Right button                                  |
| PA15 | DIN_STM     | TIM2_CH1 AF     | PWM data line for WS2812B LEDs                |
| PB0  | Accel       | Input, EXTI↑    | BMA400 INT1 — orientation-change interrupt    |
| PB1  | BATT        | Analog, ADC_IN9 | Battery voltage measurement                   |
| PB3  | Buzzer      | Output PP fast  | Passive buzzer                                |
| PB6  | I2C1_SCL    | AF OD           | I2C clock (BH1750 + BMA400)                   |
| PB7  | I2C1_SDA    | AF OD           | I2C data  (BH1750 + BMA400)                   |

---

## Peripherals

| Peripheral | Purpose |
|------------|---------|
| **ADC1** (CH9, PB1) | 12-bit battery voltage reading |
| **I2C1** (PB6/PB7) | Communicates with BH1750FVI and BMA400 |
| **RTC** | 24-hour timekeeping, clocked from LSE |
| **TIM2** (PWM, period=60) + **DMA1_CH5** | Generates WS2812B bit-stream on PA15 |
| **TIM21** (free-running) | Microsecond delay (`delay_us`) for buzzer timing |

---

## Display — RGB LED Ring (12 LEDs)

- 12 addressable WS2812B-compatible LEDs arranged in a clock ring
- Driven via **TIM2 PWM + DMA** (circular DMA, memory → peripheral)
- Protocol: T1H = 20 timer counts, T0H = 5 timer counts (period = 60)
- Data format: GRB, MSB first, 24 bits per LED
- The `leds[]` array holds `LEDs` structs (g[8], r[8], b[8]) pre-encoded as PWM compare values
- **Analog clock face**: one LED for hours, one for minutes, brightness controlled by ambient light
- LEDs 0–11 map to positions 12, 1, 2 … 11 o'clock

### LED driving flow
```
turn_spec_LEDs(leds, color_pattern)   // encode colors into PWM compare values
HAL_TIM_PWM_Start_DMA(...)            // stream to LED strip
HAL_Delay(...)                        // hold display on
clear_LEDs(leds)                      // zero out array
HAL_TIM_PWM_Start_DMA(...)            // send reset pulse (low for ≥50 µs)
```

---

## Display — Charlieplex 7-Segment Matrix

- 6 GPIO pins (PA2–PA7) drive a Charlieplexed LED matrix
- Maximum addressable LEDs: 6×5 = 30
- Used to show digital time **HH:MM** and settings text
- `Digital_show(hours, minutes, date)`: iterates look-up tables of `{high_pin, low_pin}` pairs per digit, calling `Charlieplex_Light_LED()` for each segment
- `Digital_OnOff_show(value)`: shows "OFF" (0), "ON" (1), or "ONL" (2) — wrist-wake setting indicator
- All other pins are set to high-impedance (input) between segments to avoid ghost currents

---

## Sensors

### BH1750FVI — Ambient Light Sensor (I2C 0x23)
- Continuous high-resolution mode (1 lx resolution)
- `BH1750_ReadLightLevel()` returns a brightness value 2–252, mapped **logarithmically** from 50–60 000 lux
- This value is used as the alpha (brightness) channel for the RGB LEDs

### BMA400 — Accelerometer (I2C 0x14)
- Configured for **orientation-change interrupt** on INT1 (→ PB0)
- Z-axis only, 100 Hz ODR, ±2g range, 1 Hz lowpass filter
- Threshold: 55 LSB (≈440 mg), duration: 8 × 10ms = 80 ms
- `BMA400_IsFaceToMe()` checks if Z-axis is sufficiently negative (face toward user): threshold 800 raw codes ≈ 0.78 g
- On interrupt: reads XYZ, checks "face-to-me", sets `showingDigital` or `showingLeds`, re-arms reference

---

## Application Logic (main loop)

Each iteration of the main `while(1)` loop:

1. **ADC** reads battery voltage
2. If battery ≤ 2100 (raw): disable all modes, disable accelerometer interrupt
3. If any display mode is active: read RTC time, compute `shifted_hours` (0–11) and `shifted_minutes` (0–11, rounded to 5-min interval)
4. Run the appropriate display block based on flags:
   - `showingLeds` → RGB analog clock display
   - `showingDigital` → Charlieplex digital time display
   - `charging` → Color-coded LED ring showing battery level (green/yellow/orange/red)
   - `changeTime` → Simultaneous analog + digital time-setting UI
   - `changeColor` → Analog display cycling through color themes
5. Turn off 5 V rail, disable DMA clock, enter **STOP mode** (`WFI`)

The watch spends most of its time in STOP mode. It wakes up via EXTI (button press or accelerometer interrupt), at which point `SystemClock_Config()` and `HAL_ResumeTick()` are called in the EXTI callback to restore the PLL.

---

## Button Behavior

| Button | Short press | Long press (≥1 s) | Context action |
|--------|-------------|-------------------|----------------|
| **Button_R** (PA10) | Show RGB LED clock | Enter `changeTime` mode | In `changeTime`: confirm & save to RTC; in `changeColor`: confirm |
| **Button_LT** (PA8) | Show digital clock; increment `numPresses1` | Enter `charging` display mode | In `changeTime`: increment minutes |
| **Button_LB** (PA1) | Show digital clock; increment `numPresses2` | Enter `changeColor` mode | In `changeTime`: increment hours; in `changeColor`: cycle theme |

**Multi-press shortcuts** (detected during `showingDigital` or `showingLeds`):
- Button_R ×2 while in LED mode: toggle sound on/off
- Button_LB ×2 while in digital mode: cycle `wristWake` (0 → 1 → 2 → 0)

---

## Wrist-Wake Modes (wristWake)

| Value | Behavior |
|-------|----------|
| 0 | Wrist-wake disabled; PB0 (Accel EXTI) is de-initialized |
| 1 | Wrist raise → show **digital** time |
| 2 | Wrist raise → show **RGB LED** time |

The BMA400 interrupt is still active in modes 1 and 2; mode 0 physically deinitializes the GPIO.

---

## Color Themes (6 total)

| Index | Hour color | Minute color |
|-------|------------|--------------|
| 0 (default) | Green/cyan `(80,255,70)` | Blue `(20,50,255)` |
| 1 | Red `(255,10,0)` | Yellow `(250,200,0)` |
| 2 | Pink `(255,15,50)` | Purple `(144,10,255)` |
| 3 | Red `(255,10,0)` | Blue `(20,50,255)` |
| 4 | Yellow `(250,200,0)` | Blue `(20,50,255)` |
| 5 | Purple `(144,10,255)` | Green `(0,204,0)` |

Brightness (alpha) is always overridden by the BH1750 reading.

---

## Buzzer

- Passive buzzer on PB3
- Frequency ≈ **2.7 kHz** (185 µs high + 185 µs low, using TIM21-based `delay_us`)
- 10 000 cycles per beep ≈ ~3.7 seconds (note: this is a blocking call)
- Controlled by `soundON` boolean; toggled via double-press of Button_R

---

## Battery Monitoring

- ADC reads PB1 (battery voltage via resistor divider), 12-bit result
- Raw threshold values (not converted to volts — divider ratio unknown):
  - `≤ 2100`: critically low — disables all features
  - `2100–2600`: mapped linearly to 0–12 LEDs for charging indicator

### Charging display LED colors

| LEDs lit | Color |
|----------|-------|
| 10–12 | Green `(0,250,0)` |
| 7–9 | Yellow `(250,200,0)` |
| 4–6 | Orange `(255,100,0)` |
| 1–3 | Red `(255,0,0)` |

LED frame is only re-sent when `chargedLEDnum` changes (`prev_chargedLEDnum = 0xFF` sentinel before the loop). This prevents flicker from repeated DMA restarts every 25 ms.

---

## Power Management

- **Active**: 32 MHz, all peripherals on
- **Sleep**: `HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI)`
  - Low-power voltage regulator
  - DMA clock disabled before sleep, re-enabled in EXTI callback
  - 5 V rail (EN_5V) driven low before sleep
  - RTC keeps running from LSE

---

## RTC Notes

- Time is initialized to 00:00:00 on flash but the RTC is **held in init mode** (paused) until the user explicitly sets the time for the first time via Button_R in `changeTime` mode.
- Date is fixed to 2025-01-01 Monday (date display not implemented in UI, `date` param in `Digital_show` is unused).
- No backup register usage; time resets on power loss.

---

## Interrupt-Driven Input Handling

All user input (buttons and accelerometer) is handled exclusively through **GPIO EXTI interrupts**, not polling. This is the central pattern of the firmware.

### How it works

1. A button press or accelerometer event fires an EXTI line, waking the MCU from STOP mode.
2. The ISR in `stm32l0xx_it.c` calls `HAL_GPIO_EXTI_IRQHandler()` which calls the HAL callback.
3. **`HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)`** in `main.c` is the single entry point for all input events. It:
   - Calls `SystemClock_Config()` and `HAL_ResumeTick()` to restore the PLL after STOP mode
   - Re-enables the DMA clock
   - Sets or clears the boolean state flags that control the main loop

### State flags modified by the callback

| Flag | Set by | Cleared by |
|------|--------|------------|
| `showingLeds` | Button_R, Accel (wristWake≠1) | main loop (after display) |
| `showingDigital` | Button_LT, Button_LB, Accel (wristWake==1) | main loop (after display) |
| `changeTime` | long-press Button_R (polled in main) | Button_R interrupt |
| `changeColor` | long-press Button_LT (polled in main) | Button_R interrupt |
| `colorTheme` | Button_LB interrupt (when `changeColor==true`) | — |
| `numPresses1` | Button_LT, Button_R interrupts | main loop |
| `numPresses2` | Button_LB interrupt | main loop |
| `wrist_move` | Accel interrupt | main loop |

### While-loop exit mechanism

The `while(changeTime)` and `while(changeColor)` loops in `main.c` block the main loop and continuously update the display. They are **exited by the interrupt callback**, not by polling:

- **Button_R interrupt** sets `changeTime = false` or `changeColor = false`
- On the next iteration the while condition is false and execution continues
- The RTC is written to in the same interrupt callback before `changeTime` is cleared

This means no `break` or explicit exit statement is needed inside the while loops — the interrupt modifies the loop condition from outside.

### Long-press detection

Long presses are detected by **polling inside the while loop** in `main.c` (not in the ISR):
```c
while (HAL_GPIO_ReadPin(Button_R_GPIO_Port, Button_R_Pin)) {
    x++;
    if (x >= 100) {           // 100 × 10ms = 1 second
        changeTime = true;
        break;
    }
    HAL_Delay(10);
}
```
This polling only runs while a display mode is already active (inside `showingLeds` / `showingDigital` blocks).

### EXTI line assignments

| EXTI handler | Pins served |
|---|---|
| `EXTI0_1_IRQHandler` | PB0 (Accel), PA1 (Button_LB) |
| `EXTI4_15_IRQHandler` | PA8 (Button_LT), PA10 (Button_R) |

---

## rgb_color struct field order

`rgb_color` fields are `{R, G, B, A}` where A is the alpha/brightness channel overridden by the BH1750 reading. All color literals in the codebase follow this order (e.g. `{255, 10, 0, 250}` = red).

---

## Known Gaps

- **NFC**: Scrapped during development; PCB has NFC hardware but no firmware code exists. Project name was not updated.
- **Battery ADC calibration**: Raw thresholds (2100, 2600) are used with no documented resistor divider ratio, so exact voltage levels are unknown.
- **Charlieplex physical layout**: The mapping between `{high, low}` pin index pairs and physical LED positions is now partially documented — see Charlieplex segment map below.

---

## Charlieplex Segment Map (4-digit display)

The display has 4 digit slots driven by 6 Charlieplex pins (indices 0–5). Each slot uses the 7-segment pairs below (standard a–g labeling):

| Segment | Slot 0 (H-tens) | Slot 1 (H-ones) | Slot 2 (M-tens) | Slot 3 (M-ones) |
|---------|-----------------|-----------------|-----------------|-----------------|
| a (top) | `{1,0}` | `{3,1}` | `{5,2}` | `{1,4}` |
| b (top-right) | `{2,0}` | `{4,1}` | `{0,3}` | `{2,4}` |
| c (bot-right) | `{3,0}` | `{5,1}` | `{1,3}` | `{3,4}` |
| d (bottom) | `{4,0}` | `{0,2}` | `{2,3}` | `{5,4}` |
| e (bot-left) | `{5,0}` | `{1,2}` | `{4,3}` | `{0,5}` |
| f (top-left) | `{0,1}` | `{3,2}` | `{5,3}` | `{1,5}` |
| g (middle) | `{2,1}` | `{4,2}` | `{0,4}` | `{2,5}` |

Display rows in ASCII art map to:
- Row 1 (tops): `{1,0}` `{3,1}` `{5,2}` `{1,4}`
- Row 2 (upper verticals L/R per slot): `{0,1}` `{2,0}` | `{3,2}` `{4,1}` | `{5,3}` `{0,3}` | `{1,5}` `{2,4}`
- Row 3 (middles): `{2,1}` `{4,2}` `{0,4}` `{2,5}`
- Row 4 (lower verticals L/R per slot): `{5,0}` `{3,0}` | `{1,2}` `{5,1}` | `{4,3}` `{1,3}` | `{0,5}` `{3,4}`
- Row 5 (bottoms): `{4,0}` `{0,2}` `{2,3}` `{5,4}`

---

## Simultaneous Button Press (Button_R + Button_LB)

Pressing Button_R and Button_LB together enters/exits the **shooting game**. Implemented in `HAL_GPIO_EXTI_Callback`:

- When one EXTI fires and sees the other button held: toggles `simultaneousRB` (bool), sets `simulGuard = true` to prevent the second EXTI from double-toggling.
- `simulGuard` is reset in the main loop after `while(simultaneousRB)` exits.
- **Guard condition**: simultaneous detection is blocked when `changeTime` or `changeColor` is active (`&& !changeTime && !changeColor`), preventing hardlock where both flags are set simultaneously.
- The `else if (simultaneousRB)` game branch also checks `!changeTime && !changeColor` so Button_R can always exit those modes even if `simultaneousRB` was set.

### Game-specific interrupt flags (volatile)
| Flag | Set by | Consumed by |
|------|--------|-------------|
| `gameMoveUp` | Button_LT EXTI (when `simultaneousRB`) | game loop |
| `gameMoveDown` | Button_LB EXTI (when `simultaneousRB`) | game loop |
| `gameShooting` | Button_R EXTI (when `simultaneousRB && !changeTime && !changeColor`) | game loop |

---

## Shooting Game

Entered via Button_R + Button_LB simultaneous press. Lives in `while(simultaneousRB)` in the main loop.

### Field layout (RGB ring)
- **Left arc** (enemy territory): LEDs 1–5
- **Right arc** (player territory): LEDs 7–11 (player clamped to arc indices 1–5 = LEDs 11,10,9,8,7)
- **Borders**: LED 0 and LED 6 — always purple `{144,10,255}`, game over if any enemy reaches them
- **Player**: starts at LED 9, color blue `{20,50,255}`

### Enemy types (GameEnemy struct: pos, dir, colorIdx, active)
- **RIGHT type** (`dir = -1`): spawns at random LED 2–4, moves toward LED 0
- **LEFT type** (`dir = +1`): spawns at random LED 2–4, moves toward LED 6
- Types alternate predictably (`typeToggle % 2`); colors cycle red/pink (`colorIdx % 2`)
- Max 3 enemies on field (`GAME_MAX_ENEMIES`)
- Spawn position: `(HAL_GetTick() % 3) + 2` — pseudo-random LED 2, 3, or 4

### Timing / difficulty
- Enemy movement tick: starts 3000 ms, reduces by 250 ms per 5 kills, floor at 2000 ms (`game_get_tick_rate()`)
- Spawn timer: every 5000 ms; after 10 kills, two enemies spawn per tick
- Spawn timer and movement timer are independent

### Controls in game
| Button | Action |
|--------|--------|
| Button_LT | Move player toward LED 0 (up) |
| Button_LB | Move player toward LED 6 (down) |
| Button_R | Shoot |
| Button_R + Button_LB | Exit game |

### Shooting
- Target LED: `(rightArc[playerArcIdx] + 6) % 12`
- Sequence: `beep()` → `game_shoot_animation()` → hit check → if hit: deactivate enemy, `game_draw()`, show score on Charlieplex (500 POV iterations), return
- Score: +10 per kill, displayed as `Digital_show(score/100, score%100, 0)`

### Bullet animation (Charlieplex)
Each player position maps to a diagonal line pattern on the 4-digit display:

| Player LED | Arc index | Pairs |
|------------|-----------|-------|
| LED 11 | 1 | `{3,1}{4,1}{5,1}{2,3}` |
| LED 10 | 2 | `{1,0}{3,1}{4,1}{5,1}{2,3}{5,4}` |
| LED 9  | 3 | `{2,1}{4,2}{0,4}{2,5}` (horizontal) |
| LED 8  | 4 | `{5,2}{1,4}{4,1}{5,1}{4,0}{0,2}` |
| LED 7  | 5 | `{5,2}{4,1}{5,1}{0,2}` |

Animation: 300 POV iterations then `Charlieplex_Reset_All()`.

### Game over
Enemy reaches LED 0 or LED 6 → clear LEDs → `beep(); HAL_Delay(150); beep()` → `simultaneousRB = false` (exits game loop back to watch).

### `rightArc` array
```c
static const uint8_t rightArc[7] = {0, 11, 10, 9, 8, 7, 6};
```
Player arc index 0 = LED 0 (border), index 6 = LED 6 (border). Player is clamped to indices 1–5.

---

## Bug fixes applied (2026-03-20)

- **`HAL_GPIO_EXTI_Callback` clock restore condition** (`main.c:1001`): was `!changeTime || !changeColor` (always true, called `SystemClock_Config()` on every interrupt even during I2C-active changeTime/changeColor loops). Fixed to `!changeTime && !changeColor`.
- **Wrong GPIO clock enable for wristWake re-arm** (`main.c:367`): was `__HAL_RCC_GPIOA_CLK_ENABLE()`. Accel INT1 is on PB0 (GPIOB). Fixed to `__HAL_RCC_GPIOB_CLK_ENABLE()`.
- **`decToBinary()` removed** from `Din_LED.c` and `Din_LED.h`: dead code with `int binaryNum[1000]` (4 KB stack allocation) on a device with 8 KB total RAM.

## Bug fixes applied (2026-03-20, session 2)

- **Simultaneous press hardlock**: holding Button_R + Button_LB could set `simultaneousRB = true` while also triggering `changeTime` or `changeColor` via long-press polling. Button_R's `else if (simultaneousRB)` branch would then handle the exit press instead of the `else` branch that clears `changeTime`/`changeColor`. Fixed by adding `&& !changeTime && !changeColor` to both the simultaneous detection condition and the `else if (simultaneousRB)` branch in Button_R's handler.
