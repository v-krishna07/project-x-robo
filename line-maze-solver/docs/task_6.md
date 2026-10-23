# Task 6: Electronics, Wiring & Troubleshooting (36 pts)


## 1. Labelled Wiring Diagram (2S LiPo + TB6612FNG + Conflict-Free ESP32 Pinout)

```text
+-----------------------------------------------------------------------------+
| 2S LiPo Battery (7.4 V Nominal / 8.4 V Max, 1000 mAh, 30C)                  |
+-----------------------------------------------------------------------------+
     | (+) Positive Wire                                               | (-) Negative Wire
     v                                                                 |
[ SPST Master Switch ]                                                 |
     |                                                                 |
     +----[ 470 uF 25V Bulk Capacitor ]----+                           |
     |                                     |                           |
     v (7.4 V Motor Power Rail)            v (7.4 V Input)             |
+----------------------------+     +----------------------------+      |
| TB6612FNG MOSFET Driver    |     | LM2596 5V Buck Regulator   |      |
| - VM  <- 7.4 V Motor Power |     | - Steps 7.4 V down to 5.0 V|      |
| - VCC <- 3.3 V Logic Power |     +----------------------------+      |
| - AO1/AO2 -> Left 6V Motor |                   | (5.0 V to Vin)      |
|   (+ 0.1 uF ceramic cap)   |                   v                     |
| - BO1/BO2 -> Right 6V Mot  |     +----------------------------+      |
|   (+ 0.1 uF ceramic cap)   |     | ESP32-WROOM-32             |      |
+----------------------------+     |                            |      |
     ^                             | TB6612FNG Motor Pins:      |      |
     | (3.3 V Control Signals)     | - GPIO 25 -> PWMA (Left)   |      |
     +-----------------------------| - GPIO 26 -> AIN1          |      |
     |                             | - GPIO 27 -> AIN2          |      |
     |                             | - GPIO 14 -> PWMB (Right)  |      |
     |                             | - GPIO 18 -> BIN1          |      |
     |                             | - GPIO 19 -> BIN2          |      |
     |                             | - GPIO 23 -> STBY (Enable) |      |
     |                             |                            |      |
     |   +-------------------+     | 5-Ch IR Array (ADC1 Only): |      |
     |   | 5-Ch IR Sensor    |---->| - GPIO 36 <- IR_S1 (-2)    |      |
     |   | Array (3.3 V)     |---->| - GPIO 39 <- IR_S2 (-1)    |      |
     |   +-------------------+---->| - GPIO 34 <- IR_S3 ( 0)    |      |
     |            |           ---->| - GPIO 35 <- IR_S4 (+1)    |      |
     |            |           ---->| - GPIO 32 <- IR_S5 (+2)    |      |
     |            |                |                            |      |
     |   +-------------------+     | Encoders, Gyro & Red LED:  |      |
     |   | Left/Right Hall   |---->| - GPIO 16, 17 <- Left Enc  |      |
     |   | Encoders & MPU6050|---->| - GPIO 4, 5   <- Right Enc |      |
     |   | Gyro (3.3 V)      |<--->| - GPIO 21, 22 <-> SDA, SCL |      |
     |   +-------------------+     | - GPIO 13     -> Red LED   |      |
     |            |                +----------------------------+      |
     v            v                              |                     v
===============================================================================
  COMMON STAR GROUND (Battery -, TB6612 GND, Buck GND, ESP32 GND, Sensors GND)
===============================================================================
```

---

## 2. Voltage Rails, Common Ground, and Role of the Motor Driver

* **Three Voltage Rails:**
  1. **7.4 V Rail (from 2S LiPo, 8.4 V full):** Feeds high-current power straight to the **TB6612FNG `VM` pin** so heavy motor currents never pass through the ESP32 board.
  2. **5.0 V Rail (from LM2596 Buck Regulator):** Steps the $7.4\text{ V}$ battery down cleanly with ~85% efficiency to feed the ESP32 `Vin` pin without overheating.
  3. **3.3 V Rail (from ESP32 `3V3` pin):** Powers the 5-channel IR sensor array, the TB6612FNG logic side (`VCC`), the two Hall wheel encoders, the MPU6050 gyroscope, and the red finish LED.
* **Why Common Ground is Required:**
  Voltage is always measured as the difference between a signal wire and Ground (`0 V`). By tying the Battery negative wire, TB6612FNG `GND`, Buck Regulator `GND`, ESP32 `GND`, and Sensor `GND` together at a single **Star Ground** point, every chip agrees on what `0 V` (LOW) and `3.3 V` (HIGH) mean. Without a shared ground, PWM and direction signals float and cause random motor behavior.
* **Functional Role of the Motor Driver (TB6612FNG):**
  An ESP32 GPIO pin only outputs $3.3\text{ V}$ at $\sim 20\text{ mA}$ max, while each 6V N20 motor draws $\sim 210\text{ mA}$ when cruising and up to $1,100\text{ mA}$ ($1.1\text{ A}$) when starting or stalling. Connecting a motor directly to an ESP32 pin would destroy the chip instantly. The TB6612FNG uses tiny $3.3\text{ V}$ logic signals (`PWMA/B`, `AIN1/2`, `BIN1/2`, `STBY`) to switch high-current $7.4\text{ V}$ battery power through low-resistance MOSFETs ($0.5\ \Omega$), controlling both speed and forward/reverse direction with almost zero heat loss.

---

## 3. Troubleshooting Analysis: Controller Resetting When Motors Engage (Brownout)

* **Symptom:** The ESP32 boots normally and prints Serial messages while sitting still, but the instant the motors try to start or reverse, the ESP32 reboots over and over.
* **Probable Root Causes:**
  1. **Startup Stall Current Surge:** Stationary DC motors act like a short circuit for the first few milliseconds, pulling full stall current ($\sim 2.2\text{ A}$ total across both N20 motors).
  2. **Voltage Sag Across Wires/Regulator:** This sudden current spike causes the battery voltage to dip briefly. If the $5\text{ V}$ buck input or $3.3\text{ V}$ rail drops below $\sim 2.43\text{ V}$ for even a millisecond, the ESP32's internal **Brownout Detector** trips and forces a reset.
  3. **Weak Power Wiring or Missing Bulk Capacitor:** Running motor power through thin breadboard jumpers creates high resistance ($V_{\text{drop}} = I \times R$).
* **First Diagnostic Check to Perform:**
  Connect a digital multimeter (with Min/Max hold) or an oscilloscope across the ESP32's **`3V3` and `GND` pins**, hold the wheels slightly, and command a sudden motor start. Check if the $3.3\text{ V}$ rail dips below $3.0\text{ V}$ at the moment the motors engage.
* **How to Fix It and Confirm the Fix:**
  1. **Add a Bulk Reservoir Capacitor:** Place a **$470\ \mu\text{F}$ (25V) electrolytic capacitor** directly across the TB6612FNG motor power pins (`VM` and `GND`) to supply the instant startup current surge locally.
  2. **Add Motor Noise Capacitors:** Solder **$0.1\ \mu\text{F}$ ceramic capacitors** across the left and right motor terminals to absorb high-frequency brush noise.
  3. **Separate Power Tracks:** Run thick 16–20 AWG wires from the 2S LiPo battery to the TB6612FNG `VM` pin, separate from the ESP32 logic wires.
  4. **Confirming the Fix:** Re-measure the `3V3` pin during a full-speed motor start and verify that the voltage stays steady above $3.25\text{ V}$ and the ESP32 no longer resets (`Serial` loop counter increments continuously without printing boot messages).