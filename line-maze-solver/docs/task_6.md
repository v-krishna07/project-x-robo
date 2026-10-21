# Task 6: Electronics, Wiring & Troubleshooting (36 pts)

This document shows how all parts of the robot are wired together and explains how to diagnose and fix an ESP32 brownout reset when the motors start.

---

## 1. Labelled Wiring Diagram (Appendix A Pinout)

```text
+-------------------------------------------------------------------------+
| 3S LiPo Battery (11.1 V Nominal / 12.6 V Max)                           |
+-------------------------------------------------------------------------+
     | (+) Positive Wire                                           | (-) Negative Wire
     v                                                             |
[ Master Switch ]                                                  |
     |                                                             |
     +----[ 470 uF 25V Bulk Capacitor ]----+                       |
     |                                     |                       |
     v (11.1 V Motor Power Rail)           v (5.0 V Logic Rail)    |
+--------------------------+       +------------------------+      |
| L298N Motor Driver       |       | 5V Regulator (Buck /   |      |
| - Vs (+12V Motor Input)  |       | L298N 5V Output)       |      |
| - OUT1/OUT2 -> Left Mot  |       +------------------------+      |
|   (+ 0.1 uF ceramic cap) |                   | (5.0 V to Vin)    |
| - OUT3/OUT4 -> Right Mot |                   v                   |
|   (+ 0.1 uF ceramic cap) |       +------------------------+      |
+--------------------------+       | ESP32-WROOM-32         |      |
     ^                             |                        |      |
     | (3.3 V Control Signals)     | Motor Control Pins:    |      |
     +-----------------------------| - GPIO 14 -> ENA (PWM) |      |
     |                             | - GPIO 27 -> IN1       |      |
     |                             | - GPIO 26 -> IN2       |      |
     |                             | - GPIO 13 -> ENB (PWM) |      |
     |                             | - GPIO 12 -> IN3       |      |
     |                             | - GPIO 11 -> IN4       |      |
     |                             |                        |      |
     |   +-------------------+     | Sensor ADC Input Pins: |      |
     |   | 5-Ch IR Sensor    |---->| - GPIO 34 <- IR_S1 (-2)|      |
     |   | Array             |---->| - GPIO 35 <- IR_S2 (-1)|      |
     |   | (Powered by ESP32 |---->| - GPIO 32 <- IR_S3 ( 0)|      |
     |   |  3V3 Pin = 3.3 V) |---->| - GPIO 33 <- IR_S4 (+1)|      |
     |   +-------------------+---->| - GPIO 25 <- IR_S5 (+2)|      |
     |            |                +------------------------+      |
     |            |                            |                   |
     v            v                            v                   v
===========================================================================
  COMMON STAR GROUND (Battery -, Driver GND, ESP32 GND, Sensor GND Joined)
===========================================================================
```

*(Hardware Note: On a real ESP32-WROOM-32 build, `GPIO 11` is wired to internal SPI flash memory and `GPIO 12` is a boot strapping pin. We keep the exact Appendix A pins in code as required, and would move `IN3/IN4` to safe pins `GPIO 18/19` on physical hardware.)*

---

## 2. Voltage Rails, Common Ground, and Role of the Motor Driver

* **Three Voltage Rails:**
  1. **11.1 V Rail (from 3S LiPo):** Feeds high-current power straight to the motor driver (`Vs`) so heavy motor currents stay away from the microcontroller.
  2. **5.0 V Rail (from Regulator):** Steps down battery voltage to feed the ESP32 `Vin` pin.
  3. **3.3 V Rail (from ESP32 `3V3` pin):** Powers the 5-channel IR sensor array and sets the logic level for GPIO signals.
* **Why Common Ground is Required:**
  Voltage is always measured as the difference between a signal wire and Ground (`0 V`). By tying the Battery negative wire, Motor Driver `GND`, Regulator `GND`, ESP32 `GND`, and IR Sensor `GND` together at one point, the motor driver and ESP32 agree on what `0 V` (LOW) and `3.3 V` (HIGH) mean. Without a shared ground, control signals float and motors spin randomly.
* **Functional Role of the Motor Driver:**
  An ESP32 GPIO pin only outputs $3.3\text{ V}$ at $\sim 20\text{ mA}$ max, while a DC motor needs $6\text{--}12\text{ V}$ and draws up to $1,100\text{ mA}$ ($1.1\text{ A}$) when starting. Connecting a motor directly to an ESP32 pin would burn the chip instantly. The motor driver acts like an electronic relay: it uses tiny low-current logic signals from the ESP32 to switch high-current battery power to the motors, and flips polarity (`IN1..IN4`) to spin wheels forward or backward.

---

## 3. Troubleshooting Analysis: Controller Resetting When Motors Engage (Brownout)

* **Symptom:** The ESP32 turns on and prints Serial messages normally while sitting still, but the instant the motors try to start spinning, the ESP32 reboots over and over.
* **Probable Root Causes:**
  1. **Startup Stall Current Surge:** Stationary DC motors act like a short circuit for the first few milliseconds, pulling full stall current ($\sim 2.2\text{ A}$ total for both motors).
  2. **Voltage Sag Across Wires/Regulator:** This sudden current spike causes the battery voltage (and the 5V regulator output) to dip briefly. When the ESP32's $3.3\text{ V}$ rail drops below $\sim 2.43\text{ V}$ for even a millisecond, the ESP32's internal **Brownout Detector** trips and forces a reset to protect the CPU.
  3. **Weak Power Wiring or Missing Bulk Capacitor:** Running motor power through thin breadboard jumpers creates high resistance ($V_{\text{drop}} = I \times R$).
* **First Diagnostic Check to Perform:**
  Set a digital multimeter (with Min/Max hold) or an oscilloscope across the ESP32's **`3V3` and `GND` pins**, hold the wheels slightly, and trigger a motor start. Check if the $3.3\text{ V}$ rail dips below $3.0\text{ V}$ at the moment the motors engage.
* **How to Fix It and Confirm the Fix:**
  1. **Add a Bulk Reservoir Capacitor:** Place a **$470\ \mu\text{F}$ (25V) electrolytic capacitor** right across the motor driver power input (`Vs` and `GND`) to supply the instant current surge locally.
  2. **Add Motor Noise Capacitors:** Solder **$0.1\ \mu\text{F}$ ceramic capacitors** across the left and right motor terminals to stop high-frequency brush noise.
  3. **Separate Power Tracks:** Run dedicated thick wires from the battery to the motor driver, separate from the ESP32 power wires.
  4. **Confirming the Fix:** Re-measure the `3V3` pin during a full-speed motor start and verify that the voltage stays steady above $3.25\text{ V}$ and the ESP32 no longer resets (`Serial` loop counter increments continuously without printing boot messages).
