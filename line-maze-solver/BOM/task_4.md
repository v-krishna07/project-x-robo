# Task 4: Bill of Materials (BOM) Preparation (27 pts)

This document lists all parts needed to build the LMS / Meshmerize robot well within the **₹10,000** budget limit.

* **Total Cost (with Shipping + 10% Contingency):** ₹5,638
* **Unspent Budget Reserve:** ₹4,362
* **Size & Safety Compliance:** Fits inside $220 \times 220 \times 220\text{ mm}$ and runs below the $24\text{ V}$ limit.

---

## 1. Master BOM Table

| # | Item & Exact Model | Qty | Unit (₹) | Subtotal (₹) | Running Total (₹) | Supplier Search (`VERIFY`) | Key Specifications | Selection Rationale |
| :- | :--- | :-: | --: | --: | --: | :--- | :--- | :--- |
| 1 | **Controller:** Espressif ESP32-WROOM-32 (30-Pin Dev Board) | 1 | 420 | 420 | 420 | `VERIFY`: search `"ESP32 30 pin development board CP2102"` on `robu.in` | Dual-core 240 MHz, 3.3V logic, 12-bit ADC (`0..4095`), Wi-Fi/BLE | Required by Appendix A; fast enough for 100 Hz PID and maze memory. |
| 2 | **Line Sensor:** 5-Channel Analog IR Reflectance Array (TCRT5000) | 1 | 280 | 280 | 700 | `VERIFY`: search `"5 channel analog IR line sensor array"` on `robu.in` | 5x analog outputs (0–3.3V), ~12 mm spacing | Gives smooth readings across the 30 mm line for weighted centroid math. |
| 3 | **Motors + Encoders:** N20 6V 400 RPM Metal Gear Motor with Encoder | 2 | 620 | 1,240 | 1,940 | `VERIFY`: search `"N20 6V 400 RPM metal gear motor with encoder"` on `robu.in` | 6V nominal, 400 RPM, 0.21A run, 1.1A stall, 350 ticks/rev | Lightweight metal gears give crisp turns; encoders keep straight driving even. |
| 4 | **Motor Driver:** Toshiba TB6612FNG Dual MOSFET Driver (plus L298N spare for Appendix A bench test) | 1 | 195 | 195 | 2,135 | `VERIFY`: search `"TB6612FNG dual motor driver module"` on `robu.in` | 1.2A cont. / 3.2A peak per channel, $0.5\ \Omega$ resistance | Runs cool with only ~0.3V drop (vs ~2V drop on L298N). |
| 5 | **Gyroscope:** MPU6050 3-Axis Gyro + Accelerometer Module | 1 | 140 | 140 | 2,275 | `VERIFY`: search `"MPU6050 6 DOF gyro module"` on `robu.in` | I2C interface, 3.3V, $\pm 2000^\circ\text{/s}$ yaw rate | Measures exact 90° and 180° turns even if battery voltage changes. |
| 6 | **Battery Pack:** Orange 7.4V 1000 mAh 30C 2S LiPo (or 11.1V 3S for L298N setup) | 1 | 890 | 890 | 3,165 | `VERIFY`: search `"Orange 7.4V 1000mAh 30C 2S LiPo"` on `robu.in` | 7.4V nominal (8.4V full), 1000 mAh, 30C (30A max), ~58 g | High 30C current rating prevents voltage sag when motors start. |
| 7 | **Charger:** iMAX B3 Pro 2S/3S LiPo Balance Charger | 1 | 650 | 650 | 3,815 | `VERIFY`: search `"B3 Pro 2S 3S LiPo balance charger"` on `robu.in` | 2S/3S balance charger, 800 mA per cell | Charges all LiPo cells evenly and safely. |
| 8 | **5V Buck Regulator:** LM2596 DC-DC Step-Down Module | 1 | 85 | 85 | 3,900 | `VERIFY`: search `"LM2596 DC-DC buck converter module"` on `electronicscomp.com` | Input 4.5–28V, output tuned to 5.0V, 2A max | Feeds clean 5V power to ESP32 `Vin` without overheating. |
| 9 | **Chassis & Mounts:** 2WD Robot Base Plate + 2x N20 Brackets | 1 | 380 | 380 | 4,280 | `VERIFY`: search `"N20 motor bracket and 2WD robot chassis"` on `robu.in` | $150 \times 130\text{ mm}$ plate with mounting slots | Compact frame fits easily inside the $220\text{ mm}$ cube limit. |
| 10 | **Wheels:** 42 mm Silicone Grip Wheels for N20 D-Shaft | 2 | 110 | 220 | 4,500 | `VERIFY`: search `"42mm N20 rubber wheel"` on `robu.in` | 42 mm diameter, high-grip silicone tread | Stops wheel slipping during fast turns and stops. |
| 11 | **Front Caster:** 15 mm Mini Metal Ball Caster | 1 | 75 | 75 | 4,575 | `VERIFY`: search `"mini metal ball caster wheel"` on `robu.in` | Steel ball bearing, 15 mm height | Supports front nose and keeps IR array 3 mm above floor. |
| 12 | **Main Switch:** SPST 2-Pin Rocker Switch (10A) | 1 | 25 | 25 | 4,600 | `VERIFY`: search `"SPST 2 pin rocker switch"` on `electronicscomp.com` | 10A rating at 12V DC | Single master switch required by competition rules. |
| 13 | **Finish LED:** 5 mm Red LED + $100\ \Omega$ Resistor | 1 | 20 | 20 | 4,620 | `VERIFY`: search `"5mm red LED and 100 ohm resistor"` on `electronicscomp.com` | Red LED, ~13 mA current from 3.3V pin | Glows at the $400 \times 400\text{ mm}$ END box (5 points). |
| 14 | **Capacitor Kit:** 1x $470\ \mu\text{F}$ 25V Electrolytic + 4x $0.1\ \mu\text{F}$ Ceramic | 1 | 35 | 35 | 4,655 | `VERIFY`: search `"470uF 25V electrolytic and 0.1uF ceramic capacitor"` on `electronicscomp.com` | Bulk reservoir cap + motor noise filter caps | Stops motor startup surges from resetting the ESP32. |
| 15 | **Wires & Hardware:** XT60 Pigtail, Jumper Wires, M3 Standoffs | 1 | 320 | 320 | 4,975 | `VERIFY`: search `"XT60 male connector jumper wires M3 standoffs"` on `robu.in` | 16 AWG battery wires, Dupont jumpers, spacers | Cleanly separates motor power wires from sensor wires. |
| 16 | **Shipping:** Courier delivery (`robu.in` + `electronicscomp.com`) | 1 | 150 | 150 | 5,125 | Standard domestic shipping | Covers both orders | Included for realistic total cost. |
| 17 | **Contingency (10%):** Buffer for spare motor or sensor | 1 | 513 | 513 | **5,638** | 10% of ₹5,125 | Emergency spare budget | Covers accidental part damage during testing. |

---

## 2. Major Component Justification: TB6612FNG vs. L298N

| Feature | Toshiba TB6612FNG | ST L298N | Why It Matters |
| :--- | :--- | :--- | :--- |
| **Switch Type** | MOSFETs ($0.5\ \Omega$ resistance) | Bipolar Darlington transistors | MOSFETs waste almost no power; transistors get hot. |
| **Voltage Drop** | **~0.2V to 0.4V** | **~1.8V to 3.2V** | TB6612FNG sends full voltage to the wheels instead of wasting it as heat. |
| **Weight & Size** | **~4 g**, tiny board, no heatsink | **~28 g**, bulky metal heatsink | Saving 24 g lets the robot stop and turn faster at 90° corners. |

---

## 3. Power Block Diagram

```text
+-----------------------------------------------------------------------+
| Battery Pack (7.4V 2S LiPo or 11.1V 3S LiPo per Appendix A, 1000 mAh) |
+-----------------------------------------------------------------------+
       | (+) Positive                                            | (-) Negative
       v                                                         |
  [ SPST Master Switch ]                                         |
       |                                                         |
       +--------------------+-----------------------+            |
       |                    |                       |            |
       |              [470 uF Bulk Cap]             |            |
       |                    |                       |            |
       v (Motor Rail)       |                       v            |
+--------------------+      |            +--------------------+  |
| Motor Driver       |      |            | 5V Buck Regulator  |  |
| (Feeds Left/Right  |      |            | (Feeds ESP32 Vin)  |  |
|  N20 Gear Motors)  |      |            +--------------------+  |
+--------------------+      |                       |            |
                            |                       v (5.0V)     |
                            |            +--------------------+  |
                            |            | ESP32-WROOM-32     |  |
                            |            | 3V3 Out ---------->+--+--> 5-Ch IR Array + Red LED
                            |            +--------------------+  |
                            v                       v            v
                     =================================================
                       COMMON STAR GROUND (All GNDs Tied Together)
                     =================================================
```
