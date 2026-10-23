# Task 7: Wireless Sensor Diagnostics (39 pts)

**Scenario:** Small bumps on the maze floor cause one or more IR sensors to give glitchy readings while driving. We need to spot which sensor is faulty in real time without stopping or touching the robot.

---

## 1. Wireless Protocol Selection: Why We Pick BLE (or Bluetooth Serial) Over Wi-Fi

The ESP32 supports **Wi-Fi**, **Classic Bluetooth (SPP)**, and **Bluetooth Low Energy (BLE)**. We choose **BLE (UART service)** for three practical reasons:

| Criterion | BLE (Selected) | Classic Bluetooth (SPP) | Wi-Fi (Rejected) |
| :--- | :--- | :--- | :--- |
| **Hardware Pin Conflict (`GPIO 25`)** | **Can be read safely** between short BLE packets (or on `ADC1` pins). | Heavy continuous radio usage. | **Breaks `IR_S5` (`GPIO 25`)!** Although Appendix A calls `GPIO 25` `ADC1_CH8`, on real ESP32 hardware `GPIO 25` is on **`ADC2`**, which is completely disabled whenever Wi-Fi is active. |
| **Power Draw** | **Very Low (~12–20 mA)** | Medium (~45 mA) | **High (~180–240 mA)** — heats up the 3.3V regulator. |
| **Latency & Setup** | **~5–10 ms**, connects straight to phone/PC with no router. | ~10–15 ms, easy Serial stream. | Needs a Wi-Fi router/hotspot and TCP/UDP socket setup. |

*(Competition Rule Note: Official Techfest Meshmerize rules forbid active Bluetooth/Wi-Fi during judged runs. We keep this wireless telemetry inside a `DEBUG_MODE` switch used only during practice runs.)*

---

## 2. System Architecture & Stage-by-Stage Data Flow

```text
+-----------------------+     +------------------------+     +-------------------------+
| Stage 1: Sensor Read  |     | Stage 2: Frame Pack    |     | Stage 3: Ring Buffer    |
| Read IR_S1..IR_S5 via | --> | Format CSV text line:  | --> | Push string into memory |
| analogRead() (0..4095)|     | "t_ms,s1,s2,s3,s4,s5\n"|     | queue so PID never waits|
+-----------------------+     +------------------------+     +-------------------------+
                                                                          |
                                                                          v
+-----------------------+     +------------------------+     +-------------------------+
| Stage 6: Fault Check  |     | Stage 5: Live Plotter  |     | Stage 4: BLE Transmit   |
| Spot single-channel   | <-- | Phone/PC plots 5 lines | <-- | ESP32 BLE UART sends    |
| spikes vs smooth move |     | & logs CSV to file     |     | packet over air         |
+-----------------------+     +------------------------+     +-------------------------+
```

### Stage-by-Stage Explanation:
1. **Stage 1 (Sensor Read):** Every $10\text{ ms}$ ($100\text{ Hz}$), the ESP32 reads all 5 IR sensors (`s1..s5`, values `0..4095`).
2. **Stage 2 (CSV Frame Formatting):** The ESP32 formats the timestamp and readings into a short comma-separated line:
   `1020,120,140,3850,190,110\n` (`time_ms,s1,s2,s3,s4,s5`).
3. **Stage 3 (Non-Blocking Buffer):** The text line is placed in a small memory buffer so the fast PID motor loop never slows down waiting for wireless transmission.
4. **Stage 4 (Wireless Transmit):** The ESP32 sends the CSV line over BLE UART notifications to a paired phone or laptop.
5. **Stage 5 (Receive, Plot & Log):** A phone running *Serial Bluetooth Terminal* (or a laptop running Arduino Serial Plotter / Python script) draws 5 colored lines in real time and saves the log to a `.csv` file.

---

## 3. How to Tell a Real Line Shift from a Faulty Sensor / Bump

* **What a Normal Line Shift Looks Like:**
  Because the $30\text{ mm}$ line moves smoothly under the sensor bar, readings rise and fall **gradually across neighboring sensors**. For example, as the line moves from center (`S3`) to right (`S4`), `S3` goes down slowly while `S4` goes up smoothly over several loops ($50\text{--}100\text{ ms}$).
* **What a Bump or Loose-Wire Fault Looks Like:**
  A loose pin or bump causes **one single sensor** to suddenly jump to `0` (open wire) or `4095` (short circuit) in a single $10\text{ ms}$ step and snap right back, while its left and right neighbors stay completely steady.
* **Simple Automatic Detection Rule:**
  Flag sensor $i$ as faulty if:
  1. Its reading changes by more than **2500 ADC counts** in a single $10\text{ ms}$ step ($|S_i(t) - S_i(t-1)| > 2500$), **AND**
  2. Its immediate neighbors change by less than **300 ADC counts** in the same step ($|S_{i\pm 1}(t) - S_{i\pm 1}(t-1)| < 300$).
