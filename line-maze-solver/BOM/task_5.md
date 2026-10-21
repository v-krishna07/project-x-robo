# Task 5: Power Budget & Battery Selection (30 pts)

This document shows how much current every part on our robot draws, how long the battery lasts on a single charge, and why we picked a **LiPo pouch battery** over **18650 Li-ion cells**.

---

## 1. Current Draw Estimation Table (Idle, Running, and Worst-Case Stall)

Motors pull power directly from the battery rail, while the ESP32 and sensors run on the $5\text{ V} / 3.3\text{ V}$ logic rail. To be safe, we sum the worst-case current drawn from the battery pack:

| Component | Voltage Rail | Idle Current (mA) | Typical Running Current (mA) | Peak / Stall Current (mA) | Notes |
| :--- | :--- | --: | --: | --: | :--- |
| **ESP32-WROOM-32** | 5V $\rightarrow$ 3.3V | 80 | 95 | 240 | 80 mA idle; 240 mA peak only when wireless debug is ON. |
| **5-Channel IR Sensor Array** | 3.3V | 75 | 75 | 75 | 5 infrared LEDs $\times 15\text{ mA}$ each = 75 mA constant. |
| **Wheel Encoders + Gyro** | 3.3V | 18 | 18 | 18 | 2x Hall encoders (14 mA) + MPU6050 gyro (4 mA). |
| **Red End-Zone LED** | 3.3V GPIO | 0 | 0 | 13 | Turns ON only at the finish box: $(3.3\text{ V} - 2.0\text{ V}) / 100\ \Omega = 13\text{ mA}$. |
| **Motor Driver Logic** | 3.3V / 5V | 7 | 7 | 12 | Small internal current to run the driver logic gates. |
| **Left N20 Gear Motor** | Battery Rail | 0 | 210 | 1,100 | Draws ~210 mA on flat arena floor; 1,100 mA (1.1 A) if wheel is jammed. |
| **Right N20 Gear Motor** | Battery Rail | 0 | 210 | 1,100 | Same as Left motor (~210 mA running, 1,100 mA stalled). |
| **TOTAL SYSTEM CURRENT** | **Battery Total** | **180 mA** | **615 mA** | **2,558 mA (~2.56 A)** | **Worst-case sum of all parts.** |

### C-Rating Safety Check Against Peak Stall Current
Our **1000 mAh ($1.0\text{ Ah}$) 30C LiPo battery** can safely supply:
$$\text{Max Continuous Current} = 1.0\text{ Ah} \times 30\text{C} = \mathbf{30.0\text{ Amps}}$$
Even if both motors stall at the exact same time ($2.56\text{ A}$ total), the battery only uses **$8.5\%$ of its $30\text{ A}$ limit** ($11.7\times$ safety margin).

---

## 2. Estimated Runtime Calculation

We use the standard battery runtime formula with a **$0.75$ derating factor** (so we never drain the LiPo below 25% charge, which would damage the cells):

$$\text{Runtime (hours)} = \frac{\text{Battery Capacity (mAh)}}{\text{Average Running Current (mA)}} \times 0.75$$

### Stated Assumptions:
1. **Battery Capacity:** $1000\text{ mAh}$ (at $7.4\text{ V}$ nominal for 2S, or $11.1\text{ V}$ nominal for 3S).
2. **Average Load Current:** $615\text{ mA}$ (both motors driving continuously plus ESP32 and all sensors active).
3. **Derating Factor:** $0.75$ (accounts for wire losses, aging, and stopping at $3.5\text{ V}$/cell).

### Step-by-Step Math:
$$\text{Runtime (hours)} = \frac{1000\text{ mAh}}{615\text{ mA}} \times 0.75 = 1.626 \times 0.75 = \mathbf{1.22\text{ hours}}$$

$$\text{Runtime (minutes)} = 1.22\text{ hours} \times 60 = \mathbf{73.2\text{ minutes}}$$

**Conclusion:** A full Meshmerize match takes $5.5\text{ minutes}$ ($3\text{ min}$ Dry Run + $2.5\text{ min}$ Actual Run). A $73.2\text{-minute}$ runtime gives us **over $13\times$ the time needed** on one charge.

---

## 3. Battery Chemistry Comparison: LiPo (Pouch) vs. Li-ion (18650 Cylindrical)

| Feature | LiPo Pouch Pack (`1000 mAh, 30C`) — **Selected** | Li-ion 18650 Pack (`2200 mAh, 2C–3C`) | Why LiPo is Better for Our Maze Robot |
| :--- | :--- | :--- | :--- |
| **Energy Density** | **~150 Wh/kg** (flat rectangular pouch fits neatly on chassis) | **~200 Wh/kg** (round metal cylinders hold more energy per gram) | Li-ion stores more total energy, but 73 minutes from LiPo is already plenty. |
| **C-Rating (Surge Current)** | **25C to 30C ($25\text{--}30\text{ A}$)**, very low internal resistance | **1C to 3C ($2.2\text{--}6.6\text{ A}$)** on budget cells, higher resistance | When motors reverse at $90^\circ$ corners, LiPo voltage stays steady; 18650 holders can dip and reset the ESP32. |
| **Weight** | **~58 g (2S) to ~90 g (3S)** | **~125 g (2S) to ~185 g (3S)** including plastic cell holder | **Saves 65–95 grams.** Lighter robots accelerate, brake, and turn $90^\circ$ corners with much less skidding. |
| **Cost & Reliability** | **~₹890–₹1,250**, comes with soldered XT60 plug | **~₹600–₹950**, uses spring clips | Spring holders can lose contact over arena bumps; a soldered XT60 plug never shakes loose. |

**Final Choice:** We choose the **LiPo Pouch Battery** because its high C-rating prevents ESP32 brownouts during sharp motor reversals, and its low weight lets the robot corner much faster without overshooting the $30\text{ mm}$ white line.
