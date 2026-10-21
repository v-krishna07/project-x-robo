# Task 2 — Toshiba TB6612FNG Motor Driver Datasheet Analysis

### 1. Three Design-Constraining Datasheet Parameters

1. **Motor Supply Voltage ($V_M$):**
   * **Datasheet Figure:** Absolute maximum rating $V_{M(\text{max})} = 15.0\text{ V}$.
   * **Design Impact:** A 3S LiPo battery reaches $12.6\text{ V}$, which is dangerously close to $15\text{ V}$ when motors brake or switch direction. We chose a **2S LiPo ($7.4\text{ V}$ nominal, $8.4\text{ V}$ full)** to safely power our 6V N20 motors without blowing the chip.

2. **Resistance ($R_{\text{on}}$) & Output Current ($I_{\text{out}}$):**
   * **Datasheet Figure:** Total internal switch resistance is typically $0.5\ \Omega$, continuous current is $1.2\text{ A}$ per motor, and peak current is $3.2\text{ A}$.
   * **Design Impact:** Because the resistance is so small ($0.5\ \Omega$), our motors (drawing around $0.21\text{ A}$) only lose about $0.11\text{ V}$. The chip stays cool and does not need any heavy heatsink like older drivers (such as the L298N).

3. **Logic Voltage ($V_{CC}$) & Input High Threshold ($V_{IN(H)}$):**
   * **Datasheet Figure:** Logic voltage range $V_{CC} = 2.7\text{ V}\text{ to }5.5\text{ V}$; high signal threshold $V_{IN(H)} = 0.7 \times V_{CC}$.
   * **Design Impact:** We connect the chip's logic power ($V_{CC}$) to the ESP32 **`3V3` pin ($3.3\text{ V}$)**. This sets the required "HIGH" level to $0.7 \times 3.3\text{ V} = 2.31\text{ V}$. The ESP32's $3.3\text{ V}$ control pins easily exceed this, ensuring reliable motor control.

---

### 2. Damaging Condition and Design Protection

* **Damaging Condition:** **Voltage spike above $15.0\text{ V}$ on $V_M$ or motor stall current above $1.2\text{ A}$.**
  When moving motors are suddenly stopped or reversed, they send energy backward. This can spike the voltage above $15.0\text{ V}$ and permanently burn the driver's internal switches.
* **How the Design Avoids It:**
  1. **Safe Voltage Margin:** Using an $8.4\text{ V}$ max battery leaves a large $6.6\text{ V}$ safety gap below the $15.0\text{ V}$ limit.
  2. **Capacitors for Protection:** A $470\ \mu\text{F}$ capacitor across power and ground absorbs reverse voltage spikes, and small $0.1\ \mu\text{F}$ capacitors on the motor terminals filter electrical noise.
