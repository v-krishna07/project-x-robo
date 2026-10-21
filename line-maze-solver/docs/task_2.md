# Task 2: Motor Driver Datasheet Analysis (18 pts)

We analyze the **Toshiba TB6612FNG** dual motor driver IC (and compare its ratings with the Appendix A **L298N** driver).

---

## 1. Three Datasheet Parameters That Constrain the Design

1. **Motor Supply Voltage ($V_M$):**
   * **Datasheet Spec:** Operating range is $4.5\text{ V}$ to $13.5\text{ V}$, with an absolute maximum rating of $15.0\text{ V}$ (`VERIFY` on `robu.in` / Toshiba datasheet).
   * **How it affected our design:** A fully charged 3S LiPo battery reaches $12.6\text{ V}$, which is too close to the $15.0\text{ V}$ limit when motors brake and create voltage spikes. So in our BOM (`BOM/task_4.md`), we picked a **2S LiPo battery ($7.4\text{ V}$ nominal, $8.4\text{ V}$ max)** for the TB6612FNG build (while the Appendix A L298N build handles up to $46\text{ V}$).

2. **Output Current ($I_{\text{out}}$) & Internal Resistance ($R_{\text{DS(on)}}$):**
   * **Datasheet Spec:** $1.2\text{ A}$ continuous current ($3.2\text{ A}$ peak) per channel, with $0.5\ \Omega$ internal MOSFET resistance.
   * **How it affected our design:** Our N20 motors draw $0.21\text{ A}$ during normal driving and $1.1\text{ A}$ when stalled, which fits safely below the $1.2\text{ A}$ continuous limit. Because the resistance is only $0.5\ \Omega$, the chip loses less than $0.4\text{ V}$ as heat and needs no heavy heatsink.

3. **Logic Input Voltage ($V_{CC}$ & $V_{IN(H)}$):**
   * **Datasheet Spec:** Logic supply $V_{CC} = 2.7\text{ V}$ to $5.5\text{ V}$. A "HIGH" logic signal requires at least $V_{IN(H)} = 0.7 \times V_{CC}$.
   * **How it affected our design:** We power $V_{CC}$ from the ESP32's $3.3\text{ V}$ pin. This makes the required HIGH threshold $0.7 \times 3.3\text{ V} = 2.31\text{ V}$. The ESP32 GPIO pins output $3.3\text{ V}$, so they switch the motor driver directly without needing a level shifter.

---

## 2. One Damaging Condition and How We Avoid It

* **Damaging Condition:** **Back-EMF Voltage Spike Above $15.0\text{ V}$ or Sustained Motor Stall Above $1.2\text{ A}$.**
  When spinning motors are suddenly reversed at a maze dead end, they act like generators and push a voltage spike back into the driver. If that spike passes $15.0\text{ V}$, or if a jammed wheel pulls stall current for too long, the driver burns out.
* **How Our Design Avoids It:**
  1. **Voltage Headroom:** Using an $8.4\text{ V}$ max 2S battery leaves a $6.6\text{ V}$ safety gap below $15.0\text{ V}$.
  2. **Protection Capacitors:** We wire a $470\ \mu\text{F}$ electrolytic capacitor across the motor supply pins to soak up voltage spikes, plus $0.1\ \mu\text{F}$ ceramic capacitors across the motor terminals.
  3. **Software Timeout Stop:** If the robot loses the line or stalls for more than $600\text{ ms}$, the code sets motor PWM to `0` so the motors cannot overheat.
