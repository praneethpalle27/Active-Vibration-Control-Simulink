# Active Vibration Control System (Simulink & MATLAB)

This project simulates a **2-Degree-of-Freedom (2-DOF) Mass-Spring-Damper system** and demonstrates the design and implementation of an **Active PID (Proportional-Integral-Derivative) Controller** to suppress unwanted vibrations. The entire system is modeled and simulated within **Simulink**, with parameters defined in MATLAB.

The core objective is to showcase the principles of dynamic system modeling, feedback control design, and controller tuning for improved system performance.

## Features

* **2-DOF Mechanical System Model:** Accurate Simulink block-diagram representation of a coupled mass-spring-damper system based on Newton's Laws.
* **Active PID Control:** Implementation of a PID controller to generate a control force that actively dampens oscillations and eliminates steady-state error.
* **Comprehensive Simulation:** Observe the system's response to external disturbances (e.g., step force) under both uncontrolled (passive) and actively controlled conditions.
* **Real-time Visualization:** Utilize Simulink Scopes to display displacement, velocity, and the generated control force over time.
* **Parameter-driven:** System and controller parameters are easily adjustable via the MATLAB workspace.

## How It Works

The project is structured into two main parts:

1.  **Passive System (The Plant):**
    * The 2-DOF mass-spring-damper equations of motion are translated into a Simulink block diagram.
    * This involves `Sum` blocks for forces, `Gain` blocks for mass, spring, and damping coefficients, and `Integrator` blocks to derive velocity and displacement from acceleration.
    * The system's inherent underdamped oscillatory behavior is observed when subjected to initial displacement or external forces.

2.  **Active PID Control Loop:**
    * A `PID Controller` block is introduced to generate a control force based on the error between a desired setpoint (e.g., zero displacement) and the actual displacement of `Mass 1`.
    * This control force is fed back into the `Sum` block for `Mass 1`'s acceleration, effectively creating a closed-loop system.
    * The PID gains (P, I, D) are tuned to optimize the system's response, leading to rapid suppression of oscillations and elimination of steady-state error.

## 🛠️ Technologies Used

* **MATLAB:** For defining system parameters and running the simulation environment.
* **Simulink:** For graphical modeling, simulation of dynamic systems, and control system design.

## 🚀 Getting Started

### Prerequisites

* MATLAB with Simulink.

### Installation

1.  Clone this repository or download the `.zip` file.
2.  Unzip the contents to a folder on your computer.

### How to Run

1.  Open **MATLAB**.
2.  Navigate to the folder containing the project files (`ActiveVibrationControl.slx` and `dof2.m` - or whatever your parameter script is named) in MATLAB's "Current Folder" browser.
3.  **Run the `dof2.m` script** (just type `dof2` and press Enter in the Command Window). This will load all necessary variables into the MATLAB workspace.
4.  Open the `ActiveVibrationControl.slx` file in Simulink.
5.  Optionally, double-click the `PID Controller` block and adjust the P, I, D gains (e.g., P=100, I=5, D=10 are good final values).
6.  Click the "Run" button (green triangle) in the Simulink toolbar.
7.  Double-click the `Scope` blocks (e.g., `x1 Scope`, `Control Force Scope`) to view the simulation results.

## 📈 PID Tuning Process & Results

This section illustrates the step-by-step tuning of the PID controller and its impact on the system's vibration response, using the parameters:
`m1 = 20`, `m2 = 20`, `k1 = 16`, `k2 = 10`, `c1 = 10`, `c2 = 10`, `f = 10`.

### 1. Passive System (Untuned Baseline)

* **PID Gains:** P=1, I=0, D=0
* **Observation:** Shows the inherent underdamped oscillations of the passive system with significant amplitude and slow decay. This is the baseline performance.

![Passive System Response](passive_system_response.png)
---

### 2. P-Tuning (P=100, I=0, D=0)

* **PID Gains:** P=100, I=0, D=0
* **Observation:** The system responds faster and shows a reduction in oscillation amplitude and steady-state error compared to the passive system. However, the oscillatory nature persists, and it still takes time to settle. The control force is significantly larger.

![P-Tuned System Response](p_tuned_response.png)

---

### 3. PD-Tuning (P=100, I=0, D=10)

* **PID Gains:** P=100, I=0, D=10
* **Observation:** The Derivative (D) gain dramatically suppresses the oscillations and reduces overshoot. The system now settles very quickly (within a few seconds) to a steady-state value, though a small offset may still be present. The control force shows rapid initial action to dampen the motion.

![PD-Tuned System Response](pd_tuned_response.png)

---

### 4. PID-Tuning (P=100, I=5, D=10)

* **PID Gains:** P=100, I=5, D=10
* **Observation:** The Integral (I) gain effectively eliminates the remaining steady-state error, bringing the displacement precisely back to zero (or your setpoint). The system achieves optimal performance with fast damping and accurate tracking. The control force settles to a constant value, counteracting the continuous external force.

![PID-Tuned System Response](pid_tuned_response.png)

---

## 📈 Future Enhancements

* **Controller Comparison:** Implement and compare the performance of other advanced control strategies (e.g., LQR, Sliding Mode Control) against the PID controller.
* **Robustness Analysis:** Introduce uncertainties or disturbances to system parameters and analyze the controller's robustness.
* **Graphical User Interface (GUI):** Develop a MATLAB App Designer GUI to allow real-time adjustment of system and controller parameters, and interactive visualization.
* **Hardware-in-the-Loop (HIL) Simulation:** Potentially integrate the model with physical hardware for real-world testing.

## 👤 Author
Praneeth Krishna Palle
