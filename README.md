# Active Suspension Control of a Quarter Car using GA-Optimized LQR-PID

This repository contains a MATLAB and Simulink simulation project for designing and optimizing an **active suspension control system of a quarter car (2-DOF)**. The system is designed to reduce vehicle vibration caused by uneven road profiles, thereby improving ride comfort while maintaining vehicle stability (road holding).

The controller design integrates **Linear Quadratic Regulator (LQR)** and **Proportional-Integral-Derivative (PID)** control methods using a **Backstepping Integral** transformation technique. The LQR weighting parameters are automatically optimized using a **Genetic Algorithm (GA)**.

---

## 📌 System Description and Parameters

The system is modeled using a standard *quarter car* model with the front suspension parameters of a BMW vehicle:

| Parameter | Symbol | Value | Unit | Description |
| :--- | :--- | :--- | :--- | :--- |
| **Tire Stiffness** | $kk$ | $340$ | kN/m | Tire spring constant |
| **Suspension Stiffness** | $kr$ | $30$ | kN/m | Suspension spring constant |
| **Suspension Damping** | $br$ | $1450$ | Ns/m | Suspension damping coefficient |
| **Sprung Mass** | $mc$ | $408$ | kg | Vehicle body/chassis mass |
| **Unsprung Mass** | $mus$ | $48.3$ | kg | Wheel and axle assembly mass |

### State-Space Representation
The state-space model is defined as:
$$\dot{x} = A x + B u + B_d z_r$$

With the following state variables:
*   $x_1$: Wheel/unsprung mass vertical displacement ($z_{us}$)
*   $x_2$: Wheel/unsprung mass vertical velocity ($\dot{z}_{us}$)
*   $x_3$: Vehicle body/sprung mass vertical displacement ($z_c$)
*   $x_4$: Vehicle body/sprung mass vertical velocity ($\dot{z}_c$)

System inputs:
*   $u$: Active control force from the suspension actuator.
*   $z_r$: Road vertical displacement disturbance.

---

## 🛠️ LQR-PID Control Structure with Backstepping Integral

1.  **System Augmentation**: The system is augmented to accommodate an integrator structure for PID control. The state-space matrices are expanded from 4-state to 6-state ($A_{aug}$ and $B_{aug}$).
2.  **LQR to PID Gain Transformation**: The LQR gain matrix ($K$) is mapped to PID gains ($K_p, K_i, K_d$) through the matrix transformation relation:
    $$K_{hat} = K \cdot \gamma^{\dagger}$$
    Where $\gamma$ is constructed from output matrix $C$ and system matrices $A, B$, and $\gamma^{\dagger}$ is the pseudoinverse of $\gamma$.
3.  **Genetic Algorithm (GA) Optimization**: GA is utilized to search for optimal weights of the diagonal matrices $Q$ and $R$ by minimizing an objective function that combines:
    *   **Integral Square Error (ISE)** of the state deviations.
    *   Penalties for suspension response **Overshoot**.
    *   Penalties for **Settling Time** (vibration recovery time).
    *   Penalties for **Steady-state Error**.

---

## 📂 Project Directory Structure

*   **`CariLQR_PID.slx`**: The Simulink model file representing the quarter car dynamics complete with the active control loop and road profile disturbances.
*   **`FindLQR_PID.m`**: The main MATLAB script that executes GA optimization for the augmented LQR system, automatically runs the Simulink model `CariLQR_PID.slx` for dynamic simulation, and plots the comparisons between baseline and optimized controllers.
*   **`GA.m`**: A standalone MATLAB script to test GA optimization on the standard 4-state state-space model (without Simulink) and validate transient responses as well as closed-loop eigenvalue stability.
*   **`Report.pdf`**: A comprehensive report document detailing the design and theoretical analysis of the control system.

---

## 🚀 Simulation Guidelines (Step-by-Step)

To run this project in MATLAB, follow these steps:

### 1. Prerequisites
Ensure that your MATLAB installation includes the following toolboxes:
*   **Control System Toolbox** (for functions like `lqr`, `ss`, etc.)
*   **Global Optimization Toolbox** (for `ga` and `optimoptions` functions)
*   **Simulink** (to open and simulate the `.slx` model)

### 2. Running the LQR-PID Simulation (With Simulink)
1.  Open MATLAB and set the *Current Folder* to this project directory.
2.  Run the main script by typing the following command in the Command Window:
    ```matlab
    run('FindLQR_PID.m')
    ```
3.  The Genetic Algorithm optimization will run for a maximum of 1000 generations with a population size of 20 (by default). The fitness cost progress will be displayed iteratively in the command window.
4.  Once completed, MATLAB will automatically trigger the Simulink model `CariLQR_PID.slx` to simulate for 30 seconds.
5.  Comparison plots for **State vs Time**, **Output vs Time**, **Control Input**, and the **Cost Convergence Curve** will be generated automatically.

### 3. Running Standalone Optimization (Without Simulink)
If you wish to test only the numerical calculations and standard transient responses using the MATLAB ODE solver:
1.  Run the `GA.m` script in the Command Window:
    ```matlab
    run('GA.m')
    ```
2.  The program will optimize LQR gains using GA and plot the comparative vertical displacement response of the vehicle body between the optimized and baseline controllers.

---

## 📈 Sample Visualization Results

Upon completing the simulation, the program yields the following graphs to evaluate suspension performance:

1.  **Output Response (Sprung Mass Displacement)**: Demonstrates the capability of the optimized active suspension system in suppressing vertical displacement of the car chassis, significantly reducing vibrations compared to the baseline controller.
2.  **Control Effort**: Illustrates the active force profile deployed by both the optimized and baseline controllers, ensuring that actuator forces operate within physical capacity limits.
3.  **Cost Convergence Curve**: Tracks the reduction of the fitness cost over generations of the Genetic Algorithm until it reaches a stable minimum value.
