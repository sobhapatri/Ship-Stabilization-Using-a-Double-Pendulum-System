# ⚓️ Ship Stabilization Using a Double Pendulum System  

---

## 🧠 Project Overview

This project investigates the use of a **double pendulum system** as a stabilizer to mitigate ship motion caused by ocean waves. The approach is grounded in classical mechanics and modern control theory, aiming to restore equilibrium through intelligent feedback mechanisms.

---

## 🔧 Key Objectives

- Model the dynamics of a ship stabilizer based on a double pendulum mechanism.
- Linearize the inherently nonlinear system using small-angle approximations.
- Analyze system **controllability**, **observability**, and **stability** using MATLAB.
- Design and simulate **two controllers**:  
  - Linear Quadratic Regulator (LQR)  
  - Pole Placement (State Feedback)
- Evaluate the dynamic response and performance metrics for both controllers.

---

## 📐 System Modeling

The double pendulum system was simplified using the following assumptions:

- Rods are **massless**.
- **Frictionless pivots**.
- Only **gravitational** and **coupling** forces are considered.
- Small-angle approximations:  
  `sin(θ) ≈ θ` and `cos(θ) ≈ 1`.

The system is represented in **state-space form** and treated as a **Single-Input-Single-Output (SISO)** system.

---

## 💻 Simulation & Results

MATLAB simulations were conducted to compare the performance of LQR and Pole Placement controllers.

| Metric                  | LQR Controller | Pole Placement |
|-------------------------|----------------|----------------|
| Rise Time               | 0.08 s         | 0.09 s         |
| Overshoot               | 1.42%          | 1.58%          |
| Settling Time           | ~∞ (slow)      | 20 s           |
| Steady-State Error      | 0.9681         | 0.7445         |
| Control Effort Required | High           | Low            |

- **LQR**: Faster convergence with more control effort.
- **Pole Placement**: Lower energy usage, higher overshoot, slower settling.

---

## 📽️ Animation

An animated MATLAB simulation of the pendulum’s motion is included and can be viewed here:  
🔗 [Double Pendulum Simulation](https://drive.google.com/file/d/1GfUJmM17u52X6-HfGSz8fUG08kgEYxtH/view?usp=sharing)

---

## 📌 Key Learnings

- Controller selection involves trade-offs between speed, overshoot, and energy use.
- Small-angle approximations help simplify nonlinear systems.
- The project demonstrates real-world applications of classical control theory in marine stability systems.

---
