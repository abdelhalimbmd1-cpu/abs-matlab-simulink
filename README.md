# Anti-lock Braking System (ABS) Simulation Suite in MATLAB/Simulink

An autonomous, end-to-end engineering simulation, controller benchmark, and validation suite for Anti-lock Braking Systems (ABS), developed in MATLAB R2025a and Simulink.

---

## Quick Start

To run the entire suite from a clean MATLAB workspace:

```matlab
% In MATLAB command window:
cd('C:\Users\ZORO\.gemini\antigravity\scratch\abs_project');
run_all
```

`run_all.m` executes the complete project workflow end-to-end:
1. Configures paths and sets software rendering (`painters`) to avoid headless graphics device timeouts.
2. Initializes the parameter database (`params/init_params.m`).
3. Evaluates and plots the Pacejka Magic Formula tire curves across 4 road surfaces.
4. Programmatically generates and compiles `models/abs_quarter_car.slx` and `models/abs_full_car.slx`.
5. Executes the 7-domain automated test suite (`tests/run_all_tests.m`).
6. Executes the 95-simulation batch matrix across 19 scenarios and 5 controllers (`scenarios/batch_runner.m`).
7. Exports publication figures (both `.png` and `.fig` formats) and summary tables to `results/`.
8. Displays the final executive performance summary table.

---

## Requirements

- **Environment:** MATLAB R2025a (or newer)
- **Base Products:**
  - Simulink
  - Stateflow
  - Control System Toolbox
- **Operating System:** Windows 10/11, Linux, or macOS.

---

## Project Structure

```
abs_project/
├── params/
│   └── init_params.m              # Vehicle, tire, brake actuator, sensor, and controller parameters
├── models/
│   ├── pacejka_tire.m             # Calibrated Pacejka Magic Formula friction and force engine
│   ├── sim_quarter_car.m          # High-fidelity quarter-car simulation engine (with lag & delay)
│   ├── sim_full_car.m             # 4-wheel full vehicle model (pitch load transfer & split-mu)
│   ├── wheel_speed_sensor.m       # 48-tooth encoder sensor with pulse quantization and noise
│   ├── build_quarter_car_model.m  # Programmatic generator for abs_quarter_car.slx
│   ├── build_full_car_model.m     # Programmatic generator for abs_full_car.slx
│   ├── abs_quarter_car.slx        # Simulink quarter-car ABS model
│   └── abs_full_car.slx           # Simulink 4-wheel vehicle ABS model
├── controllers/
│   ├── controller_rule_based.m    # Industrial Bosch 3-phase threshold ABS (BUILD/HOLD/DUMP)
│   ├── controller_pid.m           # Discrete velocity-form incremental PID slip controller
│   ├── controller_smc.m           # Continuous Super-Twisting 2nd-order sliding mode controller
│   ├── controller_adaptive.m      # Adaptive target-slip controller with real-time mu estimator
│   └── estimate_reference_speed.m # Vehicle reference speed estimator from slipping wheel speeds
├── scenarios/
│   ├── scenario_definitions.m     # 19 operational test scenarios (speeds, surfaces, drivers)
│   └── batch_runner.m             # Automated batch sweep runner (95 simulations)
├── analysis/
│   ├── compute_metrics.m          # Standardized metric computation (stopping distance, decel, slip, chatter)
│   ├── generate_phase6_figures.m  # Publication figure generator
│   └── plot_tire_curves.m         # Pacejka curve plotting utility
├── tests/
│   ├── test_phase1_baseline.m     # Quarter-car baseline and open-loop lockup test
│   ├── test_phase2_controllers.m  # Comparative controller benchmark test
│   ├── test_phase3_full_car.m     # 4-wheel dynamics, pitch load transfer, and split-mu test
│   ├── test_phase4_robustness.m   # Speed estimator accuracy and parameter robustness sweeps
│   └── run_all_tests.m            # Master 7-domain automated test suite
├── results/                       # Generated figures (.png + .fig), .mat traces, and CSV summary
├── report/
│   └── REPORT.md                  # Comprehensive engineering and theoretical technical report
├── LOG.md                         # Detailed step-by-step engineering audit log
├── README.md                      # Project documentation and user guide
└── run_all.m                      # Master execution script
```

---

## Model Formulations

### 1. Quarter-Car Dynamics
- Equations of motion:
  $$m_q \frac{dv}{dt} = -F_x, \quad J_w \frac{d\omega}{dt} = -T_b + F_x R$$
- Slip ratio: $\lambda = (v - \omega R) / \max(v, \epsilon_v)$
- Tire force: $F_x = \mu(\lambda) \cdot F_z$ via Pacejka Magic Formula.

### 2. Brake Actuator Dynamics
- Actuator torque: $T_b = K_b \cdot P_{act}$ ($K_b = 1.8 \times 10^{-4}\text{ N}\cdot\text{m/Pa}$).
- Fluid transport delay ($5\text{ ms}$), first-order compressibility lag ($\tau = 25\text{ ms}$), asymmetric rate limits ($800\text{ bar/s}$ build, $1200\text{ bar/s}$ dump), and pressure saturation $[0, 150\text{ bar}]$.

### 3. Full 4-Wheel Vehicle & Split-mu Stability
- Longitudinal acceleration: $m_{total} \dot{v} = -\sum_{i=1}^4 F_{x,i}$.
- Pitch load transfer:
  $$F_{z,front} = \frac{m g b - m a_x h}{L}, \quad F_{z,rear} = \frac{m g a + m a_x h}{L}$$
- Split-$\mu$ yaw dynamics ($I_z \ddot{\psi} = M_z$) mitigated via:
  - **Rear Axle Select-Low:** $P_{RL} = P_{RR} = \min(P_{RL}, P_{RR})$.
  - **Front Axle Yaw Moment Limitation (YML):** $|P_{FL} - P_{FR}| \le 35\text{ bar}$.

---

## Controllers Summary

| Controller | Algorithm | Key Characteristics | Dry 100 km/h Stopping Dist |
| :--- | :--- | :--- | :---: |
| **No-ABS** | Open Loop | Complete wheel lockup ($\lambda = 1.0$), sliding friction | 54.91 m |
| **Rule-Based** | Bosch 3-Phase | Threshold FSM (BUILD/HOLD/DUMP), proportional dump, ramped build | 48.76 m (-11.2%) |
| **PID Slip** | Incremental Velocity Form | Filtered derivative ($N=60$), rate-limited, anti-windup | 46.56 m (-15.2%) |
| **SMC** | Continuous Super-Twisting | Boundary layer ($\Phi = 0.035$) chattering elimination | 47.35 m (-13.8%) |
| **Adaptive** | Dynamic Friction Estimator | Real-time $\hat{\mu}$ estimation, adaptive target slip $\lambda^*(\hat{\mu})$ | **44.11 m (-19.7%)** |

---

## Automated Validation Test Suite

Run all validation tests at any time via:

```matlab
cd('C:\Users\ZORO\.gemini\antigravity\scratch\abs_project\tests');
run_all_tests
```

All 7 test suites pass automatically:
- `phase1`: Baseline quarter-car wheel lockup verified ($\lambda = 1.000$).
- `phase2`: Controller benchmark and chatter metrics verified.
- `phase3`: Dynamic pitch load transfer (+30.5% front, -38.4% rear) and split-$\mu$ yaw reduction (61.7%) verified.
- `phase4`: Speed estimator tracking error ($0.72\text{ m/s}$) and parameter sweeps verified.
- `energy_sanity`: Work dissipated balances initial kinetic energy within 2.9% ($154.2\text{ kJ}$ vs $158.8\text{ kJ}$).
- `textbook_benchmarks`: Deceleration matches automotive benchmarks ($9.10\text{ m/s}^2$ on dry, $7.11\text{ m/s}^2$ on wet).
- `simulink`: Programmatic compilation and simulation of `abs_quarter_car.slx` and `abs_full_car.slx` verified.
