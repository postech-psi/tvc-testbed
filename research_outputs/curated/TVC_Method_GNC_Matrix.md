# Method × GNC comparison — all 13 papers

This focused view is part of the [complete TVC Ashby project guide](TVC_Ashby_Project_Guide.md), which also contains project context, all paper summaries and the source audit.

## Methods and their roles

**GNC names the job; PID, MPC, RL and adaptive control name ways of doing a job.** One paper may combine several methods, and two papers called “MPC” may have different architectures.

| Function | Main question | Typical output |
|---|---|---|
| Guidance | Where should the vehicle go, and along what feasible trajectory? | Reference position, velocity, attitude or thrust history |
| Navigation/state estimation | What is the vehicle's current state? | Estimated position, velocity, orientation and angular rate |
| Feedback control | What commands make the state follow the reference? | Desired forces/moments or direct actuator commands |
| Allocation, within control | How should actuators realize those demands? | Motor and gimbal commands |
| Modeling/identification/adaptation | What dynamics, parameters or uncertainty should the system account for? | Identified model, parameter estimates or disturbance compensation |

Navigation feeds guidance and control. Guidance supplies control references. Control and allocation produce actuator commands. An optimizer or learned policy may combine control and allocation, while still relying on a separate estimator and supplied references.

![All 13 studies compared by method and GNC role](tvc_method_gnc_matrix.png)

The text table below preserves the comparison even when image preview is unavailable. Cells describe components used or discussed, not a claim that every component is a new contribution. `?` means insufficiently checked. `—` means no separate guidance contribution was established from the inspected material. An asterisk marks provisional source depth.

| First author / main method | Guidance | Navigation/estimation | Feedback control | Allocation | Model/learning support |
|---|---|---|---|---|---|
| Spannagl (2021): MPC + PID | Optimal trajectory | State EKF and offset filtering | Position MPC; attitude PID | Separate actuator map | Propulsion lag and disturbance model |
| Linsen (2022): optimal control | Optimal trajectory | State filter; disturbance/offset EKF | NMPC tracking/stabilization | Actuator commands within NMPC | Thrust/torque identification |
| Denton (2022): PID | — | Complementary attitude filter | Cascaded feedback | Gimbal and differential RPM | Vehicle modeling |
| Cai (2024): PID | — | Madgwick IMU fusion | Attitude/rate feedback | Thrust steering and differential RPM | Vehicle modeling |
| Chen (2024): nonlinear control | — | IMU/ToF; separate Pixhawk hover setup | Nonlinear feedback and damping | Nonlinear mapping; separate hover implementation | Six-DoF model |
| Denton (2025): identification | — | Complementary attitude filter | PID test baseline | Gimbal and differential RPM | Hover dynamics identification |
| Chih (2024)*: robust PID | ? | ? | LMI-based robust PID | LM nonlinear inverse map | Nonlinear dynamics |
| Chih (2026)*: identification | ? | ? | Supporting PD stabilization | ? | Filtering-based parameter identification |
| Osedo (2023): direct RL | Given attitude reference | Pixhawk state measurements | PPO/LSTM policy | Policy commands thrust/vector | Actuator identification; randomized model |
| Xie (2023)*: RL-assisted robust control | ? | ? | Actor–critic and fixed-time control | ? | Learned model uncertainty |
| Durán-Delfín (2026)*: energy/passivity | ? | ? | Energy-based landing control | ? | Nonlinear model |
| Li (2024): servo-integrated NMPC | Given pose/trajectory references | Motion capture and EKF | NMPC | Joint thrust/servo optimization | Actuator identification |
| Cuniato (2024): direct RL | Given pose reference | Motion capture/IMU fusion | Learned pose/velocity policy | Actuator-derivative outputs | Randomized actuator dynamics |

The contrast between **Spannagl and Linsen** is especially useful: the former retains separate attitude PID loops and allocation beneath position MPC, while the latter solves tracking/stabilization with actuator constraints inside NMPC. The comparison between **Li and Cuniato** concerns model-based versus learned actuator command generation; both use external state-estimation infrastructure. Sources and section locations are in the individual notes and [machine-readable GNC audit](method_gnc_sources.json).

System identification belongs in the supporting-model column. Filtering data to identify parameters does not automatically constitute a new navigation method. Similarly, demonstrating landing does not automatically establish a new guidance/planning algorithm.


For the control interfaces and learning distinctions, see [residual RL and Neural-Fly](TVC_Ashby_Project_Guide.md#7-residual-rl-and-neural-fly-as-transfer-ideas). Source links for each paper are in [the reading notes](TVC_Ashby_Project_Guide.md#6-read-the-13-papers-one-by-one).
