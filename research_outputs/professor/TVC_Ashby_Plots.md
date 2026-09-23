# Archived six-figure TVC plot guide (superseded)

The user-approved current version is the [three-figure focused guide](C:/Users/tae06/CODE/tvc-testbed/research_outputs/professor/TVC_Focused_Plot_Guide.md) and its three-page deck. The six-figure text below is retained only as an audit trail; Plots 2, 4, and 5 are not part of the current presentation, and the earlier hardware note about Spannagl has been corrected in the focused guide.

23 September 2026. This is the plot-led version of the review. It contains no comparison matrix. The first two figures are Ashby-style **physical-property** comparisons; Figures 3 and 6 are **categorical literature-positioning** maps. Figures 4 and 5 test whether a proposed software method is plausible on real hardware. None of the plots by itself establishes novelty.

The core set is Spannagl (2021), Linsen (2022), Denton (2022), Osedo (2023), Chen (2024), Santos *E-Rocket* (2026), and Santos *QuadRocket* (2026). Torrente (2021) and Zhang (2026) are explicitly marked as **method-transfer** examples, not close vehicle precedents. Neural-Fly is learning-based adaptive control, not reinforcement learning; it is therefore not plotted as an RL result. Li (2024), Elke, Chih, Liu, and the overlapping Denton (2025) item are not additional core dots. See the earlier notes for the selection audit, not for the meeting graphics.

## 01. Hardware: vehicle mass versus thrust

![Vehicle mass against available thrust](C:/Users/tae06/CODE/tvc-testbed/research_outputs/professor/plots/01_hardware_mass_thrust.png)

The dashed lines are constant static thrust-to-weight ratios, calculated as \(T/(mg)\). This is the closest figure here to a conventional Ashby chart: both axes describe physical properties and the contours express a simple engineering constraint. The spread is informative, but it is not a controller ranking. QuadRocket obtains high thrust with a quadrotor actuator under a universal joint, not the same coaxial-gimbal arrangement. Denton's thrust value is derived from the paper's approximate maximum \(T/W\approx2\); Chen's 11.4 N is calculated from the reported coaxial-thrust calibration at 1600 μs, **not** an independently certified maximum. The E-Rocket's **stated motor capability** of “3 kgf” and Linsen's “2.3 kg” thrust are converted to newtons with 9.80665 N/kgf. E-Rocket also reports a static thrust fit that evaluates to about 26 N at its maximum normalized command, below the stated 29.4 N capability; the plot uses the stated capability and does not reconcile those test conditions.

The POSTECH point combines the current **1.328 kg design mass estimate** and **17.79 N thrust parameter/bench-derived limit** in [`vehicle.yaml`](C:/Users/tae06/CODE/tvc-testbed/src/tvc_control/tvc_control/settings/vehicle.yaml). It does not represent a final weighed flight vehicle. Spannagl reports vehicle mass but no clear maximum-thrust value in the final paper, so there is no honest y-coordinate. Osedo's constrained one-axis apparatus is not a free-flight thrust-to-weight comparator.

## 02. Hardware: gimbal travel versus thrust margin

![Gimbal travel against thrust margin](C:/Users/tae06/CODE/tvc-testbed/research_outputs/professor/plots/02_hardware_gimbal_margin.png)

This chart asks whether a vehicle has enough static thrust *and* directional reach for lateral force. Gray contours are the ideal upright expression \((T_{\max}/m)\sin\delta\), reported as m/s² after division by mass. They ignore attitude changes, drag, rate limits, servo lag, coupling, battery sag, and control stability; they are **not achieved flight acceleration**. The present POSTECH model has substantially smaller gimbal travel (nominal ±7°) than the Linsen, Denton, and E-Rocket reported mechanisms. That is a testable design constraint and potentially a useful thesis motivation, not proof that a new controller is necessary.

QuadRocket's 40° universal-joint mechanical reach is excluded because it is not a servo-commanded two-axis gimbal limit. Chen is excluded because we did not find a numeric gimbal limit in the paper. The points use Linsen ±15°, Denton ±30°, E-Rocket ±30°, and POSTECH nominal ±7°.

## 03. Software: controller family versus demonstrated GNC scope

![Controller family against demonstrated GNC scope](C:/Users/tae06/CODE/tvc-testbed/research_outputs/professor/plots/03_software_control_gnc.png)

This is a **categorical map**, not a quantitative Ashby property plot. It shows that control method and GNC scope must be compared separately: a paper can have sophisticated trajectory control but no online guidance, or an RL attitude controller with no translational navigation. The vertical placement means the **demonstrated task in the selected experiments**, not every task the authors' architecture might in principle support. Label spacing within a cell has no meaning.

Spannagl and Linsen combine optimization-based control with guidance. E-Rocket provides a close-platform PID tracking baseline. Osedo demonstrates direct PPO control only on a constrained pitch rig. Zhang demonstrates direct RL on a different overactuated tilting-rotor UAV, while Torrente demonstrates learning-enhanced MPC on a conventional quadrotor. **Direct RL is not residual RL.** The absence of a selected close free-flight residual-RL point is a literature-search result with this scope, **not** a claim that no such paper exists anywhere.

## 04. Software: reported loop rate versus compute location

![Reported periodic function rate against compute location](C:/Users/tae06/CODE/tvc-testbed/research_outputs/professor/plots/04_software_rate_location.png)

This plot is about implementability. The points refer to the **named function** beside each paper: for example, Spannagl's MPC is 25 Hz, Linsen's NMPC 50 Hz, Chen's controller 250 Hz, Osedo's policy 50 Hz, QuadRocket's offboard tracking loop 100 Hz, Torrente's offboard GP-MPC 50 Hz, and Zhang's onboard policy 100 Hz. These are not all the same loop layer. A faster number does not imply a better controller or greater closed-loop bandwidth. In particular, Chen's actuator PWM is reported separately at 100 Hz, so the 250 Hz control update is not 250 Hz physical actuation. Denton reports 1 kHz gyroscope sampling and 200 Hz attitude estimation, but we do not assign an unreported comparable controller rate. E-Rocket does not state a PID update rate.

## 05. Software: computation time versus deadline

![Controller computation time against update period](C:/Users/tae06/CODE/tvc-testbed/research_outputs/professor/plots/05_software_compute_deadline.png)

The dashed diagonal is the simple condition that execution time equal the update period. The three published numbers are **different statistics**: Spannagl reports a 30 ms *mean* MPC solve time at a 40 ms period; Linsen says tracking computation did not exceed 18 ms at a 20 ms period; Zhang reports about 0.3 ms policy inference at a 10 ms period on a different aircraft. This chart helps frame a measurable deployment question: if a residual learner or adaptive correction is added to a baseline GNC stack, how much timing margin remains on the actual processor? It is not a worst-case timing guarantee for any paper.

## 06. Validation: experimental setting versus demonstrated task

![Experimental setting against demonstrated task](C:/Users/tae06/CODE/tvc-testbed/research_outputs/professor/plots/06_validation_task_setting.png)

Every core paper appears here once at its strongest reported combination of environment and task. This prevents the one-axis RL rig from being mistaken for free flight, or indoor tracking from being mistaken for outdoor landing. POSTECH is marked at its current model/bench stage; it is **not** yet a free-flight result. Chen's point is the custom controller's ascent test; the separate hover demonstration uses a different flight-control stack. Denton's outdoor point concerns launch-to-hover/stabilization, not autonomous landing. Spannagl and Linsen report outdoor controlled-descent/landing-related demonstrations, with different mission scales.

## What the figures support saying to the professor

The strongest current positioning statement is not “RL for rockets is unexplored.” It is narrower and testable: **our limited gimbal authority and measured actuator dynamics need to be carried through guidance, control, and validation on our own platform.** A learning component would be justified only after a specific residual error is measured against a sound baseline, and it would need the same hardware timing and safety tests. The plots therefore separate *problem selection* (hardware and validation constraints) from *method selection* (MPC, adaptive, or learning-based control).

An immediately defensible next experiment is to measure the final assembled vehicle mass, static thrust across battery voltage, gimbal travel under load, and closed-loop actuator response. Recompute Figures 1–2 with those measurements. Then demonstrate an agreed task ladder: stabilized hover, indoor trajectory tracking, and eventually constrained landing/diversion if facilities permit. Those results would turn the plotted white space into an evidence-based thesis contribution rather than a visual gap claim.

## Source trail and uncertainty

The figure builder is [`build_ashby_plots.py`](C:/Users/tae06/CODE/tvc-testbed/research_outputs/professor/build_ashby_plots.py); every PNG has a corresponding editable SVG in [`plots`](C:/Users/tae06/CODE/tvc-testbed/research_outputs/professor/plots). No paper supplied a complete, directly comparable record for all hardware and software axes. Missing points are therefore omissions, not zeros.

- [Spannagl et al. (2021), final IROS paper](https://doi.org/10.1109/IROS51168.2021.9636430): vehicle design; control architecture and rates; experimental and solver-timing sections.
- [Linsen et al. (2022), ICRA](https://doi.org/10.1109/ICRA46639.2022.9811938): Section II mass/thrust; Section V servo constraints and NMPC; Section VII flight/timing.
- [Denton et al. (2022), *International Journal of Micro Air Vehicles*](https://doi.org/10.1177/17568293221117189): vehicle mass, coaxial rotor thrust test, gimbal range, and launch/hover experiments.
- [Osedo et al. (2023), *ROBOMECH Journal*](https://doi.org/10.1186/s40648-023-00260-0): uniaxial rig, PPO and domain randomization, onboard 50 Hz policy.
- [Chen et al. (2024), *IEEE Transactions on Robotics*](https://doi.org/10.1109/TRO.2024.3354161): actual-platform parameters, Fig. 12 coaxial-thrust fit, embedded controller and ascent tests. The plotted fit-at-1600 μs thrust is derived, not a quoted maximum.
- [Santos et al. (2026), E-Rocket accepted author version](https://arxiv.org/html/2512.06535v2): mass, reported combined thrust, gimbal bounds, PX4/ROS 2 architecture, and indoor tracking.
- [Santos et al. (2026), QuadRocket accepted author version](https://arxiv.org/html/2607.02474v1): mass/thrust, universal joint, offboard tracking rate, indoor motion-capture flight.
- [Torrente et al. (2021), GP-MPC](https://arxiv.org/pdf/2102.05773): method-transfer comparator and offboard experiment rate only.
- [Zhang et al. (2026), uploaded version 2](C:/Users/tae06/Downloads/2602.21583v2.pdf): different overactuated UAV, direct RL versus NMPC, 100 Hz onboard update, approximately 0.3 ms policy inference. Accepted-author status does not make it a close hardware comparator.

The repository's [`vehicle.yaml`](C:/Users/tae06/CODE/tvc-testbed/src/tvc_control/tvc_control/settings/vehicle.yaml) is the source for the marked POSTECH model point. Its mass comment explicitly says design estimate; do not cite this point as a measured flight-vehicle result.
