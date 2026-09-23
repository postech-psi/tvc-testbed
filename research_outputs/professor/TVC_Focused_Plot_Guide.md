# Four standalone figures for the TVC thesis-positioning discussion

23 September 2026. **This is the current professor-facing guide.** There are no matrices or tables. Figure 1 is the numerical **Ashby-style property chart**; Figures 2a, 2b, and 3 are **categorical literature/evidence maps**, not numerical Ashby charts or novelty scores. A newer combined control/validation map is provided below as a single-plot alternative to the separate categorical views. Published-paper dots are black circles labeled by first author and year; the blue circle is labeled “Our Vehicle” without a year. Figures 2a and 2b are separate plots with the same GNC y-axis; Figure 2a displays the first three occupied controller categories, while Figure 2b retains all five. This distinguishes close two-axis coaxial TVC systems from papers whose methods may transfer. The blue point shows current design/simulation/bench status, never a free-flight result.

## Figure 1 — hardware: mass versus thrust evidence

![Mass versus published thrust evidence](C:/Users/tae06/CODE/tvc-testbed/research_outputs/professor/focused_plots/01_hardware_thrust.png)

This is a log–log physical-property chart with six same-sized circular dots: five close papers and our current vehicle model. Dashed lines denote \(T/(mg)=1\) and 2 using the **plotted thrust value**. These are performance-index guides, not flight-performance claims. Thrust evidence is not standardized, so the axis says “thrust reference,” not “measured maximum thrust.” Detailed qualifiers are here rather than attached to individual dots.

Spannagl reports a 1.16 kg vehicle and gives measured thrust as a 3-D surface in **Figure 6 (paper p. 6349)**, but not as one tabulated maximum. The dot at **16.5 N is the midpoint of an approximate 15–18 N visual reading**, not an author-reported specification or uncertainty estimate. Denton's dot is calculated from the paper's approximate maximum \(T/W\approx2\). Chen's 11.4 N is calculated from its published coaxial-thrust fit at the highest plotted 1600 μs operating point, not a certified maximum. Linsen reports approximately 2.3 kgf maximum thrust. E-Rocket states 3 kgf motor capability, although its separate static fit implies about 26 N at normalized maximum command; **those test conditions have not been reconciled**. Our vehicle uses the current **1.328 kg design mass estimate** and **17.79 N model/bench-derived thrust limit** in [`vehicle.yaml`](C:/Users/tae06/CODE/tvc-testbed/src/tvc_control/tvc_control/settings/vehicle.yaml), not a final assembled-vehicle measurement.

## Figure 2a — close-system control versus demonstrated GNC function

![Close-system control/GNC map](C:/Users/tae06/CODE/tvc-testbed/research_outputs/professor/focused_plots/02a_close_tvc_control.png)

Figure 2a contains Denton, Chen, Santos's E-Rocket, Spannagl, and Linsen: close coaxial two-axis TVC vehicles with flight experiments. It also shows our vehicle's current **simulation-only** PID/cascade fixed-target controller in blue. Its two previously blank rightmost controller columns are omitted for readability, not as a claim that those methods do not exist on related systems.

## Figure 2b — transferable methods on different systems

![Transfer-method control/GNC map](C:/Users/tae06/CODE/tvc-testbed/research_outputs/professor/focused_plots/02b_transfer_methods.png)

Figure 2b retains the **same GNC y-axis** and all five controller categories; it is a method-transfer comparison, not more evidence of close-system coverage. Its three dots are Li's tiltable quadrotor, Torrente's conventional quadrotor, and Osedo's constrained pitch-axis TVC rig. All three have physical-system experiments. Wilson's simulated Mars lander is excluded from the plotted set.

The **x-axis names the distinguishing implemented controller/policy architecture**, not every layer of the flight stack. “PID/cascade” includes an onboard PID tracker even when its trajectory was planned offline. “MPC/NMPC” includes inner P/PID feedback where present. Torrente goes in “Residual-model learning” because its distinguishing method is a learned Gaussian-process residual *inside MPC*; it is **not residual RL**. The categories are descriptive families, not an ordered sophistication score.

The **y-axis records the highest GNC decision function demonstrated**, not mission phase or a quality score:

- **Fixed/pilot target:** Denton's commanded attitude and Chen's target-height ascent; our vehicle's simulated hover/position hold; Osedo's commanded pitch angle on a constrained rig.
- **Given trajectory:** Santos's offline-planned mission reference; Li's and Torrente's supplied tracking references. “Given” does not imply that all three trajectories were created the same way.
- **Onboard guidance or goal policy:** Spannagl and Linsen generate trajectories onboard and track with MPC/NMPC. Wilson provides a simulated learned-policy precedent discussed outside the dots; it does not have a free-flying experimental vehicle.

The **upper-left Figure 2a cell** is empty: among these five close papers, none demonstrates onboard guidance with a PID-only outer trajectory tracker. Also, none demonstrates RL on the same close architecture in free flight. These are scoped observations, **not global novelty claims**. A thesis contribution would need a targeted wider search plus a meaningful performance, robustness, actuator-awareness, or validation result.

## Figure 3 — validation: task versus experimental setting

![Validation setting versus demonstrated task](C:/Users/tae06/CODE/tvc-testbed/research_outputs/professor/focused_plots/03_validation.png)

This plot keeps the same five published close systems and adds our vehicle as a qualified model/bench point. It separates experimental setting from controller choice. Denton's outdoor result is launch-to-hover/stabilization, not autonomous landing. Chen's plotted indoor result is custom-controller target-height ascent; its separate hover demonstration uses a different control stack. E-Rocket demonstrates indoor trajectory tracking, while Spannagl and Linsen report outdoor landing/diversion-related flights. Our blue point is **not a free-flight validation claim**.

## Combined alternative — control architecture versus validation setting

![Combined controller and validation evidence](C:/Users/tae06/CODE/tvc-testbed/research_outputs/professor/focused_plots/02_control_validation_profile.png)

This single figure puts the **eight physically tested papers**, without our vehicle, on one controller/validation grid. The translucent ellipses mark the two literature families—close TVC systems and other platforms—in an Ashby-inspired visual style. They are **illustrative groupings, not quantitative or uncertainty envelopes**; overlap does not make Li's quadrotor a close TVC system. Its y-axis refers to the validation setting of the cited result, not overall quality or the most ambitious task in a publication. Wilson is excluded because its lander plant was simulated. Osedo's RL controlled a constrained one-axis TVC rig, so it is in “Constrained hardware rig,” not free flight. Li and Torrente are indoor free-flight method precedents on non-coaxial quadrotors. The five close systems retain the placements described in Figure 3. The figure deliberately gives all published papers identical black markers; hardware comparability and task details must be read from the caption or [per-paper guide](TVC_Plotted_Papers_Guide.md), not inferred from marker shape or color. Empty cells are scoped to these eight papers and **do not prove novelty**.

## Figure 4 — control architecture versus demonstrated task

![Controller architecture versus demonstrated task](C:/Users/tae06/CODE/tvc-testbed/research_outputs/professor/focused_plots/04_control_task_profile.png)

This companion plot uses the same five controller categories but replaces validation setting with a representative demonstrated task: attitude regulation, ascent/hover, trajectory tracking, or landing/diversion. It contains **eight papers with physical-system experiments**. Wilson is deliberately excluded because its Mars descent plant was simulated, although its RL policy ran on a real Jetson Nano. Osedo remains because its one-axis TVC rig is a physical experiment; the attitude point must not be interpreted as free flight. The two translucent ellipses only distinguish close coaxial TVC systems from other platforms. Task rows are not an ordinal score, and an individual paper can demonstrate tasks beyond the one selected for its dot.

## Relevant papers kept outside the dots

[Santos's QuadRocket (2026)](https://arxiv.org/html/2607.02474v1) is useful for adaptive backstepping and actuator-dynamics treatment, with indoor tracking evidence. But its rocket body is carried above a **quadrotor through a universal joint**; that is not our central coaxial servo-gimbal actuator. It belongs in the explanation, not at an equivalent hardware point.

[Elke et al. (2021)](<C:/Users/tae06/Downloads/Elke Pei Caverly and Gebre-Egziabher (2021) - A Low-Cost and Low-Risk Testbed for Control Design of Launch Vehicles and Landing Systems.pdf>) and [Elke and Caverly (2024)](<C:/Users/tae06/Downloads/Elke and Caverly (2024) - Dynamics Guidance and Control of a Low-Cost Quadcopter-Based Space Vehicle Testbed.pdf>) use quadcopter-based launch-vehicle surrogates. Their validation and surrogate-fidelity questions are relevant, but their actuation topology is not our two-axis coaxial gimbal.

[Neural-Fly (O'Connell et al., 2022)](https://doi.org/10.1126/scirobotics.abm6597) combines offline supervised representation learning with online adaptive control; it is **not RL**. [Liu (2022)](https://doi.org/10.1109/IROS47612.2022.9981182) uses **residual RL on a blimp**. Both are useful for method selection but remain outside the dots by the project's earlier scope decision. [Zhang (2026)](https://arxiv.org/abs/2602.21583v2) is a newer actuator-level RL example on an overactuated tilt-rotor UAV, but remains an arXiv preprint in this corpus and was not added to the professor plot.

## What can and cannot be claimed

The close-platform evidence already includes PID, nonlinear feedback/allocation, and onboard optimal guidance with MPC/NMPC. Figure 2b shows that direct RL, learned-residual MPC, and servo-integrated NMPC have precedents elsewhere. Therefore “put MPC on an electric TVC rocket,” “use RL,” or “perform guided landing” is not, by itself, a thesis contribution. Potential directions should be tested against a reproducible baseline on **our measured actuator authority and lag**, with matched disturbances and validation on the actual vehicle. An empty cell is a question to investigate, not a claim of novelty.

## Primary evidence for plotted points

- [Spannagl et al. (2021), final IROS publication](https://doi.org/10.1109/IROS51168.2021.9636430): pp. 6345–6350 for vehicle, thrust Figure 6, onboard guidance/control, and outdoor flights.
- [Linsen et al. (2022), ICRA publication](https://doi.org/10.1109/ICRA46639.2022.9811938): hardware Section II, guidance/control Section V, and flight Section VII.
- [Denton et al. (2022), journal paper](https://doi.org/10.1177/17568293221117189): mass, rotor test, cascaded attitude controller, and flight experiments.
- [Chen et al. (2024), journal paper](https://doi.org/10.1109/TRO.2024.3354161): actual drone parameters; Section VI-E target-height flight; Figure 12 thrust fit.
- [Santos et al. (2026), E-Rocket accepted author text](https://arxiv.org/html/2512.06535v2): mass/thrust, software architecture, offline planning, PID indoor tracking.
- [Osedo et al. (2023), ROBOMECH Journal](https://doi.org/10.1186/s40648-023-00260-0): PPO/domain randomization, fixed pitch target, constrained one-axis apparatus.
- [Li et al. (2024), IEEE Robotics and Automation Letters](https://doi.org/10.1109/LRA.2024.3451391): servo-integrated NMPC and onboard pose-tracking experiments on a tiltable quadrotor.
- [Torrente et al. (2021), IEEE Robotics and Automation Letters](https://kelia.github.io/publication/data-driven-mpc/): GP residual dynamics inside MPC; real-world quadrotor trajectory tracking.
Wilson and Riccardi (2022) remains an [unplotted simulation-only reference](https://strathprints.strath.ac.uk/81480/): the RL policy ran on a real Jetson Nano, but the descent plant was simulated.

Editable SVGs, presentation PNGs, and single-page vector PDFs are in [`focused_plots`](C:/Users/tae06/CODE/tvc-testbed/research_outputs/professor/focused_plots). The reproducible figure builder is [`build_focused_ashby_plots.py`](C:/Users/tae06/CODE/tvc-testbed/research_outputs/professor/build_focused_ashby_plots.py). The older six-figure guide is retained only for historical/source-audit context.
