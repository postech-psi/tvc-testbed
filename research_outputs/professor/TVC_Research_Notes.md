# Electric TVC: literature positioning and thesis-direction notes

23 September 2026 | Scoped discussion draft for the professor, not a systematic review or a confirmed novelty claim.

## 1. What to take to the meeting

Use the [four-page professor brief](C:/Users/tae06/CODE/tvc-testbed/output/pdf/TVC_Professor_Brief.pdf): (1) positioning map, (2) GNC evidence matrix, (3) transferable methods, and (4) candidate research questions with decision criteria. This Markdown is the readable research record behind it.

The current question is **what can we investigate convincingly on our electric TVC testbed?** It is not “which fashionable controller can fill an empty square?” Your professor's requested Ashby-style figure is treated here as a **literature-positioning map**, following your clarification. Its categorical axes do not claim engineering-property trade-offs or controller rankings.

Our plant is a counter-rotating coaxial propulsion unit on a two-axis central gimbal. The project README describes a Raspberry Pi/Pixhawk implementation direction, while the documented current control demonstration is simulation-based. We must not count planned hardware or simulator parameters as measured flight evidence. See the [project README](C:/Users/tae06/CODE/tvc-testbed/README.md).

The main comparison has seven papers, not seven interchangeable aircraft: five close coaxial-gimbal studies, one articulated rocket surrogate, and one constrained TVC experiment. Seven is the result of the scope rule, not a target count. Transfer-method papers remain readable below but do not inflate the main map.

## 2. Selection decisions

| Paper | Decision for the professor package | Reason |
|---|---|---|
| Spannagl (2021) | Main map | Close plant; integrated optimal guidance and offset-free tracking |
| Linsen (2022) | Main map | Close plant; joint trajectory/attitude NMPC and augmented estimation |
| Santos (2026), E-Rocket | Main map | Close platform and PID implementation baseline; IFAC acceptance/program checked |
| Santos (2026), QuadRocket | Main map, separate platform row | Relevant adaptive rocket-surrogate controller, but quadrotor actuation differs |
| Chen (2024) | Main map | Closely related coaxial-gimbal coupling/allocation |
| Denton (2022) | One representative main point | Platform/control demonstration already in your slides |
| Denton (2025) | Follow-up citation, no second point | New flight-data identification on the same platform family; relevant if identification becomes our contribution |
| Osedo (2023) | Main map, explicitly adjacent rig row | TVC hardware + RL/model-variation experiment; not free flight |
| Li (2024) | Method-transfer section only | Useful servo-integrated NMPC; overactuated tiltable quadrotor differs |
| Torrente (2021) | Method-transfer section; retain substance | Residual dynamics learning inside MPC; not RL |
| Zhang (2026), supplied `2602.21583v2.pdf` | Method-transfer section only | Actual uploaded paper; actuator-level RL versus NMPC on a different plant |
| Cuniato (2024) | Remove from current map; defer detailed use | Different plant; supplied 2026 file is not this paper |
| O'Connell (2022), Neural-Fly | Idea note only, no map point | Learning-based adaptive disturbance compensation, not RL |
| Liu (2022) | Idea note only, no map point | Residual-RL architecture on a blimp; distant physical system |
| Elke (2021), Elke (2024) | Background only | Rocket-surrogate fidelity/flexible modes; not our current low-level control comparison |
| Chih (2024), Chih (2026) | Exclude from current plotted set | Full texts unavailable to this review; theoretical/simulation emphasis outside the chosen experimental scope |
| Smeur (2018) | Not carried into the present brief | Optional future baseline reading, not necessary to make this meeting package coherent |

The five mission-context papers already in your slides (Dumke, Scharf, Rmili, Trawny and Carson) remain background references, not extra low-level controller dots. Their detailed evidence is not newly audited here.

An arXiv-hosted **accepted author manuscript** is not automatically an unpublished study. E-Rocket is explicitly IFAC-accepted and appears in the official program. QuadRocket has a TAES DOI. Zhang v2 reports acceptance on 12 September 2026, and its [author record](https://arxiv.org/abs/2602.21583) says accepted to RA-L; final journal DOI/pages were not verified. This distinction does not make Zhang a close physical peer.

Excluding Chih from the figure is a scope/access decision, not evidence that its ideas do not exist. Do not use the filtered figure to claim that nobody has investigated close-system robust control or parameter identification.

## 3. How to read the map

![Seven-paper positioning map](C:/Users/tae06/CODE/tvc-testbed/research_outputs/professor/tvc_positioning_map.png)

- X: primary **control architecture**: feedback with explicit allocation; optimization-based control; adaptive nonlinear control; direct RL.
- Y: **experimental platform class**, separating close actuation topology, an articulated rocket surrogate, and a constrained TVC apparatus.
- All markers are identical. Each has a first-author/year label; the two Santos papers also carry platform names.
- Multiple papers within a categorical cell are vertically separated for legibility only. Their small within-cell offsets are not scores.
- Position represents the main architecture, not everything in the paper. For example, Spannagl contains conventional inner feedback, and both optimal-control papers include estimation for mismatch.
- Empty cells mean no selected example here, not established novelty. Learned residual models, residual RL and Neural-Fly are explained separately; we do not fabricate main-map peers for them.

The map answers “which kinds of controller have been demonstrated on which relevant plants?” The matrix below answers “which GNC layer does each paper actually address?” Neither compares unmatched RMSE, altitude, cost or processor numbers as if they came from one benchmark.

## 4. GNC evidence matrix

G = guidance/reference generation; N = navigation/state and disturbance estimation; C = feedback/tracking control; A = allocation and actuator realization. Using an estimator is not necessarily a navigation research contribution. A prescribed target is not an optimal guidance algorithm.

![GNC comparison matrix](C:/Users/tae06/CODE/tvc-testbed/research_outputs/professor/tvc_gnc_matrix.png)

| Paper | G | N | C and A | Demonstration boundary |
|---|---|---|---|---|
| Spannagl (2021) | Free-final-time optimal guidance; retargeting | State EKF plus offset estimation | Offset-free MPC, inner attitude/rate feedback, inverse allocation, thrust-response model | Indoor tracking and 19 outdoor missions; G/C onboard |
| Linsen (2022) | Free-time terminal-target optimization | Pixhawk filtering plus disturbance/parameter EKF | NMPC jointly tracks position and stabilizes attitude; angle/rate and motor constraints | Indoor pattern and wind test; outdoor apogee and landing |
| E-Rocket (2026) | Offline references and mission logic | PX4 EKF with motion capture indoors | PID tracking cascade and allocation | Indoor flight; not a demonstrated online optimal landing planner |
| QuadRocket (2026) | Supplied tracking reference | Motion-capture-supported state feedback | Adaptive backstepping, transformed control point, dynamic-surface treatment of actuation | Flight on an articulated surrogate; main tracking computation offboard |
| Chen (2024) | Ascent target; manual maneuvering also shown | IMU/ToF for custom ascent; separate Pixhawk hover | Nonlinear allocation with damping | Do not attribute the separate Pixhawk hover to the same custom controller |
| Denton (2022) | Pilot commands; launch-to-hover sequence | Onboard attitude estimation | Cascaded attitude/rate feedback; gimbal and differential speed | Rig tests and free flight; not autonomous landing guidance |
| Osedo (2023) | Prescribed pitch target | Pitch/rate feedback | PPO/LSTM commands thrust and vectoring increments | One-axis rig, not translation or autonomous landing |

Source locators and reading depth appear in the individual notes and audit below. A navigation gap cannot be inferred merely because the plotted control papers use existing filters.

## 5. Main papers, one by one

### Spannagl (2021): the closest integrated GNC baseline

**Design, Optimal Guidance and Control of a Low-cost Re-usable Electric Model Rocket**, IROS, pp. 6344-6351. [DOI](https://doi.org/10.1109/IROS51168.2021.9636430).

The provided final publication resolves the earlier manuscript discrepancy: Section IV-C, printed p.6350, reports **19 successful outdoor flights**, with campaign maxima of **100 m altitude**, **50 m diversion**, and **3 m/s velocity**. These are campaign maxima, not a claim that every flight met all three simultaneously. Guidance and MPC ran onboard; mean solve times were reported as 120 ms and 30 ms.

Its architecture matters more than these headline numbers: optimal guidance feeds offset-free MPC, which uses inner attitude/rate feedback and actuator allocation. Section III-B, Eq.(2e), already includes a first-order thrust response. Section III-A also constrains/penalizes thrust-rate variation. Section IV-B and Fig.9 show altitude regulation as battery voltage decreases, with an estimated offset compensating mismatch. Thus “prior electric TVC ignores actuator dynamics, battery variation and repeated flights” is untenable.

What remains open **for our experiment** is a matched comparison showing when a specific refinement adds value over a well-tuned simpler baseline. I did not find a controlled PID-versus-MPC benchmark or isolated thrust-lag ablation in this main text. That is a paper-specific observation, not a field-wide novelty claim.

### Linsen (2022): a different NMPC architecture, not another copy of Spannagl

**Optimal Thrust Vector Control of an Electric Small-Scale Rocket Prototype**, ICRA, pp.1996-2002. [DOI](https://doi.org/10.1109/ICRA46639.2022.9811938).

The introduction explicitly distinguishes its design from Spannagl: **NMPC simultaneously tracks the optimal reference and stabilizes attitude**, without the same separate PID orientation cascade. Guidance uses a point-mass free-time optimal-control formulation. Tracking includes the full rigid-body model, experimentally fitted propeller maps, servo-angle/rate limits, individual propeller bounds and a ground-height constraint (Section V, Eq.10).

Section VI augments estimation with thrust/torque scaling and external forces/torques; the paper connects mismatch to battery decrease, ground effect and imperfect balancing. Section VII reports indoor pattern tracking, a **3.1 m/s fan-array wind experiment indoors**, and outdoor flight to a 3 m apogee followed by landing 2 m away. Do not describe the fan test as an outdoor wind trial.

This is the closest reference to consult before claiming that joint constrained control or online model-offset correction is missing. Its local PDF was read through the main text for this revision.

### Santos (2026): E-Rocket

**The E-Rocket: Low-cost Testbed for TVC Rocket GNC Validation.** [Accepted manuscript](https://arxiv.org/html/2512.06535v2) and [official IFAC program](https://ifac.papercept.net/conferences/conferences/IFAC26/program/IFAC26_ContentListWeb_2.html#tuc26_03).

A close hardware/software baseline: central coaxial gimbal, Raspberry Pi/Pixhawk, ROS 2 control and indoor motion capture. Offline reference trajectories are tracked by a PID-based cascade. Its simplifying actuation/coupling assumptions are useful experimental starting points, not evidence that PID is inadequate. Use this as the implementation baseline while using Spannagl and Linsen to avoid overstating control novelty. Main HTML and Sections 3-5 were inspected; final proceedings metadata are not confirmed.

### Santos (2026): QuadRocket

**QuadRocket: An Aerial Robotic Testbed for Adaptive Thrust-Vector Control of Rocket-Like Vehicles.** [TAES DOI](https://doi.org/10.1109/TAES.2026.3706328); [accepted manuscript](https://arxiv.org/html/2607.02474v1).

A quadrotor below a joint supplies the rocket body's vectored thrust. Adaptive backstepping, a control-point transformation and dynamic-surface control address tracking, non-minimum-phase behavior and actuation. It already provides an adaptive rocket-surrogate precedent. Keep it on a separate platform row: a quadrotor thrust actuator is not our serial servo gimbal. Experimental tracking uses an offboard main loop; do not conflate it with fully onboard execution. Formulation and experimental sections inspected; stability proofs not independently verified.

### Chen (2024): coupling and allocation

**Design, Modeling, and Control of a Coaxial Drone**, IEEE TRO. [DOI](https://doi.org/10.1109/TRO.2024.3354161).

The close feature is the two-rotor/two-serial-servo actuation topology. Nonlinear allocation and added damping address the underactuated dynamics. Section VI-E, printed p.1661, is essential: custom ascent uses IMU and ToF measurements, but longer operation suffers state-measurement error. The separate Pixhawk 4 Mini hover test uses its sensor fusion and an existing allocation framework. The latter validates the platform without proving that the custom proposed controller produced every flight result. This also shows why a controller benchmark must hold estimation quality fixed.

### Denton (2022): retain one platform representative

**Design, development, and flight testing of a tube-launched coaxial-rotor based micro air vehicle.** [DOI](https://doi.org/10.1177/17568293221117189).

The paper demonstrates a compact coaxial thrust-vectoring platform with cascaded feedback, single-axis wind-tunnel testing and indoor/outdoor flight. Its mission is not autonomous VTVL landing. It remains the representative dot because this map organizes platform/control architectures, not every contribution from the same research line.

**Denton (2025) is not a bibliographic or scientific duplicate.** [DOI](https://doi.org/10.1177/17568293251361078). It identifies hover flight dynamics from measured excitation data, derives state-space coefficients, and validates against independent measurements (pp.12-16). Keep it in the reading record and cite it if our thesis concerns identification or model fidelity. If that becomes the thesis focus, replace the 2022 representative with 2025 instead of adding a second platform dot.

### Osedo (2023): useful adjacent TVC RL evidence

**Uniaxial attitude control of uncrewed aerial vehicle with thrust vectoring under model variations by deep reinforcement learning and domain randomization.** [Open-access paper](https://doi.org/10.1186/s40648-023-00260-0).

PPO with LSTM memory commands paired thrust/vectoring increments. Training varies the physical model; hardware evaluates altered mass, inertia and center of gravity. The out-of-training-range case fails to attain the desired pitch. This makes it a useful bounded generalization test, not evidence of universal robustness. The apparatus permits only pitch rotation. Include it in a distinct rig row; never present it as free-flight or landing RL. Setup, response models, policy and evaluation sections were read. No cross-paper thrust or tracking-performance ranking is attempted.

## 6. Transferable methods: keep ideas without mixing plants

### Torrente (2021): keep as a leading experimental-design reference

**Data-Driven MPC for Quadrotors**, RA-L. [DOI](https://doi.org/10.1109/LRA.2021.3061307); [paper](https://arxiv.org/pdf/2102.05773); [code](https://github.com/uzh-rpg/data_driven_mpc).

Gaussian processes learn **dynamics error**, not control actions. The practical model maps body-frame velocity to acceleration residual, axis by axis; the corrected dynamics enter MPC. Training targets are measured-minus-predicted next-step velocity divided by the sample interval (Eq.13). It is supervised residual-model learning, **not residual RL**.

The useful comparison is nominal MPC versus linear-drag MPC versus GP-MPC, with matched control frequency/cost matrices and shared training data for fitted models. Real experiments use laptop computation and 50 Hz thrust/body-rate commands. Table III includes a low-speed default case where GP-MPC is worse than nominal MPC; gains are condition-dependent, not universal. Sources: Sections III-C/F and IV, especially Tables II-III.

**Our proposed transfer, not the paper's claim:** first test whether a residual model improves held-out TVC dynamics prediction over an identified physical model. Only then put it into control. Candidate features could include measured gimbal state, voltage and operating thrust, but require our own data and justification; copying a high-speed drag model into near-hover TVC is not enough.

### Li (2024): retain servo-aware NMPC as a mechanism

**Servo Integrated Nonlinear Model Predictive Control for Overactuated Tiltable-Quadrotors**, RA-L. [DOI](https://doi.org/10.1109/LRA.2024.3451391); [lab record](https://www.dragon.t.u-tokyo.ac.jp/publications/references/2024-ral-beetle-art-li/); [author PDF](https://arxiv.org/pdf/2405.09871).

The transferable idea is to include servo response inside prediction/control rather than assume instantaneous vectoring. Four independently tilting rotors give a different achievable force/torque set from our underactuated central gimbal. The actuator-model ablation is simulation; hardware demonstrations are separate. It is relevant prior art for actuator-aware MPC, not a directly comparable landing result. Selected formulation/evaluation sections were inspected in the earlier audit, not fully reread in this revision.

### Zhang (2026): what your new upload actually says

**Learning End-to-End Control for Omnidirectional Aerial Motion on Overactuated Tilt-rotor Quadrotors.** [Versioned author record](https://arxiv.org/abs/2602.21583v2). Wentao Zhang is first author; Eugenio Cuniato is a coauthor. The supplied eight-page v2 is not the Cuniato (2024) paper. Acceptance to RA-L is reported; final journal metadata remain to be checked.

Four tilting rotors, four servos and an eight-dimensional action make this a different plant. PPO trains an asymmetric actor-critic: the actor observes a 33-component state/reference vector; the training critic also receives rotor thrust/torque information. The policy directly produces four joint commands and four rotor-thrust commands. **It is direct RL, not residual RL**: the constant hover-thrust offset in Eq.4 is action centering, not a separate nominal feedback controller being corrected.

The reward combines pose convergence with penalties for velocity, command changes, limits, thrust and joint motion (Table I). The sim-to-real pipeline identifies voltage-dependent thrust mappings, second-order servo response and latency, with limited domain randomization (Section II-C). These are particularly relevant lessons for our testbed even though independent omnidirectional pose control does not transfer.

The comparison reuses Li's servo-integrated NMPC, retuned to this platform; both run at 100 Hz on the same onboard stack. Table V reports three repeats of the waypoint sequence. Its steady-state mean position-error norms are **0.077 m RL versus 0.042 m NMPC**, and orientation-error norms **4.261 degrees versus 4.350 degrees**. These are not RMSE and not comparisons against our vehicle. The five-repeat trajectory test reports larger mean errors for RL: 0.16 m versus 0.05 m, and 8.06 degrees versus 5.19 degrees. RL offers fast transitions and low inference time, but the paper does not show that RL wins every objective.

Limits to retain: continuous allocation-singularity traversal is **simulation only**; the 90-degree hardware test uses a reconfigured vehicle and a retrained policy. “Zero-shot deployment” does not mean one unchanged policy for every hardware configuration. Sections II-III, Tables I-VII and the conclusion were read; key hardware-result pages 6-7 were visually checked.

### Neural-Fly and Liu: preserve only the architecture ideas

O'Connell (2022), **Neural-Fly enables rapid learning for agile flight in strong winds**, [DOI](https://doi.org/10.1126/scirobotics.abm6597): offline learned features with online coefficient adaptation compensate aerodynamic uncertainty. This is learning-based adaptive control, not reinforcement learning. Liu (2022), [IROS DOI](https://doi.org/10.1109/IROS47612.2022.9981182), provides a residual-RL idea on a blimp. Both remain outside the main positioning map as requested; their full papers were not newly audited in this revision.

| Architecture | What is learned/adapted? | Where it enters | Role here |
|---|---|---|---|
| Identified actuator-aware control | Physical response parameters from data | Physical prediction/compensation model | Strong non-learning benchmark |
| Torrente-style residual model | Prediction error from supervised data | `f_corrected = f_physics + residual_model` | Candidate method-transfer direction |
| Neural-Fly-style adaptation | Offline representation; online coefficients | Estimated disturbance compensation | Idea only; not RL |
| Direct RL, e.g. Osedo/Zhang | Policy through reward optimization | State/reference to actuator commands | Alternative architecture; plant/task limits explicit |
| Residual RL, Liu-inspired | Policy correction through reward optimization | `u = u_baseline + delta_u_policy` | Possible later study, not presumed necessary |

These are schematic classifications, not claims that the papers use identical input/output interfaces. A residual's magnitude bound alone does not establish closed-loop safety.

## 7. What the uploaded Elke papers contribute

### Elke (2021)

**A Low-Cost and Low-Risk Testbed for Control Design of Launch Vehicles and Landing Systems.** The supplied IEEE Aerospace paper contains a review and a proposed quadcopter with flexible inverted/hanging pendulums. Sections 3-4 and the appendix compare linearized planar modes: pendulum instability, flexibility and slosh analogues. The pole match uses some unrealistic testbed parameters, acknowledged explicitly on p.7; translation and rotor/engine dynamics are omitted from that planar analysis. Construction and 3D comparison are future work. This supports a careful discussion of similitude, not a free-flight controller claim.

### Elke and Caverly (2024)

**Dynamics, Guidance, and Control of a Low-Cost Quadcopter-Based Space Vehicle Testbed**, AIAA SciTech. [DOI](https://doi.org/10.2514/6.2024-0778).

The 21-page upload extends the CRQS multibody modeling, applies an existing booster soft-landing formulation, and uses an LQR designed from a simplified rigid configuration. Section IV-B assumes perfect controller state knowledge. Section V compares four configurations with and without flexibility and hanging-pendulum effects. Unmodeled bending can destabilize the loop; a prototype photograph and measured static thrust do not turn those results into flight validation. Fabrication/flight remain future work in Section VI.

**Decision:** keep both as background only. They become central if we deliberately choose “which rocket dynamics can our surrogate reproduce?” That is a different research program requiring a specified target rocket/model and similarity criteria; our electric rigid-body demonstrator does not reproduce propellant slosh or changing mass just by resembling a rocket.

## 8. Research-direction matrix: questions, not empty-cell claims

These are proposed experiments, not conclusions drawn from the plot. Do not promise all four as one undergraduate project.

| Candidate question | Closest prior art / novelty boundary | Smallest useful comparison | Decision condition |
|---|---|---|---|
| A. When does measured actuator fidelity materially improve constrained TVC tracking? | Spannagl already models thrust lag; Linsen constrains servo rates; Li integrates servo response | Same plant/controller family: static actuation model vs identified dynamic model; same estimator/reference/limits | Continue only if held-out actuator predictions and closed-loop outcomes improve reproducibly |
| B. Does a learned residual add value beyond physical identification and simple mismatch compensation? | Linsen/Spannagl estimate mismatch; Torrente learns residual dynamics; neither alone establishes novelty for us | Identified baseline vs simple parametric/offset correction vs residual model; shared data and test conditions | Continue only if residual structure persists and improvement survives new runs/conditions |
| C. What robustness/computation trade-off does learned control offer under TVC uncertainty? | Osedo tests constrained TVC RL; Zhang compares RL and NMPC on a different plant | Tuned conventional baseline vs one learned architecture, with identical action authority and sensing | Requires a validated simulator, training resources and credible safety envelope; avoid “RL is better” as the thesis |
| D. Does actuator-feasible guidance reduce tracking/landing failures near constraints? | Spannagl and Linsen already provide constrained optimal guidance/control | Hold tracking controller fixed; compare references generated with and without the selected actuator-feasibility condition | Requires reliable tracking first, and a specific constraint missed by the existing formulation |

**Recommended immediate step: an identification-and-baseline gate, not a committed thesis title.** Measure motor/thrust response, gimbal response/rate limits, command/measurement latency and static mappings within a safe fixture. Fit the simplest useful models. Use different complete runs for fitting and validation. If the physical model and conventional feedback already meet the intended envelope, adding learning is not justified solely to make the plot look novel.

If repeatable residual structure remains, B becomes a strong candidate to discuss, with Torrente as an experimental-design template. If errors are mainly actuator limits/lag, A or D is better aligned. If neither hardware measurements nor credible simulation validation are available, a claimed flight-ready RL contribution is premature.

## 9. Minimal evaluation contract

1. Define one task before controller selection: initially safe attitude/hover tracking; landing only after stable tracking is established. Fix identical references, estimator, actuator limits and sampling conditions across comparisons.
2. Separate model error from controller error. Compare held-out actuator/dynamics predictions before comparing closed-loop controllers. Do not split adjacent samples randomly and call them independent test runs.
3. Vary one meaningful factor at a time initially: operating thrust/voltage, gimbal slew demand, known small payload/CG changes, or controlled disturbances. Select magnitudes from measured safe capability, not a borrowed paper's values.
4. Report tracking RMSE with explicit definition, peak error, recovery/settling behavior, saturation time, missed deadlines and failures. Report controller-runtime distributions, not only means. For learning, include training data/compute and held-out conditions.
5. As a pilot protocol, target five independent runs per condition if resources and safety permit, then assess whether variability warrants more. This is a proposed starting point, not a claim of statistical sufficiency. Randomize/balance order and account for battery state.
6. Hold tuning effort and action authority comparable; document differences rather than silently favor one method. A first-order versus second-order actuator model is itself an experimental factor.

For GNC attribution: hold G and N fixed when studying C; hold C and N fixed when studying G. A future N contribution requires separate ground truth and estimator-specific baselines. Do not advertise a full GNC contribution when only C changes.

## 10. What was actually read, and what still needs checking

| Source | Access and reading depth supporting this revision |
|---|---|
| Spannagl final IROS PDF | Entire main text extracted/read; final outdoor results and battery figure visually checked |
| Linsen local ICRA PDF | Entire main text read, including Eqs.9-17 and all results subsections |
| Chen local TRO PDF | Targeted control/experiment/conclusion audit, especially VI-E; not every derivation independently verified |
| Denton 2022 and 2025 local PDFs | Targeted control, experiment, identification-validation and conclusion sections; not an exhaustive mathematical audit |
| Elke 2021 local PDF | Proposed-model, analysis, limitations, conclusion and appendix inspected; review background not independently source-checked |
| Elke 2024 local PDF | Introduction/contribution, controller assumptions, results and conclusion inspected; full multibody derivation not independently verified |
| Zhang supplied v2 PDF | Main method, training, results and conclusion read; results pages visually checked; acceptance checked against author record |
| E-Rocket / QuadRocket | Accepted author HTML; architecture and experimental evidence inspected; prior status checks retained |
| Osedo | Publisher full-text setup, model, policy, model-variation experiments and limitations inspected |
| Torrente | Author full-text methodology/implementation and real/sim evaluation inspected; not all derivations audited |
| Li / Cuniato / Neural-Fly / Liu | Earlier scoped reading retained where stated; not presented as newly completed full-paper audits |
| Chih 2024/2026 | No full-text evidence used; excluded from plotted set |

The original Excel corpus is no longer the authoritative selection. The original review slides remain the starting bibliography; this package corrects scope and version details without changing those originals.

No additional upload is needed to use this brief at the meeting. Before committing to a novel technical claim, perform a focused search for that exact question, including the excluded close-system theoretical literature. If you later want a detailed Cuniato (2024) comparison, obtain the actual 2024 paper rather than reuse Zhang's 2026 PDF.

## 11. Suggested opening statement to the professor

“I narrowed the review to experimentally relevant electric TVC platforms, while separating methods developed on different vehicles. The close literature already covers optimal guidance, MPC, PID-based integration, nonlinear allocation and adaptive control. Rather than claim that these methods are absent, I want to identify a measurable limitation on our platform and test one improvement against a matched baseline. The first decision is whether our dominant issue is actuator fidelity, remaining model mismatch, or guidance feasibility.”

Ask for agreement on the **research question and evidence required**, not approval of an algorithm name alone.

## 12. Local PDFs for reading one by one

These links open the existing source files; the originals were not modified. Other papers have publisher/author links in their notes above.

- [Spannagl (2021), final IROS version](<C:/Users/tae06/OneDrive - postech.ac.kr/02 Papers/UGRP/Design, Optimal Guidance and Control of a Low-cost Re-usable Electric Model Rocket - Spannagl et al. - 2021.pdf>)
- [Linsen (2022), ICRA](<C:/Users/tae06/OneDrive - postech.ac.kr/02 Papers/UGRP/Optimal Thrust Vector Control of an Electric Small-Scale Rocket Prototype - Linsen et al. - 2022.pdf>)
- [Chen (2024), TRO](<C:/Users/tae06/OneDrive - postech.ac.kr/02 Papers/UGRP/Design, Modeling, and Control of a Coaxial Drone - Chen et al. - 2024.pdf>)
- [Denton (2022), platform/control paper](<C:/Users/tae06/OneDrive - postech.ac.kr/02 Papers/UGRP/Design, development, and flight testing of a tube-launched coaxial-rotor based micro air vehicle - .pdf>)
- [Denton (2025), identification follow-up](<C:/Users/tae06/OneDrive - postech.ac.kr/02 Papers/UGRP/System identification of a thrust-vectoring, coaxial-rotor-based gun-launched micro air vehicle in h - .pdf>)
- [Zhang (2026), uploaded accepted v2](C:/Users/tae06/Downloads/2602.21583v2.pdf)
- [Elke (2021), proposed surrogate](<C:/Users/tae06/Downloads/Elke Pei Caverly and Gebre-Egziabher (2021) - A Low-Cost and Low-Risk Testbed for Control Design of Launch Vehicles and Landing Systems.pdf>)
- [Elke and Caverly (2024), dynamics/guidance/control](<C:/Users/tae06/Downloads/Elke and Caverly (2024) - Dynamics Guidance and Control of a Low-Cost Quadcopter-Based Space Vehicle Testbed.pdf>)
