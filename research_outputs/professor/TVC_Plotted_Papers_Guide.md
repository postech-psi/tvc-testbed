# Papers represented in the TVC positioning plots

This is a reading companion to the four standalone figures in [`focused_plots/`](focused_plots/): Figure 1 (hardware mass versus thrust), Figure 2a (close-system control/GNC), Figure 2b (transferable methods), and Figure 3 (validation). There are **eight distinct physically tested publications** in the plots, not eight papers in every plot. Figure 2a and 2b deliberately separate similar hardware from methods demonstrated on different plants. A dot is a descriptive position, not a ranking or a claim of novelty. Wilson is retained below only as an unplotted simulation-based reference.

The blue **Our Vehicle** marker is a project design/simulation/bench point, **not a publication or a free-flight result**. Figure 1 thrust values are not all measured to the same standard; see the [plot guide](TVC_Focused_Plot_Guide.md) before comparing magnitudes. The summaries below describe the papers' demonstrated scope, not everything their methods might eventually accomplish.

## Close thrust-vectoring systems (Figures 1, 2a, and 3)

### Denton et al. (2022) — compact coaxial MAV

**Full citation.** Hunter Denton, Moble Benedict, and Hao Kang. “Design, development, and flight testing of a tube-launched coaxial-rotor based micro air vehicle.” *International Journal of Micro Air Vehicles* 14 (2022). [DOI: 10.1177/17568293221117189](https://doi.org/10.1177/17568293221117189).

**What it does.** This is a small, tube-launched coaxial-rotor aircraft. Vectoring the rotor thrust controls pitch and roll, while differential rotor speed supplies yaw authority. The controller uses cascaded attitude/rate feedback rather than an onboard landing-guidance optimizer.

**What was tested.** The authors report component/rig characterization and indoor/outdoor flight, including launch-to-hover behavior. The publication establishes that this compact configuration can fly; it does **not** demonstrate autonomous VTVL landing or online trajectory optimization. Its dot is the small-system, fixed/pilot-target feedback reference. The 2025 Denton system-identification paper is a substantive follow-up, but is intentionally not a second dot for the same platform family.

**Why read it for our project.** It is a useful hardware and basic-stabilization comparison. It is less useful as evidence about guidance or advanced-model control, because those are not its central experiment.

### Chen et al. (2024) — coaxial gimbal and control allocation

**Full citation.** Liangming Chen, Jiaping Xiao, Yumin Zheng, N. Arun Alagappan, and Mir Feroskhan. “Design, Modeling, and Control of a Coaxial Drone.” *IEEE Transactions on Robotics* 40 (2024): 1650–1663. [DOI: 10.1109/TRO.2024.3354161](https://doi.org/10.1109/TRO.2024.3354161).

**What it does.** This is especially close mechanically: two coaxial rotors and a two-axis servo gimbal. The work develops a dynamics model and nonlinear control allocation with damping to address force/torque coupling. Its plotted custom-control task is ascent toward a target height, rather than a generated landing trajectory.

**What was tested.** The custom ascent experiment uses onboard IMU and time-of-flight sensing. The paper also presents a separate hover demonstration using a Pixhawk-based estimation/control setup. Do not attribute the latter's result wholesale to the custom ascent controller. The authors identify state-measurement error as a limit on longer operation.

**Why read it for our project.** The actuation geometry and coupling are directly relevant. It is also a warning that controller comparisons must distinguish the controller from the estimator and allocation actually used in each flight.

### Spannagl et al. (2021) — onboard optimal guidance and offset-free MPC

**Full citation.** Lukas Spannagl, Elias Hampp, Andrea Carron, Jerome Sieber, Carlo Alberto Pascucci, Aldo U. Zgraggen, Alexander Domahidi, and Melanie N. Zeilinger. “Design, Optimal Guidance and Control of a Low-cost Re-usable Electric Model Rocket.” In *2021 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, 6344–6351. IEEE, 2021. [DOI: 10.1109/IROS51168.2021.9636430](https://doi.org/10.1109/IROS51168.2021.9636430).

**What it does.** A close electric rocket-like TVC vehicle runs free-final-time optimal guidance onboard and tracks the resulting reference using offset-free MPC, inner attitude/rate feedback, and actuator allocation. The model already includes a first-order thrust response and the controller estimates persistent mismatch. “MPC” in Figure 2a names the distinguishing tracking architecture; it does not imply the absence of inner feedback.

**What was tested.** The final IROS paper reports indoor experiments and **19 successful outdoor flights**. The reported campaign maxima—100 m altitude, 50 m diversion, and 3 m/s velocity—are not necessarily from one flight. This is real onboard guidance/control evidence, not merely a simulated trajectory.

**Why read it for our project.** It is a strong integrated-GNC baseline. A thesis cannot safely claim that electric TVC has never used onboard optimal guidance, offset correction, thrust lag, or outdoor flights. A narrower contribution would need a controlled comparison tied to a measured deficiency on our system.

### Linsen et al. (2022) — joint rigid-body NMPC

**Full citation.** Raphael Linsen, Petr Listov, Alberic de Lajarte, Roland Schwan, and Colin N. Jones. “Optimal Thrust Vector Control of an Electric Small-Scale Rocket Prototype.” In *2022 International Conference on Robotics and Automation (ICRA)*, 1996–2002. IEEE, 2022. [DOI: 10.1109/ICRA46639.2022.9811938](https://doi.org/10.1109/ICRA46639.2022.9811938).

**What it does.** On a rocket-like electric TVC platform, a free-time point-mass guidance problem generates a reference; NMPC then jointly tracks position and stabilizes attitude using a rigid-body model. It incorporates propeller maps, gimbal angle/rate and motor limits, a ground-height constraint, and an estimator for disturbances and model parameters. This is meaningfully different from Spannagl's offset-free MPC plus separate attitude cascade, even though both plot in the MPC/NMPC category.

**What was tested.** The paper reports indoor trajectory tracking, an **indoor** fan-array disturbance test, and outdoor ascent/landing experiments. Do not describe the fan experiment as an outdoor wind trial.

**Why read it for our project.** It is the closest check against claims that constraint-aware joint control or online mismatch estimation is absent from TVC rockets. Its specific modelling and estimation choices can motivate a carefully matched actuator-model or robustness experiment.

### Santos et al. (2026) — E-Rocket implementation baseline

**Full citation.** Pedro Santos, André Fonte, Pedro Martins, and Paulo Oliveira. “The E-Rocket: Low-cost Testbed for TVC Rocket GNC Validation.” Accepted for IFAC publication (2026); [accepted author manuscript, arXiv:2512.06535v2](https://arxiv.org/html/2512.06535v2); [official IFAC program](https://ifac.papercept.net/conferences/conferences/IFAC26/program/IFAC26_ContentListWeb_2.html#tuc26_03). **Final proceedings volume, pages, and DOI have not been verified**, so do not invent them in a thesis bibliography.

**What it does.** E-Rocket uses a counter-rotating coaxial motor pair on a two-axis servo gimbal. A Pixhawk/PX4 handles lower-level interfaces and estimation, while a Raspberry Pi/ROS 2 layer runs mission and custom GNC components. The published baseline uses an outer position PID and inner attitude PID to track trajectories prepared offline.

**What was tested.** Indoor motion-capture flight demonstrates trajectory tracking on the physical platform. The architecture could host online guidance, but this paper's demonstrated reference generation is **offline**. Its simplified baseline neglects some coupling/actuator dynamics; that is a modelling choice to test, not proof that PID is inadequate.

**Why read it for our project.** This is probably the cleanest close hardware/software implementation comparison. It makes a simple onboard PID tracker a *known* result on a related coaxial TVC vehicle, while leaving room for a specifically evidenced question about what additional modelling or guidance buys on our platform.

## Method-transfer systems (Figure 2b)

These papers justify **possible methods**, not directly comparable hardware-performance dots. Their plant differences should be explicit when presenting Figure 2b.

### Li et al. (2024) — servo-integrated NMPC

**Full citation.** Jinjie Li, Junichiro Sugihara, and Moju Zhao. “Servo Integrated Nonlinear Model Predictive Control for Overactuated Tiltable-Quadrotors.” *IEEE Robotics and Automation Letters* 9, no. 10 (2024): 8770–8777. [DOI: 10.1109/LRA.2024.3451391](https://doi.org/10.1109/LRA.2024.3451391).

**What it does.** The controller places servo response in the NMPC prediction instead of treating rotor tilt as instantaneous. This is the transferable mechanism. Four independently tilting rotors create an overactuated force/torque set, unlike our central coaxial gimbal.

**What was tested.** The paper reports pose-tracking experiments on a physical tiltable quadrotor. Its isolated actuator-model ablation is in simulation; the hardware demonstrations are a separate evidence type. It is therefore prior art for servo-aware predictive control, not a directly comparable TVC-rocket landing result.

**Why read it for our project.** It suggests testing whether measured gimbal lag changes prediction and closed-loop performance under otherwise matched control conditions.

### Torrente et al. (2021) — learned residual dynamics inside MPC

**Full citation.** Guillem Torrente, Elia Kaufmann, Philipp Föhn, and Davide Scaramuzza. “Data-Driven MPC for Quadrotors.” *IEEE Robotics and Automation Letters* 6, no. 2 (2021): 3769–3776. [DOI: 10.1109/LRA.2021.3061307](https://doi.org/10.1109/LRA.2021.3061307); [authors' project and paper](https://kelia.github.io/publication/data-driven-mpc/).

**What it does.** A Gaussian process learns errors left by a physics-based quadrotor model; the corrected dynamics are used in MPC. The learned function predicts a **dynamics residual**, not actuator commands. This is supervised residual-model learning, **not residual reinforcement learning**.

**What was tested.** Real quadrotor trajectory-tracking experiments compare nominal MPC, a drag-model variant, and GP-augmented MPC. The gains depend on flight condition; the paper does not establish that a learned residual always helps. Its particular high-speed aerodynamic residual should not simply be copied into near-hover coaxial TVC.

**Why read it for our project.** The experimental logic is valuable: establish a physical/identified baseline, show a residual that persists on held-out data, then ask whether adding it to control improves results under matched conditions.

### Osedo et al. (2023) — RL on a constrained TVC attitude rig

**Full citation.** Atsushi Osedo, Daichi Wada, and Shinsaku Hisada. “Uniaxial attitude control of uncrewed aerial vehicle with thrust vectoring under model variations by deep reinforcement learning and domain randomization.” *ROBOMECH Journal* 10, no. 1 (2023), article 20. [DOI: 10.1186/s40648-023-00260-0](https://doi.org/10.1186/s40648-023-00260-0).

**What it does.** A PPO/LSTM reinforcement-learning policy commands thrust and vectoring changes to regulate a specified pitch angle. Domain randomization varies model parameters during training to improve tolerance to uncertain mass/inertia/center of gravity.

**What was tested.** The physical apparatus permits **one-axis pitch motion**, not free flight, translational control, or autonomous landing. Hardware tests include model variations; the out-of-training-range case does not attain the desired angle. The result supports a bounded claim about attitude-regulation robustness, not universal transfer.

**Why read it for our project.** It is close in actuator idea but narrow in task. It shows how to define and test an RL robustness envelope, and why a free-flying TVC RL claim would require substantially more validation.

### Unplotted reference: Wilson and Riccardi (2022) — embedded RL for simulated powered descent

**Full citation.** Callum Wilson and Annalisa Riccardi. “Enabling intelligent onboard guidance, navigation, and control using reinforcement learning on near-term flight hardware.” *Acta Astronautica* 199 (2022): 374–385. [DOI: 10.1016/j.actaastro.2022.07.013](https://doi.org/10.1016/j.actaastro.2022.07.013); [university repository and final paper](https://strathprints.strath.ac.uk/81480/).

**What it does.** An RL agent is initialized using nominal optimal-control demonstrations, then uses an online output-layer update mechanism (Extreme Q-Learning Machine) to handle uncertain powered descent. The policy runs on an NVIDIA Jetson Nano. This is an RL **goal-policy/guidance-and-control** example, not a learned residual model inserted into a conventional TVC tracker.

**What was tested.** The processor is real, but the Mars descent plant is **simulated**. It establishes embedded computational feasibility and simulated descent performance; it is not experimental evidence of a free-flying lander or of our gimbal hardware.

**Why read it for our project.** It is useful when discussing onboard learned decision-making and computation, provided the simulation-versus-flight boundary stays visible.

## How to use the plots to discuss a thesis direction

The literature does not support a blanket gap such as “no onboard guidance,” “no PID,” “no learning,” or “no servo dynamics.” Instead, the defensible next question is **which measured limitation of our vehicle persists after a fair baseline**. If actuator lag/limits dominate, compare a measured actuator model against a static model (Li gives method precedent; Spannagl and Linsen already account for aspects of actuation). If repeatable unmodelled dynamics dominate, compare physical identification, simple offset compensation, and a residual model (Torrente gives the experimental template). If a learned policy is considered, keep its task, sensing, safety envelope, and validation level distinct from Osedo's rig and Wilson's simulated lander. These are candidate comparisons, **not claims that the literature leaves each one untouched**.

For deeper evidence and paper-reading status, see [`TVC_Research_Notes.md`](TVC_Research_Notes.md). The present file is a per-dot reading guide plus one explicitly unplotted reference, not a claim that every mathematical proof in all cited papers was independently audited.
