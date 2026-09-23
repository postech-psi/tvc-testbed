# Electric TVC testbed: literature positioning and thesis-direction screening

> Superseded selection: use the [narrowed professor package](C:/Users/tae06/CODE/tvc-testbed/research_outputs/professor/TVC_Research_Notes.md). The broader 17-paper map below is retained as an earlier scoping draft, not the current recommendation. The supplied final Spannagl PDF has resolved the version discrepancy noted in the original draft.

Date: 23 September 2026. Status: first-pass scoping review, not a completed systematic review or a novelty claim.

## 1. What we are starting from

This restart uses Lee Taeho's **Literature Review on Electric TVC VTVL Testbeds**, dated 9 September 2026, rather than treating the earlier spreadsheet or 13-paper selection as the final scope. Both supplied files have 11 slides/pages. I inspected every PDF page, extracted the PowerPoint text, and checked its speaker notes, which were empty. The original files are unchanged.

Your clarification is that the professor probably means a **literature-positioning map**. The working aim is therefore to show which research problems have been addressed, how, and with what evidence, then identify questions worth investigating. We do not need artificial physical-property envelopes or numerical scores for “controller complexity.”

The project in slide 2 is an electric TVC VTVL testbed for RLV technology. The repository currently describes a counter-rotating coaxial propulsion unit on a two-axis gimbal, with a future Raspberry Pi 5/Pixhawk implementation. Its documented current demonstration is simulation-based attitude/hover control, not completed autonomous VTVL hardware flight. That distinction matters when judging feasible thesis experiments. See [project README](C:/Users/tae06/CODE/tvc-testbed/README.md).

The most useful starting question comes from slide 10:

> Under which operating conditions does additional controller or model complexity produce a measurable benefit on our electric TVC system?

This is a direction for investigation, not the only possible thesis. Guidance, navigation, model identification and transferability remain alternatives.

## 2. What the presentation already covers

The bibliography contains **11 papers**, but the main vehicle comparison on slide 4 has **six**. Those are different counts, not missing dots.

| Existing paper | Role in the new review | Initial disposition |
|---|---|---|
| Spannagl (2021), EmboRockETH | Close electric TVC GNC baseline | Keep centrally |
| Linsen (2022), electric small-scale rocket | Close integrated optimal GNC baseline | Keep centrally |
| Santos (2026), E-Rocket | Close architecture and implementation baseline | Keep centrally; acceptance checked below |
| Santos (2026), QuadRocket | Rocket-control surrogate and adaptive-control comparator | Keep, with different actuation topology explicit |
| Chen (2024), coaxial drone | Closely related coupling/allocation problem | Keep, separating experiments and controller implementations |
| Denton (2022), tube-launched coaxial MAV | Closely related gimbal/propulsion topology | Keep; distinguish launch/ISR mission from VTVL |
| Dumke (2020), EAGLE | Turbine-powered VTVL GNC context | Keep as mission/system context |
| Scharf (2017), onboard powered-descent guidance | Real-vehicle guidance validation context | Keep in guidance comparison |
| Rmili (2019), FROG | Turbojet/rocket testbed development context | Keep as mission/system context |
| Trawny (2015), terrain-relative navigation and divert guidance | Navigation/guidance validation context | Keep in G/N comparison |
| Carson (2015), ALHAT on Morpheus | Precision-landing navigation context | Keep in G/N comparison |

The five context papers are retained in the inventory, not treated as interchangeable low-level controller experiments. Their details in this restart come from your bibliography and presentation; I have not newly audited their full texts.

## 3. Scope rules for the restart

Use three groups, with no fixed target paper count:

1. **Platform and model studies:** central-gimbal coaxial vehicles and purpose-built electric rocket surrogates. Similar physical structure or a directly relevant surrogate-design question is the inclusion reason.
2. **Method comparators:** a limited set of experimental studies that address a specific candidate mechanism, such as servo-aware control, learned disturbance models, or residual RL. A quadrotor or blimp paper belongs here only for that mechanism, not as an equivalent TVC vehicle.
3. **Mission/GNC context:** rocket and turbine testbeds that explain the target mission, guidance or navigation requirements.

Reviews can help find references but should not be plotted as original experimental contributions. Exclude preprint-only studies from the main evidence set under your preferred publication rule. **An author manuscript hosted on arXiv is still usable when the underlying paper has a verified journal/conference publication or acceptance.** Record acceptance separately from final proceedings publication.

Simulation-only papers on almost the same architecture should remain visible as modeling competitors, with their validation limitation explicit. Excluding them would make a proposed modeling or control novelty look stronger than it is. They must not count as demonstrated flight evidence.

## 4. First positioning map

![Draft literature-positioning map](C:/Users/tae06/CODE/tvc-testbed/research_outputs/restart/tvc_literature_positioning_draft.png)

[Editable SVG](C:/Users/tae06/CODE/tvc-testbed/research_outputs/restart/tvc_literature_positioning_draft.svg) · [Placement data](C:/Users/tae06/CODE/tvc-testbed/research_outputs/restart/positioning_data.json)

**X axis:** primary research emphasis: platform/integrated GNC, actuation/coupling, identification/fidelity, or uncertainty/disturbance rejection.

**Y axis:** main approach used: feedback/allocation, optimization, robust/adaptive/incremental control, identification, learned models/adaptation, direct RL, or residual RL.

Both axes are categorical. The order is a reading arrangement, not a ranking of quality, sophistication, robustness or research maturity. Every label uses first author and year. The two Santos papers also carry their platform names. Color, shape and marker size encode nothing.

The map contains **17 distinct papers: 10 platform/model studies and 7 method comparators**. With the five retained mission-context papers, the first-pass reading inventory contains **22 papers: the original 11 plus 11 additions**. This is a screened starting set, not a claim that these are all relevant papers.

One primary placement per paper keeps the map readable. This is an editorial classification: Spannagl and Linsen also address mismatch, and QuadRocket also addresses coupling and actuation. The GNC matrix below preserves these distinctions. Empty cells mean “no example selected in this first pass,” not “nobody has done this.” Do not draw an “our novel work” dot yet.

## 5. The six papers already central to your slides

### Spannagl (2021): EmboRockETH

**Title:** Design, Optimal Guidance and Control of a Low-cost Re-usable Electric Model Rocket. IROS. [DOI](https://doi.org/10.1109/IROS51168.2021.9636430) · [author manuscript](https://arxiv.org/pdf/2103.04709).

Keep as a close architecture and integrated GNC baseline. It combines optimal guidance, offset-free position MPC, lower-level PID and actuator mapping. Thus “MPC versus PID” is not an accurate description of two mutually exclusive systems here.

**Updated source check (23 September 2026):** the supplied final IROS PDF, Section IV-C, printed p.6350, confirms 19 successful outdoor flights and campaign maxima of 100 m altitude, 50 m divert and 3 m/s. The earlier accessible manuscript was a different version; the slide's figures are supported. Section IV-B includes a battery-voltage/offset-compensation experiment, and Eq.(2e) models first-order thrust response. Calling the paper only a flight demonstration or claiming it ignores actuator dynamics would understate its evidence.

**Updated reading depth:** final publication main text read; printed p.6350 and its figures visually inspected. No independent proof audit.

### Linsen (2022): electric small-scale rocket

**Title:** Optimal Thrust Vector Control of an Electric Small-Scale Rocket Prototype. ICRA. [DOI](https://doi.org/10.1109/ICRA46639.2022.9811938) · [EPFL manuscript](https://infoscience.epfl.ch/server/api/core/bitstreams/ad287a78-7931-46c9-8028-1e541a1e3a40/content).

Keep as a close GNC competitor. The paper formulates guidance and tracking as optimal-control problems and estimates external disturbances and actuator offsets using an EKF. Any proposed “model-based control plus online mismatch correction” must distinguish itself from this.

**Reading depth:** this restart rechecked the institutional abstract; the full-text endpoint was intermittent. Detailed architecture notes exist in the earlier review, but performance and ablation claims require another check against the complete PDF. Do not turn the slide's “no repeated-run metrics” into a field-wide statement.

### Santos (2026): E-Rocket

**Title:** The E-Rocket: Low-cost Testbed for TVC Rocket GNC Validation. [Accepted author manuscript](https://arxiv.org/html/2512.06535v2) · [official IFAC 2026 program, session TuC26.3](https://ifac.papercept.net/conferences/conferences/IFAC26/program/IFAC26_ContentListWeb_2.html#tuc26_03).

Keep as a particularly close platform baseline. Acceptance is stated in the manuscript and the title/authors appear in the official program. Final proceedings metadata still need checking; it should not be excluded merely because the accessible copy is on arXiv.

Its baseline omits actuator dynamics and simplifies force–torque coupling. These are useful hypotheses to test on our platform, not proof that more elaborate control must help. Section 2 describes approximately three minutes of hover capability; the roughly 53 s illustrated run is not an endurance limit. Section 5 describes the position deviations as root-mean-squared.

**Reading depth:** full HTML main text inspected, especially Sections 2–5. The platform/integration contribution should be described neutrally, rather than dismissed as incapable of contributing beyond construction.

### Santos (2026): QuadRocket

**Title:** QuadRocket: An Aerial Robotic Testbed for Adaptive Thrust-Vector Control of Rocket-Like Vehicles. IEEE TAES. [DOI](https://doi.org/10.1109/TAES.2026.3706328) · [accepted author manuscript](https://arxiv.org/html/2607.02474v1).

Keep as an adaptive-control comparator with a different actuator: a quadrotor beneath a universal joint. It uses adaptive backstepping, a control-point transformation for non-minimum-phase behavior, and dynamic-surface control that accounts for the thrust-vector actuator's dynamics. This rules out broad claims that existing electric rocket surrogates ignore adaptation or actuation dynamics.

Its treatment does not automatically solve a serial gimbal's backlash, angle/rate limits or motor–servo interaction. Those distinctions need experiments, not an assumption of novelty. The experiment's main tracking loop runs through a Simulink/RC setup; distinguish that from fully onboard execution.

**Reading depth:** publication status, formulation and selected experimental sections inspected, not every stability proof.

### Chen (2024): coaxial drone

**Title:** Design, Modeling, and Control of a Coaxial Drone. IEEE Transactions on Robotics. [DOI](https://doi.org/10.1109/TRO.2024.3354161) · [author PDF](https://faculty.sustech.edu.cn/wp-content/uploads/2024/05/2024050116133964.pdf).

Keep for the closely related nonlinear actuation/allocation problem. The earlier full-text review flagged an important implementation distinction: custom ascent control and the separate Pixhawk-based hover experiment should not be merged into one validation claim for the same proposed controller.

**Reading depth:** presentation checked now; detailed Section VI-E reading is carried forward from the earlier audit and must be rechecked before extracting quantitative comparisons. This restart does not claim a new full reading.

### Denton (2022): tube-launched coaxial MAV

**Title:** Design, development, and flight testing of a tube-launched coaxial-rotor based micro air vehicle. International Journal of Micro Air Vehicles. [DOI](https://doi.org/10.1177/17568293221117189).

Keep for actuator topology and demonstrated platform operation. Its launch/ISR objective differs from controlled VTVL landing. It supplies an engineering baseline, not direct evidence about landing-controller superiority. Read it together with Denton (2025) rather than assuming the 2022 paper exhausts this group's model-validation work.

**Reading depth:** presentation checked now; architecture details also appear in the earlier review. No new full-paper audit in this restart.

## 6. Papers to add, one by one

### Chih (2024): close-system nonlinear allocation and robust PID

**Title:** Dynamics modeling and nonlinear attitude controller design for a rocket-type unmanned aerial vehicle. ISA Transactions. [DOI](https://doi.org/10.1016/j.isatra.2024.06.029) · [authors' institutional record](https://researchoutput.ncku.edu.tw/en/publications/dynamics-modeling-and-nonlinear-attitude-controller-design-for-a-/).

**Why add:** nearly the same central-gimbal coaxial actuation problem. LMI-based PID design and Levenberg–Marquardt inversion address robust stabilization and coupled allocation. This is a direct competitor to a thesis about improving the allocator or designing a robust baseline.

**Limit:** the abstract reports numerical simulations, not free flight. **Reading:** abstract/institutional record only; full PDF needed before detailed coding.

### Denton (2025): experimentally identified hover dynamics

**Title:** System identification of a thrust-vectoring, coaxial-rotor-based gun-launched micro air vehicle in hovering flight. International Journal of Micro Air Vehicles. [Publisher PDF](https://journals.sagepub.com/doi/pdf/10.1177/17568293251361078).

**Why add:** directly addresses extracting a dynamic model from flight data on a related coaxial TVC vehicle. It is essential before claiming that close-platform research lacks identification or empirical model validation.

**Limit:** hover linearization does not establish a validated nonlinear VTVL envelope. **Reading:** publisher abstract checked in this restart; selected control/identification sections were inspected previously. Model-validation details still need a dedicated extraction pass.

### Chih (2026): nonlinear parameter identification

**Title:** Modeling, Control Stabilization and Parameter Identification of a Thrust Vectoring Rocket-Type Aerial Robot. Applied Mathematical Modelling. [DOI](https://doi.org/10.1016/j.apm.2026.117000) · [publisher record](https://www.sciencedirect.com/science/article/pii/S0307904X26002611).

**Why add:** tests a filtering/regression identification approach for the rocket-type model. Relevant to choosing the model and identifying parameters under measurement noise.

**Limit:** reported validation is simulation. Available online in April 2026; the institutional record assigns it to the October 2026 issue. Online availability precedes that issue date. **Reading:** abstract, highlights and introduction excerpts, not the full paper. Request PDF.

### Elke (2024): rocket-surrogate fidelity

**Title:** Dynamics, Guidance, and Control of a Low-Cost Quadcopter-Based Space Vehicle Testbed. AIAA SciTech. [DOI](https://doi.org/10.2514/6.2024-0778) · [institutional record](https://experts.umn.edu/en/publications/dynamics-guidance-and-control-of-a-low-cost-quadcopter-based-spac/) · [author project and PDFs](https://sites.google.com/umn.edu/rcaverly/research-projects).

**Why add:** directly relevant to slide 10's transferability question. CRQS uses pendulum elements to represent additional rocket-like dynamic effects and compares progressively richer configurations, with landing guidance and LQR tracking.

**Limit:** this AIAA paper reports numerical simulations. The author's separately listed 2024 fabrication/flight paper must not be silently merged into its evidence. **Reading:** institutional abstract/project page; author PDF link found but its viewer did not expose readable content here. Obtain PDF if pursuing this direction.

### Li (2024): actuator-aware NMPC

**Title:** Servo Integrated Nonlinear Model Predictive Control for Overactuated Tiltable-Quadrotors. IEEE Robotics and Automation Letters. [Author/lab publication record](https://www.dragon.t.u-tokyo.ac.jp/publications/references/2024-ral-beetle-art-li/) · [paper](https://arxiv.org/pdf/2405.09871) · [DOI](https://doi.org/10.1109/LRA.2024.3451391).

**Why add:** directly tests the consequences of including servo dynamics in NMPC. It is a crucial comparator for any claim that identifying actuator lag and including it in control is itself novel.

**Limit:** an overactuated tiltable quadrotor has a different feasible wrench set from our central-gimbal vehicle. The actuator-model ablation in Section IV is simulation; real flight validation is a separate section. **Reading:** selected formulation and simulation sections, plus publication record; not an exhaustive read.

### Osedo (2023): RL under model variation on TVC hardware

**Title:** Uniaxial attitude control of uncrewed aerial vehicle with thrust vectoring under model variations by deep reinforcement learning and domain randomization. ROBOMECH Journal. [Open-access paper](https://robomechjournal.springeropen.com/counter/pdf/10.1186/s40648-023-00260-0.pdf).

**Why add:** connects RL and domain randomization to thrust-vector actuation and controlled model variation. Useful for defining training-range versus out-of-range tests.

**Limit:** uniaxial constrained experiments do not demonstrate free-flying six-degree-of-freedom VTVL. **Reading:** abstract rechecked; detailed rig/model classification comes from the prior selected-section review. A complete quantitative extraction remains necessary.

### Cuniato (2024): RL on a multi-actuator tilt-rotor

**Title:** Learning to Fly Omnidirectional Micro Aerial Vehicles with an End-to-End Control Network. Experimental Robotics, ISER proceedings. [Published record](https://link.springer.com/book/10.1007/978-3-031-63596-0?page=2) · [institutional paper](https://www.research-collection.ethz.ch/bitstreams/df9403d3-3a3d-46f7-835a-b8594adde8d3/download).

**Why add:** a hardware-supported direct-RL comparator for a vehicle with tilt-actuator dynamics and substantial control-allocation demands. It prevents the map's RL category from containing only a one-axis rig.

**Limit:** overactuation changes what the controller can achieve. Do not imply that sim-to-real was tuning-free. **Reading:** publication metadata and abstract checked; selected method/experiment sections were inspected earlier, not exhaustively reread now.

### O'Connell (2022): Neural-Fly

**Title:** Neural-Fly enables rapid learning for agile flight in strong winds. Science Robotics. [Accepted paper, supplement and publication record](https://authors.library.caltech.edu/records/q3grb-3vz72).

**Why add:** represents learning-based adaptive disturbance compensation. It learns a shared representation offline, then adapts a small coefficient vector online. It is not reinforcement learning.

**Limit:** quadrotor wind-rejection results do not establish transfer to gimbal servos or the coupled TVC input map. Identify which residual we want to learn before importing the method. **Reading:** institutional abstract and metadata checked here; a derivation-level assessment req neural fluuires the paper and supplement, both available openly.

### Liu (2022): residual RL in real aerial control

**Title:** Deep Residual Reinforcement Learning based Autonomous Blimp Control. IROS. [Institutional publication record](https://is.mpg.de/ps/publications/liu_iros_22) · [author manuscript](https://arxiv.org/abs/2203.05360) · [DOI](https://doi.org/10.1109/IROS47612.2022.9981182).

**Why add:** supplies an aerial example of combining baseline control and a learned policy correction, rather than using a manipulation paper as our principal aerial comparator.

**Limit:** buoyancy and slow blimp dynamics make it a method-transfer reference, not a close plant. Do not infer a safety guarantee merely because a PID baseline is retained. **Reading:** publication record/abstract screened; exact action mixing, reward and real-flight comparisons need full-text extraction.

### Smeur (2018): strong non-learning disturbance rejection

**Title:** Cascaded incremental nonlinear dynamic inversion for MAV disturbance rejection. Control Engineering Practice. [Institutional record and manuscript](https://research.tudelft.nl/en/publications/cascaded-incremental-nonlinear-dynamic-inversion-for-mav-disturba/).

**Why add:** provides a conventional INDI comparator with wind-disturbance experiments. A learning thesis should face a credible non-learning disturbance-rejection method, not only an arbitrarily tuned PID.

**Limit:** transferring INDI requires appropriate measurements, filtering, control effectiveness and actuator treatment on our vehicle. **Reading:** abstract/metadata screened. Read the [corrigendum](https://doi.org/10.1016/j.conengprac.2022.105093) alongside the original; it corrects an actuator transfer function and reports unchanged conclusions.

### Torrente (2021): learned residual dynamics inside MPC

**Title:** Data-Driven MPC for Quadrotors. IEEE Robotics and Automation Letters. [Author paper](https://arxiv.org/pdf/2102.05773) · [author code/publication page](https://github.com/uzh-rpg/data_driven_mpc) · [DOI](https://doi.org/10.1109/LRA.2021.3061307).

**Why add:** separates learning a residual dynamics model from learning a residual policy. Gaussian-process aerodynamic corrections are incorporated into MPC. This is a useful alternative if we want model-based constraint handling with a learned correction.

**Limit:** its high-speed quadrotor aerodynamic problem may not dominate our initial hover regime. **Reading:** abstract and selected introduction/model rationale, not complete experiments or implementation.

## 7. Method and GNC matrix

G = reference/trajectory generation. N = estimating vehicle state. C = feedback tracking/stabilization. Allocation converts requested forces/torques to actuator commands and is part of control. Identification/learning may support any layer; it is not automatically navigation.

This table describes the role of each work, not a novelty score. “Not established” means the present reading does not justify a separate contribution claim. It does not mean the vehicle flies without a state estimate or a reference.

| Paper | G | N | C / allocation | Model or learning role |
|---|---|---|---|---|
| Spannagl (2021) | Optimal guidance | State estimation and offset estimation | Position MPC, inner PID, actuator map | Mismatch and propulsion modeling |
| Linsen (2022) | Optimal guidance | EKF estimates disturbance/offset terms within the architecture | NMPC tracking/stabilization | Identified maps and mismatch compensation |
| Santos (2026), E-Rocket | Supplied offline trajectory | PX4 EKF and indoor motion capture | PID cascade and mapping | Static actuator calibration |
| Santos (2026), QuadRocket | Supplied smooth trajectory | Motion-capture-based state information | Adaptive backstepping and actuator tracking | Disturbance adaptation; coupled model |
| Chen (2024) | Not established as a main contribution | Implementation differs between tests | Nonlinear control/allocation | Coaxial vehicle model |
| Denton (2022) | Launch/hover operation, not an optimal-guidance claim | Details to recheck in full text | Cascaded PID and TVC actuation | Vehicle development/modeling |
| Chih (2024) | Reference tracking | Novel navigation not established | Robust PID and nonlinear inverse allocation | Nonlinear GCRS model |
| Denton (2025) | Excitation maneuvers for identification | Measurement setup supports identification | Stabilizing controller supports experiment | Flight-data hover system identification |
| Chih (2026) | Not established | Filtering for identification is not a navigation claim | Supporting stabilization | Nonlinear parameter identification |
| Elke (2024) | Optimal landing trajectory | Not established from abstract | LQR tracking | Multibody surrogate/fidelity comparison |
| Li (2024) | Supplied reference | State estimate used, not the main contribution | Servo-integrated NMPC | Actuator identification and explicit lag |
| Osedo (2023) | Attitude reference | State measurements used | Learned uniaxial policy | Domain randomization/model variation |
| Cuniato (2024) | Supplied pose task | External state information used | Direct learned control | Sim-to-real training/randomization |
| O'Connell (2022) | Supplied trajectory | Do not classify force adaptation as vehicle-state navigation | Model-based tracking plus adaptive compensation | Offline learned basis; online coefficients |
| Liu (2022) | Aerial task/waypoints | Full interface needs extraction | Baseline plus RL correction | Policy learning rather than explicit physical-model fitting |
| Smeur (2018) | Supplied references | Measurement/filtering supports control | Cascaded INDI | Incremental feedback, not RL |
| Torrente (2021) | Supplied trajectory | State estimate used | MPC | Learned GP residual dynamics |

### Mission-context records retained from the presentation

| Paper | Why retained | Source | Audit status |
|---|---|---|---|
| Dumke (2020) | EAGLE system/GNC validation | [DOI](https://doi.org/10.1007/s12567-019-00269-5) | Slide/bibliography only this pass |
| Scharf (2017) | Onboard powered-descent guidance | [DOI](https://doi.org/10.2514/1.G000399) | Slide/bibliography only this pass |
| Rmili (2019) | FROG testbed and propulsion context | [DOI](https://doi.org/10.13009/EUCASS2019-197) | Slide/bibliography only this pass |
| Trawny (2015) | Terrain-relative N and large-divert G | [DOI](https://doi.org/10.2514/6.2015-4418) | Slide/bibliography only this pass |
| Carson (2015) | ALHAT precision-landing technology | [DOI](https://doi.org/10.2514/6.2015-4417) | Slide/bibliography only this pass |

## 8. Distinctions needed before choosing a learning direction

These are schematic interfaces, not equations copied from every listed paper:

| Candidate mechanism | What it computes | Typical interface |
|---|---|---|
| Conventional feedback / robust control | A command from an error and model/measurements | `u = controller(reference, estimate)` |
| Online parameter or disturbance adaptation | A changing estimate inside a designed control law | `u = controller(..., estimated_parameters)` |
| Learned residual dynamics | A correction to a physics model | `f = f_nominal + residual_model(x,u)` |
| Neural-Fly-style adaptation | Learned basis with low-dimensional online coefficients | `estimated_force = phi(x) a_hat` |
| Direct RL | A policy-selected command | `u = policy(observation)` |
| Residual RL | A learned correction combined with baseline commands | `u = constrained_mix(u_baseline, policy(observation))` |

For TVC, specify **where** a correction enters: desired acceleration, force/torque, gimbal command, motor command, or model prediction. They are different research designs. Summing commands after allocation can violate limits or alter the baseline's stability properties. Keeping a conventional controller does not automatically preserve its guarantees.

Neither an EKF nor an adaptive coefficient update is, by itself, RL. Likewise, a learned force residual is not necessarily a learned action residual.

## 9. Thesis questions the enlarged map can test

These are candidate hypotheses, not established gaps and not a recommendation to implement every method.

| Direction | Testable question on our system | Closest literature to challenge the claim | Evidence needed |
|---|---|---|---|
| Actuator/model fidelity | When do measured lag, rate limits, coupling or thrust-map drift change controller performance or controller ranking? | E-Rocket, Spannagl, Linsen, Chen, Chih, Li, Denton (2025) | Held-out actuator/model validation; matched controller comparisons over a justified operating range |
| Hybrid learning or adaptation | Does a specified learned residual improve robustness beyond a tuned baseline and a strong non-learning compensator? | Linsen, QuadRocket, Smeur, Neural-Fly, Torrente, Osedo, Cuniato, Liu | Same plant, references, estimator and limits; held-out disturbances; no-learning/no-adaptation ablations; training cost and failures |
| Actuation-feasible guidance | Does making the reference respect the actual TVC system's capability improve mission success beyond improving tracking alone? | Spannagl, Linsen, Li; Scharf as mission context | Compare guidance changes separately from controller changes; track constraint violations and landing/task success |
| Electric-to-rocket surrogate validity | Which controller conclusions remain valid when the electric plant is replaced by a specified rocket model? | Elke, QuadRocket, EAGLE/FROG and the real-rocket GNC references | Explicit target dynamics and similarity assumptions; model-to-model sensitivity analysis; separate simulation transfer from real-rocket validation |

A potential thesis need not be “the first RL controller on our shape of vehicle.” A defensible contribution might be a new constraint-aware mechanism, an experimentally supported failure boundary, a validated model-selection rule, or an explanation of when one architecture is worth its cost. A simple benchmark is not automatically a sufficient contribution either; it must answer a consequential, nontrivial question.

Given the repository's present state, actuator identification and a matched simulation benchmark are enabling work for several directions. That is a practical sequencing inference, not evidence that model fidelity must become the thesis.

If onboard navigation without motion capture becomes a central goal, conduct a dedicated N-literature search. The present method additions are mainly C/model studies and do not close that separate literature gap.

## 10. How to turn the scoping map into evidence for the professor

For each retained main paper, extract the same fields into Markdown:

- Research question, claimed contribution, and exact source section/page.
- Physical plant, free-flight versus constrained rig versus simulation.
- G, N, feedback and allocation interfaces, with onboard/offboard execution distinguished.
- Controller model versus simulation model versus real plant.
- Treatment of motor/servo response, rate/travel/saturation limits, coupling, battery effects and uncertainty.
- Comparator tuning, shared test conditions, number of runs, failures and reported variability.
- What the result supports, and what it does not support for our proposed thesis.

Use `unknown/not reported in the inspected material`, never a zero, for missing evidence. Reported wind speeds, position errors and computation times cannot be pooled into a performance ranking without reconciling trajectories, vehicle authority, sensing, hardware and metric definitions.

For our later experiments, a reasonable comparison would hold the reference, estimator, allocator and actuator limits fixed when testing only a feedback mechanism. If one of those components is the proposed contribution, vary it deliberately and state the comparison. Choose trial count through pilot variability and an uncertainty/power argument, rather than inventing a universal minimum.

## 11. PDFs and unresolved evidence

The most useful uploads now would be:

1. **Spannagl's final IROS paper**, or the exact source for slide 5's 100 m / 50 m / 19-test values. The accessible manuscript differs.
2. **Chih (2024)** and **Chih (2026)** full papers. Their abstracts establish relevance but not sufficient implementation detail for a detailed matrix.
3. **Elke (2024), AIAA 2024-0778**, if rocket-surrogate fidelity remains a candidate. The author's public PDF link was located but not readable through the current viewer. Keep its separate fabrication/flight paper separate.

Linsen's institutional PDF endpoint was intermittent. A local copy would also help if that continues. Neural-Fly and its supplement are openly available; no upload is currently necessary just to establish their availability.

I have read your complete presentation, **not all 22 underlying papers end to end**. The per-paper entries above say what was actually inspected. This is enough to justify the additions and a provisional map, but not enough to assert an exhaustive gap or select a final thesis.

## 12. Working recommendation

Keep the six central papers from the slides. Add the four platform/model competitors and seven targeted method comparators above. Retain the original five mission references in their own GNC context section. Use the problem–approach map with the GNC/evidence matrix, and reserve a final thesis claim until the closest competitors and candidate mechanism have received full-text checks.

The next substantive decision is **which unresolved physical/control problem the testbed should expose**, not whether its controller should be called RL, MPC or adaptive control.
