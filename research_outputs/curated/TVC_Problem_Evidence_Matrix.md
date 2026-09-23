# Problem × evidence comparison — all 13 papers

This focused view is part of the [complete TVC Ashby project guide](TVC_Ashby_Project_Guide.md), including all paper notes and source links. Use the guide as the primary reader.

## Problems, methods and validation

![All 13 studies by method and validation, labeled by first author and year](tvc_control_method_map.png)

This is a **categorical evidence plot**. Every selected paper has one black dot, labeled with first author and year. Small offsets separate papers sharing a cell. Distances between method categories have no numerical meaning. The separated “hardware reported; setup unverified” row is unclassified, not an evidence rank.

Xie is labeled **2023**, the journal publication year; its DOI and early online publication date contain 2022. Its full experimental configuration remains unresolved. The revised plot no longer treats the earlier constrained-rig assignment as verified.

### Problem × evidence matrix

To keep the matrix readable, each paper appears under its **principal problem** below. A paper can address additional issues; these assignments are a reading organization, not mutually exclusive scientific categories. The validation column describes the paper's reported evidence, which must still be matched to the exact tested controller.

| Principal problem | Simulation | Constrained rig | Indoor free flight | Outdoor free flight | Hardware configuration unresolved |
|---|---|---|---|---|---|
| Optimal guidance and trajectory tracking | — | — | — | Spannagl (2021); Linsen (2022) | — |
| Conventional stabilization and vehicle transition | — | — | — | Denton (2022); Cai (2024) | — |
| Coupled nonlinear control/allocation | Chih (2024)* | — | Chen (2024), with implementation caveat | — | — |
| Vehicle/system identification | Chih (2026)* | — | Denton (2025) | — | — |
| Servo dynamics inside constrained control | — | — | Li (2024) | — | — |
| Learned control under model variation | — | Osedo (2023) | Cuniato (2024) | — | Xie (2023)* |
| Energy/passivity-based landing control | Durán-Delfín (2026)* | — | — | — | — |
| Residual RL or Neural-Fly-style adaptation on the selected architectures | — | — | — | — | — |

The last row means these methods are absent from the **13-paper selection**. Section 7 contains existing demonstrations on other systems. The matrix therefore supports questions about architecture transfer and validation, rather than a universal claim that the methods have not been used.

Our project would currently be listed separately as **simulation: conventional hover/attitude control and allocation**. It is not a published comparison point or a demonstrated learning-controller result.


## Remaining evidence checks

The guide is complete as a consolidated reading document; the evidence collection remains incomplete. The following limits affect how strongly its conclusions can be used:

| Issue | Current handling | Needed to strengthen the claim |
|---|---|---|
| Chih (2024), Chih (2026), Xie (2023) full text unavailable in the audit | Detailed cells marked provisional or unknown | Obtain and inspect methods, baselines, apparatus and results |
| Durán-Delfín (2026) detailed coding not rechecked | Provisional source-depth flag | Re-open the paper and verify assumptions and experiments |
| Xie experiment type unresolved | Separate unclassified hardware row | Verify whether the apparatus is free flight, constrained or another setup |
| Chen uses different experimental implementations | Explicit caveat in its entry | Attribute each metric and experiment to the actual implemented controller |
| Physical data missing or incomparable | Three audited literature points only | Verify additional all-up mass/thrust pairs and test conditions |
| Scope is a selected search | Empty cells treated as leads | Reproducible database queries and backward/forward citation checks |
| Our final hardware properties not verified here | Simulation values identified as settings | Final mass, thrust, actuator response and sensor measurements |

For a formal review, record search databases, dates, query strings, inclusion/exclusion decisions and duplicate publications. Search the close architecture together with terms such as thrust vector control, coaxial gimbal, residual reinforcement learning, adaptive neural control, system identification and free flight. Read the control law and apparatus before assigning method and validation categories.

The author-year labels use publication years consistently. Xie is 2023 with an early online date in 2022; Cuniato is the 2024 publication with an earlier author preprint. Reading an author copy does not create a separate publication entry.

