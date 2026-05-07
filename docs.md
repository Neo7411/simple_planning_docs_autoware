# `docs/` — Upstream Package Description

```
docs/
└── package_description.md
```

The package's `docs/` folder ships **one** file — a long-form description of the package's
purpose, architecture, and APIs. It complements the shorter
[`README.md`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/README.md)
at the package root.

---

## [`docs/package_description.md`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/docs/package_description.md)

A **detailed package description** maintained by the upstream Autoware project. It covers:

| Section | What it documents |
|---|---|
| **1. Purpose** | Why a 2D-only, GPU-free, pure-C++ simulator exists. |
| **2. High-level Architecture** | An ASCII diagram of the node, the model pointer, the timer callback, and the I/O flow. |
| **3. Package Layout** | Tree of `include/`, `src/`, `launch/`, `param/`, `data/`, `media/`, `test/`. |
| **4. The `SimplePlanningSimulator` Node** | Construction, the simulation loop (`on_timer`), command translation (`set_input`), auxiliary callbacks, pitch/Z handling, measurement noise & covariance. |
| **5. ROS Interface (Topics & Services)** | Same tables as the README, but more detail. |
| **6. Vehicle Models** | Full catalogue of the 11 model classes with state-vector layouts and naming conventions. |
| **7. Helper Utilities** | `csv_loader` and `mechanical_controller` summary. |
| **8. Parameters** | High-level parameter overview pointing back to the README's tables. |
| **9. Build & Run** | One-paragraph build / launch instructions. |
| **10. Limitations & Known Caveats** | The gotchas: 2D-only, no collision, no slip, fall-guard at low speed, identity `map → odom`, lanelet-direction limitation, minimal input validation. |
| **11. Where to Look When…** | A cookbook table mapping common questions to the relevant source files. |
| **12. References** | Pointer to the Autoware.AI ancestor (`wf_simulator`) and to companion packages. |

### Why it's worth reading

The doc was written specifically as an **onboarding reference**. If you are reading the
SIMPLANNER_DOCS_CLAUDE docs (this directory) for the first time, that file is a good
intermediate level of detail between the brief README and the line-by-line walkthroughs
here.

It is also the source of the **architecture diagrams** referenced in the rest of the
documentation:

- The block diagram of `SimplePlanningSimulator` ↔ `SimModelInterface` is in section 2.
- The model-naming-convention table (`I_ST_V`, `I_ST_A`, `D_ST_V`, …) is in section 6.

### Relationship to this folder's docs

| Topic | Where to read |
|---|---|
| **High-level architecture** | `package_description.md` §2, or [`overview.md`](overview.md). |
| **Per-file C++ deep-dive** | [`include.md`](include.md), [`src.md`](src.md). |
| **All ROS parameters** | [`README.md`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/README.md) "Inner-workings / Algorithms" table. |
| **Defaults shipped in YAML** | [`param.md`](param.md). |
| **Launch wiring** | [`launch.md`](launch.md). |
| **Tests** | [`test.md`](test.md). |
| **Default actuation maps** | [`data.md`](data.md). |

The upstream `docs/` does **not** currently contain the file
[`docs/actuation_cmd_sim.md`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/docs/actuation_cmd_sim.md)
that the README links to (look in
[`media/vgr_sim.drawio.svg`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/media/vgr_sim.drawio.svg)
for the visual).
