# MPC_PID_System — Layered Architecture

## Authority order

Universal Constitution → Project Profile → Architecture Manifest → Work Package → Implementation → Test/Gate → Evidence → Release Gate.

This document is explanatory. The machine-readable sources of truth are:

- `.blueprint/constitution-adoption.json`
- `.blueprint/project-profile.json`
- `.blueprint/architecture.json`
- `.blueprint/work-packages.json`

## Non-negotiable rules

1. Fix the root cause in the layer that owns it. Do not patch symptoms across layers.
2. Presentation never imports `src/core/**` directly. UI calls the Application layer.
3. The Application layer coordinates use cases; it does not contain control mathematics.
4. Core mathematics never imports React, browser APIs or presentation state.
5. Simulator truth is not controller truth when estimation is enabled.
6. Predicted QP feasibility and actual plant safety are separate evidence.
7. Adaptive identification remains shadow-only until an explicit promote/rollback gate exists and passes.
8. Constitution PASS, CI PASS or merge does not mean Production PASS.
9. Migration is incremental. Existing verified behavior is preserved at every work-package boundary.
10. Only one work package may be ACTIVE.

## Layers

| Layer | Name | Responsibility |
|---|---|---|
| L00 | Governance | Constitution, blueprint, evidence, release policy |
| L01 | Contracts | Stable config/result/capability contracts |
| L02 | Domain Model | Plant mathematics |
| L03 | Estimation | Sensors, noise, Kalman/RLS primitives |
| L04 | Optimization | MPC prediction, constraints, QP solvers |
| L05 | Control & Safety | PID, event trigger, safety governor |
| L06 | Orchestration | Closed-loop simulator and experiment execution |
| L07 | Research Shadow | Adaptation/identification without implicit control authority |
| L08 | Application | UI-facing workbench/use-case facade |
| L09 | Presentation | React web-app and visualization |
| L10 | Platform | Static shell/assets/offline platform |
| L11 | Verification | Architecture checks, smoke, benchmark, release evidence |

## Dependency direction

Outer layers may depend inward only through the dependencies declared in `.blueprint/architecture.json`.

The first enforced migration rule is:

```
Presentation → Application → Core
```

Direct Presentation → Core imports are forbidden after WP01.

## Migration strategy

The repository is migrated in place, not rebuilt as a big-bang rewrite:

- WP00 establishes authority and machine checks.
- WP01 inserts the application boundary without changing controller behavior.
- WP02 decomposes the simulator behind stable contracts.
- WP03 tightens control/optimization dependencies.
- WP04 isolates experiment/application services.
- WP05 isolates presentation state/routes.
- WP06 hardens offline/recovery behavior.
- WP07 performs Release-mode hardening and exact-head evidence.

Every WP must keep regression gates green before the next WP becomes ACTIVE.
