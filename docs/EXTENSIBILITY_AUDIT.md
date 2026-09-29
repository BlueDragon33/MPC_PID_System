# MPC_PID_System — E1 Current Extensibility Audit

## Status

Architecture track: `MPC_PID_EXTENSIBLE_CONTROL_PLATFORM_ARCHITECTURE`

Package: `E1 — Current extensibility audit`

Bootstrap revision audited: `760ee1ab43df7f1084f43aaa2318c3b904201f4e`

This document is evidence and migration guidance. The canonical package status remains in `.blueprint/work-packages.json`.

## Scope

E1 does not introduce a registry, provider runtime, plugin framework, or repository rewrite. It establishes a repeatable baseline of the current architecture before migration.

The audit measures:

- vehicle-specific orchestration;
- direct orchestration coupling to model/controller/estimator implementations;
- Application-layer direct Core imports;
- Presentation-to-Core violations;
- current Registry/Provider/Manifest surface;
- preservation of the three validated UGV/UAV/USV runtimes.

## Current findings

### Preserved strengths

- Layer authority is machine-readable in `.blueprint/architecture.json`.
- Presentation remains behind the Application boundary.
- Existing research implementations are separated into model/controller/estimation/orchestration modules.
- UGV, UAV and USV research regressions are already independently gated.
- Development is local-first and does not require a cloud runtime.

### Real coupling found

Current vehicle experiments still use plant-specific orchestration modules:

- `src/core/orchestration/ugvBicycleSimulator.js`
- `src/core/orchestration/planarUavSimulator.js`
- `src/core/orchestration/planarUsvSimulator.js`

Those orchestrators directly import concrete model/controller/estimator/predictive implementations. This is valid for the current research system, but it is the main evidence that the future capability/provider boundary is not yet established.

Application services also import concrete Core entry points such as simulator, experiment and solver modules. That remains valid under the current layered architecture, but future migration should move discovery/composition behind stable capability contracts rather than adding more plant-specific imports.

### Missing extension surface

At E1 baseline there is no first-class platform Registry/Provider/Manifest runtime under `src/`. This is not treated as a defect by itself. It is evidence for the next architecture packages.

The project must not jump directly to a large plugin framework. E2 should introduce only the minimum capability descriptor contract needed by real existing implementations, then E3 can add a registry only after the descriptor is exercised by real providers.

## Migration order derived from evidence

1. **E2 — Capability Descriptor Foundation**
   - define stable descriptor identity/version/classification/capabilities;
   - exercise it with existing real implementations;
   - do not change research behavior.

2. **E3 — Registry Foundation**
   - add deterministic capability registration/resolution;
   - migrate one real capability through the registry;
   - retain backward-compatible direct paths until regression is proven.

3. Plant/controller/estimator provider contracts follow only after the common pattern is demonstrated.

## Non-goals

E1 does not:

- rewrite UGV/UAV/USV;
- merge their mathematical semantics;
- create a God Object;
- add a universal switch statement;
- expose truth state to controllers;
- alter safety authority;
- alter experiment schemas;
- change Production authority.

## Repeatable evidence

Run:

```bash
node scripts/extensibilityAudit.mjs
```

The audit emits `mpc-pid-extensibility-audit/v1` evidence. It blocks Presentation→Core regression and requires the current three validated vehicle runtimes to remain accounted for. Missing registries/providers are reported as migration evidence, not falsely treated as a runtime defect.


## Measured E1 evidence

Architecture Migration Gate implementation-head evidence:

```text
vehicle-specific orchestrators:          3
orchestration direct provider imports:  19
Application direct Core imports:         9
Presentation direct Core imports:        0
Registry files under src:                0
Provider-named files under src:          0
Code manifest files under src:           0
```

Implementation head: `d23adbf57c6924a665400253795989bf3608feb8`

Architecture Migration Gate run: `36543141580` — PASS.

Release Readiness run: `36543141671` — PASS.

All inherited UGV/UAV/USV research gates, Control Safety, Resilience, Architecture Boundaries, Presentation, Experiment Services, Fast CI and build remained green.

## E1 conclusion

The architecture is layered and regression-protected, but the extension boundary has not yet been introduced. The next justified migration is **E2 — Capability Descriptor Foundation**.

E2 should define a small stable descriptor contract and adopt it in real existing implementations. It must not introduce a large registry/plugin runtime before descriptor semantics are proven.
