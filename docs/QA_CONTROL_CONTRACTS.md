# QA-E2 — Control Contract Regression

## Scope

QA track: `MPC_PID_CONTINUOUS_RESEARCH_PRODUCT_QUALITY`

Package: `QA-E2 — Control Contract Regression`

Base main audited before work:

```text
aee08ba1474f82f357fb5ad83724941a792a8c73
```

QA-E2 verifies the control-system contract boundaries required before deeper numerical QA.

## Defects found and fixed

### QA-004 — Invalid nonlinear plant state/config could enter model math

**Severity:** P1 — research/runtime correctness risk.

Affected nonlinear platforms:

- UGV kinematic bicycle;
- planar UAV;
- planar USV.

Examples previously accepted without a hard boundary failure included:

- zero/negative physical time step;
- zero wheelbase;
- zero mass/inertia;
- reversed actuator/speed bounds;
- negative physical drag;
- NaN/Infinity plant state or input.

**Root cause:** vehicle-specific model entry points trusted caller data and relied on clamp/math behavior rather than an explicit domain contract.

**Fix:** reusable control contracts in `src/contracts/controlContracts.js` now validate physical configuration and finite runtime state/input before model integration.

### QA-005 — Nonlinear EKFs silently sanitized invalid data

**Severity:** P1 — estimator/research-evidence risk.

The UGV/UAV/USV nonlinear EKFs used a finite fallback helper that could turn invalid initial or measurement values into numeric defaults.

That behavior could hide a sensor/configuration defect and allow apparently valid estimator output.

**Fix:** EKF constructors and runtime update/predict boundaries now fail closed for:

- incorrect state vector length;
- non-finite initial state;
- negative/non-finite covariance;
- non-finite process covariance;
- non-finite measurement variance;
- non-finite measurement;
- non-finite control input.

The existing numerical covariance stabilization inside the valid estimator path remains unchanged.

## PID contract

PID construction now rejects invalid:

- `dt <= 0`;
- negative gain/anti-windup values;
- reversed output bounds.

Runtime PID calculation rejects non-finite target/value instead of allowing NaN propagation.

## Solver contract

QA-E2 checks all current solver backends:

```text
constrained-qp
box-qp
projected-gradient
```

For valid solver results the regression requires:

- finite first control command;
- finite non-empty sequence;
- sequence length equal to horizon;
- solver backend identity;
- no contradictory non-fallback/non-finite diagnostic state;
- a `solved` constrained-QP result may not carry a hard feasibility violation above tolerance.

Unknown solver IDs continue to fail before execution through the simulation configuration validator.

## Safety authority regression

A deliberately unsafe proposal was injected:

```text
proposed command = 99
previous command = 0
allowed rate interval = [-0.25, +0.25]
```

Observed final command:

```text
accepted command = +0.25
intervened = true
```

This confirms the test case cannot bypass the final safety authority.

Predicted feasibility and actual plant safety remain separate concepts; this test verifies command authority, not a universal safe-system claim.

## Invalid payload regression

QA-E2 explicitly verifies application-facing experiment/config data fails closed for:

- `dt = 0`;
- unsupported solver backend.

QA-E1's wider configuration/import/recovery checks remain active and passed unchanged.

## Implementation-head evidence

Implementation head before ledger/document closeout:

```text
bcc78705bd8a0cdff463afda2b33593bec32d6b9
```

Key workflow results:

- QA-E2 Control Contracts run `36554371017`: PASS
- Release Readiness run `36554371141`: PASS
- QA-E1 Baseline: PASS
- Control Safety Gate: PASS
- Architecture Boundaries: PASS
- Architecture Migration Gate: PASS
- Resilience Gate: PASS
- Presentation Gate: PASS
- UI Visual Audit: PASS
- Gate 4B: PASS
- Gate 5A–5H inherited research regressions: PASS
- Development Fast CI: PASS

QA-E2 output:

```text
plantContracts: UGV, UAV, USV
estimatorContracts: UGV EKF, UAV EKF, USV EKF
solverBackends: constrained-qp, box-qp, projected-gradient
safetyProjection: 99 -> 0.25
invalidPayloadsRejected: 2
```

## Result

QA-E2 establishes explicit fail-closed control boundaries without weakening existing tests or changing valid research behavior.

No unresolved P0/P1 remains in QA-E2 scope.

Next package:

```text
QA-E3 — Numerical Verification Framework
```
