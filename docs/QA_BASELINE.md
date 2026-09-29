# MPC_PID_System — QA-E1 Current System Baseline Audit

## Scope

Track: `MPC_PID_CONTINUOUS_RESEARCH_PRODUCT_QUALITY`

Package: `QA-E1 — Current System Baseline Audit`

Base main audited before implementation:

```text
fbad404b3b8fbf1efc9fb9b5b615c713ef3a42f9
```

The QA baseline challenged current behavior across configuration validity, numerics, control/safety regression, experiment persistence, reproducibility, browser behavior, accessibility, responsive rendering, offline/recovery contracts and release readiness.

## Test matrix

| QA area | Evidence |
| --- | --- |
| Static / architecture | architectureCheck + Constitution + architecture workflows |
| Configuration contract | fail-closed validation + 10 invalid-config regressions |
| Numerical correctness | finite-input checks + independent metric calculation |
| Control / safety | full `npm run smoke` + Control Safety + Gate 4B/5A-5H |
| Estimation / reproducibility | same-seed trace equality + different-seed measurement divergence |
| Experiment import/export | valid roundtrip + invalid import rejection |
| Storage / recovery | quota/write failure + corrupt recovery fail-closed |
| Browser journey | navigation, import, run, save failure recovery |
| Accessibility baseline | one h1, visible focus, SVG title/desc semantics |
| Responsive | 1920/1440/1280/1024/768/430/390 |
| Themes | dark + light |
| Offline/PWA | resilience contract + Release Readiness offline smoke |
| Deployment boundary | development-only; protected Production not invoked |

## Fixed defects

### QA-001 — Invalid experiment/recovery config could reach simulation runtime

**Severity:** P1

**Root cause:** configuration merge supplied defaults but did not validate scientific/numerical invariants before runtime/import/recovery.

A schema-valid payload could contain values such as:

```text
dt = 0
duration <= 0
horizon = 0
uMin > uMax
positionMin > positionMax
negative covariance
invalid trigger interval ordering
unsupported solver
```

The most dangerous case was `dt = 0`, because the simulation step count derived from `duration / dt` could become unbounded before the loop.

**Fix:** added a fail-closed simulation configuration contract before:

- simulation runtime;
- experiment import;
- experiment export;
- workbench recovery.

The contract also rejects pathological simulations above an explicit 1,000,000-step execution budget before entering the loop.

**Regression:** `scripts/qaBaselineSmoke.mjs`.

**Retest:** PASS.

### QA-002 — Experiment save could throw on storage/quota failure

**Severity:** P2

**Root cause:** `saveExperimentLocal` called `localStorage.setItem` without failure containment.

**Fix:** storage write failure now returns `false`; storage read unavailability produces a controlled user-facing load error.

**Browser evidence:** Save under injected `QuotaExceededError` reports local storage unavailable and does not emit uncaught page/console errors.

**Retest:** PASS.

### QA-003 — Accessibility baseline lacked primary heading and textual chart description

**Severity:** P2

**Root cause:** visual hierarchy existed without a semantic `h1`; SVG charts had accessible labels but no descriptive text node.

**Fix:**

- one semantic `h1` in Research Context;
- `title` + `desc` for primary control charts;
- regression contract protects both.

**Retest:** PASS across browser and visual matrix.

### QA-HARNESS-001 — Ambiguous browser locator

**Classification:** Test defect, not product defect.

First QA browser attempt used non-exact `Simulation` button lookup and Playwright correctly rejected three matching controls.

**Fix:** exact accessible-name locator; the test was preserved and rerun, not skipped or weakened.

**Retest:** PASS.

## Numerical / reproducibility findings

QA-E1 independently recomputed metrics from simulation samples on a short PID run:

```text
IAE                         0.38076639061888246
Control effort              1.6504035765023823
Max actual safety violation 0
```

The independently calculated values matched product metrics within the QA tolerance.

Seeded estimation checks also verified:

```text
same revision + same config + same seed
→ identical measurement / estimate / command trace

different seed
→ different noisy measurement trace
```

No claim is made beyond the tested deterministic contract.

## Browser acceptance

Final QA browser run verified:

```text
invalidImportRejected    true
validImportAccepted      true
storageFailureRecovered  true
consoleErrors            []
pageErrors               []
mobile width             390
mobile horizontal scroll 390 == viewport
status visible           true
actual safety visible    true
```

## Cross-device visual evidence

Final QA visual audit:

```text
1920
1440
1280
1024
768
430
390
```

was executed in both Dark and Light themes.

Result:

```text
defects                    []
horizontal overflow         none
tiny visible text           0
Research Context Header     present
Actual Safety               visible
Theme control               present
h1 count                    1
keyboard focus outline      visible
```

QA artifact:

```text
workflow run 36550437779
artifact     11024960306
sha256       c6ee18a4f43096860d387bada91f1399f64edc545f96b5215c9db94839fe1bc4
```

## Regression result

On implementation head:

```text
5ff29be0252d4909cc8546f5198aae605de3cb44
```

the following passed:

- QA-E1 Baseline;
- Universal Constitution Compliance;
- Architecture Boundaries;
- Architecture Migration;
- Development Fast CI;
- Presentation Gate;
- UI Visual Audit;
- Resilience Gate;
- Experiment Services Gate;
- Control Safety Gate;
- Gate 4B;
- Gate 5A UGV;
- Gate 5B UGV EKF;
- Gate 5C UGV Predictive Governor;
- Gate 5D UAV;
- Gate 5E UAV EKF;
- Gate 5F UAV Predictive;
- Gate 5G USV;
- Gate 5H USV EKF;
- Release Readiness.

Release Readiness run:

```text
36550437846
PASS
```

## Human QA review

### Senior QA Engineer

No P0/P1 remains in QA-E1 scope after fixes and regression.

### Control Engineer

No change to controller law, estimator routing, solver authority or actual-safety definition was introduced by the QA fixes. Existing control/safety gates remain green.

### Product Designer / New Researcher

Research Context and Actual Safety remain immediately visible. Input/import/storage failures now fail with controlled behavior rather than allowing corrupted runtime state.

### Power User

No existing navigation or controller-selection flow was removed. Full research regressions remain green.

## Known deferred issue

### QA-DEBT-001 — Mobile dashboard remains vertically long

**Severity:** P2 UX debt.

The 390/430 px layouts have no horizontal overflow, hidden safety state or unreachable primary control, but the complete dashboard still requires substantial vertical scrolling.

This is not patched in QA-E1 because a structural fix belongs to the already-defined Responsive Architecture / Mobile Experience UI epochs. A CSS compression patch here would violate root-cause and no-patchwork rules.

This debt is documented rather than presented as complete.

## QA-E1 conclusion

```text
SCOPE PASS
NO P0/P1
CONTROL REGRESSION PASS
SAFETY REGRESSION PASS
REPRODUCIBILITY PASS
BROWSER PASS
RESPONSIVE BASELINE PASS
OFFLINE/RECOVERY PASS
RELEASE READINESS PASS
```

Final exact-head gates must be rerun after this evidence/ledger commit before merge.

Next QA package:

```text
QA-E2 — Control Contract Regression
```

Protected Production Release remains owner-only and is not part of QA-E1.
