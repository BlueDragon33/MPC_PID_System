# MPC_PID_System — UI-E1 Current Visual + UX Audit

## Authority

UI track: `MPC_PID_FUTURE_PROFESSIONAL_INTERFACE_SYSTEM`

Package: `UI-E1 — Current Visual + UX Audit`

Canonical status remains in `.blueprint/work-packages.json`.

## Baseline audited

Current-main base for UI-E1:

```text
3579bf54d853c6a3fdc02e97338af7870d1daa69
```

The baseline was rendered in Chromium at 1920, 1440, 1280, 1024, 768, 430 and 390 px widths.

Baseline visual audit:
- workflow run: `36545499542`
- artifact: `11022303361`
- screenshots: 7 dark-mode captures + report

## Baseline UI debt

The audit and screenshot review identified:

1. Missing Research Context Header for plant/controller/estimator/revision.
2. Mobile hid the top application status.
3. Repeated 7.5–10 px typography created excessive visual density.
4. No first-class Light/Dark control.
5. Solver Health hard-coded an `Optimal` claim.
6. Development revision was not visible in research context.
7. Mobile had no horizontal overflow, but the dashboard remained vertically long.

The mobile length is retained as explicit later-epoch debt rather than hidden by CSS patches.

## UI-E1 fixes

### Research context

A compact Research Context Header now exposes Plant, Controller, Estimator, Actual Safety and Revision.

Actual safety uses evidence-scoped wording such as:

```text
No violations observed
```

rather than an unconditional `Safe` claim.

### Runtime status

Top-level status is evidence-based:

```text
Idle
Completed
Completed · fallback
Completed · degraded
Unsafe observed
```

Mobile no longer hides the status.

### Solver wording

The hard-coded `Optimal` badge was removed. Observed status now uses `Converged`, `Degraded` or `Fallback observed` based on current metrics.

### Typography readability floor

Authored UI CSS now has:

```text
font declarations below 11 px: 0
```

This is enforced by `presentationContractSmoke.mjs`.

### Theme foundation

A user-facing Light/Dark control was added. UI-E1 does not claim full token-system maturity; deeper token normalization belongs to UI-E2.

### Revision traceability

GitHub Pages build exports `VITE_BUILD_SHA = github.sha`; the Research Context Header shows the short deployed revision. Local builds show `local-dev`.

## Final visual gate

Implementation head reviewed before evidence closeout:

```text
174bd3c1a38ec779206b0a7ccdceb06b3a0a7de5
```

Visual audit:
- workflow run: `36547093154`
- artifact: `11022179976`
- 14 screenshots: 7 viewports × dark/light
- defects: `[]`
- horizontal overflow: none
- Research Context Header: present
- Actual Safety context: present
- mobile status: visible
- theme switching: verified
- keyboard visible focus: verified
- authored CSS below 11 px: zero

Release Readiness run `36547093182` passed on that implementation head, together with inherited architecture, research, safety and resilience regressions.

## Professional review

### Product Designer lens

PASS for UI-E1 scope: hierarchy is clearer, research identity is visible, micro-text density is reduced, and both themes are coherent.

### Control Engineer lens

PASS for UI-E1 scope: Actual Safety remains distinct from predicted feasibility, and solver wording no longer overclaims optimality.

### UX Researcher / New User lens

The first screen now answers plant, controller, estimator, observed safety and revision context.

### Frontend Architect lens

No duplicate shell was introduced, no parallel legacy/new UI was created, and cross-device visual audit is now a reusable blocking gate.

## Deferred UI debt

Not mislabeled as complete:

- UI-E2 Design Token Foundation
- UI-E3 Typography / Surface / Spacing
- UI-E4 Unified App Shell
- UI-E5 Navigation Architecture
- UI-E6 Dashboard Reconstruction
- UI-E7 Experiment Workspace
- UI-E9 Control Chart System
- UI-E12 Safety Visualization
- UI-E16 Responsive Architecture
- UI-E17 Mobile Experience
- UI-E19 Accessibility
- UI-E27 Final Professional Product Audit

## UI-E1 conclusion

UI-E1 establishes an evidence-driven visual baseline and fixes the highest-confidence shell defects discovered by that baseline without rewriting unrelated research UI.

Next queued package:

```text
UI-E2 — Design Token Foundation
```
