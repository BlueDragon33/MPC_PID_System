# UI Interaction Consistency QA

## Scope

This regression covers the research workbench controls reported as visually interactive but behaviorally inconsistent. It changes presentation and presentation-state orchestration only. Control, optimization, estimator, safety, and plant math remain unchanged.

## Root causes and corrections

| Area | Root cause | Correction |
| --- | --- | --- |
| Control mode | `activeMode` only changed header context and the trigger timeline; charts, metrics, solver health, governor state, recent events, and status still consumed the full comparison or `HYBRID_SAFE` result. | A single normalized selected-mode view model now owns every Simulation panel and application status. Analysis remains the explicit four-mode comparison. |
| Response selector | The chart selector was static text with a chevron. | It is now a labeled Position/Velocity select and changes the plotted signal. |
| Configuration heading | The heading displayed a disclosure chevron without a control action. | It is now an accessible disclosure button with `aria-expanded`. |
| Run and reset | Run had no visible acknowledgement. Reset left the selected mode and matrix output stale. | Run exposes an `aria-live` revision acknowledgement. Reset restores default config, baseline preset, `HYBRID_SAFE`, and clears matrix output. |
| Current selections | Loaded scenario/preset and selected best/worst actions remained clickable, while button typing varied. | Current actions expose disabled state and all JSX buttons declare `type="button"`. |

## Repeatable gate

```sh
npm run qa:ui-interactions
```

The gate verifies selected-mode ownership, invalid-mode fallback, response-signal normalization, semantic button types, working disclosure/run feedback, reset semantics, matrix stale-state clearing, and this evidence/work-package record.

## Verification matrix

- UI interaction regression
- Presentation contract smoke
- Architecture boundary check
- QA baseline and control contract regression
- Full control smoke suite
- Full research benchmark suite
- Extensibility and release security checks
- Production build
- Post-deploy browser acceptance on GitHub Pages

The exact commit and GitHub Actions run remain authoritative release evidence in the pull request and deployment history.
