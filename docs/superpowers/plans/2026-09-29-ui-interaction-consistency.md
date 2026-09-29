# UI Interaction Consistency Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make every mode and visible control in the simulation workbench produce truthful, consistent, testable UI behavior.

**Architecture:** Keep controller mathematics and comparison execution unchanged. Add a small presentation contract that selects the active controller result and chart signal, then make Simulation render only the selected mode while Analysis remains the four-mode comparison surface. Normalize reset, disclosure, loaded-state, and run feedback behavior in presentation state.

**Tech Stack:** React 19, JavaScript ES modules, Vite 8, Node assertion scripts.

**Spec:** `/workspace/scratch/e790b1bfbb38/upload/Văn bản đã dán (1)(5).txt`

## Global Constraints

- Current `main` is authoritative.
- Presentation imports only Application or Presentation modules.
- Control mathematics, safety semantics, and numerical results must remain unchanged.
- Predicted feasibility and actual plant safety remain separate.
- Fixes require regression coverage, build, smoke, browser verification, merge, and Development Pages publish.

## Review Focus

- Selecting PID/MPC/HYBRID/HYBRID_SAFE must render exactly that mode on Simulation charts and cards.
- Velocity/position signal selection must change the response chart without changing simulation results.
- Reset must restore default config, default mode, and clear stale interaction feedback.
- Controls that display a disabled/current state must not still act like enabled operations.
- Analysis must retain full four-mode comparison after Simulation becomes mode-specific.

---

### Task 1: Active-mode presentation contract

**Files:**
- Create: `src/components/app/simulationViewModel.js`
- Create: `scripts/uiInteractionRegression.mjs`
- Modify: `src/components/app/SimulationDashboard.jsx`
- Modify: `src/App.jsx`

**Interfaces:**
- Produces: `buildSimulationView(results, activeMode)` returning `{ active, visibleResults }`.
- Produces: `RESPONSE_SIGNALS` and `normalizeResponseSignal(value)`.

- [ ] Write regression assertions for one visible mode, active metrics, invalid-mode fallback, and supported chart signals.
- [ ] Run the regression and observe the expected missing-module failure.
- [ ] Implement the presentation contract and use it for charts, KPI, health, safety, recent events, and the selected-mode metric table.
- [ ] Run regression and verify PASS.

### Task 2: Functional and consistent controls

**Files:**
- Modify: `src/components/app/SimulationDashboard.jsx`
- Modify: `src/components/app/ControlSidebar.jsx`
- Modify: `src/components/app/ExperimentMatrix.jsx`
- Modify: `src/components/app/useWorkbenchController.js`
- Modify: `src/styles.css`
- Test: `scripts/uiInteractionRegression.mjs`

**Interfaces:**
- Consumes: Task 1 presentation contract.
- Produces: functional response-signal selector, collapsible control section, coherent reset, disabled loaded buttons, cleared stale matrix result, visible run feedback.

- [ ] Add failing source/contract assertions for semantic controls and reset behavior.
- [ ] Run and confirm failures match the inactive controls.
- [ ] Implement minimal control behavior and state feedback.
- [ ] Run regression and build.

### Task 3: Whole-system verification and release

**Files:**
- Modify: `package.json`
- Modify: `docs/QA_UI_INTERACTIONS.md`
- Modify: `.blueprint/work-packages.json`

**Interfaces:**
- Produces: repeatable `qa:ui-interactions` gate and recorded evidence.

- [ ] Add the UI interaction regression command and evidence record.
- [ ] Run UI regression, architecture check, smoke, benchmark, build, and relevant release checks.
- [ ] Verify the deployed browser behavior for every mode, chart selector, navigation, tabs, reset, theme, persistence buttons, scenario buttons, and matrix actions.
- [ ] Commit, push branch, open PR, wait for required gates, merge, wait for Pages deploy, and verify the public URL.
