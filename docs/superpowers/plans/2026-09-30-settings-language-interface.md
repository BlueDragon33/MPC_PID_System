# Settings Language and Interface Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add persistent six-mode localization and user-selectable typography/background/font/contrast controls while standardizing icon geometry and making 16 px the default text scale.

**Architecture:** A focused UI-preferences module owns validated values, storage serialization, and DOM attributes. A React provider exposes preferences and translation through the existing app; Settings edits the provider while control-domain state remains separate. CSS data attributes and shared variables implement appearance without touching simulation logic.

**Tech Stack:** React 19, Vite 8, JavaScript modules, Lucide React, Node contract tests, Playwright visual audit.

**Spec:** `docs/specs/settings-language-interface.md`

## Global Constraints

- Supported languages are exactly `en`, `ru`, `vi`, `en-ru`, `vi-ru`, `en-vi`.
- Text sizes are exactly 14, 16, and 18 px; default is 16 px.
- Backgrounds are exactly dark, light, and soft gray; fonts are Inter, System, and Serif; contrast is Standard or High.
- Preferences apply immediately and persist in local storage.
- No changes to control, simulation, estimator, solver, experiment, or safety logic.

## Review Focus

- Corrupt or stale local-storage data must recover to defaults without breaking startup; Task 1 tests invalid JSON and invalid enum values.
- Bilingual output must preserve primary/secondary order and avoid duplicated technical strings; Task 1 tests both cases.
- Theme shortcut and Settings background selector must remain synchronized; Task 2 tests shared preference ownership.
- Larger text must not create horizontal page overflow at mobile widths; Task 3 runs the responsive browser audit.
- Icons must retain square optical boxes and not shrink beside long bilingual labels; Task 3 tests computed geometry.

---

### Task 1: Preferences and localization domain

**Files:**
- Create: `src/interface/uiPreferences.js`
- Create: `src/interface/messages.js`
- Create: `scripts/uiPreferencesContract.mjs`
- Modify: `package.json`

**Interfaces:**
- Produces: `DEFAULT_UI_PREFERENCES`, `normalizeUiPreferences(value)`, `readUiPreferences(storage)`, `writeUiPreferences(storage, value)`, `LANGUAGE_OPTIONS`, `translateMessage(key, language)`.

- [ ] **Step 1: Write the failing contract test** for exact option sets, safe normalization/storage recovery, monolingual translation, bilingual ordering, and identical technical strings.
- [ ] **Step 2: Run `node scripts/uiPreferencesContract.mjs` and verify it fails because the modules do not exist.**
- [ ] **Step 3: Implement the minimal pure preference and message modules.**
- [ ] **Step 4: Run `node scripts/uiPreferencesContract.mjs` and verify all assertions pass.**
- [ ] **Step 5: Add `qa:ui-preferences` to `package.json` and commit.**

### Task 2: React provider and Settings controls

**Files:**
- Create: `src/interface/UiPreferencesContext.jsx`
- Modify: `src/main.jsx`
- Modify: `src/App.jsx`
- Modify: `src/components/app/ControlSidebar.jsx`
- Modify: `src/components/app/WorkspaceViews.jsx`
- Create: `scripts/uiPreferencesIntegration.mjs`

**Interfaces:**
- Consumes: Task 1 preference and translation APIs.
- Produces: `UiPreferencesProvider`, `useUiPreferences()`, synchronized shell/background shortcut, Language and Interface settings cards.

- [ ] **Step 1: Write the failing integration contract** for provider wiring, six language options, four interface control groups, shared background setter, and translated shell/sidebar/settings copy.
- [ ] **Step 2: Run `node scripts/uiPreferencesIntegration.mjs` and verify the missing integration fails.**
- [ ] **Step 3: Implement the provider and wire it into the app shell.**
- [ ] **Step 4: Add Settings language/interface controls and translate the primary shell/sidebar/settings/documentation surfaces.**
- [ ] **Step 5: Run the integration contract and existing interaction/presentation contracts; verify they pass.**
- [ ] **Step 6: Commit.**

### Task 3: Typography, icon geometry, themes, and responsive verification

**Files:**
- Modify: `src/styles.css`
- Modify: `src/components/app/experimentMatrix.css`
- Modify: `src/components/app/researchAnalysis.css`
- Modify: `src/components/app/batchAnalysis.css`
- Modify: `src/components/app/caseExplorer.css`
- Create: `scripts/uiAppearanceContract.mjs`
- Modify: `scripts/uiVisualAudit.mjs`
- Modify: `package.json`

**Interfaces:**
- Consumes: DOM data attributes emitted by Task 2.
- Produces: scalable 14/16/18 typography variables, dark/light/soft backgrounds, font stacks, standard/high contrast, and normalized Lucide icon boxes.

- [ ] **Step 1: Write the failing appearance contract** for default 16 px, data-attribute theme mappings, icon geometry selectors, and no fixed 11 px UI text.
- [ ] **Step 2: Run `node scripts/uiAppearanceContract.mjs` and verify it fails on current CSS.**
- [ ] **Step 3: Implement CSS variables, appearance modes, and icon alignment; mechanically migrate fixed small text to scale variables.**
- [ ] **Step 4: Extend browser audit to check 16 px default, icon square geometry, 18 px mobile overflow, and preference persistence.**
- [ ] **Step 5: Run appearance, integration, interaction, presentation, full smoke, production build, and browser visual audit; verify all pass.**
- [ ] **Step 6: Commit.**

