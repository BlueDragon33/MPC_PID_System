import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';

import {
  buildSimulationView,
  normalizeResponseSignal,
  RESPONSE_SIGNALS,
} from '../src/components/app/simulationViewModel.js';

const results = [
  { mode: 'PID', metrics: { iae: 1 }, samples: [{ t: 0, x: 0, v: 0, u: 0 }] },
  { mode: 'MPC', metrics: { iae: 2 }, samples: [{ t: 0, x: 0, v: 0, u: 0 }] },
  { mode: 'HYBRID', metrics: { iae: 3 }, samples: [{ t: 0, x: 0, v: 0, u: 0 }] },
  { mode: 'HYBRID_SAFE', metrics: { iae: 4 }, samples: [{ t: 0, x: 0, v: 0, u: 0 }] },
];

const pidView = buildSimulationView(results, 'PID');
assert.equal(pidView.active.mode, 'PID', 'selected mode must own Simulation metrics');
assert.deepEqual(pidView.visibleResults.map((result) => result.mode), ['PID'], 'Simulation charts must show one selected mode');

const fallbackView = buildSimulationView(results, 'UNKNOWN');
assert.equal(fallbackView.active.mode, 'HYBRID_SAFE', 'invalid recovered mode must fail safely to HYBRID_SAFE');
assert.deepEqual(fallbackView.visibleResults.map((result) => result.mode), ['HYBRID_SAFE']);

assert.deepEqual(RESPONSE_SIGNALS.map(({ value }) => value), ['x', 'v']);
assert.equal(normalizeResponseSignal('v'), 'v');
assert.equal(normalizeResponseSignal('unsupported'), 'x');
assert.equal(normalizeResponseSignal(null), 'x');

function walk(dir) {
  return fs.readdirSync(dir, { withFileTypes: true }).flatMap((entry) => {
    const full = path.join(dir, entry.name);
    return entry.isDirectory() ? walk(full) : [full];
  });
}

const jsxFiles = walk(path.join(process.cwd(), 'src')).filter((file) => file.endsWith('.jsx'));
const missingButtonTypes = jsxFiles.flatMap((file) => {
  const source = fs.readFileSync(file, 'utf8');
  return [...source.matchAll(/<button\b[^>]*>/g)]
    .filter((match) => !/\btype=/.test(match[0]))
    .map((match) => `${path.relative(process.cwd(), file)}:${source.slice(0, match.index).split('\n').length}`);
});
assert.deepEqual(missingButtonTypes, [], `all buttons must declare type=button: ${missingButtonTypes.join(', ')}`);

const sidebarSource = fs.readFileSync(path.join(process.cwd(), 'src/components/app/ControlSidebar.jsx'), 'utf8');
assert.match(sidebarSource, /aria-expanded=\{controlExpanded\}/, 'configuration chevron must be a functional disclosure control');
assert.match(sidebarSource, /sidebar\.runApplied/, 'Run Simulation needs localized visible confirmation');

const controllerSource = fs.readFileSync(path.join(process.cwd(), 'src/components/app/useWorkbenchController.js'), 'utf8');
assert.match(controllerSource, /normalizeControlMode/, 'recovered control mode must be normalized');
assert.match(controllerSource, /setActiveMode\('HYBRID_SAFE'\)/, 'Reset must restore the default control mode');

const matrixSource = fs.readFileSync(path.join(process.cwd(), 'src/components/app/ExperimentMatrix.jsx'), 'utf8');
assert.match(matrixSource, /setBatchResult\(null\)/, 'Reset dimensions must clear stale batch results');
assert.match(matrixSource, /disabled=\{preset\.id === presetId\}/, 'already loaded presets must expose a disabled state');
assert.match(matrixSource, /executeExperimentMatrixInWorker/, 'long matrix runs must execute off the main UI thread');
assert.match(matrixSource, /aria-busy=\{isRunning\}/, 'matrix execution must expose visible busy state');

const stylesSource = fs.readFileSync(path.join(process.cwd(), 'src/styles.css'), 'utf8');
assert.match(stylesSource, /:root\[data-theme="light"\] \.chart-selector select/, 'response selector must remain readable in light theme');
assert.match(stylesSource, /:root\[data-theme="light"\] \.sidebar-disclosure:hover/, 'configuration disclosure hover must remain readable in light theme');

const packageJson = JSON.parse(fs.readFileSync(path.join(process.cwd(), 'package.json'), 'utf8'));
assert.equal(packageJson.scripts?.['qa:ui-interactions'], 'node scripts/uiInteractionRegression.mjs && node scripts/experimentMatrixWorkerContract.mjs', 'UI interaction QA must include the background matrix contract');

const qaEvidencePath = path.join(process.cwd(), 'docs/QA_UI_INTERACTIONS.md');
assert.equal(fs.existsSync(qaEvidencePath), true, 'UI interaction QA evidence must be documented');

const workPackages = JSON.parse(fs.readFileSync(path.join(process.cwd(), '.blueprint/work-packages.json'), 'utf8'));
const interactionPackage = workPackages.packages.find(({ id }) => id === 'QA-E2A');
assert.equal(interactionPackage?.status, 'PASS', 'QA-E2A must record the completed UI interaction regression');
assert.equal(interactionPackage?.productionAuthority, false, 'UI interaction QA must not grant production authority');

console.log('UI interaction regression: PASS');
