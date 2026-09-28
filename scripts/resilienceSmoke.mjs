import assert from 'node:assert/strict';
import fs from 'node:fs';

import { defaultConfig } from '../src/core/orchestration/simulationConfig.js';
import {
  loadWorkbenchRecovery,
  saveWorkbenchRecovery,
  WORKBENCH_RECOVERY_KEY,
  WORKBENCH_RECOVERY_SCHEMA,
} from '../src/application/persistence/workbenchRecovery.js';

class MemoryStorage {
  constructor() {
    this.values = new Map();
  }
  getItem(key) {
    return this.values.has(key) ? this.values.get(key) : null;
  }
  setItem(key, value) {
    this.values.set(key, String(value));
  }
  removeItem(key) {
    this.values.delete(key);
  }
}

globalThis.localStorage = new MemoryStorage();

const draftCfg = {
  ...defaultConfig,
  pid: { ...defaultConfig.pid, kp: 6.7 },
  estimation: { ...defaultConfig.estimation, enabled: true, seed: 91 },
};
const runCfg = {
  ...draftCfg,
  setpoint: 1.25,
};

assert.equal(saveWorkbenchRecovery({
  draftCfg,
  runCfg,
  presetId: 'noisy-estimation',
  activeMode: 'HYBRID_SAFE',
}), true);

const restored = loadWorkbenchRecovery();
assert(restored, 'recovery snapshot must load');
assert.equal(restored.draftCfg.pid.kp, 6.7);
assert.equal(restored.draftCfg.pid.ki, defaultConfig.pid.ki, 'recovery must restore nested defaults');
assert.equal(restored.runCfg.setpoint, 1.25);
assert.equal(restored.presetId, 'noisy-estimation');
assert.equal(restored.activeMode, 'HYBRID_SAFE');

const stored = JSON.parse(localStorage.getItem(WORKBENCH_RECOVERY_KEY));
assert.equal(stored.schema, WORKBENCH_RECOVERY_SCHEMA);

localStorage.setItem(WORKBENCH_RECOVERY_KEY, '{broken-json');
assert.equal(loadWorkbenchRecovery(), null, 'corrupt recovery must degrade to defaults');
assert.equal(localStorage.getItem(WORKBENCH_RECOVERY_KEY), null, 'corrupt recovery must be discarded');

localStorage.setItem(WORKBENCH_RECOVERY_KEY, JSON.stringify({
  schema: 'future-schema',
  draftCfg: {},
}));
assert.equal(loadWorkbenchRecovery(), null, 'unknown recovery schema must not be applied');

const sw = fs.readFileSync(new URL('../public/sw.js', import.meta.url), 'utf8');
for (const eventName of ['install', 'activate', 'message', 'fetch']) {
  assert(sw.includes(`addEventListener('${eventName}'`), `service worker must handle ${eventName}`);
}
assert(sw.includes('CACHE_URLS'), 'service worker must accept runtime asset cache messages');
assert(sw.includes("request.mode === 'navigate'"), 'service worker must provide navigation fallback');

const manifest = JSON.parse(fs.readFileSync(new URL('../public/manifest.webmanifest', import.meta.url), 'utf8'));
assert.equal(manifest.display, 'standalone');
assert.equal(manifest.start_url, './');
assert.equal(manifest.scope, './');

console.log('resilience smoke: PASS', {
  recoverySchema: WORKBENCH_RECOVERY_SCHEMA,
  pwaDisplay: manifest.display,
});
