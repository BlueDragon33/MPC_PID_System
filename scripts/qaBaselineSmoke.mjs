import assert from 'node:assert/strict';

import { defaultConfig, runSimulation } from '../src/core/simulator.js';
import {
  MAX_SIMULATION_STEPS,
  SimulationConfigValidationError,
  assertValidSimulationConfig,
  validateSimulationConfig,
} from '../src/core/orchestration/simulationConfigValidation.js';
import { mergeSimulationConfig } from '../src/core/orchestration/simulationConfig.js';
import { safetyViolationAt } from '../src/core/orchestration/simulationMetrics.js';
import {
  EXPERIMENT_SCHEMA,
  parseExperimentPayload,
  serializeExperiment,
} from '../src/core/experiments/serialization.js';
import {
  loadExperimentLocal,
  saveExperimentLocal,
} from '../src/application/persistence/experimentPersistence.js';
import {
  loadWorkbenchRecovery,
  WORKBENCH_RECOVERY_KEY,
  WORKBENCH_RECOVERY_SCHEMA,
} from '../src/application/persistence/workbenchRecovery.js';

const close = (a,b,tol=1e-10) => Math.abs(a-b) <= tol;

assert.deepEqual(validateSimulationConfig(defaultConfig), []);
assert.equal(assertValidSimulationConfig(defaultConfig), defaultConfig);

const invalidCases = [
  ['zero dt', { dt:0 }, /dt must be > 0/],
  ['negative dt', { dt:-0.01 }, /dt must be > 0/],
  ['zero duration', { duration:0 }, /duration must be > 0/],
  ['non-finite setpoint', { setpoint:Number.NaN }, /setpoint must be finite/],
  ['zero horizon', { mpc:{ horizon:0 } }, /mpc\.horizon must be >= 1/],
  ['reversed actuator bounds', { mpc:{ uMin:2,uMax:-2 } }, /mpc\.uMin must be <= mpc\.uMax/],
  ['reversed state envelope', { mpc:{ stateConstraintsEnabled:true,positionMin:2,positionMax:1 } }, /mpc\.positionMin must be <= mpc\.positionMax/],
  ['negative covariance', { estimation:{ initialPositionVariance:-1 } }, /estimation\.initialPositionVariance must be >= 0/],
  ['invalid trigger ordering', { trigger:{ minInterval:1,maxInterval:0.5 } }, /trigger\.minInterval must be <= trigger\.maxInterval/],
  ['unknown solver', { mpc:{ solver:'magic-solver' } }, /mpc\.solver is unsupported/],
];

for(const [name,patch,pattern] of invalidCases){
  const cfg=mergeSimulationConfig(patch);
  assert.throws(()=>assertValidSimulationConfig(cfg),pattern,name);
  assert.throws(()=>runSimulation('PID',patch),SimulationConfigValidationError,name);
}

const budgetCfg=mergeSimulationConfig({
  duration:(MAX_SIMULATION_STEPS+1)*defaultConfig.dt,
});
assert.throws(
  ()=>assertValidSimulationConfig(budgetCfg),
  /exceeds execution budget/,
  'huge imported experiments must fail before entering the simulation loop',
);

const validEncoded=serializeExperiment(defaultConfig,{name:'qa-baseline'});
const validDecoded=parseExperimentPayload(validEncoded);
assert.equal(validDecoded.schema,EXPERIMENT_SCHEMA);
assert.equal(validDecoded.config.dt,defaultConfig.dt);

const invalidImport=JSON.stringify({
  schema:EXPERIMENT_SCHEMA,
  config:{mpc:{},dt:0,duration:2},
});
assert.throws(()=>parseExperimentPayload(invalidImport),/dt must be > 0/);
assert.throws(()=>parseExperimentPayload('{broken-json'),SyntaxError);
assert.throws(
  ()=>parseExperimentPayload({schema:'mpc-pid-experiment/future',config:{mpc:{}}}),
  /Unsupported experiment schema/,
);

const baseline=runSimulation('PID',{
  duration:0.4,
  disturbance:{enabled:false},
});
const dt=baseline.samples[1]?.t??defaultConfig.dt;
const independentIae=baseline.samples.reduce((sum,sample)=>sum+Math.abs(defaultConfig.setpoint-sample.x)*dt,0);
const independentEffort=baseline.samples.reduce((sum,sample)=>sum+Math.abs(sample.u)*dt,0);
const independentSafety=Math.max(...baseline.samples.map((sample)=>safetyViolationAt(sample,baseline.config)));
assert(close(baseline.metrics.iae,independentIae),'IAE formula must match independent QA calculation');
assert(close(baseline.metrics.controlEffort,independentEffort),'control effort formula must match independent QA calculation');
assert(close(baseline.metrics.maxActualSafetyViolation,independentSafety),'actual safety metric must match independent truth-plant audit');

const stochasticCfg={
  duration:0.5,
  disturbance:{enabled:false},
  estimation:{
    enabled:true,
    measurementNoiseStd:0.12,
    seed:7319,
  },
};
const seededA=runSimulation('PID',stochasticCfg);
const seededB=runSimulation('PID',stochasticCfg);
assert.deepEqual(
  seededA.samples.map(({measurement,estimateX,estimateV,u})=>({measurement,estimateX,estimateV,u})),
  seededB.samples.map(({measurement,estimateX,estimateV,u})=>({measurement,estimateX,estimateV,u})),
  'same seed/config/revision must reproduce the stochastic closed-loop trace',
);
const seededOther=runSimulation('PID',{
  ...stochasticCfg,
  estimation:{...stochasticCfg.estimation,seed:7320},
});
assert.notDeepEqual(
  seededA.samples.map((sample)=>sample.measurement),
  seededOther.samples.map((sample)=>sample.measurement),
  'different seed must change the noisy measurement trace',
);
assert(
  seededA.samples.some((sample)=>Math.abs(sample.controllerX-sample.x)>1e-8),
  'estimated-state run must expose a controller state distinct from truth under sensor noise',
);

class MemoryStorage {
  constructor(){ this.values=new Map(); }
  getItem(key){ return this.values.has(key)?this.values.get(key):null; }
  setItem(key,value){ this.values.set(key,String(value)); }
  removeItem(key){ this.values.delete(key); }
}

const originalStorage=globalThis.localStorage;
globalThis.localStorage=new MemoryStorage();
assert.equal(saveExperimentLocal(defaultConfig,{name:'qa-storage'}),true);
assert(loadExperimentLocal(),'saved experiment must load');

globalThis.localStorage={
  getItem(){ throw new Error('blocked'); },
  setItem(){ throw new Error('quota'); },
  removeItem(){},
};
assert.equal(saveExperimentLocal(defaultConfig,{name:'qa-storage-failure'}),false,'storage write failure must not crash or fake success');
assert.throws(()=>loadExperimentLocal(),/Local experiment storage is unavailable/);

const recoveryStorage=new MemoryStorage();
globalThis.localStorage=recoveryStorage;
recoveryStorage.setItem(WORKBENCH_RECOVERY_KEY,JSON.stringify({
  schema:WORKBENCH_RECOVERY_SCHEMA,
  draftCfg:{dt:0},
  runCfg:{dt:0},
  activeMode:'HYBRID_SAFE',
}));
assert.equal(loadWorkbenchRecovery(),null,'invalid recovery state must fail closed');
assert.equal(recoveryStorage.getItem(WORKBENCH_RECOVERY_KEY),null,'invalid recovery state must be discarded');

if(originalStorage===undefined) delete globalThis.localStorage;
else globalThis.localStorage=originalStorage;

console.log('QA-E1 baseline smoke: PASS',{
  invalidConfigCases:invalidCases.length,
  maxSimulationSteps:MAX_SIMULATION_STEPS,
  seededSamples:seededA.samples.length,
  iae:baseline.metrics.iae,
  controlEffort:baseline.metrics.controlEffort,
  maxActualSafetyViolation:baseline.metrics.maxActualSafetyViolation,
});
