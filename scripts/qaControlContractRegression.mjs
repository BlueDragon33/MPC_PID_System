import assert from 'node:assert/strict';

import {
  ControlContractError,
  assertSafetyGovernorResultContract,
  assertSolverResultContract,
} from '../src/contracts/controlContracts.js';
import { createPIDController } from '../src/core/controllers/pid.js';
import {
  createBicycleConfig,
  stepKinematicBicycle,
} from '../src/core/models/kinematicBicycle.js';
import {
  createPlanarUavConfig,
  stepPlanarUav,
} from '../src/core/models/planarUav.js';
import {
  createPlanarUsvConfig,
  stepPlanarUsv,
} from '../src/core/models/planarUsv.js';
import { createUGVBicycleEkf } from '../src/core/estimation/ugvBicycleEkf.js';
import { createPlanarUavEkf } from '../src/core/estimation/planarUavEkf.js';
import { createPlanarUsvEkf } from '../src/core/estimation/planarUsvEkf.js';
import {
  defaultConfig,
  mergeSimulationConfig,
} from '../src/core/orchestration/simulationConfig.js';
import { assertValidSimulationConfig } from '../src/core/orchestration/simulationConfigValidation.js';
import { solveMPC, SOLVER_BACKENDS } from '../src/core/solvers/index.js';
import {
  applySafetyGovernor,
  computePhysicalCommandInterval,
} from '../src/core/safety/shortHorizonGovernor.js';
import {
  EXPERIMENT_SCHEMA,
  parseExperimentPayload,
} from '../src/core/experiments/serialization.js';

function mustThrowContract(fn, pattern, label) {
  assert.throws(fn, (error) => {
    assert(error instanceof ControlContractError, `${label}: expected ControlContractError, got ${error?.name}`);
    return pattern.test(error.message);
  }, label);
}

// Plant configuration contracts: physically invalid parameters fail before math runs.
assert.deepEqual(createBicycleConfig({}), createBicycleConfig({}));
mustThrowContract(()=>createBicycleConfig({dt:0}),/bicycle\.dt must be > 0/,'UGV zero dt');
mustThrowContract(()=>createBicycleConfig({wheelbase:0}),/wheelbase must be > 0/,'UGV zero wheelbase');
mustThrowContract(()=>createBicycleConfig({minSpeed:5,maxSpeed:2}),/minSpeed must be <= bicycle\.maxSpeed/,'UGV reversed speed bounds');

assert(createPlanarUavConfig({}).mass>0);
mustThrowContract(()=>createPlanarUavConfig({mass:0}),/uav\.mass must be > 0/,'UAV zero mass');
mustThrowContract(()=>createPlanarUavConfig({inertia:-1}),/uav\.inertia must be > 0/,'UAV negative inertia');
mustThrowContract(()=>createPlanarUavConfig({minThrust:20,maxThrust:10}),/minThrust must be <= uav\.maxThrust/,'UAV reversed thrust bounds');

assert(createPlanarUsvConfig({}).mass>0);
mustThrowContract(()=>createPlanarUsvConfig({yawInertia:0}),/usv\.yawInertia must be > 0/,'USV zero yaw inertia');
mustThrowContract(()=>createPlanarUsvConfig({surgeLinearDrag:-1}),/surgeLinearDrag must be >= 0/,'USV negative drag');
mustThrowContract(()=>createPlanarUsvConfig({minSurgeForce:8,maxSurgeForce:4}),/minSurgeForce must be <= usv\.maxSurgeForce/,'USV reversed surge force bounds');

// Runtime state/input contracts: no NaN/Infinity can enter plant integration.
mustThrowContract(
  ()=>stepKinematicBicycle({x:0,y:0,yaw:Number.NaN,v:1},{steer:0,accel:0}),
  /bicycle\.state\.yaw must be finite/,
  'UGV non-finite state',
);
mustThrowContract(
  ()=>stepPlanarUav({x:0,z:1,theta:0,vx:0,vz:0,q:0},{thrust:Number.POSITIVE_INFINITY,torque:0}),
  /uav\.input\.thrust must be finite/,
  'UAV non-finite input',
);
mustThrowContract(
  ()=>stepPlanarUsv({x:0,y:0,psi:0,u:1,v:0,r:0},{surgeForce:0,yawMoment:Number.NaN}),
  /usv\.input\.yawMoment must be finite/,
  'USV non-finite input',
);

// PID contract rejects invalid sampling/bounds and non-finite runtime values.
mustThrowContract(
  ()=>createPIDController({kp:1,ki:0,kd:0,uMin:-1,uMax:1,antiWindup:0.5},0),
  /pid\.dt must be > 0/,
  'PID zero dt',
);
mustThrowContract(
  ()=>createPIDController({kp:1,ki:0,kd:0,uMin:2,uMax:-2,antiWindup:0.5},0.02),
  /pid\.uMin must be <= pid\.uMax/,
  'PID reversed bounds',
);
const pid=createPIDController({kp:1,ki:0,kd:0,uMin:-1,uMax:1,antiWindup:0.5},0.02);
mustThrowContract(()=>pid.update(1,Number.NaN),/pid\.value must be finite/,'PID non-finite measurement');

// Estimator constructor + measurement/update contracts.
mustThrowContract(
  ()=>createUGVBicycleEkf({cfg:{},initialCovariance:[0.3,-0.1,0.08,0.2]}),
  /ugvEkf\.initialCovariance\[1\] must be >= 0/,
  'UGV EKF negative covariance',
);
mustThrowContract(
  ()=>createPlanarUavEkf({cfg:{},measurementVariance:[0.01,0.01,0.01,0.01,0.01,Number.NaN]}),
  /uavEkf\.measurementVariance\[5\] must be finite/,
  'UAV EKF non-finite variance',
);
mustThrowContract(
  ()=>createPlanarUsvEkf({cfg:{},initialState:[0,0,0,0,0]}),
  /usvEkf\.initialState must contain exactly 6 values/,
  'USV EKF wrong state length',
);

const ugvEkf=createUGVBicycleEkf({cfg:{}});
mustThrowContract(
  ()=>ugvEkf.update({x:0,y:0,yaw:Number.NaN,v:0}),
  /ugvEkf\.measurement\.yaw must be finite/,
  'UGV EKF invalid measurement',
);
const uavEkf=createPlanarUavEkf({cfg:{}});
mustThrowContract(
  ()=>uavEkf.predict({thrust:Number.NaN,torque:0}),
  /uavEkf\.input\.thrust must be finite/,
  'UAV EKF invalid input',
);
const usvEkf=createPlanarUsvEkf({cfg:{}});
mustThrowContract(
  ()=>usvEkf.update({x:0,y:0,psi:0,u:0,v:Number.POSITIVE_INFINITY,r:0}),
  /usvEkf\.measurement\.v must be finite/,
  'USV EKF invalid measurement',
);

// Solver contracts: valid backend result stays finite; unknown backend fails closed.
for(const backend of Object.values(SOLVER_BACKENDS)){
  const cfg=mergeSimulationConfig({
    disturbance:{enabled:false},
    mpc:{
      solver:backend,
      horizon:8,
      iterations:8,
      qpIterations:24,
      qpProjectionCycles:6,
    },
  });
  assertValidSimulationConfig(cfg);
  const result=solveMPC({x:0,v:0},1,0,cfg);
  assertSolverResultContract(result,cfg.mpc.horizon);
}
const invalidSolverCfg=mergeSimulationConfig({mpc:{solver:'unsupported-backend'}});
assert.throws(
  ()=>assertValidSimulationConfig(invalidSolverCfg),
  /mpc\.solver is unsupported/,
  'invalid solver config must fail before solve',
);

// Safety authority: unsafe proposal cannot bypass physical/slew interval.
const safetyCfg=mergeSimulationConfig({
  disturbance:{enabled:false},
  mpc:{
    uMin:-2,
    uMax:2,
    deltaUMin:-0.25,
    deltaUMax:0.25,
    stateConstraintsEnabled:false,
    outputConstraintsEnabled:false,
  },
});
const physical=computePhysicalCommandInterval(0,safetyCfg);
assert.equal(physical.feasible,true);
assert.deepEqual({lower:physical.lower,upper:physical.upper},{lower:-0.25,upper:0.25});
const governed=applySafetyGovernor({
  state:{x:0,v:0},
  proposedU:99,
  previousU:0,
  lastMpcSafeU:0,
  continuationSequence:null,
  cfg:safetyCfg,
});
assertSafetyGovernorResultContract(governed);
assert.equal(governed.u,0.25,'safety authority must clamp unsafe proposal to admissible command');
assert.equal(governed.intervened,true);
assert(governed.u>=physical.lower&&governed.u<=physical.upper);

// Experiment/config payload remains fail-closed at application-facing data boundary.
assert.throws(
  ()=>parseExperimentPayload(JSON.stringify({
    schema:EXPERIMENT_SCHEMA,
    config:{...defaultConfig,dt:0},
  })),
  /dt must be > 0/,
);
assert.throws(
  ()=>parseExperimentPayload(JSON.stringify({
    schema:EXPERIMENT_SCHEMA,
    config:{...defaultConfig,mpc:{...defaultConfig.mpc,solver:'not-real'}},
  })),
  /mpc\.solver is unsupported/,
);

console.log('QA-E2 control contract regression: PASS',{
  plantContracts:['UGV','UAV','USV'],
  estimatorContracts:['UGV EKF','UAV EKF','USV EKF'],
  solverBackends:Object.values(SOLVER_BACKENDS),
  safetyProjection:{proposed:governed.proposedU,accepted:governed.u},
  invalidPayloadsRejected:2,
});
