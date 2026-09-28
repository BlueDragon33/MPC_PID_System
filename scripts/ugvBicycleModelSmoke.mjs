import assert from 'node:assert/strict';

import {
  createUgvInitialState,
  normalizeAngle,
  saturateUgvCommand,
  stepKinematicBicycle,
} from '../src/core/models/kinematicBicycle.js';
import {
  ugvReferenceAtX,
  ugvSafetyViolation,
  ugvTrackingErrors,
} from '../src/core/models/ugvReferencePath.js';
import { mergeUgvConfig } from '../src/core/orchestration/ugvSimulationConfig.js';
import { createUgvClassicalController } from '../src/core/controllers/ugvClassicalController.js';

const cfg = mergeUgvConfig();
const state = { x: 0, y: 0, yaw: 0, v: 4 };
const straight = stepKinematicBicycle(state, { acceleration: 0, steering: 0 }, cfg);
assert(Math.abs(straight.x - 0.2) < 1e-10, `straight x step mismatch: ${straight.x}`);
assert(Math.abs(straight.y) < 1e-12);
assert(Math.abs(straight.yaw) < 1e-12);
assert(Math.abs(straight.v - 4) < 1e-12);

const turning = stepKinematicBicycle(state, { acceleration: 0, steering: 0.2 }, cfg);
assert(turning.yaw > 0, 'positive steering must increase yaw');

const saturated = saturateUgvCommand(
  { acceleration: 100, steering: 100 },
  { acceleration: 0, steering: 0 },
  cfg,
);
assert.equal(saturated.acceleration, cfg.ugv.accelerationRateMax * cfg.dt);
assert.equal(saturated.steering, cfg.ugv.steeringRateMax * cfg.dt);

assert(Math.abs(normalizeAngle(3 * Math.PI) - Math.PI) < 1e-12);

const reference = ugvReferenceAtX(7.5, cfg);
for (const value of Object.values(reference)) assert(Number.isFinite(value));
const initial = createUgvInitialState();
const error = ugvTrackingErrors(initial, ugvReferenceAtX(initial.x, cfg));
assert(Number.isFinite(error.lateral));
assert(Number.isFinite(error.heading));
assert(Number.isFinite(error.speed));

const safety = ugvSafetyViolation(initial, cfg);
assert.equal(safety.total, 0, 'default initial UGV state must be inside the declared safety envelope');

const controller = createUgvClassicalController(cfg);
const control = controller.update(initial);
assert(Number.isFinite(control.command.acceleration));
assert(Number.isFinite(control.command.steering));
assert(control.command.acceleration <= cfg.ugv.accelerationMax + 1e-12);
assert(control.command.acceleration >= cfg.ugv.accelerationMin - 1e-12);
assert(control.command.steering <= cfg.ugv.steeringMax + 1e-12);
assert(control.command.steering >= cfg.ugv.steeringMin - 1e-12);

console.log('UGV bicycle model smoke PASS', {
  straightX: straight.x,
  turningYaw: turning.yaw,
  initialLateralError: error.lateral,
  firstCommand: control.command,
});
