import assert from 'node:assert/strict';
import { runPlanarUavClassicalBaseline } from '../src/core/orchestration/planarUavSimulator.js';
function estimatedScenario(seed=20260928){return {estimation:{enabled:true,seed,noiseStd:{x:0.08,z:0.06,theta:0.015,vx:0.08,vz:0.07,q:0.04},processCovariance:[1e-5,1e-5,1e-6,3e-4,3e-4,4e-4],initialCovariance:[0.12,0.10,0.03,0.15,0.15,0.08]}};}
const truth=runPlanarUavClassicalBaseline();
const estimatedA=runPlanarUavClassicalBaseline(estimatedScenario(20260928));
const estimatedB=runPlanarUavClassicalBaseline(estimatedScenario(20260928));
const estimatedOtherSeed=runPlanarUavClassicalBaseline(estimatedScenario(20260929));
const trace=result=>result.samples.map(s=>[s.measurementX,s.measurementZ,s.measurementTheta,s.measurementVx,s.measurementVz,s.measurementQ,s.controllerX,s.controllerZ,s.controllerTheta,s.controllerVx,s.controllerVz,s.controllerQ,s.thrust,s.torque]);
assert.deepEqual(trace(estimatedA),trace(estimatedB),'same seed must reproduce UAV measurement/estimate/command trace exactly');
assert.notDeepEqual(trace(estimatedA),trace(estimatedOtherSeed),'different seed must change noisy UAV closed-loop trace');
assert.equal(estimatedA.metrics.estimationEnabled,true);
assert(estimatedA.samples.some(s=>Math.abs(s.controllerX-s.x)>1e-9||Math.abs(s.controllerZ-s.z)>1e-9||Math.abs(s.controllerTheta-s.theta)>1e-9||Math.abs(s.controllerVx-s.vx)>1e-9||Math.abs(s.controllerVz-s.vz)>1e-9||Math.abs(s.controllerQ-s.q)>1e-9),'controller state unexpectedly equals ground truth at every sample');
const m=estimatedA.metrics;
for(const key of ['x','z','theta','vx','vz','q']){assert(Number.isFinite(m.measurementRmse[key]),`measurement RMSE ${key} is not finite`);assert(Number.isFinite(m.estimateRmse[key]),`estimate RMSE ${key} is not finite`);assert(m.estimateRmse[key]<m.measurementRmse[key],`EKF failed to improve ${key}: measurement=${m.measurementRmse[key]}, estimate=${m.estimateRmse[key]}`);}
assert(Number.isFinite(m.averageCovarianceTrace)&&m.averageCovarianceTrace>0,'covariance trace is invalid');
assert.equal(m.unsafeSamples,0,`estimated-state loop produced ${m.unsafeSamples} unsafe samples`);
assert(m.xRmse<=truth.metrics.xRmse*1.35,`estimated-state x RMSE degraded excessively: ${truth.metrics.xRmse} -> ${m.xRmse}`);
assert(m.zRmse<=truth.metrics.zRmse*1.5,`estimated-state z RMSE degraded excessively: ${truth.metrics.zRmse} -> ${m.zRmse}`);
assert(m.vxRmse<=truth.metrics.vxRmse*1.35,`estimated-state vx RMSE degraded excessively: ${truth.metrics.vxRmse} -> ${m.vxRmse}`);
assert(m.vzRmse<=truth.metrics.vzRmse*1.5,`estimated-state vz RMSE degraded excessively: ${truth.metrics.vzRmse} -> ${m.vzRmse}`);
assert(m.attitudeTrackingRmse<=truth.metrics.attitudeTrackingRmse*1.7,`estimated-state attitude RMSE degraded excessively: ${truth.metrics.attitudeTrackingRmse} -> ${m.attitudeTrackingRmse}`);
assert(m.maxThrust<=estimatedA.config.plant.maxThrust+1e-9,`thrust bound exceeded under estimation: ${m.maxThrust}`);
assert(m.maxTorque<=estimatedA.config.plant.maxTorque+1e-9,`torque bound exceeded under estimation: ${m.maxTorque}`);
console.log('Gate 5E UAV planar nonlinear EKF closed-loop smoke PASS');
console.log(JSON.stringify({schema:'mpc-pid-gate5e-uav-ekf/v1',truth:{xRmse:truth.metrics.xRmse,zRmse:truth.metrics.zRmse,vxRmse:truth.metrics.vxRmse,vzRmse:truth.metrics.vzRmse,attitudeTrackingRmse:truth.metrics.attitudeTrackingRmse},estimated:{xRmse:m.xRmse,zRmse:m.zRmse,vxRmse:m.vxRmse,vzRmse:m.vzRmse,attitudeTrackingRmse:m.attitudeTrackingRmse,measurementRmse:m.measurementRmse,estimateRmse:m.estimateRmse,averageCovarianceTrace:m.averageCovarianceTrace,unsafeSamples:m.unsafeSamples,computePerStepUs:m.computePerStepUs}},null,2));
