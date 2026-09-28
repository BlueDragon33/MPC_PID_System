import { createAugmentedDisturbanceKalmanFilter } from '../estimation/augmentedDisturbanceKalmanFilter.js';
import { createLinearKalmanFilter } from '../estimation/linearKalmanFilter.js';
import { createMeasurementSensor } from '../estimation/measurementSensor.js';

function covarianceTraceOf(diagnostics) {
  return diagnostics?.covarianceTrace ?? null;
}

function snapshotFrom({
  enabled,
  disturbanceStateEnabled,
  measurementSample,
  state,
  estimate,
}) {
  const covariance = enabled ? estimate?.covariance ?? null : null;
  const diagnostics = enabled ? estimate?.diagnostics ?? null : null;
  return {
    controllerState: enabled
      ? { x: estimate.x, v: estimate.v }
      : { x: state.x, v: state.v },
    measurementSample,
    diagnostics,
    covariance,
    covarianceTrace: enabled ? covarianceTraceOf(diagnostics) : null,
    disturbanceEstimate: disturbanceStateEnabled ? estimate?.d ?? 0 : 0,
    disturbanceVariance: disturbanceStateEnabled ? covariance?.[2]?.[2] ?? null : null,
  };
}

export function createEstimatorRuntime({ cfg, model, initialState = { x: 0, v: 0 } }) {
  const enabled = Boolean(cfg.estimation.enabled);
  const disturbanceStateEnabled = Boolean(enabled && cfg.estimation.disturbanceStateEnabled);
  const disturbancePredictionEnabled = Boolean(
    disturbanceStateEnabled && cfg.estimation.disturbancePredictionEnabled,
  );
  const mpcDisturbanceCompensationEnabled = Boolean(
    disturbanceStateEnabled && cfg.estimation.mpcDisturbanceCompensationEnabled,
  );

  const sensor = createMeasurementSensor({
    C: model.C,
    noiseStd: enabled ? cfg.estimation.measurementNoiseStd : 0,
    seed: cfg.estimation.seed,
    bias: enabled ? cfg.estimation.measurementBias : 0,
  });

  let measurementSample = sensor.read(initialState);
  let estimator = null;
  let snapshot;

  if (enabled) {
    estimator = disturbanceStateEnabled
      ? createAugmentedDisturbanceKalmanFilter({
          A: model.A,
          B: model.B,
          E: model.E,
          C: model.C,
          processCovariance: [
            cfg.estimation.processPositionVariance,
            cfg.estimation.processVelocityVariance,
            cfg.estimation.disturbanceProcessVariance,
          ],
          measurementVariance: Math.max(1e-12, cfg.estimation.measurementNoiseStd ** 2),
          initialState: [initialState.x, initialState.v, 0],
          initialCovariance: [
            cfg.estimation.initialPositionVariance,
            cfg.estimation.initialVelocityVariance,
            cfg.estimation.initialDisturbanceVariance,
          ],
          disturbanceRetention: cfg.estimation.disturbanceRetention,
        })
      : createLinearKalmanFilter({
          A: model.A,
          B: model.B,
          C: model.C,
          processCovariance: [
            cfg.estimation.processPositionVariance,
            cfg.estimation.processVelocityVariance,
          ],
          measurementVariance: Math.max(1e-12, cfg.estimation.measurementNoiseStd ** 2),
          initialState: [initialState.x, initialState.v],
          initialCovariance: [
            cfg.estimation.initialPositionVariance,
            cfg.estimation.initialVelocityVariance,
          ],
        });

    snapshot = snapshotFrom({
      enabled,
      disturbanceStateEnabled,
      measurementSample,
      state: initialState,
      estimate: estimator.update(measurementSample.value),
    });
  } else {
    snapshot = snapshotFrom({
      enabled,
      disturbanceStateEnabled,
      measurementSample,
      state: initialState,
      estimate: null,
    });
  }

  return {
    flags: Object.freeze({
      estimationEnabled: enabled,
      disturbanceStateEnabled,
      disturbancePredictionEnabled,
      mpcDisturbanceCompensationEnabled,
    }),

    current() {
      return snapshot;
    },

    step(controlU, plantState) {
      measurementSample = sensor.read(plantState);
      snapshot = enabled
        ? snapshotFrom({
            enabled,
            disturbanceStateEnabled,
            measurementSample,
            state: plantState,
            estimate: estimator.step(controlU, measurementSample.value),
          })
        : snapshotFrom({
            enabled,
            disturbanceStateEnabled,
            measurementSample,
            state: plantState,
            estimate: null,
          });
      return snapshot;
    },
  };
}
