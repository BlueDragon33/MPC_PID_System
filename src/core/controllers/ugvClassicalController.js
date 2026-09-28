import { normalizeAngle, saturateUgvCommand } from '../models/kinematicBicycle.js';
import { ugvReferenceAtX, ugvTrackingErrors } from '../models/ugvReferencePath.js';

const clamp = (value, lower, upper) => Math.max(lower, Math.min(upper, value));

export function createUgvClassicalController(cfg) {
  let speedIntegral = 0;
  let previousSpeedError = 0;
  let previousCommand = { acceleration: 0, steering: 0 };

  return {
    reset() {
      speedIntegral = 0;
      previousSpeedError = 0;
      previousCommand = { acceleration: 0, steering: 0 };
    },

    update(state) {
      const reference = ugvReferenceAtX(state.x, cfg);
      const error = ugvTrackingErrors(state, reference);
      const gains = cfg.ugvClassical;
      const dt = cfg.dt;

      const speedError = reference.speed - state.v;
      const nextIntegral = clamp(
        speedIntegral + speedError * dt,
        -gains.speedIntegralLimit,
        gains.speedIntegralLimit,
      );
      const speedDerivative = (speedError - previousSpeedError) / Math.max(dt, 1e-9);
      const rawAcceleration =
        gains.speedKp * speedError
        + gains.speedKi * nextIntegral
        + gains.speedKd * speedDerivative;

      const headingCorrection = -normalizeAngle(error.heading);
      const crossTrackCorrection = -Math.atan2(
        gains.stanleyGain * error.lateral,
        Math.abs(state.v) + gains.softeningSpeed,
      );
      const feedforward = gains.curvatureFeedforward
        * Math.atan(cfg.ugv.wheelbase * reference.curvature);
      const rawSteering = headingCorrection + crossTrackCorrection + feedforward;

      const command = saturateUgvCommand(
        { acceleration: rawAcceleration, steering: rawSteering },
        previousCommand,
        cfg,
      );

      const accelerationSaturated = Math.abs(command.acceleration - rawAcceleration) > 1e-12;
      if (!accelerationSaturated || Math.sign(speedError) !== Math.sign(rawAcceleration)) {
        speedIntegral = nextIntegral;
      }
      previousSpeedError = speedError;
      previousCommand = command;

      return {
        command,
        raw: {
          acceleration: rawAcceleration,
          steering: rawSteering,
        },
        reference,
        error,
      };
    },
  };
}
