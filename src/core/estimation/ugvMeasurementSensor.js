import { createSeededGaussian } from './deterministicNoise.js';
import { wrapAngle } from '../models/kinematicBicycle.js';

export function createUGVMeasurementSensor({
  seed = 20260928,
  noiseStd = { x: 0.18, y: 0.18, yaw: 0.035, v: 0.12 },
  bias = { x: 0, y: 0, yaw: 0, v: 0 },
} = {}) {
  const gaussian = createSeededGaussian(seed);
  return {
    read(state) {
      const nx = gaussian(0, noiseStd.x ?? 0);
      const ny = gaussian(0, noiseStd.y ?? 0);
      const nyaw = gaussian(0, noiseStd.yaw ?? 0);
      const nv = gaussian(0, noiseStd.v ?? 0);
      return {
        x: state.x + (bias.x ?? 0) + nx,
        y: state.y + (bias.y ?? 0) + ny,
        yaw: wrapAngle(state.yaw + (bias.yaw ?? 0) + nyaw),
        v: state.v + (bias.v ?? 0) + nv,
        truth: { ...state },
        noise: { x: nx, y: ny, yaw: nyaw, v: nv },
      };
    },
  };
}
