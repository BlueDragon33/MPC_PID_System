import { createSeededGaussian } from './deterministicNoise.js';

export function createMeasurementSensor({ C = [1, 0], noiseStd = 0, seed = 1, bias = 0 } = {}) {
  const gaussian = createSeededGaussian(seed);
  const sigma = Math.max(0, Number.isFinite(noiseStd) ? noiseStd : 0);
  const offset = Number.isFinite(bias) ? bias : 0;

  return {
    read(state) {
      const truth = C[0] * state.x + C[1] * state.v;
      const noise = sigma > 0 ? gaussian(0, sigma) : 0;
      return {
        value: truth + offset + noise,
        truth,
        noise,
        bias: offset,
      };
    },
  };
}
