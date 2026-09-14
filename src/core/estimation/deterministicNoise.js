export function createSeededUniform(seed = 1) {
  let state = (Number(seed) >>> 0) || 1;
  return function nextUniform() {
    state += 0x6D2B79F5;
    let t = state;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

export function createSeededGaussian(seed = 1) {
  const uniform = createSeededUniform(seed);
  let spare = null;

  return function nextGaussian(mean = 0, standardDeviation = 1) {
    if (!(standardDeviation > 0)) return mean;
    if (spare != null) {
      const value = spare;
      spare = null;
      return mean + standardDeviation * value;
    }

    let u1 = 0;
    let u2 = 0;
    while (u1 <= Number.EPSILON) u1 = uniform();
    u2 = uniform();

    const radius = Math.sqrt(-2 * Math.log(u1));
    const angle = 2 * Math.PI * u2;
    const z0 = radius * Math.cos(angle);
    spare = radius * Math.sin(angle);
    return mean + standardDeviation * z0;
  };
}
