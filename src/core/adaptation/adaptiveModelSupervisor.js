import { createRecursiveLeastSquaresPlantEstimator } from '../estimation/recursiveLeastSquaresPlantEstimator.js';

const finite = (value, fallback) => Number.isFinite(value) ? value : fallback;
const clamp = (value, lo, hi) => Math.max(lo, Math.min(hi, value));

function cloneParameters(value) {
  return {
    stiffness: finite(value?.stiffness, 1),
    damping: finite(value?.damping, 1),
    gain: finite(value?.gain, 1),
  };
}

function relativeStep(from, to, key) {
  return Math.abs(to[key] - from[key]) / Math.max(Math.abs(from[key]), 1e-9);
}

function boundedBlend(from, to, maxRelativeStep) {
  const out = {};
  for (const key of ['stiffness', 'damping', 'gain']) {
    const base = from[key];
    const span = Math.max(Math.abs(base), 1e-6) * maxRelativeStep;
    out[key] = clamp(to[key], base - span, base + span);
  }
  return out;
}

export function createAdaptiveModelSupervisor({
  dt,
  nominalParameters,
  windowSize = 2,
  forgettingFactor = 0.998,
  initialCovariance = 60,
  bounds = {
    stiffnessMin: 0.2,
    stiffnessMax: 4,
    dampingMin: 0.1,
    dampingMax: 3,
    gainMin: 0.2,
    gainMax: 2,
  },
  minExcitation = 0.08,
  minUpdatesBeforePublish = 80,
  publishEveryUpdates = 20,
  maxCovarianceTrace = 5,
  maxRelativePublishStep = 0.03,
} = {}) {
  if (!Number.isFinite(dt) || dt <= 0) throw new Error('Adaptive model supervisor requires dt > 0.');
  const nominal = cloneParameters(nominalParameters);
  const W = Math.max(1, Math.round(windowSize));
  const rls = createRecursiveLeastSquaresPlantEstimator({
    dt,
    initialParameters: nominal,
    initialCovariance,
    forgettingFactor,
    bounds,
  });

  let published = { ...nominal };
  let candidate = { ...nominal };
  let buffer = [];
  let acceptedWindows = 0;
  let rejectedLowExcitation = 0;
  let publishCount = 0;
  let lastPublishUpdate = 0;
  let lastWindow = null;

  function considerPublish() {
    const diagnostics = rls.getDiagnostics();
    const updates = diagnostics?.updates ?? 0;
    const covarianceTrace = diagnostics?.covarianceTrace ?? Number.POSITIVE_INFINITY;
    const enoughData = updates >= minUpdatesBeforePublish;
    const cadenceReady = updates - lastPublishUpdate >= publishEveryUpdates;
    const confidenceReady = covarianceTrace <= maxCovarianceTrace;
    if (!enoughData || !cadenceReady || !confidenceReady) {
      return { published: false, enoughData, cadenceReady, confidenceReady };
    }

    const next = boundedBlend(published, candidate, maxRelativePublishStep);
    const moved = ['stiffness', 'damping', 'gain'].some((key) => Math.abs(next[key] - published[key]) > 1e-12);
    if (moved) {
      published = next;
      publishCount += 1;
    }
    lastPublishUpdate = updates;
    return { published: moved, enoughData, cadenceReady, confidenceReady };
  }

  function update({ previousState, nextState, u }) {
    const previous = {
      x: finite(previousState?.x, 0),
      v: finite(previousState?.v, 0),
    };
    const next = {
      x: finite(nextState?.x, previous.x),
      v: finite(nextState?.v, previous.v),
    };
    const input = finite(u, 0);
    buffer.push({ state: previous, nextState: next, u: input });
    if (buffer.length < W) {
      return { updated: false, reason: 'window-filling', candidate: { ...candidate }, publishedModel: { ...published } };
    }

    const avgX = buffer.reduce((sum, item) => sum + item.state.x, 0) / W;
    const avgV = buffer.reduce((sum, item) => sum + item.state.v, 0) / W;
    const avgU = buffer.reduce((sum, item) => sum + item.u, 0) / W;
    const target = (buffer[W - 1].nextState.v - buffer[0].state.v) / (W * dt);
    const regressor = [-avgX, -avgV, avgU];
    const excitation = Math.sqrt(regressor.reduce((sum, value) => sum + value * value, 0));
    buffer = [];

    lastWindow = { avgX, avgV, avgU, target, excitation };
    if (excitation < minExcitation) {
      rejectedLowExcitation += 1;
      return {
        updated: false,
        reason: 'low-excitation',
        excitation,
        candidate: { ...candidate },
        publishedModel: { ...published },
      };
    }

    const updateResult = rls.updateRegression({
      regressor,
      target,
      context: { aggregationWindow: W, excitation, shadowMode: true },
    });
    candidate = cloneParameters(updateResult.parameters);
    acceptedWindows += 1;
    const publish = considerPublish();
    return {
      updated: true,
      reason: publish.published ? 'published-shadow-model' : 'identified-shadow-candidate',
      excitation,
      candidate: { ...candidate },
      publishedModel: { ...published },
      publish,
      rls: updateResult,
    };
  }

  return {
    update,
    getCandidate() {
      return { ...candidate };
    },
    getPublishedModel() {
      return { ...published };
    },
    getDiagnostics() {
      const rlsDiagnostics = rls.getDiagnostics();
      return {
        shadowMode: true,
        windowSize: W,
        acceptedWindows,
        rejectedLowExcitation,
        publishCount,
        lastPublishUpdate,
        candidate: { ...candidate },
        publishedModel: { ...published },
        candidateRelativeToPublished: {
          stiffness: relativeStep(published, candidate, 'stiffness'),
          damping: relativeStep(published, candidate, 'damping'),
          gain: relativeStep(published, candidate, 'gain'),
        },
        lastWindow: lastWindow ? { ...lastWindow } : null,
        rls: rlsDiagnostics,
      };
    },
    reset() {
      rls.reset(nominal, initialCovariance);
      published = { ...nominal };
      candidate = { ...nominal };
      buffer = [];
      acceptedWindows = 0;
      rejectedLowExcitation = 0;
      publishCount = 0;
      lastPublishUpdate = 0;
      lastWindow = null;
    },
  };
}
