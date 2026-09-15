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

function determinant3(M) {
  return M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1])
    - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
    + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
}

function informationQuality(regressors) {
  const G = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  if (!regressors.length) return { normalizedDeterminant: 0, determinant: 0, trace: 0, matrix: G };

  for (const phi of regressors) {
    for (let i = 0; i < 3; i += 1) {
      for (let j = 0; j < 3; j += 1) G[i][j] += phi[i] * phi[j];
    }
  }
  for (let i = 0; i < 3; i += 1) {
    for (let j = 0; j < 3; j += 1) G[i][j] /= regressors.length;
  }

  const trace = G[0][0] + G[1][1] + G[2][2];
  const determinant = Math.max(0, determinant3(G));
  const isotropicDeterminant = (trace / 3) ** 3;
  const normalizedDeterminant = isotropicDeterminant > 1e-18
    ? clamp(determinant / isotropicDeterminant, 0, 1)
    : 0;
  return { normalizedDeterminant, determinant, trace, matrix: G.map((row) => [...row]) };
}

function predictionRmse(parameters, samples) {
  if (!samples.length) return null;
  const squared = samples.map(({ regressor, target }) => {
    const predicted = regressor[0] * parameters.stiffness
      + regressor[1] * parameters.damping
      + regressor[2] * parameters.gain;
    const error = target - predicted;
    return error * error;
  });
  return Math.sqrt(squared.reduce((sum, value) => sum + value, 0) / squared.length);
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
  excitationHistorySize = 30,
  minInformationRatio = 0,
  validationStride = 0,
  validationBufferSize = 30,
  minValidationSamples = 10,
  minValidationImprovement = 0,
  minUpdatesBeforePublish = 80,
  publishEveryUpdates = 20,
  maxCovarianceTrace = 5,
  maxRelativePublishStep = 0.03,
} = {}) {
  if (!Number.isFinite(dt) || dt <= 0) throw new Error('Adaptive model supervisor requires dt > 0.');
  const nominal = cloneParameters(nominalParameters);
  const W = Math.max(1, Math.round(windowSize));
  const informationWindow = Math.max(3, Math.round(excitationHistorySize));
  const informationThreshold = Math.max(0, finite(minInformationRatio, 0));
  const holdoutStride = Math.max(0, Math.round(validationStride));
  const validationCapacity = Math.max(1, Math.round(validationBufferSize));
  const validationMinimum = Math.max(1, Math.round(minValidationSamples));
  const requiredValidationImprovement = clamp(finite(minValidationImprovement, 0), 0, 0.95);
  const validationEnabled = holdoutStride >= 2;
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
  let regressorHistory = [];
  let validationBuffer = [];
  let completedWindows = 0;
  let validationWindows = 0;
  let acceptedWindows = 0;
  let rejectedLowExcitation = 0;
  let rejectedInformationFilling = 0;
  let rejectedLowInformation = 0;
  let rejectedValidation = 0;
  let publishCount = 0;
  let lastPublishUpdate = 0;
  let lastWindow = null;
  let lastInformation = informationQuality([]);
  let lastValidation = {
    enabled: validationEnabled,
    samples: 0,
    candidateRmse: null,
    publishedRmse: null,
    improvementRatio: null,
    ready: !validationEnabled,
    passed: !validationEnabled,
  };

  function evaluateValidation() {
    if (!validationEnabled) {
      return {
        enabled: false,
        samples: validationBuffer.length,
        candidateRmse: null,
        publishedRmse: null,
        improvementRatio: null,
        ready: true,
        passed: true,
      };
    }
    const candidateRmse = predictionRmse(candidate, validationBuffer);
    const publishedRmse = predictionRmse(published, validationBuffer);
    const ready = validationBuffer.length >= validationMinimum
      && Number.isFinite(candidateRmse)
      && Number.isFinite(publishedRmse);
    const improvementRatio = ready && publishedRmse > 1e-12
      ? 1 - candidateRmse / publishedRmse
      : null;
    const passed = ready && candidateRmse <= publishedRmse * (1 - requiredValidationImprovement);
    return {
      enabled: true,
      samples: validationBuffer.length,
      candidateRmse,
      publishedRmse,
      improvementRatio,
      ready,
      passed,
    };
  }

  function considerPublish() {
    const diagnostics = rls.getDiagnostics();
    const updates = diagnostics?.updates ?? 0;
    const covarianceTrace = diagnostics?.covarianceTrace ?? Number.POSITIVE_INFINITY;
    const enoughData = updates >= minUpdatesBeforePublish;
    const cadenceReady = updates - lastPublishUpdate >= publishEveryUpdates;
    const confidenceReady = covarianceTrace <= maxCovarianceTrace;
    lastValidation = evaluateValidation();
    const validationReady = lastValidation.ready;
    const validationPassed = lastValidation.passed;
    if (!enoughData || !cadenceReady || !confidenceReady || !validationReady || !validationPassed) {
      if (validationEnabled && validationReady && !validationPassed) rejectedValidation += 1;
      return {
        published: false,
        enoughData,
        cadenceReady,
        confidenceReady,
        validationReady,
        validationPassed,
        validation: { ...lastValidation },
      };
    }

    const next = boundedBlend(published, candidate, maxRelativePublishStep);
    const moved = ['stiffness', 'damping', 'gain'].some((key) => Math.abs(next[key] - published[key]) > 1e-12);
    if (moved) {
      published = next;
      publishCount += 1;
      lastValidation = evaluateValidation();
    }
    lastPublishUpdate = updates;
    return {
      published: moved,
      enoughData,
      cadenceReady,
      confidenceReady,
      validationReady,
      validationPassed,
      validation: { ...lastValidation },
    };
  }

  function update({ previousState, nextState, u }) {
    const previous = { x: finite(previousState?.x, 0), v: finite(previousState?.v, 0) };
    const next = { x: finite(nextState?.x, previous.x), v: finite(nextState?.v, previous.v) };
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
    completedWindows += 1;

    const holdoutWindow = validationEnabled && completedWindows % holdoutStride === 0;
    if (holdoutWindow) {
      validationBuffer.push({ regressor: [...regressor], target });
      if (validationBuffer.length > validationCapacity) validationBuffer.shift();
      validationWindows += 1;
      lastValidation = evaluateValidation();
      lastWindow = {
        avgX,
        avgV,
        avgU,
        target,
        excitation,
        informationRatio: lastInformation.normalizedDeterminant,
        holdout: true,
      };
      return {
        updated: false,
        reason: 'validation-holdout',
        excitation,
        validation: { ...lastValidation },
        candidate: { ...candidate },
        publishedModel: { ...published },
      };
    }

    regressorHistory.push([...regressor]);
    if (regressorHistory.length > informationWindow) regressorHistory.shift();
    lastInformation = informationQuality(regressorHistory);
    lastWindow = {
      avgX,
      avgV,
      avgU,
      target,
      excitation,
      informationRatio: lastInformation.normalizedDeterminant,
      holdout: false,
    };

    if (excitation < minExcitation) {
      rejectedLowExcitation += 1;
      return {
        updated: false,
        reason: 'low-excitation',
        excitation,
        informationRatio: lastInformation.normalizedDeterminant,
        candidate: { ...candidate },
        publishedModel: { ...published },
      };
    }

    if (informationThreshold > 0 && regressorHistory.length < informationWindow) {
      rejectedInformationFilling += 1;
      return {
        updated: false,
        reason: 'information-history-filling',
        excitation,
        informationRatio: lastInformation.normalizedDeterminant,
        candidate: { ...candidate },
        publishedModel: { ...published },
      };
    }

    if (informationThreshold > 0 && lastInformation.normalizedDeterminant < informationThreshold) {
      rejectedLowInformation += 1;
      return {
        updated: false,
        reason: 'low-persistent-excitation',
        excitation,
        informationRatio: lastInformation.normalizedDeterminant,
        candidate: { ...candidate },
        publishedModel: { ...published },
      };
    }

    const updateResult = rls.updateRegression({
      regressor,
      target,
      context: {
        aggregationWindow: W,
        excitation,
        informationRatio: lastInformation.normalizedDeterminant,
        shadowMode: true,
      },
    });
    candidate = cloneParameters(updateResult.parameters);
    acceptedWindows += 1;
    const publish = considerPublish();
    return {
      updated: true,
      reason: publish.published ? 'published-shadow-model' : 'identified-shadow-candidate',
      excitation,
      informationRatio: lastInformation.normalizedDeterminant,
      candidate: { ...candidate },
      publishedModel: { ...published },
      publish,
      rls: updateResult,
    };
  }

  return {
    update,
    getCandidate() { return { ...candidate }; },
    getPublishedModel() { return { ...published }; },
    getDiagnostics() {
      const rlsDiagnostics = rls.getDiagnostics();
      lastValidation = evaluateValidation();
      return {
        shadowMode: true,
        windowSize: W,
        excitationHistorySize: informationWindow,
        minInformationRatio: informationThreshold,
        validationEnabled,
        validationStride: holdoutStride,
        validationBufferSize: validationCapacity,
        minValidationSamples: validationMinimum,
        minValidationImprovement: requiredValidationImprovement,
        completedWindows,
        validationWindows,
        acceptedWindows,
        rejectedLowExcitation,
        rejectedInformationFilling,
        rejectedLowInformation,
        rejectedValidation,
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
        information: {
          normalizedDeterminant: lastInformation.normalizedDeterminant,
          determinant: lastInformation.determinant,
          trace: lastInformation.trace,
          matrix: lastInformation.matrix.map((row) => [...row]),
        },
        validation: { ...lastValidation },
        rls: rlsDiagnostics,
      };
    },
    reset() {
      rls.reset(nominal, initialCovariance);
      published = { ...nominal };
      candidate = { ...nominal };
      buffer = [];
      regressorHistory = [];
      validationBuffer = [];
      completedWindows = 0;
      validationWindows = 0;
      acceptedWindows = 0;
      rejectedLowExcitation = 0;
      rejectedInformationFilling = 0;
      rejectedLowInformation = 0;
      rejectedValidation = 0;
      publishCount = 0;
      lastPublishUpdate = 0;
      lastWindow = null;
      lastInformation = informationQuality([]);
      lastValidation = {
        enabled: validationEnabled,
        samples: 0,
        candidateRmse: null,
        publishedRmse: null,
        improvementRatio: null,
        ready: !validationEnabled,
        passed: !validationEnabled,
      };
    },
  };
}
