import { createAdaptiveModelSupervisor } from './adaptiveModelSupervisor.js';

const finite = (value, fallback = 0) => Number.isFinite(value) ? value : fallback;

export function runAdaptiveModelShadow(simulationResult, options = {}) {
  if (!simulationResult?.samples?.length || !simulationResult?.config) {
    throw new Error('Adaptive shadow replay requires a completed simulation result.');
  }

  const cfg = simulationResult.config;
  const supervisor = createAdaptiveModelSupervisor({
    dt: cfg.dt,
    nominalParameters: cfg.plant,
    windowSize: 2,
    minExcitation: 0,
    excitationHistorySize: 10,
    minInformationRatio: 1e-4,
    validationStride: 4,
    validationBufferSize: 24,
    minValidationSamples: 12,
    minValidationImprovement: 0,
    minUpdatesBeforePublish: 60,
    publishEveryUpdates: 20,
    maxCovarianceTrace: 8,
    maxRelativePublishStep: 0.03,
    ...options,
  });

  const events = [];
  for (let k = 0; k < simulationResult.samples.length - 1; k += 1) {
    const current = simulationResult.samples[k];
    const next = simulationResult.samples[k + 1];
    const update = supervisor.update({
      previousState: {
        x: finite(current.controllerX, current.x),
        v: finite(current.controllerV, current.v),
      },
      nextState: {
        x: finite(next.controllerX, next.x),
        v: finite(next.controllerV, next.v),
      },
      u: finite(current.u),
    });

    if (update.reason !== 'window-filling') {
      events.push({
        t: current.t,
        reason: update.reason,
        updated: Boolean(update.updated),
        excitation: update.excitation ?? null,
        informationRatio: update.informationRatio ?? null,
        validation: update.validation ?? update.publish?.validation ?? null,
        candidate: update.candidate ? { ...update.candidate } : supervisor.getCandidate(),
        publishedModel: update.publishedModel ? { ...update.publishedModel } : supervisor.getPublishedModel(),
      });
    }
  }

  return {
    shadowMode: true,
    controllerModelUnchanged: true,
    nominalModel: { ...cfg.plant },
    candidate: supervisor.getCandidate(),
    publishedModel: supervisor.getPublishedModel(),
    diagnostics: supervisor.getDiagnostics(),
    events,
  };
}
