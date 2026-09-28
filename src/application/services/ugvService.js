import {
  compareUgvControllers,
  runUgvSimulation,
  UGV_CONTROLLER_MODES,
} from '../../core/orchestration/ugvSimulator.js';
import {
  defaultUgvConfig,
  mergeUgvConfig,
} from '../../core/orchestration/ugvSimulationConfig.js';
import { assertUgvComparisonContract } from '../../contracts/workbenchContracts.js';

export {
  defaultUgvConfig,
  mergeUgvConfig,
  UGV_CONTROLLER_MODES,
};

export function executeUgvComparison(config = {}) {
  return assertUgvComparisonContract(compareUgvControllers(config));
}

export function executeUgvSimulation(mode, config = {}) {
  const result = runUgvSimulation(mode, config);
  assertUgvComparisonContract([
    mode === UGV_CONTROLLER_MODES.CLASSICAL ? result : runUgvSimulation(UGV_CONTROLLER_MODES.CLASSICAL, { ...config, duration: Math.min(config.duration ?? 0.1, 0.1) }),
    mode === UGV_CONTROLLER_MODES.LTV_MPC ? result : runUgvSimulation(UGV_CONTROLLER_MODES.LTV_MPC, { ...config, duration: Math.min(config.duration ?? 0.1, 0.1) }),
  ]);
  return result;
}
