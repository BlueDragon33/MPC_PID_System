import {
  compareUgvControllers,
  runUgvSimulation,
  UGV_CONTROLLER_MODES,
} from '../../core/orchestration/ugvSimulator.js';
import {
  defaultUgvConfig,
  mergeUgvConfig,
} from '../../core/orchestration/ugvSimulationConfig.js';
import {
  assertUgvComparisonContract,
  assertUgvResultContract,
} from '../../contracts/workbenchContracts.js';

export {
  defaultUgvConfig,
  mergeUgvConfig,
  UGV_CONTROLLER_MODES,
};

export function executeUgvComparison(config = {}) {
  return assertUgvComparisonContract(compareUgvControllers(config));
}

export function executeUgvSimulation(mode, config = {}) {
  return assertUgvResultContract(runUgvSimulation(mode, config));
}
