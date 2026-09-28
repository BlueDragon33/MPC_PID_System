import { compareControllers, runSimulation } from '../../core/simulator.js';
import { defaultConfig } from '../../core/orchestration/simulationConfig.js';
import { SOLVER_BACKENDS } from '../../core/solvers/index.js';
import {
  assertControllerComparisonContract,
  assertSimulationResultContract,
} from '../../contracts/workbenchContracts.js';

export { defaultConfig, SOLVER_BACKENDS };

export function executeControllerComparison(config) {
  return assertControllerComparisonContract(compareControllers(config));
}

export function executeSimulation(mode, config) {
  return assertSimulationResultContract(runSimulation(mode, config));
}

export const compareControllersForWorkbench = executeControllerComparison;
