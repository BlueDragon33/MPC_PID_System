export const WORKBENCH_CONTRACT_VERSION = '1.0.0';

export function assertSimulationResultContract(result, label = 'simulation result') {
  if (!result || typeof result !== 'object') throw new Error(`${label} must be an object.`);
  if (!Array.isArray(result.samples)) throw new Error(`${label}.samples must be an array.`);
  if (!result.metrics || typeof result.metrics !== 'object') throw new Error(`${label}.metrics must be an object.`);
  if (!result.config || typeof result.config !== 'object') throw new Error(`${label}.config must be an object.`);
  return result;
}

export function assertControllerComparisonContract(results) {
  if (!Array.isArray(results) || results.length === 0) {
    throw new Error('Controller comparison must contain at least one result.');
  }
  results.forEach((result, index) => assertSimulationResultContract(result, `controller result[${index}]`));
  return results;
}

export function assertExperimentMatrixContract(matrix) {
  if (!matrix || typeof matrix !== 'object') throw new Error('Experiment matrix result must be an object.');
  if (!Array.isArray(matrix.cases) || !Array.isArray(matrix.groups)) {
    throw new Error('Experiment matrix must expose cases and groups.');
  }
  return matrix;
}

export function assertUgvComparisonContract(results) {
  if (!Array.isArray(results) || results.length !== 2) {
    throw new Error('UGV comparison must contain CLASSICAL and LTV_MPC results.');
  }
  const modes = new Set(results.map((result) => result?.mode));
  if (!modes.has('CLASSICAL') || !modes.has('LTV_MPC')) {
    throw new Error('UGV comparison is missing CLASSICAL or LTV_MPC.');
  }
  for (const [index, result] of results.entries()) {
    if (!Array.isArray(result.samples) || !Array.isArray(result.solverRecords)) {
      throw new Error(`UGV result[${index}] is missing samples or solverRecords.`);
    }
    if (!result.metrics || !result.config) {
      throw new Error(`UGV result[${index}] is missing metrics or config.`);
    }
  }
  return results;
}
