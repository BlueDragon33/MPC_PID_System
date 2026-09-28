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
