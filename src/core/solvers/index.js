import { solveProjectedGradientMPC } from './projectedGradientMPC.js';

export const SOLVER_BACKENDS = {
  PROJECTED_GRADIENT: 'projected-gradient',
};

export function solveMPC(state, target, previousU, cfg, warmStart = null) {
  const backend = cfg.mpc.solver || SOLVER_BACKENDS.PROJECTED_GRADIENT;

  switch (backend) {
    case SOLVER_BACKENDS.PROJECTED_GRADIENT:
      return solveProjectedGradientMPC(state, target, previousU, cfg, warmStart);
    default:
      throw new Error(`Unsupported MPC solver backend: ${backend}`);
  }
}
