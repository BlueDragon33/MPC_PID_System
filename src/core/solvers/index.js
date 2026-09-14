import { solveProjectedGradientMPC } from './projectedGradientMPC.js';
import { solveBoxQPMPC } from './boxQPMPC.js';

export const SOLVER_BACKENDS = {
  BOX_QP: 'box-qp',
  PROJECTED_GRADIENT: 'projected-gradient',
};

export function solveMPC(state, target, previousU, cfg, warmStart = null) {
  const backend = cfg.mpc.solver || SOLVER_BACKENDS.BOX_QP;

  switch (backend) {
    case SOLVER_BACKENDS.BOX_QP:
      return solveBoxQPMPC(state, target, previousU, cfg, warmStart);
    case SOLVER_BACKENDS.PROJECTED_GRADIENT:
      return solveProjectedGradientMPC(state, target, previousU, cfg, warmStart);
    default:
      throw new Error(`Unsupported MPC solver backend: ${backend}`);
  }
}
