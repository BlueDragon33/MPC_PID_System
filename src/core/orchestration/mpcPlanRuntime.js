import { solveMPC } from '../solvers/index.js';

function toSolverRecord(solution) {
  return {
    solveMs: solution.solveMs,
    solver: solution.solver,
    status: solution.status || (solution.diagnostics?.converged ? 'solved' : null),
    fallbackUsed: Boolean(solution.fallbackUsed),
    fallbackReason: solution.fallbackReason || null,
    diagnostics: solution.diagnostics || null,
  };
}

export function createMpcPlanRuntime() {
  let warmStart = null;
  let lastSafeU = 0;
  let continuationSequence = null;
  const solverRecords = [];

  function acceptPlan(solution) {
    if (solution.fallbackUsed) return;
    continuationSequence = [...solution.sequence];
    lastSafeU = solution.u;
  }

  return {
    solve({ state, target, previousU, cfg }) {
      const solution = solveMPC(state, target, previousU, cfg, warmStart);
      warmStart = solution.sequence;
      solverRecords.push(toSolverRecord(solution));
      acceptPlan(solution);
      return solution;
    },

    shiftPlan() {
      if (!continuationSequence?.length) return;
      const tail = continuationSequence[continuationSequence.length - 1];
      continuationSequence = [...continuationSequence.slice(1), tail];
      if (Number.isFinite(continuationSequence[0])) lastSafeU = continuationSequence[0];
    },

    getContinuationSequence() {
      return continuationSequence;
    },

    getLastSafeU() {
      return lastSafeU;
    },

    getSolverRecords() {
      return solverRecords;
    },
  };
}
