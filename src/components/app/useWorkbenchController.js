import { useEffect, useMemo, useState } from 'react';

import {
  defaultConfig,
  executeControllerComparison,
  SOLVER_BACKENDS,
} from '../../application/workbench.js';
import {
  loadWorkbenchRecovery,
  saveWorkbenchRecovery,
} from '../../application/persistence/workbenchRecovery.js';
import { normalizeWorkbenchRoute, workbenchRouteHash } from './navigation.js';
import { normalizeControlMode } from './simulationViewModel.js';

const deepClone = (value) => JSON.parse(JSON.stringify(value));

function initialRoute() {
  if (typeof window === 'undefined') return 'simulation';
  return normalizeWorkbenchRoute(window.location.hash);
}

export function useWorkbenchController() {
  const [recovery] = useState(() => loadWorkbenchRecovery());
  const [draftCfg, setDraftCfg] = useState(() => deepClone(recovery?.draftCfg ?? defaultConfig));
  const [runCfg, setRunCfg] = useState(() => deepClone(recovery?.runCfg ?? defaultConfig));
  const [activeMode, setActiveMode] = useState(() => normalizeControlMode(recovery?.activeMode));
  const [activeNav, setActiveNav] = useState(initialRoute);
  const [presetId, setPresetId] = useState(recovery?.presetId ?? 'baseline');
  const [batchResult, setBatchResult] = useState(null);
  const [runRevision, setRunRevision] = useState(1);

  const results = useMemo(() => executeControllerComparison(runCfg), [runCfg]);
  const governed = results.find((result) => result.mode === 'HYBRID_SAFE') || results[0];
  const solverLabel = runCfg.mpc.solver === SOLVER_BACKENDS.CONSTRAINED_QP
    ? 'Constrained QP'
    : runCfg.mpc.solver === SOLVER_BACKENDS.BOX_QP
      ? 'Box QP'
      : 'Projected Gradient';

  useEffect(() => {
    const onHash = () => setActiveNav(normalizeWorkbenchRoute(window.location.hash));
    window.addEventListener('hashchange', onHash);
    if (!window.location.hash) {
      window.history.replaceState(null, '', workbenchRouteHash('simulation'));
    }
    return () => window.removeEventListener('hashchange', onHash);
  }, []);

  useEffect(() => {
    saveWorkbenchRecovery({
      draftCfg,
      runCfg,
      presetId,
      activeMode,
    });
  }, [draftCfg, runCfg, presetId, activeMode]);

  const navigate = (route) => {
    const nextHash = workbenchRouteHash(route);
    if (typeof window === 'undefined') {
      setActiveNav(normalizeWorkbenchRoute(nextHash));
      return;
    }
    if (window.location.hash !== nextHash) window.location.hash = nextHash;
    else setActiveNav(normalizeWorkbenchRoute(nextHash));
  };

  const runSimulation = () => {
    setRunCfg(deepClone(draftCfg));
    setRunRevision((revision) => revision + 1);
  };

  const resetSimulation = () => {
    const reset = deepClone(defaultConfig);
    setDraftCfg(reset);
    setRunCfg(reset);
    setPresetId('baseline');
    setActiveMode('HYBRID_SAFE');
    setBatchResult(null);
    setRunRevision((revision) => revision + 1);
  };

  return {
    activeMode,
    activeNav,
    batchResult,
    draftCfg,
    governed,
    navigate,
    presetId,
    resetSimulation,
    results,
    runCfg,
    runRevision,
    runSimulation,
    setActiveMode,
    setBatchResult,
    setDraftCfg,
    setPresetId,
    solverLabel,
  };
}
