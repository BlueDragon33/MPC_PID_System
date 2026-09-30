import React from 'react';
import { BarChart3, BookOpen, CircleDot, Cpu, Database, Moon, Play, Settings, ShieldCheck, Sun } from 'lucide-react';
import BatchAnalysis from './components/app/BatchAnalysis.jsx';
import ControlSidebar from './components/app/ControlSidebar.jsx';
import ExperimentMatrix from './components/app/ExperimentMatrix.jsx';
import ResearchAnalysis from './components/app/ResearchAnalysis.jsx';
import SimulationDashboard from './components/app/SimulationDashboard.jsx';
import { DocumentationView, SettingsView } from './components/app/WorkspaceViews.jsx';
import { useWorkbenchController } from './components/app/useWorkbenchController.js';
import { useUiPreferences } from './interface/UiPreferencesContext.jsx';
import './solverDiagnostics.css';

const navItems = [
  { id:'simulation', labelKey:'nav.simulation', icon:Play },
  { id:'analysis', labelKey:'nav.analysis', icon:BarChart3 },
  { id:'scenarios', labelKey:'nav.scenarios', icon:Database },
  { id:'documentation', labelKey:'nav.documentation', icon:BookOpen },
  { id:'settings', labelKey:'nav.settings', icon:Settings },
];

const MODE_CONTEXT={
  PID:'PID',
  MPC:'MPC · Constrained QP',
  HYBRID:'Event-triggered MPC + PID',
  HYBRID_SAFE:'Event-triggered MPC + PID + Safety Governor',
};

function statusSummary(result){
  if(!result?.metrics) return { key:'idle', labelKey:'status.idle' };
  if((result.metrics.safetyViolationCount??0)>0) return { key:'degraded', labelKey:'status.unsafe' };
  if((result.metrics.fallbackCount??0)>0) return { key:'fallback', labelKey:'status.fallback' };
  if((result.metrics.convergenceRate??100)<99.9) return { key:'degraded', labelKey:'status.degraded' };
  return { key:'ready', labelKey:'status.ready' };
}

export default function App(){
  const { preferences, t, toggleBackground } = useUiPreferences();
  const {
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
  } = useWorkbenchController();

  let view=null;
  if(activeNav==='simulation') view=<SimulationDashboard results={results} activeMode={activeMode} runCfg={runCfg} solverLabel={solverLabel}/>;
  if(activeNav==='analysis') view=<div className="analysis-route-stack"><ResearchAnalysis results={results} runCfg={runCfg}/><BatchAnalysis batchResult={batchResult}/></div>;
  if(activeNav==='scenarios') view=<ExperimentMatrix draftCfg={draftCfg} setDraftCfg={setDraftCfg} presetId={presetId} setPresetId={setPresetId} batchResult={batchResult} setBatchResult={setBatchResult} navigate={navigate}/>;
  if(activeNav==='documentation') view=<DocumentationView/>;
  if(activeNav==='settings') view=<SettingsView draftCfg={draftCfg} setDraftCfg={setDraftCfg}/>;

  const activeResult=results.find((result)=>result.mode===activeMode)||governed;
  const status=statusSummary(activeResult);
  const safetyViolations=activeResult?.metrics?.safetyViolationCount??0;
  const estimatorLabel=runCfg.estimation?.enabled?t('context.estimationOn'):t('common.disabled');
  const revision=(import.meta.env.VITE_BUILD_SHA||'local-dev').slice(0,10);

  return <div className="control-app">
    <header className="app-topbar">
      <button type="button" className="app-brand brand-button" onClick={()=>navigate('simulation')} aria-label="Open simulation home">
        <div className="app-logo" aria-hidden="true"><Cpu size={24}/></div>
        <div><strong>MPC-PID Control System</strong><span>{t('brand.subtitle')}</span></div>
      </button>
      <nav className="top-nav" aria-label="Main navigation">
        {navItems.map(({id,labelKey,icon:Icon})=><button
          type="button"
          key={id}
          data-nav-id={id}
          className={activeNav===id?'active':''}
          aria-current={activeNav===id?'page':undefined}
          onClick={()=>navigate(id)}
        ><Icon aria-hidden="true"/><span>{t(labelKey)}</span></button>)}
      </nav>
      <div className="app-status" aria-live="polite">
        <span className={`status-pill status-${status.key}`}><CircleDot aria-hidden="true"/>{t(status.labelKey)}</span>
        <button
          type="button"
          className="theme-control"
          data-theme-control
          aria-label={preferences.background==='dark'?t('theme.switchLight'):t('theme.switchDark')}
          onClick={toggleBackground}
        >
          {preferences.background==='dark'?<Sun aria-hidden="true"/>:<Moon aria-hidden="true"/>}
          <span>{preferences.background==='dark'?t('theme.light'):t('theme.dark')}</span>
        </button>
        <span className="repo-mark"><Cpu size={19} aria-hidden="true"/>MPC_PID_System</span>
      </div>
    </header>

    <section className="research-context-header" aria-label="Research context">
      <div className="context-primary">
        <span className="context-eyebrow">{t('context.eyebrow')}</span>
        <h1>{t('context.title')}</h1>
        <span>{t('context.subtitle')}</span>
      </div>
      <dl className="context-grid">
        <div><dt>{t('context.plant')}</dt><dd>Second-order plant</dd></div>
        <div><dt>{t('context.controller')}</dt><dd>{MODE_CONTEXT[activeMode]||activeMode}</dd></div>
        <div><dt>{t('context.estimator')}</dt><dd>{estimatorLabel}</dd></div>
        <div className={safetyViolations===0?'context-safety safe':'context-safety unsafe'}>
          <dt><ShieldCheck aria-hidden="true"/>{t('context.safety')}</dt>
          <dd>{safetyViolations===0?t('context.noViolations'):`${safetyViolations} ${t('context.violations')}`}</dd>
        </div>
        <div><dt>{t('context.revision')}</dt><dd><code>{revision}</code></dd></div>
      </dl>
    </section>

    <div className="app-layout">
      <ControlSidebar
        draftCfg={draftCfg}
        setDraftCfg={setDraftCfg}
        activeMode={activeMode}
        setActiveMode={setActiveMode}
        presetId={presetId}
        setPresetId={setPresetId}
        onRun={runSimulation}
        onReset={resetSimulation}
        runRevision={runRevision}
      />
      {view}
    </div>

    <footer className="app-footer" aria-label="Application information">
      <span>MPC_PID_System</span>
      <span>Advanced Control Research Platform</span>
      <span>Research • Simulation • Safety</span>
    </footer>
  </div>;
}
