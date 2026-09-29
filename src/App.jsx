import React from 'react';
import { BarChart3, BookOpen, CircleDot, Cpu, Database, Moon, Play, Settings, ShieldCheck, Sun } from 'lucide-react';
import BatchAnalysis from './components/app/BatchAnalysis.jsx';
import ControlSidebar from './components/app/ControlSidebar.jsx';
import ExperimentMatrix from './components/app/ExperimentMatrix.jsx';
import ResearchAnalysis from './components/app/ResearchAnalysis.jsx';
import SimulationDashboard from './components/app/SimulationDashboard.jsx';
import { DocumentationView, SettingsView } from './components/app/WorkspaceViews.jsx';
import { useWorkbenchController } from './components/app/useWorkbenchController.js';
import './solverDiagnostics.css';

const navItems = [
  { id:'simulation', label:'Simulation', icon:Play },
  { id:'analysis', label:'Analysis', icon:BarChart3 },
  { id:'scenarios', label:'Scenarios', icon:Database },
  { id:'documentation', label:'Documentation', icon:BookOpen },
  { id:'settings', label:'Settings', icon:Settings },
];

const MODE_CONTEXT={
  PID:'PID',
  MPC:'MPC · Constrained QP',
  HYBRID:'Event-triggered MPC + PID',
  HYBRID_SAFE:'Event-triggered MPC + PID + Safety Governor',
};

function statusSummary(result){
  if(!result?.metrics) return { key:'idle', label:'Idle' };
  if((result.metrics.safetyViolationCount??0)>0) return { key:'degraded', label:'Unsafe observed' };
  if((result.metrics.fallbackCount??0)>0) return { key:'fallback', label:'Completed · fallback' };
  if((result.metrics.convergenceRate??100)<99.9) return { key:'degraded', label:'Completed · degraded' };
  return { key:'ready', label:'Completed' };
}

export default function App(){
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

  const [theme,setTheme]=React.useState(()=>{
    try { return window.localStorage.getItem('mpc-pid-theme')||'dark'; }
    catch { return 'dark'; }
  });
  React.useEffect(()=>{
    document.documentElement.dataset.theme=theme;
    try { window.localStorage.setItem('mpc-pid-theme',theme); } catch {}
  },[theme]);

  const status=statusSummary(governed);
  const safetyViolations=governed?.metrics?.safetyViolationCount??0;
  const estimatorLabel=runCfg.estimation?.enabled?'Kalman estimated-state':'Disabled';
  const revision=(import.meta.env.VITE_BUILD_SHA||'local-dev').slice(0,10);

  return <div className="control-app">
    <header className="app-topbar">
      <button type="button" className="app-brand brand-button" onClick={()=>navigate('simulation')} aria-label="Open simulation home">
        <div className="app-logo" aria-hidden="true"><Cpu size={24}/></div>
        <div><strong>MPC-PID Control System</strong><span>Research • Simulation • Visualization</span></div>
      </button>
      <nav className="top-nav" aria-label="Main navigation">
        {navItems.map(({id,label,icon:Icon})=><button
          type="button"
          key={id}
          className={activeNav===id?'active':''}
          aria-current={activeNav===id?'page':undefined}
          onClick={()=>navigate(id)}
        ><Icon size={16} aria-hidden="true"/>{label}</button>)}
      </nav>
      <div className="app-status" aria-live="polite">
        <span className={`status-pill status-${status.key}`}><CircleDot size={11} aria-hidden="true"/>{status.label}</span>
        <button
          type="button"
          className="theme-control"
          data-theme-control
          aria-label={theme==='dark'?'Switch to light theme':'Switch to dark theme'}
          onClick={()=>setTheme((current)=>current==='dark'?'light':'dark')}
        >
          {theme==='dark'?<Sun size={15} aria-hidden="true"/>:<Moon size={15} aria-hidden="true"/>}
          <span>{theme==='dark'?'Light':'Dark'}</span>
        </button>
        <span className="repo-mark"><Cpu size={19} aria-hidden="true"/>MPC_PID_System</span>
      </div>
    </header>

    <section className="research-context-header" aria-label="Research context">
      <div className="context-primary">
        <span className="context-eyebrow">Research context</span>
        <strong>Linear control benchmark</strong>
        <span>Deterministic simulation workbench</span>
      </div>
      <dl className="context-grid">
        <div><dt>Plant</dt><dd>Second-order plant</dd></div>
        <div><dt>Controller</dt><dd>{MODE_CONTEXT[activeMode]||activeMode}</dd></div>
        <div><dt>Estimator</dt><dd>{estimatorLabel}</dd></div>
        <div className={safetyViolations===0?'context-safety safe':'context-safety unsafe'}>
          <dt><ShieldCheck size={13} aria-hidden="true"/>Actual safety</dt>
          <dd>{safetyViolations===0?'No violations observed':`${safetyViolations} violations observed`}</dd>
        </div>
        <div><dt>Revision</dt><dd><code>{revision}</code></dd></div>
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
