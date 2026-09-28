import React from 'react';
import { BarChart3, BookOpen, CircleDot, Cpu, Database, Play, Settings } from 'lucide-react';
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

function statusLabel(result){
  if(!result?.metrics) return 'Idle';
  if(result.metrics.fallbackCount>0) return 'Fallback';
  if((result.metrics.convergenceRate??100)<99.9) return 'Degraded';
  return 'Ready';
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

  const status = statusLabel(governed);

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
        <span className={`status-pill status-${status.toLowerCase()}`}><CircleDot size={11} aria-hidden="true"/>{status}</span>
        <span className="repo-mark"><Cpu size={19} aria-hidden="true"/>MPC_PID_System</span>
      </div>
    </header>

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
