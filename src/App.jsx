import React, { useEffect, useMemo, useState } from 'react';
import { BarChart3, BookOpen, CircleDot, Cpu, Database, Play, Settings } from 'lucide-react';
import ControlSidebar from './components/app/ControlSidebar.jsx';
import ExperimentMatrix from './components/app/ExperimentMatrix.jsx';
import ResearchAnalysis from './components/app/ResearchAnalysis.jsx';
import SimulationDashboard from './components/app/SimulationDashboard.jsx';
import { DocumentationView, SettingsView } from './components/app/WorkspaceViews.jsx';
import { compareControllers, defaultConfig } from './core/simulator.js';
import { SOLVER_BACKENDS } from './core/solvers/index.js';
import './solverDiagnostics.css';

const navItems = [
  { id:'simulation', label:'Simulation', icon:Play },
  { id:'analysis', label:'Analysis', icon:BarChart3 },
  { id:'scenarios', label:'Scenarios', icon:Database },
  { id:'documentation', label:'Documentation', icon:BookOpen },
  { id:'settings', label:'Settings', icon:Settings },
];
const VALID_ROUTES = new Set(navItems.map((item)=>item.id));
const deepCloneConfig=(value)=>JSON.parse(JSON.stringify(value));

function routeFromHash(){
  if(typeof window==='undefined') return 'simulation';
  const route=window.location.hash.replace('#','').trim();
  return VALID_ROUTES.has(route)?route:'simulation';
}
function statusLabel(result){
  if(!result?.metrics) return 'Idle';
  if(result.metrics.fallbackCount>0) return 'Fallback';
  if((result.metrics.convergenceRate??100)<99.9) return 'Degraded';
  return 'Ready';
}

export default function App(){
  const [draftCfg,setDraftCfg]=useState(()=>deepCloneConfig(defaultConfig));
  const [runCfg,setRunCfg]=useState(()=>deepCloneConfig(defaultConfig));
  const [activeMode,setActiveMode]=useState('HYBRID_SAFE');
  const [activeNav,setActiveNav]=useState(routeFromHash);
  const [presetId,setPresetId]=useState('baseline');
  const [batchResult,setBatchResult]=useState(null);
  const results=useMemo(()=>compareControllers(runCfg),[runCfg]);
  const governed=results.find((result)=>result.mode==='HYBRID_SAFE')||results[0];
  const solverLabel=runCfg.mpc.solver===SOLVER_BACKENDS.CONSTRAINED_QP
    ?'Constrained QP'
    :runCfg.mpc.solver===SOLVER_BACKENDS.BOX_QP?'Box QP':'Projected Gradient';

  useEffect(()=>{
    const onHash=()=>setActiveNav(routeFromHash());
    window.addEventListener('hashchange',onHash);
    if(!window.location.hash) window.history.replaceState(null,'','#simulation');
    return ()=>window.removeEventListener('hashchange',onHash);
  },[]);

  const navigate=(id)=>{
    if(!VALID_ROUTES.has(id)) return;
    if(window.location.hash!==`#${id}`) window.location.hash=id;
    else setActiveNav(id);
  };
  const runSimulation=()=>setRunCfg(deepCloneConfig(draftCfg));
  const resetSimulation=()=>{
    const reset=deepCloneConfig(defaultConfig);
    setDraftCfg(reset); setRunCfg(reset); setPresetId('baseline');
  };

  let view=null;
  if(activeNav==='simulation') view=<SimulationDashboard results={results} activeMode={activeMode} runCfg={runCfg} solverLabel={solverLabel}/>;
  if(activeNav==='analysis') view=<ResearchAnalysis results={results} runCfg={runCfg} batchResult={batchResult}/>;
  if(activeNav==='scenarios') view=<ExperimentMatrix draftCfg={draftCfg} setDraftCfg={setDraftCfg} presetId={presetId} setPresetId={setPresetId} batchResult={batchResult} setBatchResult={setBatchResult} navigate={navigate}/>;
  if(activeNav==='documentation') view=<DocumentationView/>;
  if(activeNav==='settings') view=<SettingsView draftCfg={draftCfg} setDraftCfg={setDraftCfg}/>;

  return <div className="control-app">
    <header className="app-topbar">
      <button className="app-brand brand-button" onClick={()=>navigate('simulation')} aria-label="Open simulation home">
        <div className="app-logo"><Cpu size={24}/></div>
        <div><strong>MPC-PID Control System</strong><span>Research • Simulation • Visualization</span></div>
      </button>
      <nav className="top-nav" aria-label="Main navigation">
        {navItems.map(({id,label,icon:Icon})=><button key={id} className={activeNav===id?'active':''} onClick={()=>navigate(id)}><Icon size={16}/>{label}</button>)}
      </nav>
      <div className="app-status">
        <span className={`status-pill status-${statusLabel(governed).toLowerCase()}`}><CircleDot size={11}/>{statusLabel(governed)}</span>
        <span className="repo-mark"><Cpu size={19}/>MPC_PID_System</span>
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

    <footer className="app-footer"><span>MPC_PID_System</span><span>Advanced Control Research Platform</span><span>Research • Simulation • Safety</span></footer>
  </div>;
}
