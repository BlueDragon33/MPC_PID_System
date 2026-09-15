import React from 'react';
import { Activity, BarChart3, BookOpen, CheckCircle2, Cpu, Database, Gauge, Settings, ShieldCheck, Sparkles, Zap } from 'lucide-react';
import { applyExperimentPreset, EXPERIMENT_PRESETS } from '../../core/experiments/presets.js';
import { defaultConfig } from '../../core/simulator.js';
import { SOLVER_BACKENDS } from '../../core/solvers/index.js';

const MODE_ORDER = ['PID','MPC','HYBRID','HYBRID_SAFE'];
const MODE_LABELS = { PID:'PID', MPC:'Periodic MPC', HYBRID:'Event MPC + PID', HYBRID_SAFE:'Event MPC + PID + Safety' };
const fmt=(v,d=2)=>Number.isFinite(v)?v.toFixed(d):'—';

function PageHeader({icon:Icon,eyebrow,title,subtitle}){
  return <div className="workspace-page-head"><div className="workspace-page-icon"><Icon size={22}/></div><div><span>{eyebrow}</span><h1>{title}</h1><p>{subtitle}</p></div></div>;
}

export function AnalysisView({results,runCfg}){
  const sorted=[...results].sort((a,b)=>a.metrics.iae-b.metrics.iae);
  const best=sorted[0];
  const safe=results.find((r)=>r.mode==='HYBRID_SAFE');
  const periodic=results.find((r)=>r.mode==='MPC');
  return <main className="dashboard-main workspace-page">
    <PageHeader icon={BarChart3} eyebrow="RUN ANALYSIS" title="Performance and compute trade-off" subtitle="Compare tracking quality, control effort, solver burden and safety for the latest executed configuration."/>
    <section className="analysis-kpis">
      <div className="analysis-kpi"><span>Best IAE</span><strong>{fmt(best?.metrics.iae,4)}</strong><small>{MODE_LABELS[best?.mode]||'—'}</small></div>
      <div className="analysis-kpi"><span>Hybrid compute avoided</span><strong>{fmt(safe?.metrics.computeReduction,1)}%</strong><small>{safe?.metrics.solveCount||0} MPC solves</small></div>
      <div className="analysis-kpi"><span>Plant safety</span><strong>{safe?.metrics.safetyViolationCount??0}</strong><small>actual violations</small></div>
      <div className="analysis-kpi"><span>Periodic baseline solves</span><strong>{periodic?.metrics.solveCount??0}</strong><small>{fmt(runCfg.duration,1)} s experiment</small></div>
    </section>
    <section className="workspace-two-col">
      <div className="dashboard-card analysis-table-card">
        <div className="card-head"><div><Gauge size={16}/><strong>Controller comparison</strong></div></div>
        <div className="research-table">
          <div className="research-row research-head"><span>Controller</span><span>IAE</span><span>Overshoot</span><span>Effort</span><span>Solves</span><span>Convergence</span><span>Safety</span></div>
          {MODE_ORDER.map((mode)=>{const r=results.find((item)=>item.mode===mode);return <div className={`research-row ${mode==='HYBRID_SAFE'?'research-highlight':''}`} key={mode}><strong>{MODE_LABELS[mode]}</strong><span>{fmt(r?.metrics.iae,4)}</span><span>{fmt(r?.metrics.overshoot,1)}%</span><span>{fmt(r?.metrics.controlEffort,2)}</span><span>{r?.metrics.solveCount??0}</span><span>{mode==='PID'?'—':`${fmt(r?.metrics.convergenceRate??100,1)}%`}</span><span>{r?.metrics.safetyViolationCount??0}</span></div>;})}
        </div>
      </div>
      <div className="analysis-stack">
        <div className="dashboard-card insight-card"><div className="card-head"><div><Cpu size={16}/><strong>Compute profile</strong></div></div><p>Event-triggered control should reduce optimization calls without materially degrading IAE or safety.</p><div className="insight-number">{fmt(safe?.metrics.computeReduction,1)}%</div><small>solve reduction vs periodic MPC</small></div>
        <div className="dashboard-card insight-card"><div className="card-head"><div><ShieldCheck size={16}/><strong>Safety authority</strong></div></div><p>The plant audit is independent from predicted feasibility, so model-safe does not automatically mean plant-safe.</p><div className="insight-number">{safe?.metrics.safetyViolationCount??0}</div><small>actual unsafe samples</small></div>
        <div className="dashboard-card insight-card"><div className="card-head"><div><Activity size={16}/><strong>Estimator</strong></div></div><p>{safe?.metrics.estimationEnabled?'Controller uses estimated state x̂ in the current run.':'Ground-truth state is used in the current run.'}</p><div className="insight-number">{safe?.metrics.estimationEnabled?fmt(safe?.metrics.estimatePositionRmse,4):'OFF'}</div><small>position estimate RMSE</small></div>
      </div>
    </section>
  </main>;
}

export function ScenariosView({draftCfg,setDraftCfg,presetId,setPresetId,onRun,navigate}){
  const select=(id)=>{setPresetId(id);setDraftCfg(applyExperimentPreset(defaultConfig,id));};
  return <main className="dashboard-main workspace-page">
    <PageHeader icon={Database} eyebrow="EXPERIMENT LIBRARY" title="Reproducible scenarios" subtitle="Load a controlled scenario into the draft configuration, inspect it, then run the same experiment from Simulation."/>
    <section className="scenario-grid">{EXPERIMENT_PRESETS.map((preset)=>{
      const active=preset.id===presetId;
      return <article key={preset.id} className={`scenario-card ${active?'scenario-active':''}`}>
        <div className="scenario-top"><span className="scenario-icon"><Sparkles size={16}/></span>{active&&<span className="scenario-selected"><CheckCircle2 size={12}/>Selected</span>}</div>
        <h3>{preset.label}</h3><p>{preset.description}</p>
        <div className="scenario-tags"><span>{preset.patch?.estimation?.enabled?'Estimator':'State truth'}</span><span>{preset.patch?.truthPlant?.enabled?'Model mismatch':'Nominal plant'}</span><span>{preset.patch?.mpc?.stateConstraintsEnabled?'State constraints':'Actuator constraints'}</span></div>
        <button onClick={()=>select(preset.id)}>{active?'Loaded':'Load scenario'}</button>
      </article>;
    })}</section>
    <section className="scenario-actionbar"><div><strong>Draft scenario: {EXPERIMENT_PRESETS.find((p)=>p.id===presetId)?.label||'Custom'}</strong><span>Changes remain draft until Run Simulation.</span></div><button onClick={()=>{onRun();navigate('simulation');}}><Zap size={15}/>Run selected scenario</button></section>
  </main>;
}

export function DocumentationView(){
  const stages=[
    ['1','State estimation','Sensor → Kalman x̂, P and optional disturbance state d̂.'],
    ['2','Event monitor','Prediction error, state-change, constraint proximity and watchdog determine when MPC is worth solving.'],
    ['3','Constrained MPC','Finite-horizon QP enforces actuator, Δu and optional state/output envelopes.'],
    ['4','Fast PID','PID tracks predictive guidance at the fast loop rate.'],
    ['5','Safety Governor','Admissibility checks constrain the first command before it reaches the actuator.'],
    ['6','Shadow adaptation','RLS + persistent excitation + holdout validation learns a candidate plant model without control authority.'],
  ];
  return <main className="dashboard-main workspace-page">
    <PageHeader icon={BookOpen} eyebrow="SYSTEM DOCUMENTATION" title="Control architecture" subtitle="A concise in-app technical reference for the current research stack. This page describes what the code actually does, not a future concept."/>
    <section className="architecture-flow"><span>Sensor</span><b>→</b><span>Kalman x̂,P,d̂</span><b>→</b><span>Event MPC</span><b>→</b><span>PID</span><b>→</b><span>Safety Governor</span><b>→</b><span>Plant</span></section>
    <section className="doc-grid">{stages.map(([n,title,text])=><article className="doc-card" key={n}><span>{n}</span><h3>{title}</h3><p>{text}</p></article>)}</section>
    <section className="dashboard-card research-rules"><div className="card-head"><div><ShieldCheck size={16}/><strong>Research gates</strong></div></div><div className="rule-grid"><p><b>QP before NMPC.</b> General constraints and diagnostics must remain trustworthy before nonlinear optimization.</p><p><b>Estimator before autonomy.</b> Controller decisions must be based on x̂ rather than privileged simulator truth.</p><p><b>Safety is measured on the plant.</b> Predicted feasibility and actual safety are reported separately.</p><p><b>Adaptation stays shadow-only.</b> A learned model needs robustness and rollback certification before it can influence MPC.</p></div></section>
  </main>;
}

function SettingToggle({label,checked,onChange,help}){return <label className="setting-line"><div><strong>{label}</strong><span>{help}</span></div><input type="checkbox" checked={checked} onChange={(e)=>onChange(e.target.checked)}/></label>;}
function SettingNumber({label,value,step=0.01,onChange,help}){return <label className="setting-number"><div><strong>{label}</strong><span>{help}</span></div><input type="number" value={value} step={step} onChange={(e)=>onChange(Number(e.target.value))}/></label>;}

export function SettingsView({draftCfg,setDraftCfg}){
  const patch=(group,key,value)=>setDraftCfg((c)=>({...c,[group]:{...c[group],[key]:value}}));
  return <main className="dashboard-main workspace-page">
    <PageHeader icon={Settings} eyebrow="WORKBENCH SETTINGS" title="Estimator, solver and safety defaults" subtitle="These values edit the draft experiment only. Use Run Simulation to apply them to the active result set."/>
    <section className="settings-grid">
      <div className="dashboard-card settings-card"><div className="card-head"><div><Activity size={16}/><strong>State estimation</strong></div></div><SettingToggle label="Enable Kalman estimation" checked={draftCfg.estimation.enabled} onChange={(v)=>patch('estimation','enabled',v)} help="Controllers use x̂ instead of simulator truth."/><SettingNumber label="Measurement noise σ" value={draftCfg.estimation.measurementNoiseStd} step={0.01} onChange={(v)=>patch('estimation','measurementNoiseStd',Math.max(0,v))} help="Sensor standard deviation."/><SettingToggle label="Disturbance state d̂" checked={draftCfg.estimation.disturbanceStateEnabled} onChange={(v)=>patch('estimation','disturbanceStateEnabled',v)} help="Augmented observer estimates equivalent disturbance."/><SettingNumber label="Disturbance retention ρ" value={draftCfg.estimation.disturbanceRetention} step={0.01} onChange={(v)=>patch('estimation','disturbanceRetention',Math.max(0,Math.min(1,v)))} help="d[k+1] = ρ d[k] + w."/></div>
      <div className="dashboard-card settings-card"><div className="card-head"><div><Cpu size={16}/><strong>MPC solver</strong></div></div><label className="setting-number"><div><strong>Backend</strong><span>Default optimization engine.</span></div><select value={draftCfg.mpc.solver} onChange={(e)=>patch('mpc','solver',e.target.value)}><option value={SOLVER_BACKENDS.CONSTRAINED_QP}>Constrained QP</option><option value={SOLVER_BACKENDS.BOX_QP}>Box QP</option><option value={SOLVER_BACKENDS.PROJECTED_GRADIENT}>Projected Gradient</option></select></label><SettingNumber label="Horizon" value={draftCfg.mpc.horizon} step={1} onChange={(v)=>patch('mpc','horizon',Math.max(3,Math.round(v)))} help="Prediction steps."/><SettingNumber label="QP iterations" value={draftCfg.mpc.qpIterations} step={10} onChange={(v)=>patch('mpc','qpIterations',Math.max(1,Math.round(v)))} help="Numerical iteration ceiling."/></div>
      <div className="dashboard-card settings-card"><div className="card-head"><div><ShieldCheck size={16}/><strong>Robust safety</strong></div></div><SettingToggle label="State constraints" checked={draftCfg.mpc.stateConstraintsEnabled} onChange={(v)=>patch('mpc','stateConstraintsEnabled',v)} help="Constrain predicted x and v."/><SettingToggle label="Output constraints" checked={draftCfg.mpc.outputConstraintsEnabled} onChange={(v)=>patch('mpc','outputConstraintsEnabled',v)} help="Constrain predicted measured output."/><SettingToggle label="Covariance tightening" checked={draftCfg.estimation.constraintTighteningEnabled} onChange={(v)=>patch('estimation','constraintTighteningEnabled',v)} help="Shrink predicted envelope using estimator uncertainty."/><SettingNumber label="kσ" value={draftCfg.estimation.constraintSigma} step={0.25} onChange={(v)=>patch('estimation','constraintSigma',Math.max(0,v))} help="Confidence multiplier for robust margins."/></div>
      <div className="dashboard-card settings-card"><div className="card-head"><div><Zap size={16}/><strong>Event scheduling</strong></div></div><SettingNumber label="Prediction error threshold" value={draftCfg.trigger.predictionError} step={0.005} onChange={(v)=>patch('trigger','predictionError',Math.max(0,v))} help="Solve MPC when model prediction deviates too far."/><SettingNumber label="Minimum interval" value={draftCfg.trigger.minInterval} step={0.02} onChange={(v)=>patch('trigger','minInterval',Math.max(0,v))} help="Anti-chatter lower bound."/><SettingNumber label="Watchdog interval" value={draftCfg.trigger.maxInterval} step={0.02} onChange={(v)=>patch('trigger','maxInterval',Math.max(draftCfg.trigger.minInterval,v))} help="Forces refresh even when event thresholds remain quiet."/></div>
    </section>
  </main>;
}
