import React from 'react';
import { Activity, BarChart3, BookOpen, CheckCircle2, Cpu, Database, Gauge, Settings, ShieldCheck, Sparkles, Zap } from 'lucide-react';
import { applyExperimentPreset, defaultConfig, EXPERIMENT_PRESETS, SOLVER_BACKENDS } from '../../application/workbench.js';
import { useUiPreferences } from '../../interface/UiPreferencesContext.jsx';
import {
  BACKGROUND_OPTIONS,
  CONTRAST_OPTIONS,
  FONT_FAMILY_OPTIONS,
  FONT_SIZE_OPTIONS,
  LANGUAGE_OPTIONS,
} from '../../interface/uiPreferences.js';

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
        <button type="button" disabled={active} onClick={()=>select(preset.id)}>{active?'Loaded':'Load scenario'}</button>
      </article>;
    })}</section>
    <section className="scenario-actionbar"><div><strong>Draft scenario: {EXPERIMENT_PRESETS.find((p)=>p.id===presetId)?.label||'Custom'}</strong><span>Changes remain draft until Run Simulation.</span></div><button type="button" onClick={()=>{onRun();navigate('simulation');}}><Zap size={15}/>Run selected scenario</button></section>
  </main>;
}

export function DocumentationView(){
  const { t } = useUiPreferences();
  const stages=[
    ['1','docs.stage.estimation.title','docs.stage.estimation.text'],
    ['2','docs.stage.monitor.title','docs.stage.monitor.text'],
    ['3','docs.stage.mpc.title','docs.stage.mpc.text'],
    ['4','docs.stage.pid.title','docs.stage.pid.text'],
    ['5','docs.stage.governor.title','docs.stage.governor.text'],
    ['6','docs.stage.adaptation.title','docs.stage.adaptation.text'],
  ];
  return <main className="dashboard-main workspace-page">
    <PageHeader icon={BookOpen} eyebrow={t('docs.eyebrow')} title={t('docs.title')} subtitle={t('docs.subtitle')}/>
    <section className="architecture-flow">{t('docs.flow').split('→').map((label,index)=><React.Fragment key={`${label}-${index}`}>{index>0&&<b>→</b>}<span>{label.trim()}</span></React.Fragment>)}</section>
    <section className="doc-grid">{stages.map(([n,titleKey,textKey])=><article className="doc-card" key={n}><span>{n}</span><h3>{t(titleKey)}</h3><p>{t(textKey)}</p></article>)}</section>
    <section className="dashboard-card research-rules"><div className="card-head"><div><ShieldCheck/><strong>{t('docs.gates')}</strong></div></div><div className="rule-grid"><p>{t('docs.gate.qp')}</p><p>{t('docs.gate.estimator')}</p><p>{t('docs.gate.safety')}</p><p>{t('docs.gate.adaptation')}</p></div></section>
  </main>;
}

function SettingToggle({label,checked,onChange,help}){return <label className="setting-line"><div><strong>{label}</strong><span>{help}</span></div><input type="checkbox" checked={checked} onChange={(e)=>onChange(e.target.checked)}/></label>;}
function SettingNumber({label,value,step=0.01,onChange,help}){return <label className="setting-number"><div><strong>{label}</strong><span>{help}</span></div><input type="number" value={value} step={step} onChange={(e)=>onChange(Number(e.target.value))}/></label>;}
function PreferenceSelect({preference,label,value,onChange,options,help,translate}){return <label className="setting-number preference-setting"><div><strong>{label}</strong><span>{help}</span></div><select data-preference={preference} value={value} onChange={(e)=>onChange(e.target.value)}>{options.map((option)=><option key={option.value} value={option.value}>{option.labelKey?translate(option.labelKey):option.label}</option>)}</select></label>;}

export function SettingsView({draftCfg,setDraftCfg}){
  const { preferences, setPreference, t } = useUiPreferences();
  const patch=(group,key,value)=>setDraftCfg((c)=>({...c,[group]:{...c[group],[key]:value}}));
  return <main className="dashboard-main workspace-page">
    <PageHeader icon={Settings} eyebrow={t('settings.eyebrow')} title={t('settings.title')} subtitle={t('settings.subtitle')}/>
    <section className="settings-grid">
      <div className="dashboard-card settings-card preference-card"><div className="card-head"><div><BookOpen/><strong>{t('settings.language')}</strong></div></div><PreferenceSelect preference="language" label={t('settings.language')} value={preferences.language} onChange={(value)=>setPreference('language',value)} options={LANGUAGE_OPTIONS} help={t('settings.languageHelp')} translate={t}/></div>
      <div className="dashboard-card settings-card preference-card"><div className="card-head"><div><Settings/><strong>{t('settings.interface')}</strong></div></div><PreferenceSelect preference="fontSize" label={t('settings.textSize')} value={preferences.fontSize} onChange={(value)=>setPreference('fontSize',Number(value))} options={FONT_SIZE_OPTIONS} help={t('settings.textSizeHelp')} translate={t}/><PreferenceSelect preference="background" label={t('settings.background')} value={preferences.background} onChange={(value)=>setPreference('background',value)} options={BACKGROUND_OPTIONS} help={t('settings.backgroundHelp')} translate={t}/><PreferenceSelect preference="fontFamily" label={t('settings.font')} value={preferences.fontFamily} onChange={(value)=>setPreference('fontFamily',value)} options={FONT_FAMILY_OPTIONS} help={t('settings.fontHelp')} translate={t}/><PreferenceSelect preference="contrast" label={t('settings.contrast')} value={preferences.contrast} onChange={(value)=>setPreference('contrast',value)} options={CONTRAST_OPTIONS} help={t('settings.contrastHelp')} translate={t}/></div>
      <div className="dashboard-card settings-card"><div className="card-head"><div><Activity/><strong>{t('settings.estimation')}</strong></div></div><SettingToggle label={t('settings.enableKalman')} checked={draftCfg.estimation.enabled} onChange={(v)=>patch('estimation','enabled',v)} help={t('settings.enableKalmanHelp')}/><SettingNumber label={t('settings.measurementNoise')} value={draftCfg.estimation.measurementNoiseStd} step={0.01} onChange={(v)=>patch('estimation','measurementNoiseStd',Math.max(0,v))} help={t('settings.measurementNoiseHelp')}/><SettingToggle label={t('settings.disturbanceState')} checked={draftCfg.estimation.disturbanceStateEnabled} onChange={(v)=>patch('estimation','disturbanceStateEnabled',v)} help={t('settings.disturbanceStateHelp')}/><SettingNumber label={t('settings.disturbanceRetention')} value={draftCfg.estimation.disturbanceRetention} step={0.01} onChange={(v)=>patch('estimation','disturbanceRetention',Math.max(0,Math.min(1,v)))} help="d[k+1] = ρ d[k] + w."/></div>
      <div className="dashboard-card settings-card"><div className="card-head"><div><Cpu/><strong>{t('settings.solver')}</strong></div></div><label className="setting-number"><div><strong>{t('settings.backend')}</strong><span>{t('settings.backendHelp')}</span></div><select value={draftCfg.mpc.solver} onChange={(e)=>patch('mpc','solver',e.target.value)}><option value={SOLVER_BACKENDS.CONSTRAINED_QP}>Constrained QP</option><option value={SOLVER_BACKENDS.BOX_QP}>Box QP</option><option value={SOLVER_BACKENDS.PROJECTED_GRADIENT}>Projected Gradient</option></select></label><SettingNumber label={t('settings.horizon')} value={draftCfg.mpc.horizon} step={1} onChange={(v)=>patch('mpc','horizon',Math.max(3,Math.round(v)))} help={t('settings.horizonHelp')}/><SettingNumber label={t('settings.qpIterations')} value={draftCfg.mpc.qpIterations} step={10} onChange={(v)=>patch('mpc','qpIterations',Math.max(1,Math.round(v)))} help={t('settings.qpIterationsHelp')}/></div>
      <div className="dashboard-card settings-card"><div className="card-head"><div><ShieldCheck/><strong>{t('settings.safety')}</strong></div></div><SettingToggle label={t('settings.stateConstraints')} checked={draftCfg.mpc.stateConstraintsEnabled} onChange={(v)=>patch('mpc','stateConstraintsEnabled',v)} help={t('settings.stateConstraintsHelp')}/><SettingToggle label={t('settings.outputConstraints')} checked={draftCfg.mpc.outputConstraintsEnabled} onChange={(v)=>patch('mpc','outputConstraintsEnabled',v)} help={t('settings.outputConstraintsHelp')}/><SettingToggle label={t('settings.covarianceTightening')} checked={draftCfg.estimation.constraintTighteningEnabled} onChange={(v)=>patch('estimation','constraintTighteningEnabled',v)} help={t('settings.covarianceTighteningHelp')}/><SettingNumber label="kσ" value={draftCfg.estimation.constraintSigma} step={0.25} onChange={(v)=>patch('estimation','constraintSigma',Math.max(0,v))} help={t('settings.confidenceHelp')}/></div>
      <div className="dashboard-card settings-card"><div className="card-head"><div><Zap/><strong>{t('settings.scheduling')}</strong></div></div><SettingNumber label={t('settings.predictionThreshold')} value={draftCfg.trigger.predictionError} step={0.005} onChange={(v)=>patch('trigger','predictionError',Math.max(0,v))} help={t('settings.predictionThresholdHelp')}/><SettingNumber label={t('settings.minimumInterval')} value={draftCfg.trigger.minInterval} step={0.02} onChange={(v)=>patch('trigger','minInterval',Math.max(0,v))} help={t('settings.minimumIntervalHelp')}/><SettingNumber label={t('settings.watchdogInterval')} value={draftCfg.trigger.maxInterval} step={0.02} onChange={(v)=>patch('trigger','maxInterval',Math.max(draftCfg.trigger.minInterval,v))} help={t('settings.watchdogIntervalHelp')}/></div>
    </section>
  </main>;
}
