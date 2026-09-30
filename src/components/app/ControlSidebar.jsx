import React, { useState } from 'react';
import { ChevronDown, Gauge, Play, RotateCcw, SlidersHorizontal } from 'lucide-react';
import ExperimentSafetyPanel from '../ExperimentSafetyPanel.jsx';
import { SOLVER_BACKENDS } from '../../application/workbench.js';
import { useUiPreferences } from '../../interface/UiPreferencesContext.jsx';

const MODES = ['PID', 'MPC', 'HYBRID', 'HYBRID_SAFE'];
const MODE_LABELS = {
  PID: 'PID Only',
  MPC: 'MPC (Periodic)',
  HYBRID: 'MPC + PID (Event)',
  HYBRID_SAFE: 'MPC + PID + Safety',
};

function NumberField({ label, value, step = 0.01, onChange }) {
  return <label className="field app-field"><span>{label}</span><input type="number" value={value} step={step} onChange={(e) => onChange(Number(e.target.value))}/></label>;
}
function SelectField({ label, value, onChange, options }) {
  return <label className="select-field app-field"><span>{label}</span><select value={value} onChange={(e) => onChange(e.target.value)}>{options.map((option) => <option key={option.value} value={option.value}>{option.label}</option>)}</select></label>;
}
function ToggleField({ label, checked, onChange }) {
  return <label className="toggle-field app-field"><span>{label}</span><input type="checkbox" checked={checked} onChange={(e) => onChange(e.target.checked)}/></label>;
}
const clamp = (value, min, max) => Math.max(min, Math.min(max, value));

export default function ControlSidebar({ draftCfg, setDraftCfg, activeMode, setActiveMode, presetId, setPresetId, onRun, onReset, runRevision }) {
  const { t } = useUiPreferences();
  const [leftTab, setLeftTab] = useState('pid');
  const [controlExpanded, setControlExpanded] = useState(true);
  const update = (group, key, value) => setDraftCfg((current) => group
    ? ({ ...current, [group]: { ...current[group], [key]: value } })
    : ({ ...current, [key]: value }));

  return <aside className="control-sidebar">
    <section className="sidebar-section control-config">
      <button type="button" className="sidebar-heading sidebar-disclosure" aria-expanded={controlExpanded} onClick={() => setControlExpanded((expanded) => !expanded)}>
        <SlidersHorizontal aria-hidden="true"/><strong>{t('sidebar.controlConfiguration')}</strong><ChevronDown className={controlExpanded ? 'expanded' : ''} aria-hidden="true"/>
      </button>
      {controlExpanded && <div className="control-config-content">
        <SelectField label={t('sidebar.controlMode')} value={activeMode} onChange={setActiveMode} options={MODES.map((mode) => ({ value: mode, label: MODE_LABELS[mode] }))}/>
        <small className="field-hint">{t('sidebar.modeHint')}</small>
        <div className="config-tabs" role="tablist" aria-label="Controller settings">
          {['pid','mpc','trigger','safety'].map((tab) => <button type="button" role="tab" aria-selected={leftTab === tab} key={tab} className={leftTab === tab ? 'active' : ''} onClick={() => setLeftTab(tab)}>{tab.toUpperCase()}</button>)}
        </div>

        {leftTab === 'pid' && <div className="tab-content" role="tabpanel">
          <NumberField label="Kp" value={draftCfg.pid.kp} step={0.1} onChange={(v) => update('pid','kp',v)}/>
          <NumberField label="Ki" value={draftCfg.pid.ki} step={0.05} onChange={(v) => update('pid','ki',v)}/>
          <NumberField label="Kd" value={draftCfg.pid.kd} step={0.02} onChange={(v) => update('pid','kd',v)}/>
          <NumberField label="Anti-windup" value={draftCfg.pid.antiWindup} step={0.1} onChange={(v) => update('pid','antiWindup',v)}/>
        </div>}

        {leftTab === 'mpc' && <div className="tab-content" role="tabpanel">
          <SelectField label="Solver" value={draftCfg.mpc.solver} onChange={(v) => update('mpc','solver',v)} options={[
            { value: SOLVER_BACKENDS.CONSTRAINED_QP, label: 'Constrained QP' },
            { value: SOLVER_BACKENDS.BOX_QP, label: 'Box QP' },
            { value: SOLVER_BACKENDS.PROJECTED_GRADIENT, label: 'Projected Gradient' },
          ]}/>
          <NumberField label="Horizon" value={draftCfg.mpc.horizon} step={1} onChange={(v) => update('mpc','horizon',Math.max(3,Math.round(v)))}/>
          <NumberField label="Q position" value={draftCfg.mpc.qPosition} step={0.5} onChange={(v) => update('mpc','qPosition',Math.max(0,v))}/>
          <NumberField label="R input" value={draftCfg.mpc.rInput} step={0.01} onChange={(v) => update('mpc','rInput',Math.max(0,v))}/>
        </div>}

        {leftTab === 'trigger' && <div className="tab-content" role="tabpanel">
          <NumberField label="Prediction error" value={draftCfg.trigger.predictionError} step={0.005} onChange={(v) => update('trigger','predictionError',Math.max(0,v))}/>
          <NumberField label="State change" value={draftCfg.trigger.stateChange} step={0.01} onChange={(v) => update('trigger','stateChange',Math.max(0,v))}/>
          <NumberField label="T min (s)" value={draftCfg.trigger.minInterval} step={0.02} onChange={(v) => update('trigger','minInterval',Math.max(0,v))}/>
          <NumberField label="Watchdog (s)" value={draftCfg.trigger.maxInterval} step={0.02} onChange={(v) => update('trigger','maxInterval',Math.max(draftCfg.trigger.minInterval,v))}/>
        </div>}

        {leftTab === 'safety' && <div className="tab-content" role="tabpanel">
          <ToggleField label="State limits" checked={draftCfg.mpc.stateConstraintsEnabled} onChange={(v) => update('mpc','stateConstraintsEnabled',v)}/>
          <ToggleField label="Output limits" checked={draftCfg.mpc.outputConstraintsEnabled} onChange={(v) => update('mpc','outputConstraintsEnabled',v)}/>
          <NumberField label="Preview horizon" value={draftCfg.safety.previewHorizon} step={1} onChange={(v) => update('safety','previewHorizon',Math.max(1,Math.round(v)))}/>
          <NumberField label="kσ tightening" value={draftCfg.estimation.constraintSigma} step={0.25} onChange={(v) => update('estimation','constraintSigma',Math.max(0,v))}/>
        </div>}
      </div>}
    </section>

    <section className="sidebar-section sim-settings">
      <div className="sidebar-heading"><Gauge aria-hidden="true"/><strong>{t('sidebar.simulationSettings')}</strong></div>
      <NumberField label={t('sidebar.simulationTime')} value={draftCfg.duration} step={1} onChange={(v) => update(null,'duration',Math.max(1,v))}/>
      <NumberField label={t('sidebar.sampleTime')} value={draftCfg.dt} step={0.005} onChange={(v) => update(null,'dt',clamp(v,0.005,0.2))}/>
      <NumberField label={t('sidebar.referenceValue')} value={draftCfg.setpoint} step={0.1} onChange={(v) => update(null,'setpoint',v)}/>
      <ToggleField label={t('sidebar.disturbance')} checked={draftCfg.disturbance.enabled} onChange={(v) => update('disturbance','enabled',v)}/>
      <button type="button" className="run-button" onClick={onRun}><Play aria-hidden="true" fill="currentColor"/>{t('sidebar.run')}</button>
      <button type="button" className="reset-button" onClick={onReset}><RotateCcw aria-hidden="true"/>{t('sidebar.reset')}</button>
      <p className="run-feedback" aria-live="polite">{t('sidebar.runApplied')} · #{runRevision}</p>
    </section>

    <section className="sidebar-section presets-section"><ExperimentSafetyPanel cfg={draftCfg} setCfg={setDraftCfg} presetId={presetId} setPresetId={setPresetId}/></section>
  </aside>;
}
