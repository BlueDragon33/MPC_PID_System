import React, { useRef, useState } from 'react';
import { Download, FolderOpen, Save, ShieldCheck, Upload } from 'lucide-react';
import EstimationPanel from './EstimationPanel.jsx';
import { defaultConfig } from '../core/simulator.js';
import { applyExperimentPreset, EXPERIMENT_PRESETS } from '../core/experiments/presets.js';
import {
  downloadExperimentJSON,
  loadExperimentLocal,
  parseExperimentPayload,
  saveExperimentLocal,
} from '../core/experiments/serialization.js';
import { SOLVER_BACKENDS } from '../core/solvers/index.js';

function NumberField({ label, value, step = 0.01, onChange }) {
  return (
    <label className="field">
      <span>{label}</span>
      <input type="number" value={value} step={step} onChange={(e) => onChange(Number(e.target.value))} />
    </label>
  );
}

function ToggleField({ label, checked, onChange }) {
  return (
    <label className="toggle-field">
      <span>{label}</span>
      <input type="checkbox" checked={checked} onChange={(e) => onChange(e.target.checked)} />
    </label>
  );
}

export default function ExperimentSafetyPanel({ cfg, setCfg, presetId, setPresetId }) {
  const [ioStatus, setIoStatus] = useState('');
  const fileInputRef = useRef(null);
  const constrained = cfg.mpc.solver === SOLVER_BACKENDS.CONSTRAINED_QP;
  const selected = EXPERIMENT_PRESETS.find((preset) => preset.id === presetId) || EXPERIMENT_PRESETS[0];
  const updateMpc = (key, value) => setCfg((current) => ({ ...current, mpc: { ...current.mpc, [key]: value } }));
  const selectPreset = (id) => {
    setPresetId(id);
    setCfg(applyExperimentPreset(defaultConfig, id));
    setIoStatus(`Loaded preset: ${id}`);
  };

  const saveLocal = () => {
    const ok = saveExperimentLocal(cfg, { name: `MPC PID ${presetId}`, presetId });
    setIoStatus(ok ? 'Experiment saved locally.' : 'Local storage is unavailable.');
  };

  const loadLocal = () => {
    try {
      const payload = loadExperimentLocal();
      if (!payload) {
        setIoStatus('No local experiment found.');
        return;
      }
      setCfg(payload.config);
      setPresetId(payload.presetId || 'custom');
      setIoStatus('Local experiment restored.');
    } catch (error) {
      setIoStatus(`Load failed: ${error.message}`);
    }
  };

  const exportJson = () => {
    const ok = downloadExperimentJSON(cfg, { name: `MPC-PID-${presetId}`, presetId });
    setIoStatus(ok ? 'Experiment JSON exported.' : 'Export is unavailable in this environment.');
  };

  const importJson = async (event) => {
    const file = event.target.files?.[0];
    event.target.value = '';
    if (!file) return;
    try {
      const payload = parseExperimentPayload(await file.text());
      setCfg(payload.config);
      setPresetId(payload.presetId || 'custom');
      setIoStatus(`Imported ${file.name}.`);
    } catch (error) {
      setIoStatus(`Import failed: ${error.message}`);
    }
  };

  return (
    <>
      <div className="control-group safety-controls">
        <div className="safety-title"><h3>Experiment + safety envelope</h3><ShieldCheck size={15}/></div>
        <label className="select-field">
          <span>Preset</span>
          <select value={EXPERIMENT_PRESETS.some((preset) => preset.id === presetId) ? presetId : 'baseline'} onChange={(e) => selectPreset(e.target.value)}>
            {EXPERIMENT_PRESETS.map((preset) => <option key={preset.id} value={preset.id}>{preset.label}</option>)}
          </select>
        </label>
        <p className="preset-description">{selected.description}</p>

        <div className="experiment-actions">
          <button type="button" onClick={saveLocal}><Save size={12}/>Save</button>
          <button type="button" onClick={loadLocal}><FolderOpen size={12}/>Load</button>
          <button type="button" onClick={exportJson}><Download size={12}/>Export</button>
          <button type="button" onClick={() => fileInputRef.current?.click()}><Upload size={12}/>Import</button>
          <input ref={fileInputRef} className="hidden-file-input" type="file" accept="application/json,.json" onChange={importJson}/>
        </div>
        {ioStatus && <p className="experiment-status">{ioStatus}</p>}

        <ToggleField label="Predicted state limits" checked={cfg.mpc.stateConstraintsEnabled} onChange={(v) => updateMpc('stateConstraintsEnabled', v)}/>
        {cfg.mpc.stateConstraintsEnabled && <div className="envelope-grid">
          <NumberField label="Position min" value={cfg.mpc.positionMin} step={0.05} onChange={(v) => updateMpc('positionMin', Math.min(v, cfg.mpc.positionMax))}/>
          <NumberField label="Position max" value={cfg.mpc.positionMax} step={0.05} onChange={(v) => updateMpc('positionMax', Math.max(v, cfg.mpc.positionMin))}/>
          <NumberField label="Velocity min" value={cfg.mpc.velocityMin} step={0.05} onChange={(v) => updateMpc('velocityMin', Math.min(v, cfg.mpc.velocityMax))}/>
          <NumberField label="Velocity max" value={cfg.mpc.velocityMax} step={0.05} onChange={(v) => updateMpc('velocityMax', Math.max(v, cfg.mpc.velocityMin))}/>
        </div>}

        <ToggleField label="Predicted output limits" checked={cfg.mpc.outputConstraintsEnabled} onChange={(v) => updateMpc('outputConstraintsEnabled', v)}/>
        {cfg.mpc.outputConstraintsEnabled && <div className="envelope-grid">
          <NumberField label="Output min" value={cfg.mpc.outputMin} step={0.02} onChange={(v) => updateMpc('outputMin', Math.min(v, cfg.mpc.outputMax))}/>
          <NumberField label="Output max" value={cfg.mpc.outputMax} step={0.02} onChange={(v) => updateMpc('outputMax', Math.max(v, cfg.mpc.outputMin))}/>
        </div>}

        {!constrained && (cfg.mpc.stateConstraintsEnabled || cfg.mpc.outputConstraintsEnabled) &&
          <p className="constraint-warning">Selected backend is a baseline and does not enforce the general state/output polyhedron. Use Constrained QP for hard predicted envelopes.</p>}
        <p className="constraint-note">Predicted constraints apply to the MPC model. Actual plant safety is audited separately from predicted feasibility.</p>
      </div>
      <EstimationPanel cfg={cfg} setCfg={setCfg}/>
    </>
  );
}
