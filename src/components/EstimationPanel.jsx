import React from 'react';
import { Radar } from 'lucide-react';

function NumberField({ label, value, step = 0.01, min = null, onChange }) {
  return (
    <label className="field">
      <span>{label}</span>
      <input
        type="number"
        value={value}
        step={step}
        min={min ?? undefined}
        onChange={(event) => onChange(Number(event.target.value))}
      />
    </label>
  );
}

function ToggleField({ label, checked, onChange }) {
  return (
    <label className="toggle-field">
      <span>{label}</span>
      <input type="checkbox" checked={checked} onChange={(event) => onChange(event.target.checked)} />
    </label>
  );
}

export default function EstimationPanel({ cfg, setCfg }) {
  const estimation = cfg.estimation;
  const update = (key, value) => setCfg((current) => ({
    ...current,
    estimation: { ...current.estimation, [key]: value },
  }));

  return (
    <div className="control-group estimation-controls">
      <div className="estimation-title"><h3>Sensor + state estimation</h3><Radar size={15}/></div>
      <ToggleField label="Kalman estimated-state control" checked={estimation.enabled} onChange={(value) => update('enabled', value)} />
      <p className="estimation-note">
        Khi bật, Event Trigger, MPC, PID và Safety Governor chỉ nhận x̂. Ground truth chỉ còn dùng để mô phỏng plant và chấm RMSE/safety.
      </p>
      {estimation.enabled && <>
        <NumberField label="Measurement noise σ" value={estimation.measurementNoiseStd} step={0.01} min={0} onChange={(value) => update('measurementNoiseStd', Math.max(0, value))} />
        <NumberField label="Measurement bias" value={estimation.measurementBias} step={0.005} onChange={(value) => update('measurementBias', value)} />
        <NumberField label="Noise seed" value={estimation.seed} step={1} min={1} onChange={(value) => update('seed', Math.max(1, Math.round(value)))} />
        <NumberField label="Q position" value={estimation.processPositionVariance} step={0.00001} min={0} onChange={(value) => update('processPositionVariance', Math.max(0, value))} />
        <NumberField label="Q velocity" value={estimation.processVelocityVariance} step={0.0001} min={0} onChange={(value) => update('processVelocityVariance', Math.max(0, value))} />
        <NumberField label="Initial P position" value={estimation.initialPositionVariance} step={0.05} min={0} onChange={(value) => update('initialPositionVariance', Math.max(0, value))} />
        <NumberField label="Initial P velocity" value={estimation.initialVelocityVariance} step={0.05} min={0} onChange={(value) => update('initialVelocityVariance', Math.max(0, value))} />
      </>}
    </div>
  );
}
