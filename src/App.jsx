import React, { useMemo, useState } from 'react';
import { Activity, BrainCircuit, Cpu, Gauge, Play, Radar, SlidersHorizontal, TimerReset, Waves } from 'lucide-react';
import { compareControllers, defaultConfig, getStateSpaceModel } from './core/simulator.js';

const MODES = ['PID', 'MPC', 'HYBRID'];

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

function ResponseChart({ data, target }) {
  const width = 960;
  const height = 320;
  const pad = 34;
  const all = data.flatMap((d) => d.samples.map((p) => p.x));
  const yMin = Math.min(-0.1, ...all);
  const yMax = Math.max(target * 1.25, ...all, 1.2);
  const tMax = Math.max(...data.flatMap((d) => d.samples.map((p) => p.t)));
  const x = (t) => pad + (t / tMax) * (width - 2 * pad);
  const y = (v) => height - pad - ((v - yMin) / Math.max(yMax - yMin, 1e-9)) * (height - 2 * pad);
  const path = (samples) => samples.map((p, i) => `${i ? 'L' : 'M'} ${x(p.t).toFixed(1)} ${y(p.x).toFixed(1)}`).join(' ');

  return (
    <svg className="chart" viewBox={`0 0 ${width} ${height}`} role="img" aria-label="Controller response comparison">
      <line x1={pad} y1={y(target)} x2={width - pad} y2={y(target)} className="target-line" />
      <line x1={pad} y1={height - pad} x2={width - pad} y2={height - pad} className="axis" />
      <line x1={pad} y1={pad} x2={pad} y2={height - pad} className="axis" />
      {data.map((d) => <path key={d.mode} d={path(d.samples)} className={`curve curve-${d.mode.toLowerCase()}`} />)}
    </svg>
  );
}

function TriggerTimeline({ samples, duration }) {
  const width = 960;
  const height = 96;
  const pad = 34;
  const x = (t) => pad + (t / duration) * (width - pad * 2);
  const events = samples.filter((p) => p.triggered);
  const disturbed = samples.filter((p) => Math.abs(p.disturbance) > 1e-9);
  const d0 = disturbed[0]?.t;
  const d1 = disturbed[disturbed.length - 1]?.t;
  const reasonY = { initial: 31, 'prediction-error': 31, 'state-change': 48, constraint: 65, watchdog: 78, periodic: 48 };

  return (
    <svg className="timeline" viewBox={`0 0 ${width} ${height}`} role="img" aria-label="MPC trigger timeline">
      {d0 != null && <rect x={x(d0)} y="12" width={Math.max(2, x(d1) - x(d0))} height="68" className="disturbance-band" />}
      <line x1={pad} y1="48" x2={width - pad} y2="48" className="axis" />
      {events.map((p, index) => (
        <g key={`${p.t}-${index}`}>
          <line x1={x(p.t)} y1="22" x2={x(p.t)} y2="76" className={`trigger-line trigger-${p.triggerReason}`} />
          <circle cx={x(p.t)} cy={reasonY[p.triggerReason] ?? 48} r="3.2" className={`trigger-dot trigger-${p.triggerReason}`} />
        </g>
      ))}
    </svg>
  );
}

function fmtMatrix(model) {
  return {
    A: model.A.map((row) => `[ ${row.map((v) => v.toFixed(4)).join('   ')} ]`).join('\n'),
    B: `[ ${model.B.map((v) => v.toFixed(4)).join('   ')} ]ᵀ`,
    C: `[ ${model.C.map((v) => v.toFixed(1)).join('   ')} ]`,
  };
}

export default function App() {
  const [cfg, setCfg] = useState(defaultConfig);
  const [activeMode, setActiveMode] = useState('HYBRID');
  const results = useMemo(() => compareControllers(cfg), [cfg]);
  const active = results.find((r) => r.mode === activeMode);
  const hybrid = results.find((r) => r.mode === 'HYBRID');
  const model = useMemo(() => fmtMatrix(getStateSpaceModel(cfg)), [cfg]);
  const update = (group, key, value) => setCfg((c) => group ? ({ ...c, [group]: { ...c[group], [key]: value } }) : ({ ...c, [key]: value }));

  return (
    <div className="app-shell">
      <aside className="sidebar">
        <div className="brand"><div className="brand-mark"><BrainCircuit size={24} /></div><div><b>MPC · PID</b><span>Control Research Lab</span></div></div>
        <nav>
          <a className="nav-active"><Gauge size={18}/>Workbench</a>
          <a><Activity size={18}/>State-Space Plant</a>
          <a><Radar size={18}/>Event Trigger</a>
          <a><Cpu size={18}/>Compute Profiler</a>
          <a><TimerReset size={18}/>Experiments</a>
        </nav>
        <div className="sidebar-note"><b>V0.2 Predictive Core</b><span>State-space · sequence MPC · predictive reference shaping · event-trigger profiler</span></div>
      </aside>

      <main>
        <header>
          <div><p className="eyebrow">ADVANCED CONTROL WORKBENCH</p><h1>Predict only when prediction is worth the cost.</h1><p className="subtitle">MPC dự đoán sai số tương lai và tạo setpoint dẫn trước có giới hạn; PID giữ vòng phản xạ nhanh. Event Trigger chỉ gọi lại optimizer khi trạng thái thực sự đáng để tái dự đoán.</p></div>
          <button className="run"><Play size={17} fill="currentColor"/>Deterministic simulation</button>
        </header>

        <section className="kpi-grid">
          <div className="kpi"><span>Architecture</span><strong>MPC predictor → PID</strong><small>event-triggered predictive guidance</small></div>
          <div className="kpi"><span>Hybrid MPC solves</span><strong>{hybrid.metrics.solveCount}</strong><small>{hybrid.metrics.triggerRate.toFixed(1)} solves/s average</small></div>
          <div className="kpi"><span>Compute avoided</span><strong>{hybrid.metrics.computeReduction.toFixed(1)}%</strong><small>vs solving MPC every sample</small></div>
          <div className="kpi"><span>Average solve</span><strong>{hybrid.metrics.avgSolveMs.toFixed(3)} ms</strong><small>projected-gradient optimizer</small></div>
        </section>

        <section className="workspace">
          <div className="panel chart-panel">
            <div className="panel-head"><div><span className="section-tag">RESPONSE</span><h2>Closed-loop comparison under disturbance</h2></div><div className="legend"><i className="l-pid"/>PID <i className="l-mpc"/>MPC <i className="l-hybrid"/>Hybrid</div></div>
            <ResponseChart data={results} target={cfg.setpoint} />
            <div className="mode-tabs">{MODES.map((m) => <button key={m} className={activeMode === m ? 'selected' : ''} onClick={() => setActiveMode(m)}>{m === 'HYBRID' ? 'MPC + PID' : m}</button>)}</div>

            <div className="timeline-wrap">
              <div className="timeline-title"><div><span className="section-tag">EVENTS</span><h3>Trigger timeline — {activeMode === 'HYBRID' ? 'event-driven solves' : activeMode === 'MPC' ? 'periodic solves' : 'no MPC solver'}</h3></div><span className="disturbance-key"><Waves size={14}/> disturbance window</span></div>
              <TriggerTimeline samples={active.samples} duration={cfg.duration} />
              <div className="trigger-legend"><span>prediction error</span><span>state change</span><span>constraint</span><span>watchdog</span></div>
            </div>
          </div>

          <div className="panel controls-panel">
            <div className="panel-head"><div><span className="section-tag">TUNING</span><h2>Experiment parameters</h2></div><SlidersHorizontal size={19}/></div>
            <div className="control-group"><h3>System</h3><NumberField label="Setpoint" value={cfg.setpoint} onChange={(v) => update(null, 'setpoint', v)}/><NumberField label="Sample time (s)" value={cfg.dt} step={0.005} onChange={(v) => update(null, 'dt', Math.max(0.005, v))}/></div>
            <div className="control-group"><h3>PID fast loop</h3><NumberField label="Kp" value={cfg.pid.kp} onChange={(v) => update('pid', 'kp', v)}/><NumberField label="Ki" value={cfg.pid.ki} onChange={(v) => update('pid', 'ki', v)}/><NumberField label="Kd" value={cfg.pid.kd} onChange={(v) => update('pid', 'kd', v)}/></div>
            <div className="control-group"><h3>Predictive MPC</h3><NumberField label="Horizon" value={cfg.mpc.horizon} step={1} onChange={(v) => update('mpc', 'horizon', Math.max(3, Math.round(v)))}/><NumberField label="Q position" value={cfg.mpc.qPosition} onChange={(v) => update('mpc', 'qPosition', Math.max(0, v))}/><NumberField label="Q velocity" value={cfg.mpc.qVelocity} onChange={(v) => update('mpc', 'qVelocity', Math.max(0, v))}/><NumberField label="R input" value={cfg.mpc.rInput} onChange={(v) => update('mpc', 'rInput', Math.max(0, v))}/><NumberField label="Terminal weight" value={cfg.mpc.terminalWeight} step={0.5} onChange={(v) => update('mpc', 'terminalWeight', Math.max(1, v))}/><NumberField label="Solver iterations" value={cfg.mpc.iterations} step={1} onChange={(v) => update('mpc', 'iterations', Math.max(1, Math.round(v)))}/></div>
            <div className="control-group guidance-controls"><h3>MPC → PID guidance</h3><NumberField label="Reference lead" value={cfg.mpc.referenceLead} step={0.05} onChange={(v) => update('mpc', 'referenceLead', Math.max(0, v))}/><NumberField label="Velocity damping" value={cfg.mpc.velocityDamping} step={0.01} onChange={(v) => update('mpc', 'velocityDamping', Math.max(0, v))}/><NumberField label="Lead clamp" value={cfg.mpc.maxReferenceLead} step={0.05} onChange={(v) => update('mpc', 'maxReferenceLead', Math.max(0, v))}/></div>
            <div className="control-group accent"><h3>Event trigger</h3><NumberField label="Prediction error" value={cfg.trigger.predictionError} step={0.005} onChange={(v) => update('trigger', 'predictionError', Math.max(0.001, v))}/><NumberField label="State change" value={cfg.trigger.stateChange} step={0.01} onChange={(v) => update('trigger', 'stateChange', Math.max(0.001, v))}/><NumberField label="Min interval (s)" value={cfg.trigger.minInterval} step={0.02} onChange={(v) => update('trigger', 'minInterval', Math.max(cfg.dt, v))}/><NumberField label="Watchdog (s)" value={cfg.trigger.maxInterval} step={0.02} onChange={(v) => update('trigger', 'maxInterval', Math.max(cfg.trigger.minInterval, v))}/></div>
            <div className="control-group disturbance-controls"><h3>Disturbance injection</h3><ToggleField label="Enabled" checked={cfg.disturbance.enabled} onChange={(v) => update('disturbance', 'enabled', v)}/><NumberField label="Start (s)" value={cfg.disturbance.start} step={0.1} onChange={(v) => update('disturbance', 'start', Math.max(0, v))}/><NumberField label="Duration (s)" value={cfg.disturbance.duration} step={0.1} onChange={(v) => update('disturbance', 'duration', Math.max(cfg.dt, v))}/><NumberField label="Amplitude" value={cfg.disturbance.amplitude} step={0.1} onChange={(v) => update('disturbance', 'amplitude', v)}/></div>
          </div>
        </section>

        <section className="lower-grid">
          <div className="panel analysis-panel">
            <div className="panel-head"><div><span className="section-tag">ANALYSIS</span><h2>Controller efficiency</h2></div></div>
            <div className="comparison-table">
              <div className="table-row table-head"><span>Controller</span><span>Overshoot</span><span>Settling</span><span>IAE</span><span>Effort</span><span>MPC solves</span><span>Solver total</span></div>
              {results.map((r) => <div className={`table-row ${r.mode === activeMode ? 'row-active' : ''}`} key={r.mode} onClick={() => setActiveMode(r.mode)}><b>{r.mode === 'HYBRID' ? 'Event MPC + PID' : r.mode}</b><span>{r.metrics.overshoot.toFixed(2)}%</span><span>{r.metrics.settling == null ? '—' : `${r.metrics.settling.toFixed(2)} s`}</span><span>{r.metrics.iae.toFixed(3)}</span><span>{r.metrics.controlEffort.toFixed(3)}</span><span>{r.metrics.solveCount}</span><span>{r.metrics.totalSolveMs.toFixed(2)} ms</span></div>)}
            </div>
          </div>

          <div className="panel model-panel">
            <div className="panel-head"><div><span className="section-tag">MODEL</span><h2>Discrete state-space</h2></div></div>
            <div className="equation">xₖ₊₁ = A xₖ + B uₖ + E dₖ</div>
            <div className="matrix-block"><span>A</span><pre>{model.A}</pre></div>
            <div className="matrix-block"><span>B</span><pre>{model.B}</pre></div>
            <div className="matrix-block"><span>C</span><pre>{model.C}</pre></div>
            <p className="model-note">State vector: x = [position, velocity]ᵀ. MPC dùng model danh định để dự đoán sai số tương lai; lớp guidance biến dự đoán đó thành setpoint dẫn trước có giới hạn cho PID.</p>
          </div>
        </section>

        <section className="concept-grid">
          <div className="concept"><span>01</span><h3>Model</h3><p>Plant được rời rạc hóa thành state-space để sau này thay trực tiếp bằng UAV, UGV hoặc USV.</p></div>
          <div className="concept"><span>02</span><h3>Optimize</h3><p>Projected-gradient MPC tối ưu control sequence có input bounds, state cost, terminal cost và Δu cost.</p></div>
          <div className="concept"><span>03</span><h3>Guide</h3><p>MPC không thay PID: nó dự đoán tương lai và tạo reference dẫn trước để PID phản ứng sớm nhưng vẫn bị giới hạn an toàn.</p></div>
          <div className="concept"><span>04</span><h3>Trigger + Profile</h3><p>Chỉ solve lại khi cần, đồng thời đo chất lượng điều khiển và chi phí tính toán của từng kiến trúc.</p></div>
        </section>
      </main>
    </div>
  );
}
