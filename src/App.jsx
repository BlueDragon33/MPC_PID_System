import React, { useMemo, useState } from 'react';
import { Activity, BrainCircuit, Cpu, Gauge, Play, Radar, SlidersHorizontal, TimerReset } from 'lucide-react';
import { compareControllers, defaultConfig } from './core/simulator.js';

const MODES = ['PID', 'MPC', 'HYBRID'];

function NumberField({ label, value, step = 0.01, onChange }) {
  return (
    <label className="field">
      <span>{label}</span>
      <input type="number" value={value} step={step} onChange={(e) => onChange(Number(e.target.value))} />
    </label>
  );
}

function Chart({ data, target }) {
  const width = 960;
  const height = 320;
  const pad = 34;
  const all = data.flatMap((d) => d.samples.map((p) => p.x));
  const yMin = Math.min(-0.1, ...all);
  const yMax = Math.max(target * 1.25, ...all, 1.2);
  const tMax = Math.max(...data.flatMap((d) => d.samples.map((p) => p.t)));
  const x = (t) => pad + (t / tMax) * (width - 2 * pad);
  const y = (v) => height - pad - ((v - yMin) / (yMax - yMin)) * (height - 2 * pad);
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

function Metric({ label, value, suffix = '' }) {
  return <div className="metric"><span>{label}</span><strong>{value}{suffix}</strong></div>;
}

export default function App() {
  const [cfg, setCfg] = useState(defaultConfig);
  const [activeMode, setActiveMode] = useState('HYBRID');
  const results = useMemo(() => compareControllers(cfg), [cfg]);
  const active = results.find((r) => r.mode === activeMode);
  const update = (group, key, value) => setCfg((c) => group ? ({ ...c, [group]: { ...c[group], [key]: value } }) : ({ ...c, [key]: value }));

  return (
    <div className="app-shell">
      <aside className="sidebar">
        <div className="brand"><div className="brand-mark"><BrainCircuit size={24} /></div><div><b>MPC · PID</b><span>Control Research Lab</span></div></div>
        <nav>
          <a className="nav-active"><Gauge size={18}/>Workbench</a>
          <a><Activity size={18}/>Plant Models</a>
          <a><Radar size={18}/>Event Trigger</a>
          <a><Cpu size={18}/>Compute Profiler</a>
          <a><TimerReset size={18}/>Experiments</a>
        </nav>
        <div className="sidebar-note"><b>V0.1 Research Core</b><span>PID · MPC · Event-triggered Hybrid</span></div>
      </aside>

      <main>
        <header>
          <div><p className="eyebrow">ADVANCED CONTROL WORKBENCH</p><h1>Predict first. Correct fast.</h1><p className="subtitle">MPC dự đoán và tối ưu; PID giữ vòng phản xạ nhanh. Trigger quyết định khi nào đáng để tái tính MPC.</p></div>
          <button className="run"><Play size={17} fill="currentColor"/>Simulation live</button>
        </header>

        <section className="kpi-grid">
          <div className="kpi"><span>Active architecture</span><strong>MPC → Trigger → PID</strong><small>Hybrid supervisory control</small></div>
          <div className="kpi"><span>MPC solves</span><strong>{active.metrics.solveCount}</strong><small>over {cfg.duration.toFixed(1)} seconds</small></div>
          <div className="kpi"><span>Settling time</span><strong>{active.metrics.settling == null ? '—' : `${active.metrics.settling.toFixed(2)} s`}</strong><small>±2% band</small></div>
          <div className="kpi"><span>Integrated error</span><strong>{active.metrics.iae.toFixed(3)}</strong><small>IAE</small></div>
        </section>

        <section className="workspace">
          <div className="panel chart-panel">
            <div className="panel-head"><div><span className="section-tag">RESPONSE</span><h2>Closed-loop comparison</h2></div><div className="legend"><i className="l-pid"/>PID <i className="l-mpc"/>MPC <i className="l-hybrid"/>Hybrid</div></div>
            <Chart data={results} target={cfg.setpoint} />
            <div className="mode-tabs">{MODES.map((m) => <button key={m} className={activeMode === m ? 'selected' : ''} onClick={() => setActiveMode(m)}>{m === 'HYBRID' ? 'MPC + PID' : m}</button>)}</div>
          </div>

          <div className="panel controls-panel">
            <div className="panel-head"><div><span className="section-tag">TUNING</span><h2>Experiment parameters</h2></div><SlidersHorizontal size={19}/></div>
            <div className="control-group"><h3>System</h3><NumberField label="Setpoint" value={cfg.setpoint} onChange={(v) => update(null, 'setpoint', v)}/><NumberField label="Sample time (s)" value={cfg.dt} step={0.005} onChange={(v) => update(null, 'dt', Math.max(0.005, v))}/></div>
            <div className="control-group"><h3>PID</h3><NumberField label="Kp" value={cfg.pid.kp} onChange={(v) => update('pid', 'kp', v)}/><NumberField label="Ki" value={cfg.pid.ki} onChange={(v) => update('pid', 'ki', v)}/><NumberField label="Kd" value={cfg.pid.kd} onChange={(v) => update('pid', 'kd', v)}/></div>
            <div className="control-group"><h3>MPC</h3><NumberField label="Horizon" value={cfg.mpc.horizon} step={1} onChange={(v) => update('mpc', 'horizon', Math.max(2, Math.round(v)))}/><NumberField label="Q state" value={cfg.mpc.q} onChange={(v) => update('mpc', 'q', v)}/><NumberField label="R input" value={cfg.mpc.r} onChange={(v) => update('mpc', 'r', v)}/></div>
            <div className="control-group accent"><h3>Event trigger</h3><NumberField label="Prediction error" value={cfg.trigger.predictionError} step={0.01} onChange={(v) => update('trigger', 'predictionError', Math.max(0.001, v))}/><NumberField label="State error" value={cfg.trigger.stateError} step={0.01} onChange={(v) => update('trigger', 'stateError', Math.max(0.001, v))}/><NumberField label="Max interval (s)" value={cfg.trigger.maxInterval} step={0.02} onChange={(v) => update('trigger', 'maxInterval', Math.max(cfg.dt, v))}/></div>
          </div>
        </section>

        <section className="panel analysis-panel">
          <div className="panel-head"><div><span className="section-tag">ANALYSIS</span><h2>Controller efficiency</h2></div></div>
          <div className="comparison-table">
            <div className="table-row table-head"><span>Controller</span><span>Overshoot</span><span>Settling</span><span>IAE</span><span>Control effort</span><span>MPC solves</span></div>
            {results.map((r) => <div className={`table-row ${r.mode === activeMode ? 'row-active' : ''}`} key={r.mode} onClick={() => setActiveMode(r.mode)}><b>{r.mode === 'HYBRID' ? 'Event MPC + PID' : r.mode}</b><span>{r.metrics.overshoot.toFixed(2)}%</span><span>{r.metrics.settling == null ? '—' : `${r.metrics.settling.toFixed(2)} s`}</span><span>{r.metrics.iae.toFixed(3)}</span><span>{r.metrics.controlEffort.toFixed(3)}</span><span>{r.metrics.solveCount}</span></div>)}
          </div>
        </section>

        <section className="concept-grid">
          <div className="concept"><span>01</span><h3>Predict</h3><p>MPC đánh giá horizon tương lai trên mô hình plant và chọn hành động có cost thấp.</p></div>
          <div className="concept"><span>02</span><h3>Trigger</h3><p>Chỉ tái solve khi prediction error, state error hoặc watchdog yêu cầu.</p></div>
          <div className="concept"><span>03</span><h3>Correct</h3><p>PID chạy ở vòng nhanh để bám reference do lớp dự đoán cung cấp.</p></div>
          <div className="concept"><span>04</span><h3>Measure</h3><p>So sánh overshoot, settling time, IAE, control effort và số lần solve MPC.</p></div>
        </section>
      </main>
    </div>
  );
}
