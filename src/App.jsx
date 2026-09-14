import React, { useMemo, useState } from 'react';
import { Activity, BrainCircuit, Cpu, Gauge, Play, Radar, ShieldCheck, SlidersHorizontal, TimerReset, Waves } from 'lucide-react';
import ExperimentSafetyPanel from './components/ExperimentSafetyPanel.jsx';
import { compareControllers, defaultConfig, getStateSpaceModel } from './core/simulator.js';
import { SOLVER_BACKENDS } from './core/solvers/index.js';
import './solverDiagnostics.css';

const MODES = ['PID', 'MPC', 'HYBRID', 'HYBRID_SAFE'];
const MODE_LABELS = {
  PID: 'PID',
  MPC: 'Periodic MPC',
  HYBRID: 'MPC + PID',
  HYBRID_SAFE: 'MPC + PID + Safety',
};

function NumberField({ label, value, step = 0.01, onChange }) {
  return (
    <label className="field">
      <span>{label}</span>
      <input type="number" value={value} step={step} onChange={(e) => onChange(Number(e.target.value))} />
    </label>
  );
}

function SelectField({ label, value, onChange, options }) {
  return (
    <label className="select-field">
      <span>{label}</span>
      <select value={value} onChange={(e) => onChange(e.target.value)}>
        {options.map((option) => <option key={option.value} value={option.value}>{option.label}</option>)}
      </select>
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

function fmtScientific(value) {
  if (value == null || !Number.isFinite(value)) return '—';
  if (value === 0) return '0';
  return value.toExponential(2);
}

export default function App() {
  const [cfg, setCfg] = useState(defaultConfig);
  const [activeMode, setActiveMode] = useState('HYBRID_SAFE');
  const [presetId, setPresetId] = useState('baseline');
  const results = useMemo(() => compareControllers(cfg), [cfg]);
  const active = results.find((r) => r.mode === activeMode);
  const hybrid = results.find((r) => r.mode === 'HYBRID');
  const governed = results.find((r) => r.mode === 'HYBRID_SAFE');
  const model = useMemo(() => fmtMatrix(getStateSpaceModel(cfg)), [cfg]);
  const update = (group, key, value) => setCfg((c) => group ? ({ ...c, [group]: { ...c[group], [key]: value } }) : ({ ...c, [key]: value }));
  const usingConstrainedQP = cfg.mpc.solver === SOLVER_BACKENDS.CONSTRAINED_QP;
  const usingBoxQP = cfg.mpc.solver === SOLVER_BACKENDS.BOX_QP;
  const usingQPSolver = usingConstrainedQP || usingBoxQP;
  const solverLabel = usingConstrainedQP ? 'Constrained QP' : usingBoxQP ? 'Box QP' : 'Projected gradient';
  const safetyEnabled = cfg.mpc.stateConstraintsEnabled || cfg.mpc.outputConstraintsEnabled;

  return (
    <div className="app-shell">
      <aside className="sidebar">
        <div className="brand"><div className="brand-mark"><BrainCircuit size={24} /></div><div><b>MPC · PID</b><span>Control Research Lab</span></div></div>
        <nav>
          <a className="nav-active"><Gauge size={18}/>Workbench</a>
          <a><Activity size={18}/>State-Space Plant</a>
          <a><Radar size={18}/>Event Trigger</a>
          <a><ShieldCheck size={18}/>Safety Governor</a>
          <a><Cpu size={18}/>Solver Diagnostics</a>
          <a><TimerReset size={18}/>Experiments</a>
        </nav>
        <div className="sidebar-note"><b>V0.3D Safety Authority</b><span>event MPC · predictive guidance · fast PID · short-horizon admissibility governor</span></div>
      </aside>

      <main>
        <header>
          <div><p className="eyebrow">ADVANCED CONTROL WORKBENCH</p><h1>Predict globally. React fast. Guard every command.</h1><p className="subtitle">MPC vẫn chỉ solve khi Event Trigger yêu cầu. PID chạy vòng nhanh. Safety Governor chạy rất nhẹ ở mỗi sample, tính miền lệnh admissible trong short horizon và không cho PID phát lệnh làm mất predicted safe envelope.</p></div>
          <button className="run"><Play size={17} fill="currentColor"/>Safety authority experiment</button>
        </header>

        <section className="kpi-grid">
          <div className="kpi"><span>Solver backend</span><strong>{solverLabel}</strong><small>{usingConstrainedQP ? 'general polyhedral QP' : 'research baseline'}</small></div>
          <div className="kpi"><span>MPC compute avoided</span><strong>{governed.metrics.computeReduction.toFixed(1)}%</strong><small>HYBRID_SAFE vs periodic solve every sample</small></div>
          <div className="kpi"><span>Governor intervention</span><strong>{governed.metrics.governorInterventionRate.toFixed(1)}%</strong><small>{governed.metrics.governorInterventionCount} fast-loop corrections</small></div>
          <div className={`kpi ${governed.metrics.maxActualSafetyViolation <= hybrid.metrics.maxActualSafetyViolation ? '' : 'kpi-warning'}`}><span>Safety gap: governed / guidance</span><strong>{safetyEnabled ? `${fmtScientific(governed.metrics.maxActualSafetyViolation)} / ${fmtScientific(hybrid.metrics.maxActualSafetyViolation)}` : 'OFF'}</strong><small>{safetyEnabled ? 'lower governed value is better' : 'load Safety Envelope preset to compare authority'}</small></div>
        </section>

        <section className="workspace">
          <div className="panel chart-panel">
            <div className="panel-head"><div><span className="section-tag">RESPONSE</span><h2>Four-layer closed-loop comparison</h2></div><div className="legend"><i className="l-pid"/>PID <i className="l-mpc"/>MPC <i className="l-hybrid"/>Hybrid <i className="l-safe"/>Governed</div></div>
            <ResponseChart data={results} target={cfg.setpoint} />
            <div className="mode-tabs">{MODES.map((m) => <button key={m} className={activeMode === m ? 'selected' : ''} onClick={() => setActiveMode(m)}>{MODE_LABELS[m]}</button>)}</div>

            <div className="timeline-wrap">
              <div className="timeline-title"><div><span className="section-tag">EVENTS</span><h3>Trigger timeline — {MODE_LABELS[activeMode]}</h3></div><span className="disturbance-key"><Waves size={14}/> disturbance window</span></div>
              <TriggerTimeline samples={active.samples} duration={cfg.duration} />
              <div className="trigger-legend"><span>prediction error</span><span>state change</span><span>constraint</span><span>watchdog</span></div>
            </div>
          </div>

          <div className="panel controls-panel">
            <div className="panel-head"><div><span className="section-tag">TUNING</span><h2>Experiment parameters</h2></div><SlidersHorizontal size={19}/></div>
            <div className="control-group"><h3>System</h3><NumberField label="Setpoint" value={cfg.setpoint} onChange={(v) => update(null, 'setpoint', v)}/><NumberField label="Sample time (s)" value={cfg.dt} step={0.005} onChange={(v) => update(null, 'dt', Math.max(0.005, v))}/></div>
            <div className="control-group"><h3>PID fast loop</h3><NumberField label="Kp" value={cfg.pid.kp} onChange={(v) => update('pid', 'kp', v)}/><NumberField label="Ki" value={cfg.pid.ki} onChange={(v) => update('pid', 'ki', v)}/><NumberField label="Kd" value={cfg.pid.kd} onChange={(v) => update('pid', 'kd', v)}/></div>

            <div className="control-group solver-controls">
              <h3>MPC solver</h3>
              <SelectField label="Backend" value={cfg.mpc.solver} onChange={(v) => update('mpc', 'solver', v)} options={[
                { value: SOLVER_BACKENDS.CONSTRAINED_QP, label: 'Constrained QP' },
                { value: SOLVER_BACKENDS.BOX_QP, label: 'Box QP' },
                { value: SOLVER_BACKENDS.PROJECTED_GRADIENT, label: 'Projected gradient' },
              ]}/>
              <span className="solver-badge"><Cpu size={12}/>{usingConstrainedQP ? 'AU≤b monitored' : usingBoxQP ? 'box KKT monitored' : 'baseline backend'}</span>
              {usingQPSolver ? <>
                <NumberField label="QP iterations" value={cfg.mpc.qpIterations} step={10} onChange={(v) => update('mpc', 'qpIterations', Math.max(1, Math.round(v)))}/>
                <NumberField label="QP tolerance" value={cfg.mpc.qpTolerance} step={0.0001} onChange={(v) => update('mpc', 'qpTolerance', Math.max(1e-10, v))}/>
                {usingConstrainedQP && <>
                  <NumberField label="Projection cycles" value={cfg.mpc.qpProjectionCycles} step={1} onChange={(v) => update('mpc', 'qpProjectionCycles', Math.max(1, Math.round(v)))}/>
                  <NumberField label="Time budget (ms)" value={cfg.mpc.qpTimeBudgetMs} step={1} onChange={(v) => update('mpc', 'qpTimeBudgetMs', Math.max(0, v))}/>
                </>}
              </> : <>
                <NumberField label="Iterations" value={cfg.mpc.iterations} step={1} onChange={(v) => update('mpc', 'iterations', Math.max(1, Math.round(v)))}/>
                <NumberField label="Learning rate" value={cfg.mpc.learningRate} step={0.01} onChange={(v) => update('mpc', 'learningRate', Math.max(0.001, v))}/>
              </>}
            </div>

            <div className="control-group"><h3>Predictive cost</h3><NumberField label="Horizon" value={cfg.mpc.horizon} step={1} onChange={(v) => update('mpc', 'horizon', Math.max(3, Math.round(v)))}/><NumberField label="Q position" value={cfg.mpc.qPosition} onChange={(v) => update('mpc', 'qPosition', Math.max(0, v))}/><NumberField label="Q velocity" value={cfg.mpc.qVelocity} onChange={(v) => update('mpc', 'qVelocity', Math.max(0, v))}/><NumberField label="R input" value={cfg.mpc.rInput} onChange={(v) => update('mpc', 'rInput', Math.max(0, v))}/><NumberField label="R Δu" value={cfg.mpc.rDelta} onChange={(v) => update('mpc', 'rDelta', Math.max(0, v))}/><NumberField label="Terminal weight" value={cfg.mpc.terminalWeight} step={0.5} onChange={(v) => update('mpc', 'terminalWeight', Math.max(1, v))}/></div>
            <div className="control-group constraint-controls"><h3>Hard actuator constraints</h3><NumberField label="u min" value={cfg.mpc.uMin} step={0.1} onChange={(v) => update('mpc', 'uMin', Math.min(v, cfg.mpc.uMax))}/><NumberField label="u max" value={cfg.mpc.uMax} step={0.1} onChange={(v) => update('mpc', 'uMax', Math.max(v, cfg.mpc.uMin))}/><NumberField label="Δu min / sample" value={cfg.mpc.deltaUMin} step={0.05} onChange={(v) => update('mpc', 'deltaUMin', Math.min(v, cfg.mpc.deltaUMax))}/><NumberField label="Δu max / sample" value={cfg.mpc.deltaUMax} step={0.05} onChange={(v) => update('mpc', 'deltaUMax', Math.max(v, cfg.mpc.deltaUMin))}/></div>
            <ExperimentSafetyPanel cfg={cfg} setCfg={setCfg} presetId={presetId} setPresetId={setPresetId}/>
            <div className="control-group governor-controls"><h3>Fast Safety Governor</h3><NumberField label="Preview horizon" value={cfg.safety.previewHorizon} step={1} onChange={(v) => update('safety', 'previewHorizon', Math.max(1, Math.round(v)))}/><NumberField label="Position margin" value={cfg.safety.positionMargin} step={0.01} onChange={(v) => update('safety', 'positionMargin', Math.max(0, v))}/><NumberField label="Velocity margin" value={cfg.safety.velocityMargin} step={0.01} onChange={(v) => update('safety', 'velocityMargin', Math.max(0, v))}/><NumberField label="Output margin" value={cfg.safety.outputMargin} step={0.01} onChange={(v) => update('safety', 'outputMargin', Math.max(0, v))}/><p className="constraint-note">Governor giả định một command ứng viên không đổi trên preview horizon để tạo interval admissible. Đây là guard bảo thủ, O(H), không phải MPC thứ hai.</p></div>
            <div className="control-group guidance-controls"><h3>MPC → PID guidance</h3><NumberField label="Reference lead" value={cfg.mpc.referenceLead} step={0.05} onChange={(v) => update('mpc', 'referenceLead', Math.max(0, v))}/><NumberField label="Velocity damping" value={cfg.mpc.velocityDamping} step={0.01} onChange={(v) => update('mpc', 'velocityDamping', Math.max(0, v))}/><NumberField label="Lead clamp" value={cfg.mpc.maxReferenceLead} step={0.05} onChange={(v) => update('mpc', 'maxReferenceLead', Math.max(0, v))}/></div>
            <div className="control-group accent"><h3>Event trigger</h3><NumberField label="Prediction error" value={cfg.trigger.predictionError} step={0.005} onChange={(v) => update('trigger', 'predictionError', Math.max(0.001, v))}/><NumberField label="State change" value={cfg.trigger.stateChange} step={0.01} onChange={(v) => update('trigger', 'stateChange', Math.max(0.001, v))}/><NumberField label="Min interval (s)" value={cfg.trigger.minInterval} step={0.02} onChange={(v) => update('trigger', 'minInterval', Math.max(cfg.dt, v))}/><NumberField label="Watchdog (s)" value={cfg.trigger.maxInterval} step={0.02} onChange={(v) => update('trigger', 'maxInterval', Math.max(cfg.trigger.minInterval, v))}/></div>
            <div className="control-group disturbance-controls"><h3>Disturbance injection</h3><ToggleField label="Enabled" checked={cfg.disturbance.enabled} onChange={(v) => update('disturbance', 'enabled', v)}/><NumberField label="Start (s)" value={cfg.disturbance.start} step={0.1} onChange={(v) => update('disturbance', 'start', Math.max(0, v))}/><NumberField label="Duration (s)" value={cfg.disturbance.duration} step={0.1} onChange={(v) => update('disturbance', 'duration', Math.max(cfg.dt, v))}/><NumberField label="Amplitude" value={cfg.disturbance.amplitude} step={0.1} onChange={(v) => update('disturbance', 'amplitude', v)}/></div>
          </div>
        </section>

        <section className="panel diagnostics-panel">
          <div className="panel-head"><div><span className="section-tag">SOLVER + SAFETY AUTHORITY</span><h2>{MODE_LABELS[activeMode]} audit</h2></div><span className="solver-badge">{cfg.mpc.solver}</span></div>
          <div className="diagnostics-grid diagnostics-grid-wide">
            <div className={`diag-card ${(active.metrics.convergenceRate ?? 100) >= 95 ? 'diag-good' : 'diag-warn'}`}><span>Convergence</span><strong>{active.metrics.convergenceRate == null ? 'N/A' : `${active.metrics.convergenceRate.toFixed(1)}%`}</strong><small>MPC optimization calls</small></div>
            <div className={`diag-card ${(active.metrics.maxFeasibilityViolation ?? 0) <= 1e-6 ? 'diag-good' : 'diag-warn'}`}><span>QP feasibility</span><strong>{fmtScientific(active.metrics.maxFeasibilityViolation)}</strong><small>predicted AU ≤ b violation</small></div>
            <div className={`diag-card ${active.metrics.maxActualSafetyViolation <= 1e-9 ? 'diag-good' : 'diag-warn'}`}><span>Real plant safety</span><strong>{safetyEnabled ? fmtScientific(active.metrics.maxActualSafetyViolation) : 'OFF'}</strong><small>{safetyEnabled ? `${active.metrics.safetyViolationRate.toFixed(1)}% unsafe samples` : 'load Safety Envelope preset'}</small></div>
            <div className="diag-card"><span>Governor intervention</span><strong>{active.metrics.governorEnabled ? `${active.metrics.governorInterventionRate.toFixed(1)}%` : 'OFF'}</strong><small>{active.metrics.governorInterventionCount} corrected samples</small></div>
            <div className={`diag-card ${active.metrics.governorEmergencyCount === 0 ? 'diag-good' : 'diag-warn'}`}><span>Governor emergency</span><strong>{active.metrics.governorEnabled ? active.metrics.governorEmergencyCount : '—'}</strong><small>{active.metrics.governorInfeasibleCount} empty admissible intervals</small></div>
            <div className="diag-card"><span>Max governor correction</span><strong>{active.metrics.governorEnabled ? active.metrics.governorMaxCorrection.toFixed(3) : '—'}</strong><small>|u_safe − u_PID|</small></div>
            <div className={`diag-card ${active.metrics.fallbackCount === 0 ? 'diag-good' : 'diag-warn'}`}><span>MPC fallback</span><strong>{active.metrics.fallbackCount}</strong><small>{active.metrics.timeoutCount} timeout · {active.metrics.infeasibleCount} infeasible</small></div>
          </div>
          <p className="solver-note">Safety Governor không thay MPC và cũng không thay PID. MPC giải bài toán horizon dài theo event; PID phản ứng nhanh; Governor chỉ kiểm tra command PID trong short horizon mỗi sample. Nếu admissible interval rỗng, hệ dùng last valid MPC move/physical fallback và ghi emergency rõ ràng.</p>
        </section>

        <section className="lower-grid">
          <div className="panel analysis-panel">
            <div className="panel-head"><div><span className="section-tag">ANALYSIS</span><h2>Controller, compute and safety authority</h2></div></div>
            <div className="comparison-table">
              <div className="table-row table-head solver-table-row"><span>Controller</span><span>IAE</span><span>MPC solves</span><span>Avg solve</span><span>Plant safety</span><span>Gov. intervention</span><span>Fallback</span></div>
              {results.map((r) => <div className={`table-row solver-table-row ${r.mode === activeMode ? 'row-active' : ''}`} key={r.mode} onClick={() => setActiveMode(r.mode)}><b>{MODE_LABELS[r.mode]}</b><span>{r.metrics.iae.toFixed(3)}</span><span>{r.metrics.solveCount}</span><span>{r.metrics.avgSolveMs.toFixed(3)} ms</span><span>{safetyEnabled ? fmtScientific(r.metrics.maxActualSafetyViolation) : '—'}</span><span>{r.metrics.governorEnabled ? `${r.metrics.governorInterventionRate.toFixed(1)}%` : '—'}</span><span>{r.metrics.fallbackCount}</span></div>)}
            </div>
          </div>

          <div className="panel model-panel">
            <div className="panel-head"><div><span className="section-tag">SAFETY ARCHITECTURE</span><h2>Two horizons, two compute budgets</h2></div></div>
            <div className="equation">MPC: X = Φx₀ + ΓU, &nbsp; min ½UᵀHU + fᵀU</div>
            <div className="equation">Governor: uPID ∈ [uL(x), uU(x)] over Hsafe</div>
            <div className="matrix-block"><span>A</span><pre>{model.A}</pre></div>
            <div className="matrix-block"><span>B</span><pre>{model.B}</pre></div>
            <div className="matrix-block"><span>C</span><pre>{model.C}</pre></div>
            <p className="model-note">MPC horizon dài tối ưu hiệu năng nhưng chạy theo event. Governor horizon ngắn chỉ tạo admissible scalar command interval nên đủ nhẹ để chạy ở vòng PID. Đây là cách giữ ý tưởng ban đầu: giảm solve MPC nhưng không bỏ quyền kiểm soát safety ở fast loop.</p>
          </div>
        </section>

        <section className="concept-grid">
          <div className="concept"><span>01</span><h3>Predict sparsely</h3><p>MPC horizon dài chỉ tái solve khi Event Trigger cho rằng prediction cũ không còn đáng tin.</p></div>
          <div className="concept"><span>02</span><h3>React fast</h3><p>PID vẫn là fast loop và nhận predictive guidance thay vì bị MPC thay thế hoàn toàn.</p></div>
          <div className="concept"><span>03</span><h3>Guard cheaply</h3><p>Safety Governor O(H) chạy mỗi sample, thu hẹp lệnh PID vào short-horizon admissible interval.</p></div>
          <div className="concept"><span>04</span><h3>Measure authority</h3><p>So sánh guidance-only với governed hybrid bằng plant-safety violation, intervention rate và tracking penalty.</p></div>
        </section>
      </main>
    </div>
  );
}
