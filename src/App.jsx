import React, { useMemo, useState } from 'react';
import { Activity, BrainCircuit, Cpu, Gauge, Play, Radar, SlidersHorizontal, TimerReset, Waves } from 'lucide-react';
import ExperimentSafetyPanel from './components/ExperimentSafetyPanel.jsx';
import { compareControllers, defaultConfig, getStateSpaceModel } from './core/simulator.js';
import { SOLVER_BACKENDS } from './core/solvers/index.js';
import './solverDiagnostics.css';

const MODES = ['PID', 'MPC', 'HYBRID'];

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
  const [activeMode, setActiveMode] = useState('HYBRID');
  const [presetId, setPresetId] = useState('baseline');
  const results = useMemo(() => compareControllers(cfg), [cfg]);
  const active = results.find((r) => r.mode === activeMode);
  const hybrid = results.find((r) => r.mode === 'HYBRID');
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
          <a><Cpu size={18}/>Solver Diagnostics</a>
          <a><TimerReset size={18}/>Experiments</a>
        </nav>
        <div className="sidebar-note"><b>V0.3C Safety Envelope</b><span>predicted state/output constraints · presets · plant-safety audit</span></div>
      </aside>

      <main>
        <header>
          <div><p className="eyebrow">ADVANCED CONTROL WORKBENCH</p><h1>Constrain the predicted future. Audit the real plant.</h1><p className="subtitle">Input, Δu, state và output constraints cùng đi vào AU ≤ b. QP feasibility cho biết model có nằm trong miền an toàn dự đoán hay không; plant-safety metric kiểm tra riêng những gì thật sự xảy ra sau PID và disturbance.</p></div>
          <button className="run"><Play size={17} fill="currentColor"/>Reproducible experiment</button>
        </header>

        <section className="kpi-grid">
          <div className="kpi"><span>Solver backend</span><strong>{solverLabel}</strong><small>{usingConstrainedQP ? 'general polyhedral QP' : 'research baseline'}</small></div>
          <div className="kpi"><span>Hybrid fallback</span><strong>{hybrid.metrics.fallbackRate.toFixed(1)}%</strong><small>{hybrid.metrics.fallbackCount} fallback solves</small></div>
          <div className="kpi"><span>Compute avoided</span><strong>{hybrid.metrics.computeReduction.toFixed(1)}%</strong><small>vs solving MPC every sample</small></div>
          <div className={`kpi ${hybrid.metrics.maxActualSafetyViolation <= 1e-9 ? '' : 'kpi-warning'}`}><span>Actual safety violation</span><strong>{safetyEnabled ? fmtScientific(hybrid.metrics.maxActualSafetyViolation) : 'OFF'}</strong><small>{safetyEnabled ? `${hybrid.metrics.safetyViolationRate.toFixed(1)}% samples outside envelope` : 'enable predicted envelope to audit'}</small></div>
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
            <div className="control-group guidance-controls"><h3>MPC → PID guidance</h3><NumberField label="Reference lead" value={cfg.mpc.referenceLead} step={0.05} onChange={(v) => update('mpc', 'referenceLead', Math.max(0, v))}/><NumberField label="Velocity damping" value={cfg.mpc.velocityDamping} step={0.01} onChange={(v) => update('mpc', 'velocityDamping', Math.max(0, v))}/><NumberField label="Lead clamp" value={cfg.mpc.maxReferenceLead} step={0.05} onChange={(v) => update('mpc', 'maxReferenceLead', Math.max(0, v))}/></div>
            <div className="control-group accent"><h3>Event trigger</h3><NumberField label="Prediction error" value={cfg.trigger.predictionError} step={0.005} onChange={(v) => update('trigger', 'predictionError', Math.max(0.001, v))}/><NumberField label="State change" value={cfg.trigger.stateChange} step={0.01} onChange={(v) => update('trigger', 'stateChange', Math.max(0.001, v))}/><NumberField label="Min interval (s)" value={cfg.trigger.minInterval} step={0.02} onChange={(v) => update('trigger', 'minInterval', Math.max(cfg.dt, v))}/><NumberField label="Watchdog (s)" value={cfg.trigger.maxInterval} step={0.02} onChange={(v) => update('trigger', 'maxInterval', Math.max(cfg.trigger.minInterval, v))}/></div>
            <div className="control-group disturbance-controls"><h3>Disturbance injection</h3><ToggleField label="Enabled" checked={cfg.disturbance.enabled} onChange={(v) => update('disturbance', 'enabled', v)}/><NumberField label="Start (s)" value={cfg.disturbance.start} step={0.1} onChange={(v) => update('disturbance', 'start', Math.max(0, v))}/><NumberField label="Duration (s)" value={cfg.disturbance.duration} step={0.1} onChange={(v) => update('disturbance', 'duration', Math.max(cfg.dt, v))}/><NumberField label="Amplitude" value={cfg.disturbance.amplitude} step={0.1} onChange={(v) => update('disturbance', 'amplitude', v)}/></div>
          </div>
        </section>

        <section className="panel diagnostics-panel">
          <div className="panel-head"><div><span className="section-tag">SOLVER + SAFETY HEALTH</span><h2>{activeMode === 'PID' ? 'No optimizer in PID mode' : `${activeMode === 'HYBRID' ? 'Hybrid' : 'Periodic MPC'} model and plant audit`}</h2></div><span className="solver-badge">{cfg.mpc.solver}</span></div>
          <div className="diagnostics-grid diagnostics-grid-wide">
            <div className={`diag-card ${(active.metrics.convergenceRate ?? 100) >= 95 ? 'diag-good' : 'diag-warn'}`}><span>Convergence</span><strong>{active.metrics.convergenceRate == null ? 'N/A' : `${active.metrics.convergenceRate.toFixed(1)}%`}</strong><small>strict stationarity tolerance</small></div>
            <div className="diag-card"><span>Average iterations</span><strong>{active.metrics.avgIterations ? active.metrics.avgIterations.toFixed(1) : '—'}</strong><small>iterations per optimization call</small></div>
            <div className="diag-card"><span>Stationarity residual</span><strong>{fmtScientific(active.metrics.avgStationarityResidual)}</strong><small>projected-gradient/KKT residual</small></div>
            <div className={`diag-card ${(active.metrics.maxFeasibilityViolation ?? 0) <= 1e-6 ? 'diag-good' : 'diag-warn'}`}><span>QP feasibility</span><strong>{fmtScientific(active.metrics.maxFeasibilityViolation)}</strong><small>maximum predicted AU ≤ b violation</small></div>
            <div className={`diag-card ${active.metrics.maxActualSafetyViolation <= 1e-9 ? 'diag-good' : 'diag-warn'}`}><span>Real plant safety</span><strong>{safetyEnabled ? fmtScientific(active.metrics.maxActualSafetyViolation) : 'OFF'}</strong><small>{safetyEnabled ? `${active.metrics.safetyViolationRate.toFixed(1)}% unsafe samples` : 'state/output envelope disabled'}</small></div>
            <div className="diag-card"><span>Safety constraint active</span><strong>{safetyEnabled ? `${active.metrics.stateConstraintActiveSolveRate.toFixed(1)}%` : 'OFF'}</strong><small>solves touching state/output boundary</small></div>
            <div className={`diag-card ${active.metrics.fallbackCount === 0 ? 'diag-good' : 'diag-warn'}`}><span>Fallback</span><strong>{active.metrics.fallbackCount}</strong><small>{active.metrics.timeoutCount} timeout · {active.metrics.infeasibleCount} infeasible</small></div>
          </div>
          <p className="solver-note">V0.3C tách rõ model safety khỏi plant safety. Periodic MPC trực tiếp thi hành nghiệm tối ưu; hybrid hiện dùng MPC để tạo guidance cho PID nên predicted feasibility chưa đồng nghĩa với bảo đảm an toàn tuyệt đối của plant thật. Khoảng cách này được đo thay vì bị che giấu.</p>
        </section>

        <section className="lower-grid">
          <div className="panel analysis-panel">
            <div className="panel-head"><div><span className="section-tag">ANALYSIS</span><h2>Controller, solver and safety efficiency</h2></div></div>
            <div className="comparison-table">
              <div className="table-row table-head solver-table-row"><span>Controller</span><span>IAE</span><span>MPC solves</span><span>Avg solve</span><span>QP viol.</span><span>Plant safety</span><span>Fallback</span></div>
              {results.map((r) => <div className={`table-row solver-table-row ${r.mode === activeMode ? 'row-active' : ''}`} key={r.mode} onClick={() => setActiveMode(r.mode)}><b>{r.mode === 'HYBRID' ? 'Event MPC + PID' : r.mode}</b><span>{r.metrics.iae.toFixed(3)}</span><span>{r.metrics.solveCount}</span><span>{r.metrics.avgSolveMs.toFixed(3)} ms</span><span>{fmtScientific(r.metrics.maxFeasibilityViolation)}</span><span>{safetyEnabled ? fmtScientific(r.metrics.maxActualSafetyViolation) : '—'}</span><span>{r.metrics.fallbackCount}</span></div>)}
            </div>
          </div>

          <div className="panel model-panel">
            <div className="panel-head"><div><span className="section-tag">MODEL + CONSTRAINTS</span><h2>Discrete state-space</h2></div></div>
            <div className="equation">xₖ₊₁ = A xₖ + B uₖ + E dₖ</div>
            <div className="equation">X = Φx₀ + ΓU</div>
            <div className="equation">min ½UᵀHU + fᵀU &nbsp; s.t. &nbsp; AU ≤ b</div>
            <div className="matrix-block"><span>A</span><pre>{model.A}</pre></div>
            <div className="matrix-block"><span>B</span><pre>{model.B}</pre></div>
            <div className="matrix-block"><span>C</span><pre>{model.C}</pre></div>
            <p className="model-note">A·U≤b hiện có thể chứa input, slew-rate, predicted position, velocity và output inequalities. Constraint activation được theo dõi riêng để biết MPC đang thực sự dùng miền an toàn hay chỉ chạy trong vùng tự do.</p>
          </div>
        </section>

        <section className="concept-grid">
          <div className="concept"><span>01</span><h3>Predict</h3><p>Φ và Γ biến tương lai của state/output thành hàm tuyến tính của toàn control sequence U.</p></div>
          <div className="concept"><span>02</span><h3>Constrain</h3><p>Input, Δu, state và output cùng nằm trong một polyhedron AU ≤ b.</p></div>
          <div className="concept"><span>03</span><h3>Reproduce</h3><p>Preset cố định scenario, disturbance và envelope để mọi kết quả có thể chạy lại và benchmark.</p></div>
          <div className="concept"><span>04</span><h3>Audit reality</h3><p>Model feasibility và plant safety được đo tách biệt, chuẩn bị cho Safety Governor nếu guidance-only chưa đủ.</p></div>
        </section>
      </main>
    </div>
  );
}
