import React, { useMemo, useState } from 'react';
import {
  Activity,
  BarChart3,
  BookOpen,
  CheckCircle2,
  ChevronDown,
  CircleDot,
  Cpu,
  Database,
  Gauge,
  Play,
  RotateCcw,
  Settings,
  ShieldCheck,
  SlidersHorizontal,
  Sparkles,
  Zap,
} from 'lucide-react';
import ExperimentSafetyPanel from './components/ExperimentSafetyPanel.jsx';
import { compareControllers, defaultConfig } from './core/simulator.js';
import { SOLVER_BACKENDS } from './core/solvers/index.js';
import './solverDiagnostics.css';

const MODES = ['PID', 'MPC', 'HYBRID', 'HYBRID_SAFE'];
const MODE_LABELS = {
  PID: 'PID Only',
  MPC: 'MPC (Periodic)',
  HYBRID: 'MPC + PID (Event)',
  HYBRID_SAFE: 'MPC + PID + Safety',
};

const MODE_SHORT = {
  PID: 'PID',
  MPC: 'MPC',
  HYBRID: 'MPC + PID',
  HYBRID_SAFE: 'MPC + PID + Safety',
};

const navItems = [
  { id: 'simulation', label: 'Simulation', icon: Play },
  { id: 'analysis', label: 'Analysis', icon: BarChart3 },
  { id: 'scenarios', label: 'Scenarios', icon: Database },
  { id: 'documentation', label: 'Documentation', icon: BookOpen },
  { id: 'settings', label: 'Settings', icon: Settings },
];

function deepCloneConfig(value) {
  return JSON.parse(JSON.stringify(value));
}

function NumberField({ label, value, step = 0.01, onChange }) {
  return (
    <label className="field app-field">
      <span>{label}</span>
      <input type="number" value={value} step={step} onChange={(e) => onChange(Number(e.target.value))} />
    </label>
  );
}

function SelectField({ label, value, onChange, options }) {
  return (
    <label className="select-field app-field">
      <span>{label}</span>
      <select value={value} onChange={(e) => onChange(e.target.value)}>
        {options.map((option) => <option key={option.value} value={option.value}>{option.label}</option>)}
      </select>
    </label>
  );
}

function ToggleField({ label, checked, onChange }) {
  return (
    <label className="toggle-field app-field">
      <span>{label}</span>
      <input type="checkbox" checked={checked} onChange={(e) => onChange(e.target.checked)} />
    </label>
  );
}

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function TimeSeriesChart({ results, valueKey, target, yLabel, compact = false }) {
  const width = 980;
  const height = compact ? 205 : 315;
  const pad = { l: 56, r: 18, t: 22, b: 38 };
  const allValues = results.flatMap((result) => result.samples.map((sample) => Number(sample[valueKey]) || 0));
  if (valueKey === 'x') allValues.push(target);
  const rawMin = Math.min(...allValues, -0.05);
  const rawMax = Math.max(...allValues, 0.05);
  const span = Math.max(rawMax - rawMin, 0.25);
  const yMin = rawMin - span * 0.12;
  const yMax = rawMax + span * 0.12;
  const tMax = Math.max(...results.flatMap((result) => result.samples.map((sample) => sample.t)), 1);
  const plotW = width - pad.l - pad.r;
  const plotH = height - pad.t - pad.b;
  const x = (t) => pad.l + (t / tMax) * plotW;
  const y = (v) => pad.t + (1 - (v - yMin) / (yMax - yMin)) * plotH;
  const path = (samples) => samples
    .map((sample, index) => `${index ? 'L' : 'M'} ${x(sample.t).toFixed(2)} ${y(Number(sample[valueKey]) || 0).toFixed(2)}`)
    .join(' ');
  const xTicks = Array.from({ length: 6 }, (_, i) => (tMax * i) / 5);
  const yTicks = Array.from({ length: 5 }, (_, i) => yMin + ((yMax - yMin) * i) / 4);

  return (
    <svg className={`timeseries-chart ${compact ? 'compact' : ''}`} viewBox={`0 0 ${width} ${height}`} role="img" aria-label={`${yLabel} chart`}>
      <g className="chart-grid">
        {xTicks.map((tick) => <line key={`x-${tick}`} x1={x(tick)} y1={pad.t} x2={x(tick)} y2={height - pad.b} />)}
        {yTicks.map((tick) => <line key={`y-${tick}`} x1={pad.l} y1={y(tick)} x2={width - pad.r} y2={y(tick)} />)}
      </g>
      {valueKey === 'x' && <line x1={pad.l} y1={y(target)} x2={width - pad.r} y2={y(target)} className="target-line" />}
      {results.map((result) => <path key={result.mode} d={path(result.samples)} className={`curve curve-${result.mode.toLowerCase()}`} />)}
      <g className="chart-labels">
        {xTicks.map((tick) => <text key={`xt-${tick}`} x={x(tick)} y={height - 13} textAnchor="middle">{tick.toFixed(tMax > 20 ? 0 : 1)}</text>)}
        {yTicks.map((tick) => <text key={`yt-${tick}`} x={pad.l - 10} y={y(tick) + 4} textAnchor="end">{tick.toFixed(1)}</text>)}
        <text x={pad.l + plotW / 2} y={height - 1} textAnchor="middle">Time (s)</text>
        <text transform={`translate(13 ${pad.t + plotH / 2}) rotate(-90)`} textAnchor="middle">{yLabel}</text>
      </g>
    </svg>
  );
}

function TriggerTimeline({ samples, duration }) {
  const width = 980;
  const height = 150;
  const pad = { l: 92, r: 18, t: 16, b: 28 };
  const rows = [
    ['prediction-error', 'Prediction'],
    ['state-change', 'State Error'],
    ['watchdog', 'Timeout'],
    ['constraint', 'Safety'],
  ];
  const x = (t) => pad.l + (t / Math.max(duration, 1e-6)) * (width - pad.l - pad.r);
  const rowY = (index) => 30 + index * 24;
  const events = samples.filter((sample) => sample.triggered || sample.governorSafetyIntervened);

  return (
    <svg className="trigger-timeline" viewBox={`0 0 ${width} ${height}`} role="img" aria-label="Trigger events timeline">
      {rows.map(([reason, label], index) => (
        <g key={reason}>
          <text x="8" y={rowY(index) + 4} className="timeline-row-label">{label}</text>
          <line x1={pad.l} y1={rowY(index)} x2={width - pad.r} y2={rowY(index)} className="timeline-row" />
        </g>
      ))}
      {events.map((sample, index) => {
        const reason = sample.governorSafetyIntervened
          ? 'constraint'
          : sample.triggerReason === 'initial' ? 'prediction-error' : sample.triggerReason;
        const rowIndex = Math.max(0, rows.findIndex(([key]) => key === reason));
        return <line key={`${sample.t}-${index}`} x1={x(sample.t)} y1={rowY(rowIndex) - 7} x2={x(sample.t)} y2={rowY(rowIndex) + 7} className={`event-mark event-${reason}`} />;
      })}
      {Array.from({ length: 6 }, (_, i) => (duration * i) / 5).map((tick) => (
        <g key={tick}>
          <text x={x(tick)} y={height - 8} textAnchor="middle" className="timeline-tick">{tick.toFixed(duration > 20 ? 0 : 1)}</text>
          <line x1={x(tick)} y1="18" x2={x(tick)} y2={height - 25} className="timeline-gridline" />
        </g>
      ))}
    </svg>
  );
}

function Metric({ label, value, accent = false }) {
  return <div className="health-row"><span>{label}</span><strong className={accent ? 'metric-good' : ''}>{value}</strong></div>;
}

function fmt(value, digits = 2, fallback = '—') {
  return Number.isFinite(value) ? value.toFixed(digits) : fallback;
}

function scientific(value) {
  if (!Number.isFinite(value)) return '—';
  if (value === 0) return '0.0e+0';
  return value.toExponential(1);
}

function statusLabel(result) {
  if (!result?.metrics) return 'Idle';
  if (result.metrics.fallbackCount > 0) return 'Fallback';
  if ((result.metrics.convergenceRate ?? 100) < 99.9) return 'Degraded';
  return 'Ready';
}

export default function App() {
  const [draftCfg, setDraftCfg] = useState(() => deepCloneConfig(defaultConfig));
  const [runCfg, setRunCfg] = useState(() => deepCloneConfig(defaultConfig));
  const [activeMode, setActiveMode] = useState('HYBRID_SAFE');
  const [activeNav, setActiveNav] = useState('simulation');
  const [presetId, setPresetId] = useState('baseline');
  const [leftTab, setLeftTab] = useState('pid');
  const results = useMemo(() => compareControllers(runCfg), [runCfg]);
  const active = results.find((result) => result.mode === activeMode) || results[0];
  const governed = results.find((result) => result.mode === 'HYBRID_SAFE') || active;
  const update = (group, key, value) => setDraftCfg((current) => group
    ? ({ ...current, [group]: { ...current[group], [key]: value } })
    : ({ ...current, [key]: value }));
  const runSimulation = () => setRunCfg(deepCloneConfig(draftCfg));
  const resetSimulation = () => {
    const reset = deepCloneConfig(defaultConfig);
    setDraftCfg(reset);
    setRunCfg(reset);
    setPresetId('baseline');
  };
  const solverLabel = runCfg.mpc.solver === SOLVER_BACKENDS.CONSTRAINED_QP
    ? 'Constrained QP'
    : runCfg.mpc.solver === SOLVER_BACKENDS.BOX_QP ? 'Box QP' : 'Projected Gradient';
  const recentEvents = [...active.samples]
    .filter((sample) => sample.triggered || sample.governorSafetyIntervened)
    .slice(-5)
    .reverse();

  return (
    <div className="control-app">
      <header className="app-topbar">
        <div className="app-brand">
          <div className="app-logo"><Cpu size={24} /></div>
          <div><strong>MPC-PID Control System</strong><span>Research • Simulation • Visualization</span></div>
        </div>
        <nav className="top-nav" aria-label="Main navigation">
          {navItems.map(({ id, label, icon: Icon }) => (
            <button key={id} className={activeNav === id ? 'active' : ''} onClick={() => setActiveNav(id)}>
              <Icon size={16} />{label}
            </button>
          ))}
        </nav>
        <div className="app-status">
          <span className={`status-pill status-${statusLabel(governed).toLowerCase()}`}><CircleDot size={11} />{statusLabel(governed)}</span>
          <span className="repo-mark"><Cpu size={19} />MPC_PID_System</span>
        </div>
      </header>

      <div className="app-layout">
        <aside className="control-sidebar">
          <section className="sidebar-section control-config">
            <div className="sidebar-heading"><SlidersHorizontal size={17} /><strong>Control Configuration</strong><ChevronDown size={15} /></div>
            <SelectField
              label="Control Mode"
              value={activeMode}
              onChange={setActiveMode}
              options={MODES.map((mode) => ({ value: mode, label: MODE_LABELS[mode] }))}
            />
            <small className="field-hint">Event-triggered MPC with PID tracking and optional Safety Governor.</small>
            <div className="config-tabs">
              {['pid', 'mpc', 'trigger', 'safety'].map((tab) => <button key={tab} className={leftTab === tab ? 'active' : ''} onClick={() => setLeftTab(tab)}>{tab.toUpperCase()}</button>)}
            </div>

            {leftTab === 'pid' && <div className="tab-content">
              <NumberField label="Kp" value={draftCfg.pid.kp} step={0.1} onChange={(v) => update('pid', 'kp', v)} />
              <NumberField label="Ki" value={draftCfg.pid.ki} step={0.05} onChange={(v) => update('pid', 'ki', v)} />
              <NumberField label="Kd" value={draftCfg.pid.kd} step={0.02} onChange={(v) => update('pid', 'kd', v)} />
              <NumberField label="Anti-windup" value={draftCfg.pid.antiWindup} step={0.1} onChange={(v) => update('pid', 'antiWindup', v)} />
            </div>}

            {leftTab === 'mpc' && <div className="tab-content">
              <SelectField label="Solver" value={draftCfg.mpc.solver} onChange={(v) => update('mpc', 'solver', v)} options={[
                { value: SOLVER_BACKENDS.CONSTRAINED_QP, label: 'Constrained QP' },
                { value: SOLVER_BACKENDS.BOX_QP, label: 'Box QP' },
                { value: SOLVER_BACKENDS.PROJECTED_GRADIENT, label: 'Projected Gradient' },
              ]} />
              <NumberField label="Horizon" value={draftCfg.mpc.horizon} step={1} onChange={(v) => update('mpc', 'horizon', Math.max(3, Math.round(v)))} />
              <NumberField label="Q position" value={draftCfg.mpc.qPosition} step={0.5} onChange={(v) => update('mpc', 'qPosition', Math.max(0, v))} />
              <NumberField label="R input" value={draftCfg.mpc.rInput} step={0.01} onChange={(v) => update('mpc', 'rInput', Math.max(0, v))} />
            </div>}

            {leftTab === 'trigger' && <div className="tab-content">
              <NumberField label="Prediction error" value={draftCfg.trigger.predictionError} step={0.005} onChange={(v) => update('trigger', 'predictionError', Math.max(0, v))} />
              <NumberField label="State change" value={draftCfg.trigger.stateChange} step={0.01} onChange={(v) => update('trigger', 'stateChange', Math.max(0, v))} />
              <NumberField label="T min (s)" value={draftCfg.trigger.minInterval} step={0.02} onChange={(v) => update('trigger', 'minInterval', Math.max(0, v))} />
              <NumberField label="Watchdog (s)" value={draftCfg.trigger.maxInterval} step={0.02} onChange={(v) => update('trigger', 'maxInterval', Math.max(draftCfg.trigger.minInterval, v))} />
            </div>}

            {leftTab === 'safety' && <div className="tab-content">
              <ToggleField label="State limits" checked={draftCfg.mpc.stateConstraintsEnabled} onChange={(v) => update('mpc', 'stateConstraintsEnabled', v)} />
              <ToggleField label="Output limits" checked={draftCfg.mpc.outputConstraintsEnabled} onChange={(v) => update('mpc', 'outputConstraintsEnabled', v)} />
              <NumberField label="Preview horizon" value={draftCfg.safety.previewHorizon} step={1} onChange={(v) => update('safety', 'previewHorizon', Math.max(1, Math.round(v)))} />
              <NumberField label="kσ tightening" value={draftCfg.estimation.constraintSigma} step={0.25} onChange={(v) => update('estimation', 'constraintSigma', Math.max(0, v))} />
            </div>}
          </section>

          <section className="sidebar-section sim-settings">
            <div className="sidebar-heading"><Gauge size={17} /><strong>Simulation Settings</strong></div>
            <NumberField label="Simulation Time (s)" value={draftCfg.duration} step={1} onChange={(v) => update(null, 'duration', Math.max(1, v))} />
            <NumberField label="Sample Time (s)" value={draftCfg.dt} step={0.005} onChange={(v) => update(null, 'dt', clamp(v, 0.005, 0.2))} />
            <NumberField label="Reference Value" value={draftCfg.setpoint} step={0.1} onChange={(v) => update(null, 'setpoint', v)} />
            <ToggleField label="Disturbance" checked={draftCfg.disturbance.enabled} onChange={(v) => update('disturbance', 'enabled', v)} />
            <button className="run-button" onClick={runSimulation}><Play size={16} fill="currentColor" />Run Simulation</button>
            <button className="reset-button" onClick={resetSimulation}><RotateCcw size={15} />Reset</button>
          </section>

          <section className="sidebar-section presets-section">
            <ExperimentSafetyPanel cfg={draftCfg} setCfg={setDraftCfg} presetId={presetId} setPresetId={setPresetId} />
          </section>
        </aside>

        <main className="dashboard-main">
          <section className="stat-strip">
            <div className="stat-card"><span className="stat-icon blue"><Gauge size={18} /></span><div><small>Simulation Time</small><strong>{fmt(runCfg.duration, 2)} s</strong><span>{Math.round(runCfg.duration / runCfg.dt)} steps</span></div></div>
            <div className="stat-card"><span className="stat-icon purple"><Cpu size={18} /></span><div><small>MPC Solves</small><strong>{governed.metrics.solveCount}</strong><span>{fmt(100 - governed.metrics.computeReduction, 1)}% of steps</span></div></div>
            <div className="stat-card"><span className="stat-icon amber"><Zap size={18} /></span><div><small>Trigger Rate</small><strong>{fmt(governed.metrics.triggerRate, 1)}/s</strong><span>{fmt(governed.metrics.computeReduction, 1)}% compute avoided</span></div></div>
            <div className="stat-card"><span className="stat-icon green"><ShieldCheck size={18} /></span><div><small>Plant Safety</small><strong>{governed.metrics.safetyViolationCount}</strong><span>violations</span></div></div>
            <div className="stat-card"><span className="stat-icon green"><CheckCircle2 size={18} /></span><div><small>Solver Success</small><strong>{fmt(governed.metrics.convergenceRate ?? 100, 0)}%</strong><span>{governed.metrics.fallbackCount} fallback</span></div></div>
          </section>

          <div className="dashboard-grid">
            <div className="center-column">
              <section className="dashboard-card response-card">
                <div className="card-head"><div><Activity size={16} /><strong>System Response</strong></div><span className="chart-selector">Position (x)<ChevronDown size={14} /></span></div>
                <div className="chart-legend">
                  <span className="ref-line">Reference</span>
                  {MODES.map((mode) => <span key={mode} className={`legend-${mode.toLowerCase()}`}>{MODE_LABELS[mode]}</span>)}
                </div>
                <TimeSeriesChart results={results} valueKey="x" target={runCfg.setpoint} yLabel="Position" />
              </section>

              <section className="dashboard-card control-chart-card">
                <div className="card-head"><div><Zap size={16} /><strong>Control Input</strong></div><span className="subtle-badge">u(t)</span></div>
                <div className="chart-legend compact-legend">{MODES.map((mode) => <span key={mode} className={`legend-${mode.toLowerCase()}`}>{MODE_LABELS[mode]}</span>)}</div>
                <TimeSeriesChart results={results} valueKey="u" target={runCfg.setpoint} yLabel="Control Input (u)" compact />
              </section>

              <section className="dashboard-card trigger-card">
                <div className="card-head"><div><Sparkles size={16} /><strong>Trigger Events</strong></div><span className="subtle-badge">{MODE_SHORT[activeMode]}</span></div>
                <div className="event-legend"><span className="dot-red">Prediction Error</span><span className="dot-blue">State Error</span><span className="dot-yellow">Timeout</span><span className="dot-purple">Safety Intervention</span></div>
                <TriggerTimeline samples={active.samples} duration={runCfg.duration} />
              </section>
            </div>

            <aside className="insight-column">
              <section className="dashboard-card metrics-card">
                <div className="card-head"><div><BarChart3 size={16} /><strong>Performance Metrics</strong></div></div>
                <div className="metric-table">
                  <div className="metric-row metric-header"><span>Metric</span>{MODES.map((mode) => <span key={mode}>{MODE_SHORT[mode]}</span>)}</div>
                  <div className="metric-row"><span>IAE</span>{results.map((r) => <span key={r.mode}>{fmt(r.metrics.iae, 4)}</span>)}</div>
                  <div className="metric-row"><span>Overshoot</span>{results.map((r) => <span key={r.mode}>{fmt(r.metrics.overshoot, 1)}%</span>)}</div>
                  <div className="metric-row"><span>Settling Time</span>{results.map((r) => <span key={r.mode}>{Number.isFinite(r.metrics.settling) ? `${fmt(r.metrics.settling, 2)} s` : '—'}</span>)}</div>
                  <div className="metric-row"><span>Control Effort</span>{results.map((r) => <span key={r.mode}>{fmt(r.metrics.controlEffort, 2)}</span>)}</div>
                  <div className="metric-row"><span>MPC Solves</span>{results.map((r) => <span key={r.mode}>{r.metrics.solveCount}</span>)}</div>
                  <div className="metric-row metric-highlight"><span>Solve Reduction</span>{results.map((r) => <span key={r.mode}>{r.mode === 'PID' ? '—' : `${fmt(r.metrics.computeReduction, 1)}%`}</span>)}</div>
                </div>
              </section>

              <section className="dashboard-card health-card">
                <div className="card-head"><div><Cpu size={16} /><strong>Solver Health</strong></div><span className="optimal-pill"><CheckCircle2 size={12} />Optimal</span></div>
                <div className="health-grid">
                  <div>
                    <Metric label="Backend" value={solverLabel} />
                    <Metric label="Convergence" value={`${fmt(governed.metrics.convergenceRate ?? 100, 1)}%`} accent />
                    <Metric label="Avg iterations" value={fmt(governed.metrics.avgIterations, 1)} />
                    <Metric label="Avg solve" value={`${fmt(governed.metrics.avgSolveMs, 2)} ms`} />
                    <Metric label="Stationarity" value={scientific(governed.metrics.avgStationarityResidual)} />
                  </div>
                  <div>
                    <Metric label="Feasibility" value={scientific(governed.metrics.maxFeasibilityViolation)} />
                    <Metric label="Active ratio" value={`${fmt(governed.metrics.avgActiveConstraintRatio * 100, 1)}%`} />
                    <Metric label="Fallback" value={governed.metrics.fallbackCount} />
                    <Metric label="Timeout" value={governed.metrics.timeoutCount} />
                    <Metric label="Infeasible" value={governed.metrics.infeasibleCount} />
                  </div>
                </div>
              </section>

              <section className="dashboard-card governor-card">
                <div className="card-head"><div><ShieldCheck size={16} /><strong>Safety Governor</strong></div></div>
                <Metric label="Intervention Rate" value={`${fmt(governed.metrics.governorInterventionRate, 1)}%`} />
                <Metric label="Max |u_safe - u_pid|" value={fmt(governed.metrics.governorMaxCorrection, 3)} />
                <Metric label="Empty Admissible Interval" value={governed.metrics.governorInfeasibleCount} />
                <Metric label="Emergency Fallback" value={governed.metrics.governorEmergencyCount} />
                <Metric label="Plant Safety Violations" value={governed.metrics.safetyViolationCount} accent />
                <Metric label="Conditioning Only" value={`${governed.metrics.governorConditioningCount} (${fmt(governed.metrics.governorConditioningRate, 1)}%)`} />
              </section>

              <section className="dashboard-card events-card">
                <div className="card-head"><div><BookOpen size={16} /><strong>Recent Events</strong></div><button className="clear-button">Clear</button></div>
                <div className="recent-events">
                  {recentEvents.length === 0 && <div className="empty-event">No trigger events in this run.</div>}
                  {recentEvents.map((sample, index) => (
                    <div className="recent-event" key={`${sample.t}-${index}`}>
                      <span className={`event-dot ${sample.governorSafetyIntervened ? 'safety' : sample.triggerReason || 'state-change'}`} />
                      <time>{fmt(sample.t, 2)}s</time>
                      <span>{sample.governorSafetyIntervened ? 'Safety intervention' : `${sample.triggerReason || 'state'} trigger`}</span>
                    </div>
                  ))}
                </div>
              </section>
            </aside>
          </div>
        </main>
      </div>

      <footer className="app-footer"><span>MPC_PID_System</span><span>Advanced Control Research Platform</span><span>Research • Simulation • Safety</span></footer>
    </div>
  );
}
