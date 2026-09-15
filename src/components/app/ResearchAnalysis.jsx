import React, { useMemo } from 'react';
import { Activity, BarChart3, BrainCircuit, CheckCircle2, Clock3, Cpu, Gauge, Radar, ShieldCheck, Sparkles } from 'lucide-react';
import { runAdaptiveModelShadow } from '../../core/adaptation/runAdaptiveModelShadow.js';
import './researchAnalysis.css';

const MODE_ORDER = ['PID', 'MPC', 'HYBRID', 'HYBRID_SAFE'];
const MODE_LABELS = {
  PID: 'PID',
  MPC: 'Periodic MPC',
  HYBRID: 'Event MPC + PID',
  HYBRID_SAFE: 'Event MPC + PID + Safety',
};

const fmt = (value, digits = 2, fallback = '—') => Number.isFinite(value) ? value.toFixed(digits) : fallback;
const sci = (value) => Number.isFinite(value) ? value.toExponential(2) : '—';
const clamp = (value, lo, hi) => Math.max(lo, Math.min(hi, value));

function PageHeader() {
  return <div className="workspace-page-head">
    <div className="workspace-page-icon"><BarChart3 size={22}/></div>
    <div>
      <span>RUN ANALYSIS</span>
      <h1>Research diagnostics and trade-offs</h1>
      <p>Pareto quality-versus-compute, solver latency, trigger causes, estimator uncertainty and adaptive shadow telemetry from the latest executed experiment.</p>
    </div>
  </div>;
}

function ParetoChart({ results }) {
  const width = 640;
  const height = 300;
  const pad = { l: 62, r: 32, t: 28, b: 48 };
  const points = results.map((result) => ({
    mode: result.mode,
    x: result.metrics.solveCount,
    y: result.metrics.iae,
  }));
  const xMax = Math.max(1, ...points.map((point) => point.x)) * 1.08;
  const yValues = points.map((point) => point.y);
  const yMinRaw = Math.min(...yValues);
  const yMaxRaw = Math.max(...yValues);
  const ySpan = Math.max(0.05, yMaxRaw - yMinRaw);
  const yMin = Math.max(0, yMinRaw - ySpan * 0.2);
  const yMax = yMaxRaw + ySpan * 0.2;
  const x = (value) => pad.l + (value / xMax) * (width - pad.l - pad.r);
  const y = (value) => height - pad.b - ((value - yMin) / Math.max(yMax - yMin, 1e-9)) * (height - pad.t - pad.b);
  const xTicks = Array.from({ length: 6 }, (_, index) => xMax * index / 5);
  const yTicks = Array.from({ length: 5 }, (_, index) => yMin + (yMax - yMin) * index / 4);

  return <svg className="research-svg pareto-svg" viewBox={`0 0 ${width} ${height}`} role="img" aria-label="IAE versus MPC solves Pareto chart">
    <g className="research-grid">
      {xTicks.map((tick) => <line key={`x-${tick}`} x1={x(tick)} y1={pad.t} x2={x(tick)} y2={height - pad.b}/>)}
      {yTicks.map((tick) => <line key={`y-${tick}`} x1={pad.l} y1={y(tick)} x2={width - pad.r} y2={y(tick)}/>)}
    </g>
    {points.map((point) => <g key={point.mode} className={`pareto-point pareto-${point.mode.toLowerCase()}`}>
      <circle cx={x(point.x)} cy={y(point.y)} r={point.mode === 'HYBRID_SAFE' ? 7 : 5}/>
      <text x={x(point.x) + 9} y={y(point.y) - 9}>{MODE_LABELS[point.mode]}</text>
    </g>)}
    <g className="research-axis-labels">
      {xTicks.map((tick) => <text key={`xt-${tick}`} x={x(tick)} y={height - 21} textAnchor="middle">{Math.round(tick)}</text>)}
      {yTicks.map((tick) => <text key={`yt-${tick}`} x={pad.l - 10} y={y(tick) + 4} textAnchor="end">{tick.toFixed(2)}</text>)}
      <text x={(pad.l + width - pad.r) / 2} y={height - 3} textAnchor="middle">MPC solves</text>
      <text transform={`translate(15 ${(pad.t + height - pad.b) / 2}) rotate(-90)`} textAnchor="middle">IAE ↓</text>
    </g>
  </svg>;
}

function LatencyHistogram({ records }) {
  const values = records.map((record) => record.solveMs).filter(Number.isFinite);
  if (!values.length) return <div className="analysis-empty">No solver timing records for this controller.</div>;
  const width = 640;
  const height = 250;
  const pad = { l: 52, r: 20, t: 20, b: 44 };
  const min = Math.min(...values);
  const max = Math.max(...values);
  const bins = 10;
  const span = Math.max(max - min, 1e-6);
  const counts = Array.from({ length: bins }, () => 0);
  values.forEach((value) => {
    const index = clamp(Math.floor(((value - min) / span) * bins), 0, bins - 1);
    counts[index] += 1;
  });
  const maxCount = Math.max(1, ...counts);
  const plotW = width - pad.l - pad.r;
  const plotH = height - pad.t - pad.b;
  const binW = plotW / bins;
  const xValue = (index) => min + span * index / bins;
  return <svg className="research-svg latency-svg" viewBox={`0 0 ${width} ${height}`} role="img" aria-label="Solver latency histogram">
    <line x1={pad.l} y1={height - pad.b} x2={width - pad.r} y2={height - pad.b} className="axis-line"/>
    {counts.map((count, index) => {
      const h = (count / maxCount) * plotH;
      return <rect key={index} x={pad.l + index * binW + 2} y={height - pad.b - h} width={Math.max(2, binW - 4)} height={h} className="latency-bar"/>;
    })}
    {Array.from({ length: 6 }, (_, index) => index).map((index) => {
      const px = pad.l + plotW * index / 5;
      const value = min + span * index / 5;
      return <text key={index} x={px} y={height - 18} textAnchor="middle" className="hist-label">{value.toFixed(2)}</text>;
    })}
    <text x={(pad.l + width - pad.r) / 2} y={height - 2} textAnchor="middle" className="hist-label">solve time (ms)</text>
  </svg>;
}

function TriggerBreakdown({ samples }) {
  const names = [
    ['prediction-error', 'Prediction error'],
    ['state-change', 'State change'],
    ['constraint', 'Constraint proximity'],
    ['watchdog', 'Watchdog'],
    ['initial', 'Initial solve'],
  ];
  const counts = Object.fromEntries(names.map(([key]) => [key, 0]));
  samples.filter((sample) => sample.triggered).forEach((sample) => {
    const key = sample.triggerReason || 'initial';
    counts[key] = (counts[key] || 0) + 1;
  });
  const total = Object.values(counts).reduce((sum, value) => sum + value, 0);
  const maxCount = Math.max(1, ...Object.values(counts));
  return <div className="trigger-breakdown">
    {names.map(([key, label]) => <div className="trigger-break-row" key={key}>
      <span>{label}</span>
      <div className="trigger-break-track"><i style={{ width: `${100 * counts[key] / maxCount}%` }}/></div>
      <strong>{counts[key]}</strong>
      <small>{total ? `${(100 * counts[key] / total).toFixed(1)}%` : '0%'}</small>
    </div>)}
  </div>;
}

function CovarianceChart({ samples }) {
  const data = samples.filter((sample) => Number.isFinite(sample.covarianceTrace));
  if (data.length < 2) return <div className="analysis-empty">Enable state estimation to inspect covariance history.</div>;
  const width = 640;
  const height = 190;
  const pad = { l: 52, r: 20, t: 18, b: 36 };
  const tMax = Math.max(...data.map((sample) => sample.t), 1);
  const maxV = Math.max(...data.map((sample) => sample.covarianceTrace), 1e-6) * 1.08;
  const x = (t) => pad.l + (t / tMax) * (width - pad.l - pad.r);
  const y = (value) => height - pad.b - (value / maxV) * (height - pad.t - pad.b);
  const path = data.map((sample, index) => `${index ? 'L' : 'M'} ${x(sample.t).toFixed(2)} ${y(sample.covarianceTrace).toFixed(2)}`).join(' ');
  return <svg className="research-svg covariance-svg" viewBox={`0 0 ${width} ${height}`} role="img" aria-label="Estimator covariance trace">
    <line x1={pad.l} y1={height - pad.b} x2={width - pad.r} y2={height - pad.b} className="axis-line"/>
    <path d={path} className="covariance-path"/>
    <text x={pad.l} y={height - 12} className="hist-label">0 s</text>
    <text x={width - pad.r} y={height - 12} textAnchor="end" className="hist-label">{tMax.toFixed(1)} s</text>
  </svg>;
}

function parameterDelta(nominal, published, key) {
  if (!Number.isFinite(nominal?.[key]) || !Number.isFinite(published?.[key])) return '—';
  return `${((published[key] / nominal[key] - 1) * 100).toFixed(1)}%`;
}

export default function ResearchAnalysis({ results, runCfg }) {
  const safe = results.find((result) => result.mode === 'HYBRID_SAFE') || results[0];
  const periodic = results.find((result) => result.mode === 'MPC') || results[0];
  const best = [...results].sort((a, b) => a.metrics.iae - b.metrics.iae)[0];
  const shadow = useMemo(() => {
    try {
      return runAdaptiveModelShadow(safe);
    } catch {
      return null;
    }
  }, [safe]);
  const triggerCount = safe.samples.filter((sample) => sample.triggered).length;
  const avgLatency = safe.metrics.avgSolveMs;
  const maxLatency = safe.metrics.maxSolveMs;
  const diagnostics = shadow?.diagnostics;
  const validation = diagnostics?.validation;

  return <main className="dashboard-main workspace-page research-analysis-page">
    <PageHeader/>

    <section className="analysis-kpis deep-analysis-kpis">
      <div className="analysis-kpi"><span>Best tracking IAE</span><strong>{fmt(best?.metrics.iae, 4)}</strong><small>{MODE_LABELS[best?.mode] || '—'}</small></div>
      <div className="analysis-kpi"><span>Compute avoided</span><strong>{fmt(safe.metrics.computeReduction, 1)}%</strong><small>{safe.metrics.solveCount} vs {periodic.metrics.solveCount} periodic solves</small></div>
      <div className="analysis-kpi"><span>Solver latency</span><strong>{fmt(avgLatency, 2)} ms</strong><small>max {fmt(maxLatency, 2)} ms</small></div>
      <div className="analysis-kpi"><span>Actual safety</span><strong>{safe.metrics.safetyViolationCount}</strong><small>unsafe samples · max {sci(safe.metrics.maxActualSafetyViolation)}</small></div>
      <div className="analysis-kpi"><span>Shadow publishes</span><strong>{diagnostics?.publishCount ?? 0}</strong><small>no control authority</small></div>
    </section>

    <section className="research-analysis-grid">
      <article className="dashboard-card research-panel pareto-panel">
        <div className="card-head"><div><Gauge size={16}/><strong>Pareto · tracking quality vs compute</strong></div><span className="analysis-chip">lower-left is better</span></div>
        <ParetoChart results={results}/>
        <div className="analysis-note">This chart uses actual IAE and actual MPC solve counts from the current run. It makes the event-trigger trade-off visible instead of comparing controllers on tracking alone.</div>
      </article>

      <article className="dashboard-card research-panel latency-panel">
        <div className="card-head"><div><Clock3 size={16}/><strong>Solver latency distribution</strong></div><span className="analysis-chip">{safe.solverRecords.length} solves</span></div>
        <LatencyHistogram records={safe.solverRecords}/>
        <div className="mini-stat-row"><span>Avg <b>{fmt(avgLatency,2)} ms</b></span><span>Max <b>{fmt(maxLatency,2)} ms</b></span><span>Iterations <b>{fmt(safe.metrics.avgIterations,1)}</b></span><span>Fallback <b>{safe.metrics.fallbackCount}</b></span></div>
      </article>

      <article className="dashboard-card research-panel trigger-panel-analysis">
        <div className="card-head"><div><Radar size={16}/><strong>Event-trigger breakdown</strong></div><span className="analysis-chip">{triggerCount} total</span></div>
        <TriggerBreakdown samples={safe.samples}/>
        <div className="mini-stat-row"><span>Rate <b>{fmt(safe.metrics.triggerRate,2)}/s</b></span><span>Reduction <b>{fmt(safe.metrics.computeReduction,1)}%</b></span><span>Conditioning <b>{fmt(safe.metrics.governorConditioningRate,1)}%</b></span></div>
      </article>

      <article className="dashboard-card research-panel estimator-panel-analysis">
        <div className="card-head"><div><Activity size={16}/><strong>Estimator uncertainty</strong></div><span className={`analysis-chip ${safe.metrics.estimationEnabled ? 'chip-good' : ''}`}>{safe.metrics.estimationEnabled ? 'x̂ active' : 'truth-state run'}</span></div>
        <div className="estimator-metric-grid">
          <div><span>Measurement RMSE</span><strong>{fmt(safe.metrics.measurementRmse,4)}</strong></div>
          <div><span>x̂ RMSE</span><strong>{fmt(safe.metrics.estimatePositionRmse,4)}</strong></div>
          <div><span>v̂ RMSE</span><strong>{fmt(safe.metrics.estimateVelocityRmse,4)}</strong></div>
          <div><span>Innovation RMSE</span><strong>{fmt(safe.metrics.innovationRmse,4)}</strong></div>
        </div>
        <CovarianceChart samples={safe.samples}/>
        <div className="analysis-note">Covariance is controller-facing uncertainty. When tightening is enabled, it shrinks the predicted safety envelope; the actual plant audit remains evaluated against the original physical envelope.</div>
      </article>

      <article className="dashboard-card research-panel solver-panel-analysis">
        <div className="card-head"><div><Cpu size={16}/><strong>Numerical health</strong></div><span className={`analysis-chip ${(safe.metrics.convergenceRate ?? 100) >= 99.9 ? 'chip-good' : 'chip-warn'}`}><CheckCircle2 size={11}/>{fmt(safe.metrics.convergenceRate ?? 100,1)}%</span></div>
        <div className="numerical-health-grid">
          <div><span>Stationarity avg</span><strong>{sci(safe.metrics.avgStationarityResidual)}</strong></div>
          <div><span>Stationarity max</span><strong>{sci(safe.metrics.maxStationarityResidual)}</strong></div>
          <div><span>Feasibility max</span><strong>{sci(safe.metrics.maxFeasibilityViolation)}</strong></div>
          <div><span>Active constraints</span><strong>{fmt((safe.metrics.avgActiveConstraintRatio || 0) * 100,1)}%</strong></div>
          <div><span>Infeasible</span><strong>{safe.metrics.infeasibleCount}</strong></div>
          <div><span>Timeout</span><strong>{safe.metrics.timeoutCount}</strong></div>
        </div>
      </article>

      <article className="dashboard-card research-panel adaptive-panel-analysis">
        <div className="card-head"><div><BrainCircuit size={16}/><strong>Adaptive Shadow Model</strong></div><span className="analysis-chip shadow-chip"><Sparkles size={11}/>shadow-only</span></div>
        {shadow ? <>
          <div className="shadow-summary-grid">
            <div><span>Accepted windows</span><strong>{diagnostics.acceptedWindows}</strong></div>
            <div><span>Holdout windows</span><strong>{diagnostics.validationWindows}</strong></div>
            <div><span>PE rejected</span><strong>{diagnostics.rejectedLowInformation}</strong></div>
            <div><span>Validation rejected</span><strong>{diagnostics.rejectedValidation}</strong></div>
            <div><span>Publishes</span><strong>{diagnostics.publishCount}</strong></div>
            <div><span>PE information</span><strong>{sci(diagnostics.information?.normalizedDeterminant)}</strong></div>
          </div>
          <div className="shadow-model-table">
            <div className="shadow-row shadow-head"><span>Parameter</span><span>Nominal</span><span>Candidate</span><span>Published</span><span>Published Δ</span></div>
            {['stiffness','damping','gain'].map((key) => <div className="shadow-row" key={key}><strong>{key}</strong><span>{fmt(shadow.nominalModel[key],4)}</span><span>{fmt(shadow.candidate[key],4)}</span><span>{fmt(shadow.publishedModel[key],4)}</span><span>{parameterDelta(shadow.nominalModel, shadow.publishedModel, key)}</span></div>)}
          </div>
          <div className="validation-strip"><span>Validation samples <b>{validation?.samples ?? 0}</b></span><span>Candidate RMSE <b>{fmt(validation?.candidateRmse,4)}</b></span><span>Published RMSE <b>{fmt(validation?.publishedRmse,4)}</b></span><span className={validation?.passed ? 'validation-pass' : ''}>Gate <b>{validation?.passed ? 'PASS' : 'WAIT'}</b></span></div>
          <div className="analysis-note"><ShieldCheck size={13}/>The learned model is replayed from the governed trajectory and remains isolated from MPC. This panel is telemetry only; it does not grant adaptation control authority.</div>
        </> : <div className="analysis-empty">Shadow replay unavailable for this run.</div>}
      </article>
    </section>
  </main>;
}
