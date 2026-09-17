import React, { useMemo } from 'react';
import { Activity, AlertTriangle, CheckCircle2, Cpu, Gauge, ShieldCheck, Zap } from 'lucide-react';
import { compareExperimentCases } from '../../core/experiments/experimentMatrix.js';
import './caseExplorer.css';

const fmt = (value, digits = 3) => Number.isFinite(value) ? value.toFixed(digits) : '—';
const sci = (value) => Number.isFinite(value) ? value.toExponential(1) : '—';

function TraceChart({ trace, valueKey, secondaryKey, yLabel, reference = false }) {
  if (!trace?.length) return <div className="case-empty">No trajectory stored for this case.</div>;
  const width = 900;
  const height = 260;
  const pad = { l: 58, r: 20, t: 18, b: 38 };
  const values = trace.flatMap((sample) => {
    const out = [Number(sample[valueKey]) || 0];
    if (secondaryKey && Number.isFinite(sample[secondaryKey])) out.push(sample[secondaryKey]);
    if (reference && Number.isFinite(sample.reference)) out.push(sample.reference);
    return out;
  });
  const rawMin = Math.min(...values, -0.05);
  const rawMax = Math.max(...values, 0.05);
  const span = Math.max(0.2, rawMax - rawMin);
  const yMin = rawMin - span * 0.12;
  const yMax = rawMax + span * 0.12;
  const tMax = Math.max(...trace.map((sample) => sample.t), 1);
  const x = (t) => pad.l + t / tMax * (width - pad.l - pad.r);
  const y = (value) => height - pad.b - (value - yMin) / Math.max(1e-9, yMax - yMin) * (height - pad.t - pad.b);
  const makePath = (key) => trace.filter((sample) => Number.isFinite(sample[key])).map((sample, index) => `${index ? 'L' : 'M'} ${x(sample.t).toFixed(2)} ${y(sample[key]).toFixed(2)}`).join(' ');
  const refPath = reference ? trace.filter((sample) => Number.isFinite(sample.reference)).map((sample, index) => `${index ? 'L' : 'M'} ${x(sample.t).toFixed(2)} ${y(sample.reference).toFixed(2)}`).join(' ') : '';
  return <svg className="case-trace-chart" viewBox={`0 0 ${width} ${height}`} role="img" aria-label={`${yLabel} trajectory`}>
    {[0,1,2,3,4,5].map((i) => { const t = tMax * i / 5; return <g key={`x-${i}`}><line x1={x(t)} y1={pad.t} x2={x(t)} y2={height-pad.b}/><text x={x(t)} y={height-13} textAnchor="middle">{t.toFixed(1)}</text></g>; })}
    {[0,1,2,3,4].map((i) => { const value = yMin + (yMax-yMin)*i/4; return <g key={`y-${i}`}><line x1={pad.l} y1={y(value)} x2={width-pad.r} y2={y(value)}/><text x={pad.l-9} y={y(value)+4} textAnchor="end">{value.toFixed(2)}</text></g>; })}
    {reference && <path d={refPath} className="case-line case-reference"/>}
    <path d={makePath(valueKey)} className="case-line case-primary"/>
    {secondaryKey && <path d={makePath(secondaryKey)} className="case-line case-secondary"/>}
    <text x={(pad.l+width-pad.r)/2} y={height-1} textAnchor="middle" className="case-axis-label">Time (s)</text>
    <text transform={`translate(14 ${(pad.t+height-pad.b)/2}) rotate(-90)`} textAnchor="middle" className="case-axis-label">{yLabel}</text>
  </svg>;
}

function TriggerTimeline({ trace }) {
  const events = trace.filter((sample) => sample.triggered || sample.governorSafetyIntervened || sample.fallbackUsed || sample.safetyViolation > 0);
  const width = 900;
  const height = 116;
  const pad = { l: 92, r: 18 };
  const tMax = Math.max(...trace.map((sample) => sample.t), 1);
  const x = (t) => pad.l + t / tMax * (width - pad.l - pad.r);
  const rows = [
    ['trigger', 24],
    ['governor', 48],
    ['fallback', 72],
    ['unsafe', 96],
  ];
  const yFor = (sample) => sample.safetyViolation > 0 ? 96 : sample.fallbackUsed ? 72 : sample.governorSafetyIntervened ? 48 : 24;
  return <svg className="case-event-timeline" viewBox={`0 0 ${width} ${height}`} role="img" aria-label="Case event timeline">
    {rows.map(([label, y]) => <g key={label}><text x="8" y={y+4}>{label}</text><line x1={pad.l} y1={y} x2={width-pad.r} y2={y}/></g>)}
    {events.map((sample, index) => <line key={`${sample.t}-${index}`} x1={x(sample.t)} y1={yFor(sample)-7} x2={x(sample.t)} y2={yFor(sample)+7} className={`case-event-mark ${sample.safetyViolation>0?'unsafe':sample.fallbackUsed?'fallback':sample.governorSafetyIntervened?'governor':'trigger'}`}/>)}
  </svg>;
}

function statusCounts(records) {
  return records.reduce((acc, record) => {
    const key = record.status || 'unknown';
    acc[key] = (acc[key] || 0) + 1;
    return acc;
  }, {});
}

export default function CaseExplorer({ batchResult, presetId, selectedCaseId, onSelectCase }) {
  const cases = useMemo(() => batchResult.cases.filter((item) => item.presetId === presetId), [batchResult, presetId]);
  const ordered = useMemo(() => [...cases].sort(compareExperimentCases), [cases]);
  const selected = cases.find((item) => item.id === selectedCaseId) || ordered[ordered.length - 1] || ordered[0];
  const best = ordered[0];
  const worst = ordered[ordered.length - 1];
  const solverCounts = statusCounts(selected?.solverRecords || []);
  const fallbackRecords = (selected?.solverRecords || []).filter((record) => record.fallbackUsed);
  const maxSolve = Math.max(0, ...(selected?.solverRecords || []).map((record) => Number.isFinite(record.solveMs) ? record.solveMs : 0));

  if (!selected) return null;

  return <section className="dashboard-card case-explorer">
    <div className="case-explorer-head">
      <div><span>CASE EXPLORER</span><h2>{selected.presetLabel}</h2><p>Inspect a stored batch trajectory without re-running the simulation.</p></div>
      <div className="case-head-actions">
        <button className={selected.id === best?.id ? 'active' : ''} onClick={() => onSelectCase(best.id)}><CheckCircle2 size={14}/>Best case</button>
        <button className={selected.id === worst?.id ? 'active danger' : ''} onClick={() => onSelectCase(worst.id)}><AlertTriangle size={14}/>Worst case</button>
      </div>
    </div>

    <div className="case-picker-row">
      <label><span>Case</span><select value={selected.id} onChange={(event) => onSelectCase(event.target.value)}>{ordered.map((item) => <option key={item.id} value={item.id}>seed {item.seed} · noise ×{item.noiseScale} · mismatch ×{item.mismatchScale}</option>)}</select></label>
      <div><span>Seed</span><strong>{selected.seed}</strong></div>
      <div><span>Noise</span><strong>×{selected.noiseScale}</strong></div>
      <div><span>Mismatch</span><strong>×{selected.mismatchScale}</strong></div>
    </div>

    <div className="case-kpis">
      <div><Gauge size={16}/><span>IAE</span><strong>{fmt(selected.metrics.iae,4)}</strong></div>
      <div><Cpu size={16}/><span>MPC solves</span><strong>{selected.metrics.solveCount}</strong></div>
      <div><CheckCircle2 size={16}/><span>Convergence</span><strong>{fmt(selected.metrics.convergenceRate ?? 100,1)}%</strong></div>
      <div className={selected.metrics.safetyViolationCount ? 'case-kpi-warn' : ''}><ShieldCheck size={16}/><span>Unsafe samples</span><strong>{selected.metrics.safetyViolationCount}</strong></div>
      <div className={selected.metrics.fallbackCount ? 'case-kpi-warn' : ''}><AlertTriangle size={16}/><span>Fallback</span><strong>{selected.metrics.fallbackCount}</strong></div>
      <div><Zap size={16}/><span>Compute avoided</span><strong>{fmt(selected.metrics.computeReduction,1)}%</strong></div>
    </div>

    <div className="case-explorer-grid">
      <article className="case-subpanel case-trajectory-panel">
        <div className="case-subhead"><Activity size={15}/><strong>Position trajectory</strong><span><i className="legend-ref"/>reference <i className="legend-x"/>plant x <i className="legend-est"/>controller x̂</span></div>
        <TraceChart trace={selected.trace} valueKey="x" secondaryKey="controllerX" yLabel="Position" reference/>
      </article>
      <article className="case-subpanel case-control-panel">
        <div className="case-subhead"><Zap size={15}/><strong>Control input</strong><span>u(t)</span></div>
        <TraceChart trace={selected.trace} valueKey="u" yLabel="Control input"/>
      </article>
      <article className="case-subpanel case-events-panel">
        <div className="case-subhead"><AlertTriangle size={15}/><strong>Trigger / safety timeline</strong><span>stored event trace</span></div>
        <TriggerTimeline trace={selected.trace}/>
      </article>
      <article className="case-subpanel case-solver-panel">
        <div className="case-subhead"><Cpu size={15}/><strong>Solver audit</strong><span>{selected.solverRecords.length} records</span></div>
        <div className="case-solver-stats">
          <div><span>Max solve</span><strong>{fmt(maxSolve,2)} ms</strong></div>
          <div><span>Feasibility max</span><strong>{sci(selected.metrics.maxFeasibilityViolation)}</strong></div>
          <div><span>Stationarity max</span><strong>{sci(selected.metrics.maxStationarityResidual)}</strong></div>
          <div><span>Active constraints</span><strong>{fmt((selected.metrics.avgActiveConstraintRatio || 0)*100,1)}%</strong></div>
        </div>
        <div className="solver-status-chips">{Object.entries(solverCounts).map(([status,count]) => <span key={status} className={status === 'solved' ? 'good' : status === 'max-iterations' ? 'warn' : 'bad'}>{status}: {count}</span>)}</div>
        {fallbackRecords.length > 0 && <div className="fallback-audit"><strong>Fallback reasons</strong>{fallbackRecords.slice(0,5).map((record,index) => <span key={index}>{record.status || 'unknown'} · {record.fallbackReason || 'unspecified'}</span>)}</div>}
      </article>
    </div>
  </section>;
}
