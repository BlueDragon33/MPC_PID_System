import React from 'react';
import { BarChart3, ShieldCheck } from 'lucide-react';

const fmt = (value, digits = 2) => Number.isFinite(value) ? value.toFixed(digits) : '—';

function RobustnessPareto({ groups }) {
  const width = 760;
  const height = 280;
  const pad = { l: 60, r: 30, t: 28, b: 46 };
  const xValues = groups.map((group) => group.meanSolveCount || 0);
  const yValues = groups.map((group) => group.meanIae || 0);
  const xMax = Math.max(1, ...xValues) * 1.08;
  const yMinRaw = Math.min(...yValues);
  const yMaxRaw = Math.max(...yValues);
  const ySpan = Math.max(0.05, yMaxRaw - yMinRaw);
  const yMin = Math.max(0, yMinRaw - ySpan * 0.18);
  const yMax = yMaxRaw + ySpan * 0.18;
  const x = (value) => pad.l + value / xMax * (width - pad.l - pad.r);
  const y = (value) => height - pad.b - (value - yMin) / Math.max(1e-9, yMax - yMin) * (height - pad.t - pad.b);
  return <svg className="batch-pareto" viewBox={`0 0 ${width} ${height}`} role="img" aria-label="Batch robustness Pareto chart">
    {[0,1,2,3,4,5].map((i) => { const value = xMax * i / 5; return <g key={`x${i}`}><line x1={x(value)} y1={pad.t} x2={x(value)} y2={height-pad.b}/><text x={x(value)} y={height-18} textAnchor="middle">{Math.round(value)}</text></g>; })}
    {[0,1,2,3,4].map((i) => { const value = yMin + (yMax-yMin)*i/4; return <g key={`y${i}`}><line x1={pad.l} y1={y(value)} x2={width-pad.r} y2={y(value)}/><text x={pad.l-9} y={y(value)+4} textAnchor="end">{value.toFixed(2)}</text></g>; })}
    {groups.map((group, index) => <g key={group.presetId} className={`batch-point batch-point-${index%6}`}><circle cx={x(group.meanSolveCount)} cy={y(group.meanIae)} r={group.totalUnsafeSamples ? 8 : 6}/><text x={x(group.meanSolveCount)+9} y={y(group.meanIae)-8}>{group.presetLabel}</text></g>)}
    <text x={(pad.l+width-pad.r)/2} y={height-2} textAnchor="middle">Mean MPC solves</text>
    <text transform={`translate(15 ${(pad.t+height-pad.b)/2}) rotate(-90)`} textAnchor="middle">Mean IAE ↓</text>
  </svg>;
}

export default function BatchAnalysis({ batchResult }) {
  if (!batchResult) return null;
  const ranked = [...batchResult.groups].sort((a,b) => {
    const unsafe = a.totalUnsafeSamples - b.totalUnsafeSamples;
    if (unsafe !== 0) return unsafe;
    const fallback = a.totalFallbacks - b.totalFallbacks;
    if (fallback !== 0) return fallback;
    return a.meanIae - b.meanIae;
  });
  return <main className="dashboard-main workspace-page batch-analysis-page">
    <div className="workspace-page-head compact-batch-head"><div className="workspace-page-icon"><BarChart3 size={21}/></div><div><span>BATCH ROBUSTNESS</span><h1>Experiment Matrix · aggregate analysis</h1><p>{batchResult.requestedCases} HYBRID_SAFE cases across {batchResult.dimensions.presetIds.length} presets, {batchResult.dimensions.seeds.length} seed(s), {batchResult.dimensions.noiseScales.length} noise scale(s) and {batchResult.dimensions.mismatchScales.length} mismatch scale(s).</p></div></div>
    <section className="batch-analysis-grid">
      <article className="dashboard-card batch-pareto-card"><div className="card-head"><div><BarChart3 size={16}/><strong>Robustness Pareto · mean IAE vs mean solves</strong></div><span className="analysis-chip">aggregate cases</span></div><RobustnessPareto groups={batchResult.groups}/></article>
      <article className="dashboard-card batch-rank-card"><div className="card-head"><div><ShieldCheck size={16}/><strong>Robustness ordering</strong></div><span className="analysis-chip">safety first</span></div><div className="batch-rank-list">{ranked.map((group,index) => <div key={group.presetId} className="batch-rank-row"><b>#{index+1}</b><span><strong>{group.presetLabel}</strong><small>{group.cases} cases</small></span><span>IAE <b>{fmt(group.meanIae,4)}</b></span><span>unsafe <b className={group.totalUnsafeSamples?'warn':'good'}>{group.totalUnsafeSamples}</b></span><span>fallback <b>{group.totalFallbacks}</b></span></div>)}</div></article>
    </section>
    <section className="dashboard-card batch-worst-table"><div className="card-head"><div><ShieldCheck size={16}/><strong>Worst-case audit</strong></div></div><div className="batch-table-row head"><span>Preset</span><span>Worst IAE</span><span>Worst safety</span><span>Mean convergence</span><span>Mean reduction</span><span>x̂ RMSE</span></div>{batchResult.groups.map((group) => <div className="batch-table-row" key={group.presetId}><strong>{group.presetLabel}</strong><span>{fmt(group.worstIae,4)}</span><span>{group.worstSafetyViolation?.toExponential?.(1) || '0'}</span><span>{fmt(group.meanConvergenceRate,1)}%</span><span>{fmt(group.meanComputeReduction,1)}%</span><span>{fmt(group.meanPositionRmse,4)}</span></div>)}</section>
  </main>;
}
