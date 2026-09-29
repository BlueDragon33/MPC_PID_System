import React, { useState } from 'react';
import { Activity, BarChart3, BookOpen, CheckCircle2, Cpu, Gauge, ShieldCheck, Sparkles, Zap } from 'lucide-react';
import { buildSimulationView, normalizeResponseSignal, RESPONSE_SIGNALS } from './simulationViewModel.js';

const MODES = ['PID', 'MPC', 'HYBRID', 'HYBRID_SAFE'];
const MODE_LABELS = { PID:'PID Only', MPC:'MPC (Periodic)', HYBRID:'MPC + PID (Event)', HYBRID_SAFE:'MPC + PID + Safety' };
const MODE_SHORT = { PID:'PID', MPC:'MPC', HYBRID:'MPC + PID', HYBRID_SAFE:'MPC + PID + Safety' };

function fmt(value, digits=2, fallback='—'){ return Number.isFinite(value) ? value.toFixed(digits) : fallback; }
function scientific(value){ if(!Number.isFinite(value)) return '—'; if(value===0) return '0.0e+0'; return value.toExponential(1); }

function TimeSeriesChart({ results, valueKey, target, yLabel, compact=false }) {
  const width=980, height=compact?205:315, pad={l:56,r:18,t:22,b:38};
  const allValues=results.flatMap((result)=>result.samples.map((sample)=>Number(sample[valueKey])||0));
  if(valueKey==='x') allValues.push(target);
  const rawMin=Math.min(...allValues,-0.05), rawMax=Math.max(...allValues,0.05), span=Math.max(rawMax-rawMin,0.25);
  const yMin=rawMin-span*0.12, yMax=rawMax+span*0.12;
  const tMax=Math.max(...results.flatMap((r)=>r.samples.map((s)=>s.t)),1);
  const plotW=width-pad.l-pad.r, plotH=height-pad.t-pad.b;
  const x=(t)=>pad.l+(t/tMax)*plotW, y=(v)=>pad.t+(1-(v-yMin)/(yMax-yMin))*plotH;
  const path=(samples)=>samples.map((s,i)=>`${i?'L':'M'} ${x(s.t).toFixed(2)} ${y(Number(s[valueKey])||0).toFixed(2)}`).join(' ');
  const xTicks=Array.from({length:6},(_,i)=>(tMax*i)/5), yTicks=Array.from({length:5},(_,i)=>yMin+((yMax-yMin)*i)/4);
  return <svg className={`timeseries-chart ${compact?'compact':''}`} viewBox={`0 0 ${width} ${height}`} role="img" aria-label={`${yLabel} chart`}>
    <title>{`${yLabel} response for ${MODE_LABELS[results[0]?.mode] || 'selected controller'}`}</title>
    <desc>{`${yLabel} over time for the selected ${MODE_LABELS[results[0]?.mode] || 'controller'} mode${valueKey==='x' ? `, with reference ${target}` : ''}.`}</desc>
    <g className="chart-grid">{xTicks.map((tick)=><line key={`x-${tick}`} x1={x(tick)} y1={pad.t} x2={x(tick)} y2={height-pad.b}/>)}{yTicks.map((tick)=><line key={`y-${tick}`} x1={pad.l} y1={y(tick)} x2={width-pad.r} y2={y(tick)}/>)}</g>
    {valueKey==='x'&&<line x1={pad.l} y1={y(target)} x2={width-pad.r} y2={y(target)} className="target-line"/>}
    {results.map((result)=><path key={result.mode} d={path(result.samples)} className={`curve curve-${result.mode.toLowerCase()}`}/>)}
    <g className="chart-labels">{xTicks.map((tick)=><text key={`xt-${tick}`} x={x(tick)} y={height-13} textAnchor="middle">{tick.toFixed(tMax>20?0:1)}</text>)}{yTicks.map((tick)=><text key={`yt-${tick}`} x={pad.l-10} y={y(tick)+4} textAnchor="end">{tick.toFixed(1)}</text>)}<text x={pad.l+plotW/2} y={height-1} textAnchor="middle">Time (s)</text><text transform={`translate(13 ${pad.t+plotH/2}) rotate(-90)`} textAnchor="middle">{yLabel}</text></g>
  </svg>;
}

function TriggerTimeline({samples,duration}){
  const width=980,height=150,pad={l:92,r:18};
  const rows=[['prediction-error','Prediction'],['state-change','State Error'],['watchdog','Timeout'],['constraint','Safety']];
  const x=(t)=>pad.l+(t/Math.max(duration,1e-6))*(width-pad.l-pad.r), rowY=(i)=>30+i*24;
  const events=samples.filter((s)=>s.triggered||s.governorSafetyIntervened);
  return <svg className="trigger-timeline" viewBox={`0 0 ${width} ${height}`} role="img" aria-label="Trigger events timeline">
    <title>Trigger and safety event timeline</title>
    <desc>{`${events.length} trigger or safety events across prediction, state, timeout and safety rows.`}</desc>
    {rows.map(([reason,label],i)=><g key={reason}><text x="8" y={rowY(i)+4} className="timeline-row-label">{label}</text><line x1={pad.l} y1={rowY(i)} x2={width-pad.r} y2={rowY(i)} className="timeline-row"/></g>)}
    {events.map((s,i)=>{const reason=s.governorSafetyIntervened?'constraint':s.triggerReason==='initial'?'prediction-error':s.triggerReason; const ri=Math.max(0,rows.findIndex(([key])=>key===reason)); return <line key={`${s.t}-${i}`} x1={x(s.t)} y1={rowY(ri)-7} x2={x(s.t)} y2={rowY(ri)+7} className={`event-mark event-${reason}`}/>;})}
    {Array.from({length:6},(_,i)=>(duration*i)/5).map((tick)=><g key={tick}><text x={x(tick)} y={height-8} textAnchor="middle" className="timeline-tick">{tick.toFixed(duration>20?0:1)}</text><line x1={x(tick)} y1="18" x2={x(tick)} y2={height-25} className="timeline-gridline"/></g>)}
  </svg>;
}

function Metric({label,value,accent=false}){ return <div className="health-row"><span>{label}</span><strong className={accent?'metric-good':''}>{value}</strong></div>; }

export default function SimulationDashboard({ results, activeMode, runCfg, solverLabel }){
  const { active, visibleResults } = buildSimulationView(results, activeMode);
  const [responseSignal, setResponseSignal] = useState('x');
  if (!active) return <main className="dashboard-main"><section className="dashboard-card"><p>No simulation result is available.</p></section></main>;
  const signal = RESPONSE_SIGNALS.find((item) => item.value === responseSignal) || RESPONSE_SIGNALS[0];
  const recentEvents=[...active.samples].filter((s)=>s.triggered||s.governorSafetyIntervened).slice(-5).reverse();
  const usesSolver=active.mode!=='PID';
  const usesGovernor=active.mode==='HYBRID_SAFE';
  const convergence=active.metrics.convergenceRate??100;
  const fallbackCount=active.metrics.fallbackCount??0;
  return <main className="dashboard-main">
    <section className="stat-strip" aria-label={`${MODE_LABELS[active.mode]} run summary`}>
      <div className="stat-card"><span className="stat-icon blue"><Gauge size={18}/></span><div><small>Simulation Time</small><strong>{fmt(runCfg.duration,2)} s</strong><span>{Math.round(runCfg.duration/runCfg.dt)} steps</span></div></div>
      <div className="stat-card"><span className="stat-icon purple"><Cpu size={18}/></span><div><small>MPC Solves</small><strong>{active.metrics.solveCount}</strong><span>{usesSolver?`${fmt(100-active.metrics.computeReduction,1)}% of steps`:'not used by PID'}</span></div></div>
      <div className="stat-card"><span className="stat-icon amber"><Zap size={18}/></span><div><small>Trigger Rate</small><strong>{fmt(active.metrics.triggerRate,1)}/s</strong><span>{usesSolver?`${fmt(active.metrics.computeReduction,1)}% compute avoided`:'periodic fast loop'}</span></div></div>
      <div className="stat-card"><span className="stat-icon green"><ShieldCheck size={18}/></span><div><small>Plant Safety</small><strong>{active.metrics.safetyViolationCount}</strong><span>violations</span></div></div>
      <div className="stat-card"><span className="stat-icon green"><CheckCircle2 size={18}/></span><div><small>Solver Success</small><strong>{usesSolver?`${fmt(convergence,0)}%`:'N/A'}</strong><span>{usesSolver?`${fallbackCount} fallback`:'PID has no solver'}</span></div></div>
    </section>
    <div className="dashboard-grid">
      <div className="center-column">
        <section className="dashboard-card response-card"><div className="card-head"><div><Activity size={16}/><strong>System Response</strong></div><label className="chart-selector"><span className="visually-hidden">Response signal</span><select aria-label="Response signal" value={responseSignal} onChange={(event)=>setResponseSignal(normalizeResponseSignal(event.target.value))}>{RESPONSE_SIGNALS.map((item)=><option key={item.value} value={item.value}>{item.label}</option>)}</select></label></div><div className="chart-legend"><span className="ref-line">Reference</span><span className={`legend-${active.mode.toLowerCase()}`}>{MODE_LABELS[active.mode]}</span></div><TimeSeriesChart results={visibleResults} valueKey={signal.value} target={runCfg.setpoint} yLabel={signal.yLabel}/></section>
        <section className="dashboard-card control-chart-card"><div className="card-head"><div><Zap size={16}/><strong>Control Input</strong></div><span className="subtle-badge">u(t)</span></div><div className="chart-legend compact-legend"><span className={`legend-${active.mode.toLowerCase()}`}>{MODE_LABELS[active.mode]}</span></div><TimeSeriesChart results={visibleResults} valueKey="u" target={runCfg.setpoint} yLabel="Control Input (u)" compact/></section>
        <section className="dashboard-card trigger-card"><div className="card-head"><div><Sparkles size={16}/><strong>Trigger Events</strong></div><span className="subtle-badge">{MODE_SHORT[active.mode]}</span></div><div className="event-legend"><span className="dot-red">Prediction Error</span><span className="dot-blue">State Error</span><span className="dot-yellow">Timeout</span><span className="dot-purple">Safety Intervention</span></div><TriggerTimeline samples={active.samples} duration={runCfg.duration}/></section>
      </div>
      <aside className="insight-column">
        <section className="dashboard-card metrics-card"><div className="card-head"><div><BarChart3 size={16}/><strong>Performance Metrics</strong></div><span className="subtle-badge">{MODE_SHORT[active.mode]}</span></div><div className="metric-table"><div className="metric-row metric-header"><span>Metric</span><span>{MODE_SHORT[active.mode]}</span></div><div className="metric-row"><span>IAE</span><span>{fmt(active.metrics.iae,4)}</span></div><div className="metric-row"><span>Overshoot</span><span>{fmt(active.metrics.overshoot,1)}%</span></div><div className="metric-row"><span>Settling Time</span><span>{Number.isFinite(active.metrics.settling)?`${fmt(active.metrics.settling,2)} s`:'—'}</span></div><div className="metric-row"><span>Control Effort</span><span>{fmt(active.metrics.controlEffort,2)}</span></div><div className="metric-row"><span>MPC Solves</span><span>{active.metrics.solveCount}</span></div><div className="metric-row metric-highlight"><span>Solve Reduction</span><span>{active.mode==='PID'?'—':`${fmt(active.metrics.computeReduction,1)}%`}</span></div></div></section>
        <section className="dashboard-card health-card"><div className="card-head"><div><Cpu size={16}/><strong>Solver Health</strong></div><span className="optimal-pill"><CheckCircle2 size={12}/>{!usesSolver?'Not used':fallbackCount>0?'Fallback observed':convergence>=99.9?'Converged':'Degraded'}</span></div>{usesSolver?<div className="health-grid"><div><Metric label="Backend" value={solverLabel}/><Metric label="Convergence" value={`${fmt(convergence,1)}%`} accent/><Metric label="Avg iterations" value={fmt(active.metrics.avgIterations,1)}/><Metric label="Avg solve" value={`${fmt(active.metrics.avgSolveMs,2)} ms`}/><Metric label="Stationarity" value={scientific(active.metrics.avgStationarityResidual)}/></div><div><Metric label="Feasibility" value={scientific(active.metrics.maxFeasibilityViolation)}/><Metric label="Active ratio" value={`${fmt((active.metrics.avgActiveConstraintRatio??0)*100,1)}%`}/><Metric label="Fallback" value={fallbackCount}/><Metric label="Timeout" value={active.metrics.timeoutCount??0}/><Metric label="Infeasible" value={active.metrics.infeasibleCount??0}/></div></div>:<p className="panel-empty-state">PID runs directly at the fast loop rate and does not invoke an optimization solver.</p>}</section>
        {usesGovernor?<section className="dashboard-card governor-card"><div className="card-head"><div><ShieldCheck size={16}/><strong>Safety Governor</strong></div><span className="optimal-pill"><CheckCircle2 size={12}/>Active</span></div><Metric label="Intervention Rate" value={`${fmt(active.metrics.governorInterventionRate,1)}%`}/><Metric label="Max |u_safe - u_pid|" value={fmt(active.metrics.governorMaxCorrection,3)}/><Metric label="Empty Admissible Interval" value={active.metrics.governorInfeasibleCount}/><Metric label="Emergency Fallback" value={active.metrics.governorEmergencyCount}/><Metric label="Plant Safety Violations" value={active.metrics.safetyViolationCount} accent/><Metric label="Conditioning Only" value={`${active.metrics.governorConditioningCount} (${fmt(active.metrics.governorConditioningRate,1)}%)`}/></section>:<section className="dashboard-card governor-card"><div className="card-head"><div><ShieldCheck size={16}/><strong>Safety Governor</strong></div><span className="subtle-badge">Inactive</span></div><p className="panel-empty-state">The selected {MODE_LABELS[active.mode]} mode does not grant command authority to the Safety Governor. Actual plant safety is still audited independently.</p><Metric label="Plant Safety Violations" value={active.metrics.safetyViolationCount} accent/></section>}
        <section className="dashboard-card events-card"><div className="card-head"><div><BookOpen size={16}/><strong>Recent Events</strong></div></div><div className="recent-events">{recentEvents.length===0&&<div className="empty-event">No trigger events in this run.</div>}{recentEvents.map((s,i)=><div className="recent-event" key={`${s.t}-${i}`}><span className={`event-dot ${s.governorSafetyIntervened?'safety':s.triggerReason||'state-change'}`}/><time>{fmt(s.t,2)}s</time><span>{s.governorSafetyIntervened?'Safety intervention':`${s.triggerReason||'state'} trigger`}</span></div>)}</div></section>
      </aside>
    </div>
  </main>;
}
