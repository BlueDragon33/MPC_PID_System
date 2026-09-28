import React, { useMemo, useState } from 'react';
import { Car, Gauge, Play, ShieldCheck } from 'lucide-react';
import { executeUgvComparison } from '../../application/workbench.js';
import './ugvResearchLab.css';

const fmt = (value, digits = 3) => Number.isFinite(value) ? value.toFixed(digits) : '—';

function pathPoints(samples, fieldX, fieldY, bounds) {
  const width = 720;
  const height = 220;
  const pad = 18;
  const spanX = Math.max(1e-9, bounds.maxX - bounds.minX);
  const spanY = Math.max(1e-9, bounds.maxY - bounds.minY);
  return samples.map((sample) => {
    const x = pad + (width - 2 * pad) * (sample[fieldX] - bounds.minX) / spanX;
    const y = height - pad - (height - 2 * pad) * (sample[fieldY] - bounds.minY) / spanY;
    return `${x.toFixed(1)},${y.toFixed(1)}`;
  }).join(' ');
}

export default function UgvResearchLab() {
  const [estimationEnabled, setEstimationEnabled] = useState(true);
  const [duration, setDuration] = useState(6);
  const [results, setResults] = useState(null);
  const [error, setError] = useState('');
  const [running, setRunning] = useState(false);

  const run = () => {
    try {
      setRunning(true);
      setError('');
      const next = executeUgvComparison({
        duration: Math.max(2, Math.min(10, duration)),
        ugvEstimation: {
          enabled: estimationEnabled,
          seed: 20260928,
        },
      });
      setResults(next);
    } catch (runError) {
      setError(runError?.message || 'UGV comparison failed.');
    } finally {
      setRunning(false);
    }
  };

  const plot = useMemo(() => {
    if (!results) return null;
    const classical = results.find((item) => item.mode === 'CLASSICAL');
    const ltv = results.find((item) => item.mode === 'LTV_MPC');
    const allSamples = [...classical.samples, ...ltv.samples];
    const xs = allSamples.flatMap((sample) => [sample.x, sample.referenceX]);
    const ys = allSamples.flatMap((sample) => [sample.y, sample.referenceY]);
    const bounds = {
      minX: Math.min(...xs),
      maxX: Math.max(...xs),
      minY: Math.min(...ys) - 0.2,
      maxY: Math.max(...ys) + 0.2,
    };
    return {
      classical,
      ltv,
      reference: pathPoints(ltv.samples, 'referenceX', 'referenceY', bounds),
      classicalPath: pathPoints(classical.samples, 'x', 'y', bounds),
      ltvPath: pathPoints(ltv.samples, 'x', 'y', bounds),
    };
  }, [results]);

  return <section className="dashboard-card ugv-lab">
    <div className="card-head">
      <div><Car size={17}/><strong>UGV Bicycle Lab</strong></div>
      <span className="ugv-lab-badge">Gate 5A · nonlinear</span>
    </div>

    <div className="ugv-lab-toolbar">
      <div>
        <strong>Kinematic bicycle · Stanley/PID vs constrained LTV-MPC</strong>
        <span>20 Hz nonlinear plant · 10 Hz predictive solve · 0.60 s selected horizon</span>
      </div>
      <label><span>Duration</span><input type="number" min="2" max="10" step="1" value={duration} onChange={(e)=>setDuration(Number(e.target.value))}/><small>s</small></label>
      <label className="ugv-toggle"><input type="checkbox" checked={estimationEnabled} onChange={(e)=>setEstimationEnabled(e.target.checked)}/><span>EKF state</span></label>
      <button type="button" onClick={run} disabled={running}><Play size={14}/>{running?'Running…':'Run UGV comparison'}</button>
    </div>

    {error && <div className="matrix-error">{error}</div>}

    {!plot && <div className="ugv-lab-empty">
      <Gauge size={22}/>
      <strong>Run on demand</strong>
      <span>The nonlinear UGV experiment is intentionally not executed during page load because constrained QP solves are compute-intensive.</span>
    </div>}

    {plot && <>
      <div className="ugv-lab-kpis">
        {[['Classical', plot.classical], ['LTV-MPC', plot.ltv]].map(([label,result]) => <article key={label}>
          <div><strong>{label}</strong><span>{estimationEnabled?'EKF':'truth state'}</span></div>
          <dl>
            <div><dt>Lateral RMSE</dt><dd>{fmt(result.metrics.lateralRmse,4)}</dd></div>
            <div><dt>Heading RMSE</dt><dd>{fmt(result.metrics.headingRmse,4)}</dd></div>
            <div><dt>Effort</dt><dd>{fmt(result.metrics.controlEffort,2)}</dd></div>
            <div><dt>Safety</dt><dd className={result.metrics.unsafeSamples?'warn':'good'}>{result.metrics.unsafeSamples}</dd></div>
            <div><dt>Fallback</dt><dd>{result.metrics.fallbackCount}</dd></div>
            <div><dt>Accepted</dt><dd>{result.metrics.acceptedRate==null?'—':`${fmt(result.metrics.acceptedRate,1)}%`}</dd></div>
            <div><dt>Avg solve</dt><dd>{result.metrics.solveCount?`${fmt(result.metrics.averageSolveMs,1)} ms`:'—'}</dd></div>
            <div><dt>Progress</dt><dd>{fmt(result.metrics.finalProgressX,1)} m</dd></div>
          </dl>
        </article>)}
      </div>

      <div className="ugv-lab-chart">
        <div className="ugv-chart-head"><strong>XY path tracking</strong><span><i className="ref"/>Reference <i className="classical"/>Classical <i className="ltv"/>LTV-MPC</span></div>
        <svg viewBox="0 0 720 220" role="img" aria-label="UGV reference path with classical and LTV-MPC trajectories">
          <polyline className="ugv-reference" points={plot.reference}/>
          <polyline className="ugv-classical" points={plot.classicalPath}/>
          <polyline className="ugv-ltv" points={plot.ltvPath}/>
        </svg>
      </div>

      <div className="ugv-lab-footnote">
        <ShieldCheck size={14}/>
        <span>Actual nonlinear plant safety is audited separately from LTV-QP feasibility. CI solve time is research evidence only, not a hardware real-time claim.</span>
      </div>
    </>}
  </section>;
}
