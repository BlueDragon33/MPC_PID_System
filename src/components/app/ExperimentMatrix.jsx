import React, { useMemo, useState } from 'react';
import { BarChart3, CheckCircle2, Database, FlaskConical, Play, RotateCcw, ShieldCheck } from 'lucide-react';
import { EXPERIMENT_PRESETS, applyExperimentPreset } from '../../core/experiments/presets.js';
import { runExperimentMatrix } from '../../core/experiments/experimentMatrix.js';
import { defaultConfig } from '../../core/simulator.js';
import './experimentMatrix.css';

const DEFAULT_PRESETS = ['baseline', 'safety-envelope', 'noisy-estimation', 'mismatch-observer'];
const fmt = (value, digits = 2) => Number.isFinite(value) ? value.toFixed(digits) : '—';

function parseList(value, fallback) {
  const values = value.split(',').map((item) => Number(item.trim())).filter(Number.isFinite);
  return values.length ? values : fallback;
}

export default function ExperimentMatrix({
  draftCfg,
  setDraftCfg,
  presetId,
  setPresetId,
  batchResult,
  setBatchResult,
  navigate,
}) {
  const [selected, setSelected] = useState(DEFAULT_PRESETS);
  const [seedText, setSeedText] = useState('20260914, 20260915');
  const [noiseText, setNoiseText] = useState('1');
  const [mismatchText, setMismatchText] = useState('1');
  const [error, setError] = useState('');
  const seeds = useMemo(() => parseList(seedText, [20260914]), [seedText]);
  const noiseScales = useMemo(() => parseList(noiseText, [1]), [noiseText]);
  const mismatchScales = useMemo(() => parseList(mismatchText, [1]), [mismatchText]);
  const caseCount = selected.length * seeds.length * noiseScales.length * mismatchScales.length;

  const togglePreset = (id) => setSelected((current) => current.includes(id)
    ? current.filter((item) => item !== id)
    : [...current, id]);

  const runMatrix = () => {
    try {
      setError('');
      const result = runExperimentMatrix({
        presetIds: selected,
        seeds,
        noiseScales,
        mismatchScales,
        maxCases: 48,
      });
      setBatchResult(result);
    } catch (matrixError) {
      setError(matrixError?.message || 'Experiment matrix failed.');
    }
  };

  const loadPreset = (id) => {
    setPresetId(id);
    setDraftCfg(applyExperimentPreset(defaultConfig, id));
  };

  return <main className="dashboard-main workspace-page matrix-page">
    <div className="workspace-page-head">
      <div className="workspace-page-icon"><Database size={22}/></div>
      <div><span>EXPERIMENT MATRIX</span><h1>Batch scenarios and robustness sweeps</h1><p>Run the same safe hybrid controller across multiple presets, sensor seeds, noise levels and model-mismatch intensities. Results are summarized here and forwarded to Analysis.</p></div>
    </div>

    <section className="matrix-builder dashboard-card">
      <div className="card-head"><div><FlaskConical size={16}/><strong>Matrix builder</strong></div><span className={`matrix-case-pill ${caseCount > 48 ? 'over' : ''}`}>{caseCount} / 48 cases</span></div>
      <div className="matrix-builder-body">
        <div className="matrix-preset-grid">
          {EXPERIMENT_PRESETS.map((preset) => {
            const checked = selected.includes(preset.id);
            return <button key={preset.id} className={`matrix-preset ${checked ? 'selected' : ''}`} onClick={() => togglePreset(preset.id)}>
              <span className="matrix-check">{checked ? <CheckCircle2 size={15}/> : <i/>}</span>
              <span><strong>{preset.label}</strong><small>{preset.description}</small></span>
            </button>;
          })}
        </div>
        <div className="matrix-dimensions">
          <label><span>Seeds</span><input value={seedText} onChange={(e) => setSeedText(e.target.value)} placeholder="20260914, 20260915"/><small>comma-separated deterministic sensor seeds</small></label>
          <label><span>Noise scale</span><input value={noiseText} onChange={(e) => setNoiseText(e.target.value)} placeholder="0.75, 1, 1.25"/><small>multiplies each preset's measurement σ</small></label>
          <label><span>Mismatch scale</span><input value={mismatchText} onChange={(e) => setMismatchText(e.target.value)} placeholder="0.8, 1, 1.2"/><small>scales existing truth-plant mismatch away from nominal</small></label>
          <div className="matrix-actions">
            <button className="matrix-run" disabled={!selected.length || caseCount > 48} onClick={runMatrix}><Play size={15}/>Run matrix</button>
            <button className="matrix-reset" onClick={() => { setSelected(DEFAULT_PRESETS); setSeedText('20260914, 20260915'); setNoiseText('1'); setMismatchText('1'); setError(''); }}><RotateCcw size={14}/>Reset dimensions</button>
          </div>
          {error && <div className="matrix-error">{error}</div>}
        </div>
      </div>
    </section>

    <section className="matrix-quick-grid">
      {EXPERIMENT_PRESETS.map((preset) => <article key={preset.id} className={`matrix-quick-card ${preset.id === presetId ? 'active' : ''}`}>
        <div><strong>{preset.label}</strong><small>{preset.patch?.truthPlant?.enabled ? 'model mismatch' : 'nominal plant'} · {preset.patch?.estimation?.enabled ? 'estimator' : 'truth state'}</small></div>
        <button onClick={() => loadPreset(preset.id)}>{preset.id === presetId ? 'Loaded' : 'Load draft'}</button>
      </article>)}
    </section>

    {batchResult && <section className="dashboard-card matrix-results">
      <div className="card-head"><div><BarChart3 size={16}/><strong>Batch result</strong></div><button className="matrix-analysis-button" onClick={() => navigate('analysis')}>Open in Analysis</button></div>
      <div className="matrix-result-kpis">
        <div><span>Cases</span><strong>{batchResult.requestedCases}</strong></div>
        <div><span>Mean IAE</span><strong>{fmt(batchResult.overall.meanIae, 4)}</strong></div>
        <div><span>Mean solve reduction</span><strong>{fmt(batchResult.overall.meanComputeReduction, 1)}%</strong></div>
        <div><span>Fallbacks</span><strong>{batchResult.overall.totalFallbacks}</strong></div>
        <div><span>Unsafe samples</span><strong>{batchResult.overall.totalUnsafeSamples}</strong></div>
      </div>
      <div className="matrix-summary-table">
        <div className="matrix-summary-row head"><span>Preset</span><span>Cases</span><span>Mean IAE</span><span>Worst IAE</span><span>Mean solves</span><span>Convergence</span><span>Fallback</span><span>Unsafe</span></div>
        {batchResult.groups.map((group) => <div className="matrix-summary-row" key={group.presetId}>
          <strong>{group.presetLabel}</strong><span>{group.cases}</span><span>{fmt(group.meanIae,4)}</span><span>{fmt(group.worstIae,4)}</span><span>{fmt(group.meanSolveCount,1)}</span><span>{fmt(group.meanConvergenceRate,1)}%</span><span>{group.totalFallbacks}</span><span className={group.totalUnsafeSamples ? 'warn' : 'good'}>{group.totalUnsafeSamples}</span>
        </div>)}
      </div>
      <div className="matrix-footnote"><ShieldCheck size={14}/>Every matrix case uses HYBRID_SAFE. Aggregates report actual plant safety separately from solver feasibility.</div>
    </section>}
  </main>;
}
