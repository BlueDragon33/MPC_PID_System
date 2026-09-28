import fs from 'node:fs';
import path from 'node:path';

const root = process.cwd();
const manifestPath = path.join(root, '.blueprint', 'architecture.json');
const manifest = JSON.parse(fs.readFileSync(manifestPath, 'utf8'));

function walk(dir) {
  if (!fs.existsSync(dir)) return [];
  const out = [];
  for (const entry of fs.readdirSync(dir, { withFileTypes: true })) {
    const full = path.join(dir, entry.name);
    if (entry.isDirectory()) out.push(...walk(full));
    else out.push(full);
  }
  return out;
}

function normalize(file) {
  return path.relative(root, file).split(path.sep).join('/');
}

function matches(pattern, file) {
  if (pattern.endsWith('/**')) return file.startsWith(pattern.slice(0, -3));
  return file === pattern;
}

function layerFor(file) {
  return manifest.layers.find((layer) => layer.paths.some((pattern) => matches(pattern, file))) || null;
}

function resolveRelativeImport(sourceFile, specifier) {
  const base = path.resolve(path.dirname(sourceFile), specifier);
  const candidates = [
    base,
    `${base}.js`,
    `${base}.jsx`,
    path.join(base, 'index.js'),
    path.join(base, 'index.jsx'),
  ];
  return candidates.find((candidate) => fs.existsSync(candidate) && fs.statSync(candidate).isFile()) || null;
}

const sourceFiles = walk(path.join(root, 'src')).filter((file) => /\.(js|jsx|mjs)$/.test(file));
const importRegex = /(?:import\s+(?:[^'"]+?\s+from\s+)?|export\s+[^'"]*?\s+from\s+)['"]([^'"]+)['"]/g;
const violations = [];
const unresolved = [];

for (const sourceFile of sourceFiles) {
  const sourceRel = normalize(sourceFile);
  const sourceLayer = layerFor(sourceRel);
  if (!sourceLayer) continue;
  const text = fs.readFileSync(sourceFile, 'utf8');
  for (const match of text.matchAll(importRegex)) {
    const specifier = match[1];
    if (!specifier.startsWith('.')) continue;
    const targetFile = resolveRelativeImport(sourceFile, specifier);
    if (!targetFile) {
      if (!/\.(css|json)$/.test(specifier)) unresolved.push({ sourceRel, specifier });
      continue;
    }
    const targetRel = normalize(targetFile);
    const targetLayer = layerFor(targetRel);
    if (!targetLayer || targetLayer.name === sourceLayer.name) continue;
    if (!sourceLayer.allowedDependencies.includes(targetLayer.name)) {
      violations.push({
        source: sourceRel,
        sourceLayer: sourceLayer.name,
        target: targetRel,
        targetLayer: targetLayer.name,
      });
    }
  }
}

if (unresolved.length) {
  console.warn('Architecture check: unresolved relative imports (non-blocking):');
  unresolved.forEach((item) => console.warn(`  ${item.sourceRel} -> ${item.specifier}`));
}

if (violations.length) {
  console.error('Architecture boundary violations:');
  violations.forEach((item) => {
    console.error(`  ${item.source} [${item.sourceLayer}] -> ${item.target} [${item.targetLayer}]`);
  });
  process.exit(1);
}

const activePackages = JSON.parse(
  fs.readFileSync(path.join(root, '.blueprint', 'work-packages.json'), 'utf8'),
).packages.filter((item) => item.status === 'ACTIVE');

if (activePackages.length > 1) {
  console.error(`Architecture governance violation: ${activePackages.length} work packages are ACTIVE.`);
  process.exit(1);
}

const presentationCoreText = sourceFiles
  .filter((file) => layerFor(normalize(file))?.name === 'presentation')
  .flatMap((file) => {
    const rel = normalize(file);
    const text = fs.readFileSync(file, 'utf8');
    return [...text.matchAll(importRegex)]
      .map((match) => match[1])
      .filter((specifier) => specifier.includes('/core/'))
      .map((specifier) => `${rel} -> ${specifier}`);
  });

if (presentationCoreText.length) {
  console.error('ARCH-001 presentation-must-not-import-core violations:');
  presentationCoreText.forEach((line) => console.error(`  ${line}`));
  process.exit(1);
}

console.log('architecture check: PASS', {
  architectureId: manifest.architectureId,
  layers: manifest.layers.length,
  activeWorkPackage: activePackages[0]?.id ?? null,
  checkedSourceFiles: sourceFiles.length,
});
