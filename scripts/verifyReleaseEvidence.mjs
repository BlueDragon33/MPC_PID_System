import crypto from 'node:crypto';
import fs from 'node:fs';
import path from 'node:path';

const root = process.cwd();
const evidencePath = path.resolve(root, process.argv[2] || 'release-evidence.json');
const distDir = path.resolve(root, process.argv[3] || 'dist');
const evidence = JSON.parse(fs.readFileSync(evidencePath, 'utf8'));

if (evidence.schema !== 'mpc-pid-release-evidence/v1') {
  throw new Error(`unsupported release evidence schema: ${evidence.schema}`);
}
if (evidence.productionAuthority !== false) {
  throw new Error('readiness evidence must not claim production authority');
}
if (!/^[0-9a-f]{40}$/i.test(evidence.candidateSha)) throw new Error('candidate SHA missing');
if (!/^[0-9a-f]{40}$/i.test(evidence.rollbackSha)) throw new Error('rollback SHA missing');

function sha256(file) {
  return crypto.createHash('sha256').update(fs.readFileSync(file)).digest('hex');
}

for (const item of evidence.artifact.files) {
  const file = path.join(distDir, item.path);
  if (!fs.existsSync(file)) throw new Error(`artifact file missing: ${item.path}`);
  const actual = sha256(file);
  if (actual !== item.sha256) {
    throw new Error(`artifact hash mismatch: ${item.path}`);
  }
}

const currentFiles = [];
function walk(dir) {
  for (const entry of fs.readdirSync(dir, { withFileTypes: true })) {
    const full = path.join(dir, entry.name);
    if (entry.isDirectory()) walk(full);
    else currentFiles.push(path.relative(distDir, full).split(path.sep).join('/'));
  }
}
walk(distDir);
currentFiles.sort();
const expectedFiles = evidence.artifact.files.map((item) => item.path).sort();

if (JSON.stringify(currentFiles) !== JSON.stringify(expectedFiles)) {
  throw new Error('artifact file set differs from evidence manifest');
}

console.log('release evidence verification: PASS', {
  candidateSha: evidence.candidateSha,
  rollbackSha: evidence.rollbackSha,
  files: expectedFiles.length,
});
