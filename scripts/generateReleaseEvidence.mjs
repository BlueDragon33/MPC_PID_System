import crypto from 'node:crypto';
import fs from 'node:fs';
import path from 'node:path';

const root = process.cwd();
const distDir = path.resolve(root, process.argv[2] || 'dist');
const output = path.resolve(root, process.argv[3] || 'release-evidence.json');

if (!fs.existsSync(distDir) || !fs.statSync(distDir).isDirectory()) {
  throw new Error(`dist directory does not exist: ${distDir}`);
}

function walk(dir) {
  return fs.readdirSync(dir, { withFileTypes: true })
    .flatMap((entry) => {
      const full = path.join(dir, entry.name);
      return entry.isDirectory() ? walk(full) : [full];
    })
    .sort();
}

function sha256(file) {
  return crypto.createHash('sha256').update(fs.readFileSync(file)).digest('hex');
}

const files = walk(distDir).map((file) => ({
  path: path.relative(distDir, file).split(path.sep).join('/'),
  bytes: fs.statSync(file).size,
  sha256: sha256(file),
}));

const candidateSha = process.env.GITHUB_SHA || process.env.RELEASE_CANDIDATE_SHA || 'unknown';
const rollbackSha = process.env.RELEASE_ROLLBACK_SHA || 'unknown';
const repository = process.env.GITHUB_REPOSITORY || 'BlueDragon33/MPC_PID_System';

if (!/^[0-9a-f]{40}$/i.test(candidateSha)) {
  throw new Error(`exact candidate SHA is required, got: ${candidateSha}`);
}
if (!/^[0-9a-f]{40}$/i.test(rollbackSha)) {
  throw new Error(`exact rollback SHA is required, got: ${rollbackSha}`);
}
if (!files.length) throw new Error('release artifact is empty');

const manifest = {
  schema: 'mpc-pid-release-evidence/v1',
  productionAuthority: false,
  repository,
  candidateSha,
  rollbackSha,
  generatedAt: new Date().toISOString(),
  artifact: {
    root: 'dist',
    fileCount: files.length,
    totalBytes: files.reduce((sum, file) => sum + file.bytes, 0),
    files,
  },
};

fs.writeFileSync(output, JSON.stringify(manifest, null, 2) + '\n');
console.log('release evidence generated', {
  candidateSha,
  rollbackSha,
  fileCount: files.length,
  totalBytes: manifest.artifact.totalBytes,
});
