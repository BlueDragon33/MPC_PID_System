import fs from 'node:fs';
import path from 'node:path';

const root = process.cwd();
const ignoredDirs = new Set(['.git', 'node_modules', 'dist', 'coverage']);
const textExtensions = new Set([
  '.js', '.jsx', '.mjs', '.cjs', '.ts', '.tsx',
  '.json', '.md', '.yml', '.yaml', '.html', '.css',
  '.env', '.txt',
]);

const patterns = [
  { id: 'private-key', regex: /-----BEGIN (?:RSA |EC |OPENSSH |DSA )?PRIVATE KEY-----/g },
  { id: 'github-classic-pat', regex: /\bghp_[A-Za-z0-9]{30,}\b/g },
  { id: 'github-fine-grained-pat', regex: /\bgithub_pat_[A-Za-z0-9_]{30,}\b/g },
  { id: 'aws-access-key', regex: /\bAKIA[0-9A-Z]{16}\b/g },
  { id: 'openai-style-secret', regex: /\bsk-[A-Za-z0-9_-]{20,}\b/g },
];

function walk(dir) {
  const out = [];
  for (const entry of fs.readdirSync(dir, { withFileTypes: true })) {
    if (entry.isDirectory() && ignoredDirs.has(entry.name)) continue;
    const full = path.join(dir, entry.name);
    if (entry.isDirectory()) out.push(...walk(full));
    else out.push(full);
  }
  return out;
}

const violations = [];
for (const file of walk(root)) {
  const rel = path.relative(root, file).split(path.sep).join('/');
  const ext = path.extname(file);
  if (!textExtensions.has(ext) && !path.basename(file).startsWith('.env')) continue;
  const text = fs.readFileSync(file, 'utf8');
  for (const { id, regex } of patterns) {
    regex.lastIndex = 0;
    for (const match of text.matchAll(regex)) {
      const before = text.slice(0, match.index);
      const line = before.split('\n').length;
      violations.push({ id, file: rel, line });
    }
  }
}

if (violations.length) {
  console.error('release security scan: FAIL');
  for (const item of violations) {
    console.error(`  ${item.id}: ${item.file}:${item.line}`);
  }
  process.exit(1);
}

const index = fs.readFileSync(path.join(root, 'index.html'), 'utf8');
if (!/Content-Security-Policy/i.test(index)) {
  console.error('release security scan: missing Content-Security-Policy in index.html');
  process.exit(1);
}
if (!/referrer[^>]+no-referrer/i.test(index)) {
  console.error('release security scan: missing no-referrer policy in index.html');
  process.exit(1);
}

console.log('release security scan: PASS', {
  scannedFiles: walk(root).length,
  highSignalPatterns: patterns.length,
});
