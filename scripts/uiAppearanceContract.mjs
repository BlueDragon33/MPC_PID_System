import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';

function walk(dir) {
  return fs.readdirSync(dir, { withFileTypes: true }).flatMap((entry) => {
    const full = path.join(dir, entry.name);
    return entry.isDirectory() ? walk(full) : [full];
  });
}

const styles = walk(path.join(process.cwd(), 'src'))
  .filter((file) => file.endsWith('.css'))
  .map((file) => fs.readFileSync(file, 'utf8'))
  .join('\n');

assert.match(styles, /--ui-font-size:\s*16px/, 'default reading size must be 16 px');
assert.match(styles, /--ui-small-font-size:\s*14px/, 'default supporting text must remain readable');
assert.match(styles, /data-font-size="14"/, '14 px preference needs a CSS mapping');
assert.match(styles, /data-font-size="18"/, '18 px preference needs a CSS mapping');
assert.match(styles, /data-background="soft"/, 'soft gray background needs a CSS mapping');
assert.match(styles, /data-font-family="inter"/, 'Inter preference needs a CSS mapping');
assert.match(styles, /data-font-family="system"/, 'system preference needs a CSS mapping');
assert.match(styles, /data-font-family="serif"/, 'serif preference needs a CSS mapping');
assert.match(styles, /data-contrast="high"/, 'high contrast preference needs a CSS mapping');

const fixedEleven = [...styles.matchAll(/font-size:\s*11px/gi)];
assert.equal(fixedEleven.length, 0, `fixed 11 px UI text bypasses user scaling (${fixedEleven.length} declarations)`);

const iconRule = styles.match(/\.top-nav button svg\s*,[\s\S]*?\{([\s\S]*?)\}/);
assert.ok(iconRule, 'interactive icon normalization rule is missing');
assert.match(iconRule[1], /width:\s*1\.1em/);
assert.match(iconRule[1], /height:\s*1\.1em/);
assert.match(iconRule[1], /flex:\s*0 0 auto/);

assert.match(styles, /@media\s*\(max-width:\s*820px\)[\s\S]*?:root\[data-font-size="18"\]\s+\.app-topbar\s*\{[^}]*grid-template-columns:\s*minmax\(0,1fr\)\s+auto/, '18 px mode needs an equally specific mobile topbar override');
assert.match(styles, /\.setting-line strong,[^}]*\.setting-number strong\{[^}]*color:var\(--text\)/, 'Settings primary text must use semantic contrast colors');
assert.match(styles, /\.setting-line span,[^}]*\.setting-number span\{[^}]*color:var\(--muted\)/, 'Settings help text must use semantic contrast colors');

console.log('UI appearance contract: PASS');
