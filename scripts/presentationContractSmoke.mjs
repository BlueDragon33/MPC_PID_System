import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';

import {
  normalizeWorkbenchRoute,
  workbenchRouteHash,
  WORKBENCH_ROUTE_IDS,
} from '../src/components/app/navigation.js';

assert.deepEqual(WORKBENCH_ROUTE_IDS, [
  'simulation',
  'analysis',
  'scenarios',
  'documentation',
  'settings',
]);
for (const route of WORKBENCH_ROUTE_IDS) {
  assert.equal(normalizeWorkbenchRoute(`#${route}`), route);
  assert.equal(workbenchRouteHash(route), `#${route}`);
}
assert.equal(normalizeWorkbenchRoute('#unknown'), 'simulation');
assert.equal(normalizeWorkbenchRoute(''), 'simulation');
assert.equal(workbenchRouteHash('unknown'), '#simulation');

const appSource = fs.readFileSync(path.join(process.cwd(), 'src', 'App.jsx'), 'utf8');
assert.match(appSource, /aria-current=/, 'top navigation must expose aria-current');
assert.match(appSource, /aria-live="polite"/, 'status region must be announced politely');
assert.match(appSource, /aria-label="Main navigation"/, 'main navigation requires an accessible label');
assert.match(appSource, /type="button"/, 'navigation buttons must declare button type');

function walk(dir) {
  if (!fs.existsSync(dir)) return [];
  return fs.readdirSync(dir, { withFileTypes: true }).flatMap((entry) => {
    const full = path.join(dir, entry.name);
    return entry.isDirectory() ? walk(full) : [full];
  });
}
const css = walk(path.join(process.cwd(), 'src'))
  .filter((file) => file.endsWith('.css'))
  .map((file) => fs.readFileSync(file, 'utf8'))
  .join('\n');
const mediaQueries = [...css.matchAll(/@media\s*\([^)]*max-width/gi)].length;
assert(mediaQueries >= 3, `responsive contract expects at least 3 max-width media queries, got ${mediaQueries}`);

console.log('presentation contract smoke: PASS', {
  routes: WORKBENCH_ROUTE_IDS.length,
  responsiveBreakpoints: mediaQueries,
});
