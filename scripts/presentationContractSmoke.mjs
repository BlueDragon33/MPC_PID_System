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
assert.match(appSource, /research-context-header/, 'research context header must be present');
assert.match(appSource, /data-theme-control/, 'theme control must be present');
assert.match(appSource, /context\.safety/, 'actual plant safety context must be explicit and localized');
assert.match(appSource, /<h1>\{t\('context\.title'\)\}<\/h1>/, 'workbench requires a localized semantic primary heading');

const dashboardSource = fs.readFileSync(path.join(process.cwd(), 'src', 'components', 'app', 'SimulationDashboard.jsx'), 'utf8');
assert.doesNotMatch(dashboardSource, />Optimal</, 'solver UI must not hard-code an Optimal claim');
assert.match(dashboardSource, /<desc>/, 'primary control charts require textual descriptions');

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
const tinyFontDeclarations=[...css.matchAll(/font-size:\s*([0-9.]+)px/gi)]
  .map((match)=>Number(match[1]))
  .filter((size)=>size>0&&size<11);
assert.equal(tinyFontDeclarations.length,0,`authored UI CSS contains ${tinyFontDeclarations.length} font declarations below 11px`);

console.log('presentation contract smoke: PASS', {
  routes: WORKBENCH_ROUTE_IDS.length,
  responsiveBreakpoints: mediaQueries,
  tinyFontDeclarations: tinyFontDeclarations.length,
});
