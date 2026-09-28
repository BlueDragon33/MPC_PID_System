export const WORKBENCH_ROUTE_IDS = Object.freeze([
  'simulation',
  'analysis',
  'scenarios',
  'documentation',
  'settings',
]);

const ROUTES = new Set(WORKBENCH_ROUTE_IDS);

export function normalizeWorkbenchRoute(hash = '') {
  const route = String(hash).replace(/^#/, '').trim();
  return ROUTES.has(route) ? route : 'simulation';
}

export function workbenchRouteHash(route) {
  return `#${ROUTES.has(route) ? route : 'simulation'}`;
}
