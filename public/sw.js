const CACHE_VERSION = 'mpc-pid-shell-v1';
const RUNTIME_CACHE = 'mpc-pid-runtime-v1';

function scopeUrl(path = './') {
  return new URL(path, self.registration.scope).href;
}

self.addEventListener('install', (event) => {
  event.waitUntil(
    caches.open(CACHE_VERSION)
      .then((cache) => cache.addAll([
        scopeUrl('./'),
        scopeUrl('./manifest.webmanifest'),
        scopeUrl('./favicon.svg'),
      ]))
      .then(() => self.skipWaiting()),
  );
});

self.addEventListener('activate', (event) => {
  event.waitUntil(
    caches.keys()
      .then((keys) => Promise.all(
        keys
          .filter((key) => ![CACHE_VERSION, RUNTIME_CACHE].includes(key))
          .map((key) => caches.delete(key)),
      ))
      .then(() => self.clients.claim()),
  );
});

self.addEventListener('message', (event) => {
  if (event.data?.type !== 'CACHE_URLS' || !Array.isArray(event.data.urls)) return;
  const urls = event.data.urls
    .map((value) => {
      try {
        const url = new URL(value, self.registration.scope);
        return url.origin === self.location.origin ? url.href : null;
      } catch {
        return null;
      }
    })
    .filter(Boolean);
  if (!urls.length) return;
  event.waitUntil(
    caches.open(RUNTIME_CACHE).then((cache) => Promise.all(
      urls.map((url) => cache.add(url).catch(() => null)),
    )),
  );
});

self.addEventListener('fetch', (event) => {
  const request = event.request;
  if (request.method !== 'GET') return;

  const url = new URL(request.url);
  if (url.origin !== self.location.origin) return;

  if (request.mode === 'navigate') {
    event.respondWith(
      fetch(request)
        .then((response) => {
          const copy = response.clone();
          caches.open(RUNTIME_CACHE).then((cache) => cache.put(request, copy));
          return response;
        })
        .catch(async () => (
          await caches.match(request)
          || await caches.match(scopeUrl('./'))
          || Response.error()
        )),
    );
    return;
  }

  event.respondWith(
    caches.match(request).then((cached) => {
      const network = fetch(request)
        .then((response) => {
          if (response.ok) {
            const copy = response.clone();
            caches.open(RUNTIME_CACHE).then((cache) => cache.put(request, copy));
          }
          return response;
        })
        .catch(() => cached || Response.error());
      return cached || network;
    }),
  );
});
