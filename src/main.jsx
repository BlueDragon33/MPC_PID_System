import React from 'react';
import ReactDOM from 'react-dom/client';
import App from './App.jsx';
import './styles.css';

ReactDOM.createRoot(document.getElementById('root')).render(
  <React.StrictMode>
    <App />
  </React.StrictMode>,
);

async function registerOfflineShell() {
  if (!import.meta.env.PROD || !('serviceWorker' in navigator)) return;
  try {
    await navigator.serviceWorker.register('./sw.js');
    const registration = await navigator.serviceWorker.ready;
    const urls = [
      window.location.href,
      ...performance.getEntriesByType('resource').map((entry) => entry.name),
    ].filter((value) => {
      try {
        return new URL(value, window.location.href).origin === window.location.origin;
      } catch {
        return false;
      }
    });
    registration.active?.postMessage({ type: 'CACHE_URLS', urls });
  } catch {
    // Offline support is additive. Failure must never block the control workbench.
  }
}

window.addEventListener('load', registerOfflineShell, { once: true });
