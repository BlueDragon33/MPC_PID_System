import assert from 'node:assert/strict';
import React from 'react';
import { renderToStaticMarkup } from 'react-dom/server';
import { createServer } from 'vite';

const server = await createServer({
  root: process.cwd(),
  server: { middlewareMode: true },
  appType: 'custom',
  logLevel: 'error',
});

try {
  const [{ UiPreferencesProvider, preferenceDomState, nextBackground }, { default: ControlSidebar }, workspace, workbench] = await Promise.all([
    server.ssrLoadModule('/src/interface/UiPreferencesContext.jsx'),
    server.ssrLoadModule('/src/components/app/ControlSidebar.jsx'),
    server.ssrLoadModule('/src/components/app/WorkspaceViews.jsx'),
    server.ssrLoadModule('/src/application/workbench.js'),
  ]);
  const { SettingsView, DocumentationView } = workspace;
  const noop = () => {};

  const renderWithPreferences = (element, initialPreferences) => renderToStaticMarkup(
    React.createElement(UiPreferencesProvider, { initialPreferences }, element),
  );

  const settingsHtml = renderWithPreferences(React.createElement(SettingsView, {
    draftCfg: workbench.defaultConfig,
    setDraftCfg: noop,
  }), {
    language: 'en-ru',
    fontSize: 16,
    background: 'dark',
    fontFamily: 'inter',
    contrast: 'standard',
  });

  const optionCount = (preference) => {
    const select = settingsHtml.match(new RegExp(`<select[^>]*data-preference="${preference}"[^>]*>([\\s\\S]*?)<\\/select>`));
    assert.ok(select, `missing ${preference} Settings control`);
    return [...select[1].matchAll(/<option/g)].length;
  };
  assert.equal(optionCount('language'), 6, 'Settings must expose exactly six language modes');
  assert.equal(optionCount('fontSize'), 3, 'Settings must expose 14, 16, and 18 px');
  assert.equal(optionCount('background'), 3, 'Settings must expose dark, light, and soft backgrounds');
  assert.equal(optionCount('fontFamily'), 3, 'Settings must expose Inter, System, and Serif');
  assert.equal(optionCount('contrast'), 2, 'Settings must expose standard and high contrast');
  assert.match(settingsHtml, /Interface preferences · Настройки интерфейса/);
  assert.match(settingsHtml, /Language · Язык/);
  assert.match(settingsHtml, /Soft gray · Мягкий серый/, 'appearance option labels must be localized');
  assert.match(settingsHtml, /Enable Kalman estimation · Включить оценивание Калмана/, 'existing Settings copy must be localized');

  const sidebarHtml = renderWithPreferences(React.createElement(ControlSidebar, {
    draftCfg: workbench.defaultConfig,
    setDraftCfg: noop,
    activeMode: 'HYBRID_SAFE',
    setActiveMode: noop,
    presetId: 'nominal',
    setPresetId: noop,
    onRun: noop,
    onReset: noop,
    runRevision: 0,
  }), {
    language: 'vi',
    fontSize: 16,
    background: 'dark',
    fontFamily: 'inter',
    contrast: 'standard',
  });
  assert.match(sidebarHtml, /Cấu hình điều khiển/);
  assert.match(sidebarHtml, /Chạy mô phỏng/);

  const docsHtml = renderWithPreferences(React.createElement(DocumentationView), {
    language: 'ru',
    fontSize: 16,
    background: 'dark',
    fontFamily: 'inter',
    contrast: 'standard',
  });
  assert.match(docsHtml, /Архитектура управления/);
  assert.match(docsHtml, /Оценка состояния/);
  assert.match(docsHtml, /Датчик → Калман/, 'documentation body copy must be localized');

  assert.deepEqual(preferenceDomState({
    language: 'en-ru',
    fontSize: 18,
    background: 'soft',
    fontFamily: 'serif',
    contrast: 'high',
  }), {
    lang: 'en',
    fontSize: '18',
    background: 'soft',
    theme: 'light',
    fontFamily: 'serif',
    contrast: 'high',
  });
  assert.equal(nextBackground('dark'), 'light');
  assert.equal(nextBackground('light'), 'dark');
  assert.equal(nextBackground('soft'), 'dark');

  console.log('UI preferences integration: PASS');
} finally {
  await server.close();
}
