import assert from 'node:assert/strict';

import {
  BACKGROUND_OPTIONS,
  CONTRAST_OPTIONS,
  DEFAULT_UI_PREFERENCES,
  FONT_FAMILY_OPTIONS,
  FONT_SIZE_OPTIONS,
  LANGUAGE_OPTIONS,
  UI_PREFERENCES_STORAGE_KEY,
  normalizeUiPreferences,
  readUiPreferences,
  resolveUiStorage,
  writeUiPreferences,
} from '../src/interface/uiPreferences.js';
import { translateMessage } from '../src/interface/messages.js';

assert.deepEqual(LANGUAGE_OPTIONS.map(({ value }) => value), [
  'en',
  'ru',
  'vi',
  'en-ru',
  'vi-ru',
  'en-vi',
]);
assert.deepEqual(FONT_SIZE_OPTIONS.map(({ value }) => value), [14, 16, 18]);
assert.deepEqual(BACKGROUND_OPTIONS.map(({ value }) => value), ['dark', 'light', 'soft']);
assert.deepEqual(FONT_FAMILY_OPTIONS.map(({ value }) => value), ['inter', 'system', 'serif']);
assert.deepEqual(CONTRAST_OPTIONS.map(({ value }) => value), ['standard', 'high']);
assert.deepEqual(DEFAULT_UI_PREFERENCES, {
  language: 'en',
  fontSize: 16,
  background: 'dark',
  fontFamily: 'inter',
  contrast: 'standard',
});

assert.deepEqual(normalizeUiPreferences({
  language: 'vi-ru',
  fontSize: 18,
  background: 'soft',
  fontFamily: 'serif',
  contrast: 'high',
}), {
  language: 'vi-ru',
  fontSize: 18,
  background: 'soft',
  fontFamily: 'serif',
  contrast: 'high',
});
assert.deepEqual(normalizeUiPreferences({
  language: 'unknown',
  fontSize: 99,
  background: 'neon',
  fontFamily: 'comic-sans',
  contrast: 'extreme',
}), DEFAULT_UI_PREFERENCES, 'invalid enum values must recover independently to defaults');
assert.deepEqual(normalizeUiPreferences(null), DEFAULT_UI_PREFERENCES);

const memory = new Map();
const storage = {
  getItem(key) { return memory.get(key) ?? null; },
  setItem(key, value) { memory.set(key, value); },
};
assert.deepEqual(readUiPreferences(storage), DEFAULT_UI_PREFERENCES);
memory.set(UI_PREFERENCES_STORAGE_KEY, '{not-json');
assert.deepEqual(readUiPreferences(storage), DEFAULT_UI_PREFERENCES, 'corrupt JSON must not break startup');

const saved = writeUiPreferences(storage, {
  language: 'ru',
  fontSize: 14,
  background: 'light',
  fontFamily: 'system',
  contrast: 'high',
});
assert.deepEqual(saved, {
  language: 'ru',
  fontSize: 14,
  background: 'light',
  fontFamily: 'system',
  contrast: 'high',
});
assert.deepEqual(readUiPreferences(storage), saved);
assert.deepEqual(writeUiPreferences(null, saved), saved, 'unavailable storage must be harmless');
assert.equal(resolveUiStorage({ get localStorage() { throw new DOMException('Storage disabled', 'SecurityError'); } }), null, 'a throwing storage getter must not crash startup');
assert.equal(resolveUiStorage(null), null);
assert.equal(resolveUiStorage({ localStorage: storage }), storage);

assert.equal(translateMessage('nav.settings', 'en'), 'Settings');
assert.equal(translateMessage('nav.settings', 'ru'), 'Настройки');
assert.equal(translateMessage('nav.settings', 'vi'), 'Cài đặt');
assert.equal(translateMessage('nav.settings', 'en-ru'), 'Settings · Настройки');
assert.equal(translateMessage('nav.settings', 'vi-ru'), 'Cài đặt · Настройки');
assert.equal(translateMessage('nav.settings', 'en-vi'), 'Settings · Cài đặt');
assert.equal(translateMessage('technical.pid', 'vi-ru'), 'PID', 'identical technical strings must not be duplicated');
assert.equal(translateMessage('unknown.key', 'vi'), 'unknown.key', 'unknown keys must fall back deterministically');
assert.equal(translateMessage('settings.option.soft', 'en-ru'), 'Soft gray · Мягкий серый');
assert.equal(translateMessage('settings.enableKalman', 'vi'), 'Bật ước lượng Kalman');
assert.equal(translateMessage('docs.stage.estimation.title', 'ru'), 'Оценка состояния');

console.log('UI preferences contract: PASS');
