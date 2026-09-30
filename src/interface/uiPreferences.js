export const UI_PREFERENCES_STORAGE_KEY = 'mpc-pid-ui-preferences-v1';

export const LANGUAGE_OPTIONS = Object.freeze([
  { value: 'en', label: 'English' },
  { value: 'ru', label: 'Русский' },
  { value: 'vi', label: 'Tiếng Việt' },
  { value: 'en-ru', label: 'English · Русский' },
  { value: 'vi-ru', label: 'Tiếng Việt · Русский' },
  { value: 'en-vi', label: 'English · Tiếng Việt' },
]);

export const FONT_SIZE_OPTIONS = Object.freeze([
  { value: 14, label: '14 px' },
  { value: 16, label: '16 px' },
  { value: 18, label: '18 px' },
]);

export const BACKGROUND_OPTIONS = Object.freeze([
  { value: 'dark', label: 'Dark', labelKey: 'theme.dark' },
  { value: 'light', label: 'Light', labelKey: 'theme.light' },
  { value: 'soft', label: 'Soft gray', labelKey: 'settings.option.soft' },
]);

export const FONT_FAMILY_OPTIONS = Object.freeze([
  { value: 'inter', label: 'Inter' },
  { value: 'system', label: 'System', labelKey: 'settings.option.system' },
  { value: 'serif', label: 'Serif', labelKey: 'settings.option.serif' },
]);

export const CONTRAST_OPTIONS = Object.freeze([
  { value: 'standard', label: 'Standard', labelKey: 'settings.option.standard' },
  { value: 'high', label: 'High', labelKey: 'settings.option.high' },
]);

export const DEFAULT_UI_PREFERENCES = Object.freeze({
  language: 'en',
  fontSize: 16,
  background: 'dark',
  fontFamily: 'inter',
  contrast: 'standard',
});

const allowedValues = {
  language: new Set(LANGUAGE_OPTIONS.map(({ value }) => value)),
  fontSize: new Set(FONT_SIZE_OPTIONS.map(({ value }) => value)),
  background: new Set(BACKGROUND_OPTIONS.map(({ value }) => value)),
  fontFamily: new Set(FONT_FAMILY_OPTIONS.map(({ value }) => value)),
  contrast: new Set(CONTRAST_OPTIONS.map(({ value }) => value)),
};

export function normalizeUiPreferences(value) {
  const candidate = value && typeof value === 'object' ? value : {};
  return Object.fromEntries(Object.entries(DEFAULT_UI_PREFERENCES).map(([key, fallback]) => [
    key,
    allowedValues[key].has(candidate[key]) ? candidate[key] : fallback,
  ]));
}

export function readUiPreferences(storage) {
  if (!storage?.getItem) return { ...DEFAULT_UI_PREFERENCES };
  try {
    const serialized = storage.getItem(UI_PREFERENCES_STORAGE_KEY);
    return serialized ? normalizeUiPreferences(JSON.parse(serialized)) : { ...DEFAULT_UI_PREFERENCES };
  } catch {
    return { ...DEFAULT_UI_PREFERENCES };
  }
}

export function resolveUiStorage(windowLike) {
  try {
    return windowLike?.localStorage ?? null;
  } catch {
    return null;
  }
}

export function writeUiPreferences(storage, value) {
  const normalized = normalizeUiPreferences(value);
  try {
    storage?.setItem?.(UI_PREFERENCES_STORAGE_KEY, JSON.stringify(normalized));
  } catch {
    // Preferences are progressive enhancement; storage failures must not block the workbench.
  }
  return normalized;
}
