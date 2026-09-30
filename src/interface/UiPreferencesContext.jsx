import React from 'react';
import { translateMessage } from './messages.js';
import {
  DEFAULT_UI_PREFERENCES,
  normalizeUiPreferences,
  readUiPreferences,
  resolveUiStorage,
  writeUiPreferences,
} from './uiPreferences.js';

const UiPreferencesContext = React.createContext(null);

export function preferenceDomState(value) {
  const preferences = normalizeUiPreferences(value);
  return {
    lang: preferences.language.split('-')[0],
    fontSize: String(preferences.fontSize),
    background: preferences.background,
    theme: preferences.background === 'dark' ? 'dark' : 'light',
    fontFamily: preferences.fontFamily,
    contrast: preferences.contrast,
  };
}

export function nextBackground(background) {
  return background === 'dark' ? 'light' : 'dark';
}

export function UiPreferencesProvider({ children, initialPreferences = null }) {
  const [preferences, setPreferences] = React.useState(() => {
    if (initialPreferences) return normalizeUiPreferences(initialPreferences);
    if (typeof window === 'undefined') return { ...DEFAULT_UI_PREFERENCES };
    return readUiPreferences(resolveUiStorage(window));
  });

  const setPreference = React.useCallback((key, value) => {
    setPreferences((current) => normalizeUiPreferences({ ...current, [key]: value }));
  }, []);

  const toggleBackground = React.useCallback(() => {
    setPreferences((current) => normalizeUiPreferences({
      ...current,
      background: nextBackground(current.background),
    }));
  }, []);

  const t = React.useCallback(
    (key) => translateMessage(key, preferences.language),
    [preferences.language],
  );

  React.useEffect(() => {
    const root = document.documentElement;
    const domState = preferenceDomState(preferences);
    root.lang = domState.lang;
    root.dataset.fontSize = domState.fontSize;
    root.dataset.background = domState.background;
    root.dataset.theme = domState.theme;
    root.dataset.fontFamily = domState.fontFamily;
    root.dataset.contrast = domState.contrast;
    writeUiPreferences(resolveUiStorage(window), preferences);
  }, [preferences]);

  const value = React.useMemo(() => ({
    preferences,
    setPreference,
    toggleBackground,
    t,
  }), [preferences, setPreference, toggleBackground, t]);

  return <UiPreferencesContext.Provider value={value}>{children}</UiPreferencesContext.Provider>;
}

export function useUiPreferences() {
  const context = React.useContext(UiPreferencesContext);
  if (!context) throw new Error('useUiPreferences must be used inside UiPreferencesProvider');
  return context;
}
