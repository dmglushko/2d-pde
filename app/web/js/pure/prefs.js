// The viewer's preferences (settings, folded sections) and probe slots.
// Pure.

const isObject = (value) => value !== null && typeof value === "object" && !Array.isArray(value);

// The app's preferences (settings.js) and their defaults.
export const SETTINGS_DEFAULTS = Object.freeze({
  checkWhileTyping: true,
  checkDelayMs: 350,
  playbackFps: 12,
  followRun: true,
  digits: 4,
  pixelRatioCap: 2,
  isolineCount: 10,
});

const SETTING_RANGES = {
  checkDelayMs: [100, 3000],
  playbackFps: [1, 60],
  digits: [2, 10],
  pixelRatioCap: [1, 4],
  isolineCount: [2, 40],
};

// Settings as stored, made whole: a missing or malformed value takes its
// default, a number out of range is clamped, unknown keys are dropped.
export function normalizeSettings(input) {
  const out = { ...SETTINGS_DEFAULTS };
  if (!isObject(input)) return out;
  for (const [key, fallback] of Object.entries(SETTINGS_DEFAULTS)) {
    const value = input[key];
    if (typeof fallback === "boolean") {
      if (typeof value === "boolean") out[key] = value;
    } else if (typeof value === "number" && Number.isFinite(value)) {
      const [low, high] = SETTING_RANGES[key];
      const clamped = Math.min(high, Math.max(low, value));
      out[key] = key === "pixelRatioCap" ? clamped : Math.round(clamped);
    }
  }
  return out;
}

// The folded sections, as stored: {id: true (folded) or false (open)}.
export function parseFolds(text) {
  try {
    const value = JSON.parse(text ?? "null");
    if (!isObject(value)) return Object.create(null);
    const out = Object.create(null);
    for (const [key, folded] of Object.entries(value)) if (typeof folded === "boolean") out[key] = folded;
    return out;
  } catch {
    return Object.create(null);
  }
}

// Probes pinned on the picture keep their slot (and so their colour)
// while others come and go: the lowest free slot of `max`, or -1.
export const MAX_PROBES = 4;
export function freeProbeSlot(probes, max = MAX_PROBES) {
  const taken = new Set(probes.map((p) => p.slot));
  for (let slot = 0; slot < max; slot++) if (!taken.has(slot)) return slot;
  return -1;
}

// The picture's height at phone width, as the viewer dragged it: at least
// 160 px and at most 70 % of the window's height; null for a stored value
// that is not a number (the default height then).
export function clampPictureHeight(height, windowHeight) {
  if (!Number.isFinite(height)) return null;
  return Math.round(Math.max(160, Math.min(height, 0.7 * windowHeight)));
}
