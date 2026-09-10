const MAX_LENGTH = 200;

export async function resolveValue(defaultValue: string) {
  try {
    const clip = await navigator.clipboard.readText();
    const sanitized = sanitizeSnippetValue(clip.slice(0, MAX_LENGTH));
    if (sanitized) return sanitized;
  } catch {
    // clipboard unavailable or permission denied
  }

  return defaultValue;
}

function sanitizeSnippetValue(text: string): string {
  if (!text) return "";

  return text
    .trim()
    .replace(/\r?\n/g, " ")
    // Monaco snippet syntax: escape backslash first, then metacharacters
    .replace(/[\\$}]/g, (ch) => `\\${ch}`);
}
