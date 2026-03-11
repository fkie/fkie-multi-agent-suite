export async function resolveValue(defaultValue: string) {
  try {
    const clip = await navigator.clipboard.readText();
    const sanitized = sanitizeSnippetValue(clip);
    if (sanitized) return sanitized;
  } catch {
    /* empty */
  }

  return defaultValue;
}

function sanitizeSnippetValue(text: string): string {
  if (!text) return "";

  return text.trim().replace(/\r?\n/g, " ").replace(/"/g, "&quot;").replace(/\$/g, "\\$").replace(/}/g, "\\}");
}
