import { LaunchIncludedFile } from "@/renderer/models";
import { Position } from "monaco-editor";

export type ResolveType = { path: string; realpath: string };
export type ResolverCacheEntry = { start: Position; end: Position; match: IncludeMatch };

export type IncludeMatch = { value: string; offset: number; resolved: string; realpath: string };

export type IncludeResolver = {
  cache: Map<string, ResolverCacheEntry[]>;
  resolve: (currentFile: string, rawPath: string) => ResolveType | undefined;
};

export function createIncludeResolver(includedFiles: LaunchIncludedFile[]): IncludeResolver {
  const map = new Map<string, ResolveType>();

  for (const f of includedFiles) {
    map.set(`${f.path}|${f.raw_inc_path}`, { path: f.inc_path, realpath: f.inc_realpath });
  }

  return {
    cache: new Map<string, ResolverCacheEntry[]>(),
    resolve(currentFile, rawPath) {
      return map.get(`${currentFile}|${rawPath}`);
    },
  };
}

/**
 * Extracts ROS/Launch/Include paths from text
 * @param text The text to parse
 * @param language "xml", ... | "python" (used to distinguish parsing strategy if needed)
 * @returns List of matches with value and offset
 */
export function extractIncludes(
  text: string,
  language: string,
  resolver: IncludeResolver,
  currentFile: string
): IncludeMatch[] {
  const matches: IncludeMatch[] = [];

  if (language === "python") {
    // --- Handle Python IncludeLaunchDescription(...) separately ---
    const prefix = "[^#]";
    const PY_INCLUDE_REGEX = new RegExp(`${prefix}\\sIncludeLaunchDescription\\s*?\\(`, "gsm");
    for (const match of text.matchAll(PY_INCLUDE_REGEX)) {
      if (match.index == null) continue; // safe check
      const startOffset = match.index;
      const block = extractPythonInclude(text, startOffset);
      if (!block) continue;

      const resolved = resolver?.resolve(currentFile, block);
      if (!resolved) continue;

      const fileMatches = extractPythonIncludeFiles(block, startOffset, resolved);
      matches.push(...fileMatches);
    }
    return matches;
  }

  // --- Regex for file, ROS paths, pkg paths ---
  const PATH_REGEX = new RegExp(
    [
      // file="..." / textfile="..." / binfile="..."
      String.raw`(?:file|textfile|binfile)\s*=\s*"([^"]+)"`,
      // $(find pkg)/path , $(find-pkg-share pkg)/path , $(dirname)/path
      String.raw`(\$\((?:find|find-pkg-share|dirname) [^)]+\)/[^"\s>]+)`,
      // pkg://pkg/path or package://pkg/path
      String.raw`((?:pkg|package):\/\/[^"\s>]+)`,
    ].join("|"),
    "g"
  );

  for (const match of text.matchAll(PATH_REGEX)) {
    if (match.index == null) continue; // safe check
    const value = match.slice(1).find((v) => v != null);
    if (!value) continue;

    const resolved = resolver?.resolve(currentFile, value);
    if (!resolved) continue;

    const offset = match.index + match[0].indexOf(value);
    matches.push({ value, offset, resolved: resolved.path, realpath: resolved.realpath });
  }

  return matches;
}

/**
 * Extracts the complete IncludeLaunchDescription(...) block using bracket counting
 */
function extractPythonInclude(text: string, start: number): string | undefined {
  let depth = 0;
  let i = start;

  for (; i < text.length; i++) {
    const c = text[i];

    if (c === "(") depth++;
    else if (c === ")") {
      depth--;
      if (depth === 0) {
        return text.slice(start, i + 1);
      }
    }
  }

  return undefined;
}

export function extractPythonIncludeFiles(block: string, blockOffset: number, resolved: ResolveType): IncludeMatch[] {
  const matches: IncludeMatch[] = [];

  // Kommentare ignorieren
  const lines = block.split("\n").filter((l) => !l.trimStart().startsWith("#"));
  const cleanBlock = lines.join("\n");

  // Alle Strings matchen, die wie Dateien aussehen
  // Das hier ist ein einfacher heuristischer Filter: endet auf .py, .launch.py etc.
  const FILE_STRING_REGEX = /['"]([^'"]+\.(?:py|launch\.py|xml|yaml))['"]/g;

  for (const match of cleanBlock.matchAll(FILE_STRING_REGEX)) {
    if (!match[1]) continue;
    const offset = blockOffset + cleanBlock.indexOf(match[0]);
    matches.push({ value: match[1], offset, resolved: resolved.path, realpath: resolved.realpath });
  }

  return matches;
}
