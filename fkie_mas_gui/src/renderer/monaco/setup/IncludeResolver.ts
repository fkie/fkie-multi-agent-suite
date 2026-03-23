import { LaunchIncludedFile } from "@/renderer/models";
import { TLaunchArg } from "@/types";
import { Position } from "monaco-editor";

export type ResolveType = { path: string; realpath: string; exists: boolean };
export type ResolverCacheEntry = { start: Position; end: Position; match: IncludeMatch };
export type ResolverIncludeArgs = { args: TLaunchArg[]; defaults: TLaunchArg[]; topLevel: TLaunchArg[]; from: string };

export type IncludeMatch = { value: string; offset: number; resolved: string; realpath: string; exists: boolean };

export type IncludeResolver = {
  cache: Map<string, ResolverCacheEntry[]>;
  resolve: (currentFile: string, rawPath: string) => ResolveType | undefined;
  getArgs: (currentFile: string) => ResolverIncludeArgs | undefined;
  update: (includedFiles: LaunchIncludedFile[]) => void;
};

// Type alias for the nested map: currentFile -> rawPath -> resolved include info
type ResolveMap = Map<string, Map<string, ResolveType>>;

export function createIncludeResolver(
  rootPath: string,
  launchArgs: TLaunchArg[],
  includedFiles: LaunchIncludedFile[]
): IncludeResolver {
  // Nested map for resolving includes
  const map: ResolveMap = new Map();
  const mapIncludeArgs: Map<string, ResolverIncludeArgs> = new Map();
  const cache = new Map<string, ResolverCacheEntry[]>();
  const topLevelArgs = launchArgs;

  /**
   * Helper to set a resolved include in the nested map
   * @param file - The current file path
   * @param raw - The raw include path from that file
   * @param value - The resolved include information
   */
  function set(file: string, raw: string, value: ResolveType) {
    let inner = map.get(file);
    if (!inner) {
      // Initialize inner map if it doesn't exist
      inner = new Map();
      map.set(file, inner);
    }
    inner.set(raw, value);
  }

  // Initialize the map with the provided included files
  mapIncludeArgs.set(rootPath, { args: launchArgs, defaults: [], topLevel: launchArgs, from: "top level" });
  for (const f of includedFiles) {
    set(f.path, f.raw_inc_path, {
      path: f.inc_path,
      realpath: f.inc_realpath,
      exists: f.exists,
    });
    mapIncludeArgs.set(f.inc_path, {
      args: f.args || [],
      defaults: f.default_inc_args || [],
      topLevel: topLevelArgs,
      from: f.path,
    });
  }

  /**
   * Resolve a raw include path from a given file
   * @param currentFile - The file doing the include
   * @param rawPath - The raw include path
   * @returns ResolveType or undefined if not found
   */
  function resolve(currentFile: string, rawPath: string) {
    return map.get(currentFile)?.get(rawPath);
  }

  /**
   * Update the resolver with a new set of included files
   * Adds new entries, updates existing ones, and removes stale entries
   */
  function update(includedFiles: LaunchIncludedFile[]) {
    // Track valid rawPaths for each current file
    const next = new Map<string, Set<string>>();

    // Add or update entries in the map
    for (const f of includedFiles) {
      set(f.path, f.raw_inc_path, {
        path: f.inc_path,
        realpath: f.inc_realpath,
        exists: f.exists,
      });

      // Record which raw paths should remain
      let s = next.get(f.path);
      if (!s) {
        s = new Set();
        next.set(f.path, s);
      }
      s.add(f.raw_inc_path);

      // update include args
      mapIncludeArgs.set(f.inc_path, {
        args: f.args || [],
        defaults: f.default_inc_args || [],
        topLevel: topLevelArgs,
        from: f.path,
      });
    }

    // Remove stale entries from the map and cache
    for (const [file, inner] of map) {
      const valid = next.get(file);

      // Delete raw paths that are no longer present
      for (const raw of inner.keys()) {
        if (!valid?.has(raw)) {
          inner.delete(raw);
          // Remove corresponding cache entries
          cache.delete(file);
        }
      }

      // Remove outer map entry if empty
      if (inner.size === 0) {
        map.delete(file);
      }
    }
  }

  function getArgs(currentFile: string) {
    return mapIncludeArgs.get(currentFile);
  }

  // Return the resolver object
  return { cache, resolve, update, getArgs };
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
      String.raw`(?:file|textfile|binfile)\s*=\s*"([^\n"]+)"`,
      // $(find pkg)/path , $(find-pkg-share pkg)/path , $(dirname)/path
      String.raw`(\$\((?:find|find-pkg-share|dirname) [^)]+\)[^\n"]*)`,
      // pkg://pkg/path or package://pkg/path
      String.raw`((?:pkg|package):\/\/[^"]*)`,
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
    matches.push({ value, offset, resolved: resolved.path, realpath: resolved.realpath, exists: resolved.exists });
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
    matches.push({
      value: match[1],
      offset,
      resolved: resolved.path,
      realpath: resolved.realpath,
      exists: resolved.exists,
    });
  }

  return matches;
}
