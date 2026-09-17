import { useEffect, useRef, useState } from "react";
import { useCustomEventListener } from "react-custom-events";
import { LaunchArgument, LaunchIncludedFile, LaunchIncludedFilesRequest, RosPackage } from "@/renderer/models";

import { TLaunchArg } from "@/types";
import { TIncludedFile } from "../models/TIncludedFile";
import {
  extractPythonInclude,
  extractPythonIncludeFiles,
  IncludeMatch,
  ResolverCacheEntry,
  ResolverIncludeArgs,
  ResolveType,
  replaceAllXmlVars,
} from "../monaco/setup/resolveUtils";
import { Provider } from "../providers";
import { EventProviderRosPackages } from "../providers/events";
import { EVENT_PROVIDER_PACKAGES } from "../providers/eventTypes";

// Type alias for the nested map: currentFile -> rawPath -> resolved include info
type ResolveMap = Map<string, Map<string, ResolveType>>;

export type IncludeResolver = {
  cache: Map<string, ResolverCacheEntry[]>;
  includedFiles: TIncludedFile[];
  fetchIncludedFiles: () => Promise<{ result: boolean; error: string }>;
  clearIncludedFiles: () => void;
  resolve: (currentFile: string, rawPath: string, lineNumber: number, fullTextBeforeMatch?: string) => ResolveType[];
  getArgs: (currentFile: string) => ResolverIncludeArgs | undefined;
  update: (includedFiles: LaunchIncludedFile[], packages: RosPackage[]) => void;
  extractIncludes(text: string, language: string, currentFile: string): IncludeMatch[];
};

/**
 * Normalize a path so that daemon and editor results become comparable.
 * Removes duplicated slashes, "." and resolves ".." segments.
 */
export function normalizePath(p: string): string {
  if (!p) return "";
  const isAbsolute = p.startsWith("/");
  const parts = p.replace(/\/{2,}/g, "/").split("/");
  const out: string[] = [];
  for (const part of parts) {
    if (part === "" || part === ".") continue;
    if (part === "..") {
      out.pop();
    } else {
      out.push(part);
    }
  }
  return (isAbsolute ? "/" : "") + out.join("/");
}

/** Key of the include *statement* - stable, independent of the resolution result. */
function rawKey(f: LaunchIncludedFile): string {
  return `${normalizePath(f.path)}|${(f.raw_inc_path || "").trim()}|${f.line_number ?? -1}`;
}

/** Key of the resolved file. */
function resolvedKey(f: LaunchIncludedFile): string {
  return `${normalizePath(f.path)}|${normalizePath(f.inc_realpath || f.inc_path)}`;
}

/**
 * Merge the daemon result with editor-discovered entries.
 * Daemon entries always win. Editor entries survive only if the daemon reported
 * neither the same include statement nor the same resolved file.
 */
function mergeIncludedFiles(daemon: TIncludedFile[], editor: TIncludedFile[]): TIncludedFile[] {
  const rawKeys = new Set(daemon.map(rawKey));
  const resolvedKeys = new Set(daemon.map(resolvedKey));
  const result: TIncludedFile[] = daemon.map((f) => ({ ...f, resolver: "daemon" as const }));

  for (const e of editor) {
    // skip entries which are already known by the daemon
    if (rawKeys.has(rawKey(e)) || resolvedKeys.has(resolvedKey(e))) continue;
    rawKeys.add(rawKey(e));
    resolvedKeys.add(resolvedKey(e));

    // the depth must match the parent, otherwise the tree builder misplaces the item
    const parent = result.find((f) => normalizePath(f.inc_path) === normalizePath(e.path));
    const insertIndex = parent ? result.indexOf(parent) + 1 : result.length;
    result.splice(insertIndex, 0, {
      ...e,
      rec_depth: (parent?.rec_depth ?? 0) + 1,
      resolver: "editor",
    });
  }
  return result;
}

export function useIncludedFiles(
  provider: Provider,
  rootFilePath: string,
  rootLaunchArgs: TLaunchArg[]
): IncludeResolver {
  const [includedFiles, setIncludedFiles] = useState<TIncludedFile[]>([]);
  // Nested map for resolving includes
  const mapRef = useRef<ResolveMap>(new Map());
  const mapIncludeArgsRef = useRef<Map<string, ResolverIncludeArgs>>(new Map());
  const cacheRef = useRef<Map<string, ResolverCacheEntry[]>>(new Map());
  const rosPackagesRef = useRef<Map<string, string>>(new Map());

  // true as soon as the daemon has answered at least once for the current request
  const daemonLoadedRef = useRef<boolean>(false);
  // editor results discovered before the daemon answered
  const pendingDiscoveredRef = useRef<TIncludedFile[]>([]);
  // guard against outdated daemon responses
  const fetchGenerationRef = useRef<number>(0);

  const map = mapRef.current;
  const mapIncludeArgs = mapIncludeArgsRef.current;
  const cache = cacheRef.current;
  const rosPackages = rosPackagesRef.current;
  const topLevelArgs = rootLaunchArgs;

  function setPackages(packages: RosPackage[]): void {
    rosPackages.clear();
    for (const p of packages) {
      rosPackages.set(p.name, p.path);
    }
  }

  /**
   * Helper to set a resolved include in the nested map
   * @param file - The current file path
   * @param raw - The raw include path from that file
   * @param value - The resolved include information
   */
  function set(file: string, raw: string, value: ResolveType): void {
    let inner = map.get(file);
    if (!inner) {
      // Initialize inner map if it doesn't exist
      inner = new Map();
      map.set(file, inner);
    }
    inner.set(raw, value);
  }

  useEffect(() => {
    // Initialize the map with the provided included files
    mapIncludeArgs.set(rootFilePath, {
      args: rootLaunchArgs,
      defaults: [],
      topLevel: rootLaunchArgs,
      from: "top level",
    });
    setPackages(provider.packages);
  }, []);

  useEffect(() => {
    update(includedFiles);
  }, [includedFiles]);

  async function fetchIncludedFiles(): Promise<{ result: boolean; error: string }> {
    if (!provider) {
      return { result: false, error: "useIncludedFiles: Provider not available" };
    }

    // invalidate older requests and block editor results until the answer arrives
    const generation = ++fetchGenerationRef.current;
    daemonLoadedRef.current = false;

    const launch = provider.launchFiles.find((l) => l.path === rootFilePath);
    const request = new LaunchIncludedFilesRequest();
    request.path = rootFilePath;
    request.unique = false;
    request.recursive = true;
    request.args =
      launch?.args?.map((t) => new LaunchArgument(t.name, t.value, t.default_value, t.description, t.choices)) || [];

    const includedFilesLocal = await provider.launchGetIncludedFiles(request);

    // ignore outdated responses
    if (generation !== fetchGenerationRef.current) {
      return { result: true, error: "" };
    }

    if (!includedFilesLocal) {
      // daemon failed: release the buffered editor results as fallback
      daemonLoadedRef.current = true;
      const pending = pendingDiscoveredRef.current;
      pendingDiscoveredRef.current = [];
      if (pending.length > 0) {
        setIncludedFiles((prev) =>
          mergeIncludedFiles(
            prev.filter((f) => f.resolver === "daemon"),
            pending
          )
        );
      }
      return { result: false, error: `error while get included launch files from ${provider.id}` };
    }

    daemonLoadedRef.current = true;
    const pending = pendingDiscoveredRef.current;
    pendingDiscoveredRef.current = [];
    setIncludedFiles(mergeIncludedFiles(includedFilesLocal as TIncludedFile[], pending));
    return { result: true, error: "" };
  }

  function clearIncludedFiles(): void {
    pendingDiscoveredRef.current = [];
    setIncludedFiles([]);
  }

  useCustomEventListener(EVENT_PROVIDER_PACKAGES, (data: EventProviderRosPackages) => {
    if (data.provider.id === provider.id) {
      setPackages(data.packages);
    }
  });

  /**
   * Resolves a raw include path from a given file.
   *
   * It performs:
   *  - ROS package path replacement (e.g. $(find pkg), $(find-pkg-share pkg), $(package://pkg/...))
   *  - XML variable replacement (via replaceAllXmlVars), which may return multiple variants
   *
   * @param currentFile         The file that contains the include statement
   * @param rawPath             The raw include value as it appears in the file
   * @param lineNumber          Line number of the raw path in full text
   * @param fullTextBeforeMatch Full text before the include position (used for XML var resolution)
   * @returns                   Array of possible resolutions (each as ResolveType)
   */
  function resolve(
    currentFile: string,
    rawPath: string,
    lineNumber: number,
    fullTextBeforeMatch?: string
  ): ResolveType[] {
    // Regex to replace ROS package expressions with actual package paths
    const pkgRegex = /\$\((?:find|find-pkg-share)\s+([^)]+)\)|\$\((?:package|pkg):\/\/([^)]+)\)/;

    const replacedPackage = rawPath.replace(pkgRegex, (_, p1, p2) => {
      const packageName = p1 || p2;
      return rosPackages.get(packageName) || "";
    });

    const result: ResolveType[] = [];
    // tracks which normalized paths have already been added
    const seenPaths = new Set<string>();

    // Check if we already have a resolved entry from the includedFiles map
    const mapped = map.get(currentFile)?.get(rawPath);
    if (mapped) {
      result.push(mapped);
      seenPaths.add(normalizePath(mapped.path));
      seenPaths.add(normalizePath(mapped.realpath));
    }

    // Resolve XML variables; may return multiple path variants
    const replacedVariants = replaceAllXmlVars(replacedPackage, currentFile, getArgs(currentFile), fullTextBeforeMatch);

    for (const variant of replacedVariants) {
      // Skip if this path was already added (either from mapped or from another variant)
      if (seenPaths.has(normalizePath(variant))) {
        continue;
      }

      result.push({
        path: variant,
        realpath: variant,
        exists: true,
        resolver: "editor",
      });
      seenPaths.add(normalizePath(variant));
      addDiscoveredInclude(currentFile, rawPath, lineNumber, variant);
    }

    return result;
  }

  /**
   * Update the resolver with a new set of included files
   * Adds new entries, updates existing ones, and removes stale entries
   */
  function update(includedFiles: LaunchIncludedFile[]): void {
    // Track valid rawPaths for each current file
    const next = new Map<string, Set<string>>();

    // Add or update entries in the map
    for (const f of includedFiles) {
      set(f.path, f.raw_inc_path, {
        path: f.inc_path,
        realpath: f.inc_realpath,
        exists: f.exists,
        resolver: (f as TIncludedFile).resolver === "editor" ? "editor" : "daemon",
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

  function getArgs(currentFile: string): ResolverIncludeArgs | undefined {
    return mapIncludeArgs.get(currentFile);
  }

  function getLineFromOffset(lineStarts: number[], offset: number): number {
    // binary search for O(log n)
    let lo = 0;
    let hi = lineStarts.length - 1;
    while (lo <= hi) {
      const mid = (lo + hi) >> 1;
      if (lineStarts[mid] <= offset) {
        lo = mid + 1;
      } else {
        hi = mid - 1;
      }
    }
    return hi + 1; // 0-based line number
  }

  /**
   * Extracts ROS/Launch/Include paths from text
   * @param text The text to parse
   * @param language "xml", ... | "python" (used to distinguish parsing strategy if needed)
   * @returns List of matches with value and offset
   */
  function extractIncludes(text: string, language: string, currentFile: string): IncludeMatch[] {
    const matches: IncludeMatch[] = [];

    const lineStarts = [0];
    for (let i = 0; i < text.length; i++) {
      if (text[i] === "\n") {
        lineStarts.push(i + 1);
      }
    }

    if (language === "python") {
      // --- Handle Python IncludeLaunchDescription(...) separately ---
      const prefix = "[^#]";
      const PY_INCLUDE_REGEX = new RegExp(`${prefix}\\sIncludeLaunchDescription\\s*?\\(`, "gsm");
      for (const match of text.matchAll(PY_INCLUDE_REGEX)) {
        if (match.index == null) continue; // safe check
        const startOffset = match.index;
        const block = extractPythonInclude(text, startOffset);
        if (!block) continue;
        const resolves = resolve(currentFile, block, getLineFromOffset(lineStarts, match.index));
        for (const resolved of resolves) {
          const fileMatches = extractPythonIncludeFiles(block, startOffset, resolved);
          matches.push(...fileMatches);
        }
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
      const fullTextBeforeMatch = text.slice(0, match.index);
      const value = match.slice(1).find((v) => v != null);
      if (!value) continue;
      const resolves = resolve(currentFile, value, getLineFromOffset(lineStarts, match.index), fullTextBeforeMatch);
      const offset = match.index + match[0].indexOf(value);
      for (const resolved of resolves) {
        matches.push({
          value,
          offset,
          resolved: resolved.path,
          realpath: resolved.realpath,
          exists: resolved.exists,
          resolver: resolved.resolver,
        });
      }
    }

    return matches;
  }

  /**
   * Register an include which was discovered by the editor itself.
   * Before the daemon answered the entry is only buffered, so no duplicates appear.
   */
  function addDiscoveredInclude(currentFile: string, rawPath: string, lineNumber: number, variant: string): void {
    const candidate: TIncludedFile = {
      host: provider.host(),
      size: -1, // unknown, file was not stat'ed by the daemon
      path: currentFile,
      raw_inc_path: rawPath,
      inc_path: variant,
      inc_realpath: variant,
      line_number: lineNumber,
      exists: true,
      rec_depth: 0,
      args: [],
      default_inc_args: [],
      conditional_excluded: false,
      resolver: "editor",
    };

    // daemon result not yet available -> buffer it and render nothing for now
    if (!daemonLoadedRef.current) {
      if (!pendingDiscoveredRef.current.some((f) => rawKey(f) === rawKey(candidate))) {
        pendingDiscoveredRef.current.push(candidate);
      }
      return;
    }

    setIncludedFiles((prev) => {
      // already known (same statement or same resolved file)?
      if (prev.some((f) => rawKey(f) === rawKey(candidate) || resolvedKey(f) === resolvedKey(candidate))) {
        return prev;
      }
      const daemonEntries = prev.filter((f) => f.resolver !== "editor");
      const editorEntries = [...prev.filter((f) => f.resolver === "editor"), candidate];
      return mergeIncludedFiles(daemonEntries, editorEntries);
    });
  }

  // Return the resolver object
  return {
    includedFiles,
    fetchIncludedFiles,
    clearIncludedFiles,
    cache,
    resolve,
    update,
    getArgs,
    extractIncludes,
  };
}
