import { LaunchArgument, LaunchIncludedFile, LaunchIncludedFilesRequest, RosPackage } from "@/renderer/models";
import { useEffect, useRef, useState } from "react";
import { useCustomEventListener } from "react-custom-events";

import { TLaunchArg } from "@/types";
import {
  extractPythonInclude,
  extractPythonIncludeFiles,
  IncludeMatch,
  replaceAllXmlVars,
  ResolverCacheEntry,
  ResolverIncludeArgs,
  ResolveType,
} from "../monaco/setup/resolveUtils";
import { Provider } from "../providers";
import { EVENT_PROVIDER_PACKAGES } from "../providers/eventTypes";
import { EventProviderRosPackages } from "../providers/events";

// Type alias for the nested map: currentFile -> rawPath -> resolved include info
type ResolveMap = Map<string, Map<string, ResolveType>>;

export type IncludeResolver = {
  cache: Map<string, ResolverCacheEntry[]>;
  includedFiles: LaunchIncludedFile[];
  fetchIncludedFiles: () => Promise<{ result: boolean; error: string }>;
  clearIncludedFiles: () => void;
  resolve: (currentFile: string, rawPath: string, fullTextBeforeMatch?: string) => ResolveType[];
  getArgs: (currentFile: string) => ResolverIncludeArgs | undefined;
  update: (includedFiles: LaunchIncludedFile[], packages: RosPackage[]) => void;
  extractIncludes(text: string, language: string, currentFile: string): IncludeMatch[];
};

export function useIncludedFiles(
  provider: Provider,
  rootFilePath: string,
  rootLaunchArgs: TLaunchArg[]
): IncludeResolver {
  const [includedFiles, setIncludedFiles] = useState<LaunchIncludedFile[]>([]);
  // Nested map for resolving includes
  const mapRef = useRef<ResolveMap>(new Map());
  const mapIncludeArgsRef = useRef<Map<string, ResolverIncludeArgs>>(new Map());
  const cacheRef = useRef<Map<string, ResolverCacheEntry[]>>(new Map());
  const rosPackagesRef = useRef<Map<string, string>>(new Map());

  const map = mapRef.current;
  const mapIncludeArgs = mapIncludeArgsRef.current;
  const cache = cacheRef.current;
  const rosPackages = rosPackagesRef.current;
  const topLevelArgs = rootLaunchArgs;

  function setPackages(packages: RosPackage[]) {
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
  function set(file: string, raw: string, value: ResolveType) {
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

    const launch = provider.launchFiles.find((l) => l.path === rootFilePath);
    const request = new LaunchIncludedFilesRequest();
    request.path = rootFilePath;
    request.unique = false;
    request.recursive = true;
    request.args =
      launch?.args?.map((t) => new LaunchArgument(t.name, t.value, t.default_value, t.description, t.choices)) || [];

    const includedFilesLocal = await provider.launchGetIncludedFiles(request);

    if (!includedFilesLocal)
      return { result: false, error: `error while get included launch files from ${provider.id}` };
    // // filter unique file names (in case multiple imports)
    // const uniqueIncludedFiles = [rootFilePath];
    // for (const f of includedFilesLocal) {
    //   if (!uniqueIncludedFiles.includes(f.inc_path)) uniqueIncludedFiles.push(f.inc_path);
    // }
    setIncludedFiles(includedFilesLocal);
    return { result: true, error: "" };
  }

  function clearIncludedFiles() {
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
  * @param fullTextBeforeMatch Full text before the include position (used for XML var resolution)
  * @returns                   Array of possible resolutions (each as ResolveType)

  */
  function resolve(currentFile: string, rawPath: string, fullTextBeforeMatch?: string): ResolveType[] {
    // Regex to replace ROS package expressions with actual package paths
    const pkgRegex = /\$\((?:find|find-pkg-share)\s+([^)]+)\)|\$\((?:package|pkg):\/\/([^)]+)\)/;

    const replacedPackage = rawPath.replace(pkgRegex, (_, p1, p2) => {
      const packageName = p1 || p2;
      return rosPackages.get(packageName) || "";
    });

    const result: ResolveType[] = [];
    const seenPaths = new Set<string>(); // tracks which paths have already been added

    // Check if we already have a resolved entry from the includedFiles map
    const mapped = map.get(currentFile)?.get(rawPath);
    if (mapped) {
      result.push(mapped);
      seenPaths.add(mapped.path);
    }

    // Resolve XML variables; may return multiple path variants
    const replacedVariants = replaceAllXmlVars(replacedPackage, currentFile, getArgs(currentFile), fullTextBeforeMatch);

    for (const variant of replacedVariants) {
      // Skip if this path was already added (either from mapped or from another variant)
      if (seenPaths.has(variant)) {
        continue;
      }

      result.push({
        path: variant,
        realpath: variant,
        exists: true,
        resolver: "editor",
      });
      seenPaths.add(variant);
      addDiscoveredInclude(currentFile, rawPath, variant);
    }

    return result;
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
        resolver: "daemon",
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

  /**
   * Extracts ROS/Launch/Include paths from text
   * @param text The text to parse
   * @param language "xml", ... | "python" (used to distinguish parsing strategy if needed)
   * @returns List of matches with value and offset
   */
  function extractIncludes(text: string, language: string, currentFile: string): IncludeMatch[] {
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

        const resolves = resolve(currentFile, block);
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

      const resolves = resolve(currentFile, value, fullTextBeforeMatch);
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

  function addDiscoveredInclude(currentFile: string, rawPath: string, variant: string) {
    setIncludedFiles((prev) => {
      // exists?
      const exists = prev.some((f) => f.path === currentFile && f.inc_path === variant);
      if (exists) return prev;

      // Template of the last Include file (if exists)
      const template = [...prev].reverse().find((f) => f.path === currentFile);

      const newEntry: LaunchIncludedFile = {
        ...(template ?? {
          args: [],
          default_inc_args: [],
        }),
        path: currentFile,
        raw_inc_path: rawPath,
        inc_path: variant,
        inc_realpath: variant,
        exists: true,
      };

      // insert position: after entry with path === currentFile
      let insertIndex = -1;
      for (let i = prev.length - 1; i >= 0; i--) {
        if (prev[i].path === currentFile) {
          insertIndex = i + 1;
          break;
        }
      }

      if (insertIndex === -1) {
        // no entry in currentFile found → add at the end
        return [...prev, newEntry];
      }

      const next = [...prev];
      next.splice(insertIndex, 0, newEntry);
      return next;
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
