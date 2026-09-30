import { useCallback, useEffect, useMemo, useRef, useState } from "react";
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

type ResolveMap = Map<string, Map<string, ResolveType>>;

export type IncludeResolver = {
  cache: Map<string, ResolverCacheEntry[]>;
  /** live value, backed by a ref -> the resolver object itself never changes identity */
  readonly includedFiles: TIncludedFile[];
  fetchIncludedFiles: () => Promise<{ result: boolean; error: string }>;
  clearIncludedFiles: () => void;
  resolve: (currentFile: string, rawPath: string, lineNumber: number, fullTextBeforeMatch?: string) => ResolveType[];
  getArgs: (currentFile: string) => ResolverIncludeArgs | undefined;
  update: (includedFiles: LaunchIncludedFile[]) => void;
  extractIncludes: (text: string, language: string, currentFile: string) => IncludeMatch[];
};

export function normalizePath(p: string): string {
  if (!p) return "";
  const isAbsolute = p.startsWith("/");
  const out: string[] = [];
  for (const part of p.replace(/\/{2,}/g, "/").split("/")) {
    if (part === "" || part === ".") continue;
    if (part === "..") out.pop();
    else out.push(part);
  }
  return (isAbsolute ? "/" : "") + out.join("/");
}

/**
 * A path is considered "resolved" only if it is absolute and free of unexpanded
 * substitutions. Editor side results are never stat'ed, so this is the best
 * available heuristic to avoid false positives in the UI.
 */
function looksResolved(p: string): boolean {
  return !!p && p.startsWith("/") && !p.includes("$(") && !p.includes("${") && !p.includes("$LaunchConfig");
}

function rawKey(f: LaunchIncludedFile): string {
  return `${normalizePath(f.path)}|${(f.raw_inc_path || "").trim()}|${f.line_number ?? -1}`;
}

/**
 * Key of the resolved target. Falls back to a unique sentinel when the target is
 * unknown, otherwise all unresolved entries of one file would collide.
 */
function resolvedKey(f: LaunchIncludedFile): string {
  const target = normalizePath(f.inc_realpath || f.inc_path || "");
  const suffix = target || `\u0000unresolved:${f.line_number ?? -1}:${(f.raw_inc_path || "").trim()}`;
  return `${normalizePath(f.path)}|${suffix}`;
}

/** normalized target of an entry, used to link children to their parent */
function targetPath(f: LaunchIncludedFile): string {
  return normalizePath(f.inc_realpath || f.inc_path || "");
}

/** cheap content comparison - avoids a state update when nothing really changed */
function sameIncludes(a: TIncludedFile[], b: TIncludedFile[]): boolean {
  if (a.length !== b.length) return false;
  for (let i = 0; i < a.length; i++) {
    const x = a[i];
    const y = b[i];
    if (
      rawKey(x) !== rawKey(y) ||
      resolvedKey(x) !== resolvedKey(y) ||
      x.exists !== y.exists ||
      x.rec_depth !== y.rec_depth ||
      x.resolver !== y.resolver
    ) {
      return false;
    }
  }
  return true;
}

/** signature used to detect real changes of the daemon part of the list */
function includesSignature(files: TIncludedFile[]): string {
  return files.map((f) => `${rawKey(f)}>${resolvedKey(f)}>${f.exists ? 1 : 0}>${f.resolver ?? ""}`).join("\n");
}

/**
 * Index of the entry that includes `child`. When the same file is included more
 * than once, the shallowest (and first) occurrence wins - this keeps the result
 * deterministic instead of depending on the array order.
 */
function findParentIndex(result: TIncludedFile[], child: TIncludedFile): number {
  const wanted = normalizePath(child.path);
  if (!wanted) return -1;
  let best = -1;
  for (let i = 0; i < result.length; i++) {
    if (targetPath(result[i]) !== wanted) continue;
    if (best === -1 || (result[i].rec_depth ?? 0) < (result[best].rec_depth ?? 0)) best = i;
  }
  return best;
}

/** first index after the whole subtree of `parentIndex` */
function subtreeEnd(result: TIncludedFile[], parentIndex: number): number {
  const parentDepth = result[parentIndex].rec_depth ?? 0;
  let i = parentIndex + 1;
  while (i < result.length && (result[i].rec_depth ?? 0) > parentDepth) i++;
  return i;
}

function mergeIncludedFiles(daemon: TIncludedFile[], editor: TIncludedFile[]): TIncludedFile[] {
  const rawKeys = new Set(daemon.map(rawKey));
  const resolvedKeys = new Set(daemon.map(resolvedKey));
  const result: TIncludedFile[] = daemon.map((f) => (f.resolver === "daemon" ? f : { ...f, resolver: "daemon" }));

  for (const e of editor) {
    const rk = rawKey(e);
    const rsk = resolvedKey(e);
    if (rawKeys.has(rk) || resolvedKeys.has(rsk)) continue;
    rawKeys.add(rk);
    resolvedKeys.add(rsk);

    // the depth must match the parent, otherwise the tree builder misplaces the item
    const parentIndex = findParentIndex(result, e);
    const insertIndex = parentIndex >= 0 ? subtreeEnd(result, parentIndex) : result.length;
    const parentDepth = parentIndex >= 0 ? (result[parentIndex].rec_depth ?? 0) : -1;
    result.splice(insertIndex, 0, { ...e, rec_depth: parentDepth + 1, resolver: "editor" });
  }
  return result;
}

export function useIncludedFiles(
  provider: Provider,
  rootFilePath: string,
  rootLaunchArgs: TLaunchArg[]
): IncludeResolver {
  const [includedFiles, setIncludedFiles] = useState<TIncludedFile[]>([]);

  const mapRef = useRef<ResolveMap>(new Map());
  const mapIncludeArgsRef = useRef<Map<string, ResolverIncludeArgs>>(new Map());
  const cacheRef = useRef<Map<string, ResolverCacheEntry[]>>(new Map());
  const rosPackagesRef = useRef<Map<string, string>>(new Map());

  // latest-ref pattern: volatile inputs must not invalidate the callbacks
  const providerRef = useRef(provider);
  const rootFilePathRef = useRef(rootFilePath);
  const rootLaunchArgsRef = useRef(rootLaunchArgs);
  const includedFilesRef = useRef(includedFiles);
  providerRef.current = provider;
  rootFilePathRef.current = rootFilePath;
  rootLaunchArgsRef.current = rootLaunchArgs;
  includedFilesRef.current = includedFiles;

  const daemonLoadedRef = useRef<boolean>(false);
  const pendingDiscoveredRef = useRef<TIncludedFile[]>([]);
  const fetchGenerationRef = useRef<number>(0);
  const inFlightRef = useRef<{ key: string; promise: Promise<{ result: boolean; error: string }> } | null>(null);
  // batching of editor-discovered includes
  const discoveredQueueRef = useRef<TIncludedFile[]>([]);
  const flushTimerRef = useRef<ReturnType<typeof setTimeout> | null>(null);
  // signature of the list the resolve maps were built from
  const updateSignatureRef = useRef<string>("");

  const setPackages = useCallback((packages: RosPackage[]): void => {
    const map = rosPackagesRef.current;
    map.clear();
    for (const p of packages || []) map.set(p.name, p.path);
  }, []);

  const set = useCallback((file: string, raw: string, value: ResolveType): void => {
    let inner = mapRef.current.get(file);
    if (!inner) {
      inner = new Map();
      mapRef.current.set(file, inner);
    }
    // daemon results always win over editor guesses
    const existing = inner.get(raw);
    if (existing?.resolver === "daemon" && value.resolver !== "daemon") return;
    inner.set(raw, value);
  }, []);

  const getArgs = useCallback((currentFile: string): ResolverIncludeArgs | undefined => {
    return mapIncludeArgsRef.current.get(currentFile);
  }, []);

  const update = useCallback(
    (files: LaunchIncludedFile[]): void => {
      const map = mapRef.current;
      const next = new Map<string, Set<string>>();
      const validArgKeys = new Set<string>([rootFilePathRef.current]);

      // process daemon entries first so that they cannot be overwritten by editor guesses
      const ordered = [...files].sort((a, b) => {
        const av = (a as TIncludedFile).resolver === "editor" ? 1 : 0;
        const bv = (b as TIncludedFile).resolver === "editor" ? 1 : 0;
        return av - bv;
      });

      for (const f of ordered) {
        if (!f.raw_inc_path) continue; // nothing to key on
        const resolver = (f as TIncludedFile).resolver === "editor" ? "editor" : "daemon";
        set(f.path, f.raw_inc_path, {
          path: f.inc_path,
          realpath: f.inc_realpath,
          exists: f.exists,
          resolver,
        });

        let s = next.get(f.path);
        if (!s) {
          s = new Set();
          next.set(f.path, s);
        }
        s.add(f.raw_inc_path);

        if (f.inc_path) {
          validArgKeys.add(f.inc_path);
          // only daemon entries carry real argument information
          if (resolver === "daemon" || !mapIncludeArgsRef.current.has(f.inc_path)) {
            mapIncludeArgsRef.current.set(f.inc_path, {
              args: f.args || [],
              defaults: f.default_inc_args || [],
              topLevel: rootLaunchArgsRef.current,
              from: f.path,
            });
          }
        }
      }

      // remove stale entries
      const dirtyFiles = new Set<string>();
      for (const [file, inner] of map) {
        const valid = next.get(file);
        for (const raw of [...inner.keys()]) {
          if (!valid?.has(raw)) {
            inner.delete(raw);
            dirtyFiles.add(file);
          }
        }
        if (inner.size === 0) map.delete(file);
      }
      for (const file of dirtyFiles) cacheRef.current.delete(file);

      // drop argument entries of files that are no longer included (prevents stale args + leak)
      for (const key of [...mapIncludeArgsRef.current.keys()]) {
        if (!validArgKeys.has(key)) mapIncludeArgsRef.current.delete(key);
      }
    },
    [set]
  );

  /** flush queued editor results in one single state update */
  const scheduleFlush = useCallback((): void => {
    if (flushTimerRef.current !== null) return;
    flushTimerRef.current = setTimeout(() => {
      flushTimerRef.current = null;
      const queue = discoveredQueueRef.current;
      discoveredQueueRef.current = [];
      if (queue.length === 0) return;

      setIncludedFiles((prev) => {
        const daemonEntries = prev.filter((f) => f.resolver !== "editor");
        const editorEntries = prev.filter((f) => f.resolver === "editor");
        const known = new Set([...prev.map(rawKey), ...prev.map(resolvedKey)]);
        let changed = false;
        for (const c of queue) {
          if (known.has(rawKey(c)) || known.has(resolvedKey(c))) continue;
          known.add(rawKey(c));
          known.add(resolvedKey(c));
          editorEntries.push(c);
          changed = true;
        }
        // keep the previous identity -> no re-render, no loop
        if (!changed) return prev;
        const merged = mergeIncludedFiles(daemonEntries, editorEntries);
        return sameIncludes(prev, merged) ? prev : merged;
      });
    }, 0);
  }, []);

  const addDiscoveredInclude = useCallback(
    (currentFile: string, rawPath: string, lineNumber: number, variant: string): void => {
      const prov = providerRef.current;
      if (!prov) return;
      // never report unresolved guesses as included files
      if (!looksResolved(variant)) return;

      const candidate: TIncludedFile = {
        host: prov.host(),
        size: -1, // unknown, file was not stat'ed by the daemon
        path: currentFile,
        raw_inc_path: rawPath,
        inc_path: variant,
        inc_realpath: variant,
        line_number: lineNumber,
        exists: true, // unverified: editor side never stats the file
        rec_depth: 0,
        args: [],
        default_inc_args: [],
        conditional_excluded: false,
        resolver: "editor",
      };

      // daemon result not yet available -> buffer it and render nothing for now
      if (!daemonLoadedRef.current) {
        if (
          !pendingDiscoveredRef.current.some(
            (f) => rawKey(f) === rawKey(candidate) || resolvedKey(f) === resolvedKey(candidate)
          )
        ) {
          pendingDiscoveredRef.current.push(candidate);
        }
        return;
      }

      // already known? -> do not even schedule an update
      if (
        includedFilesRef.current.some(
          (f) => rawKey(f) === rawKey(candidate) || resolvedKey(f) === resolvedKey(candidate)
        ) ||
        discoveredQueueRef.current.some(
          (f) => rawKey(f) === rawKey(candidate) || resolvedKey(f) === resolvedKey(candidate)
        )
      ) {
        return;
      }
      discoveredQueueRef.current.push(candidate);
      scheduleFlush();
    },
    [scheduleFlush]
  );

  const resolve = useCallback(
    (currentFile: string, rawPath: string, lineNumber: number, fullTextBeforeMatch?: string): ResolveType[] => {
      // replace ROS package expressions with actual package paths (all occurrences)
      const pkgRegex = /\$\((?:find|find-pkg-share)\s+([^)]+)\)|\$\((?:package|pkg):\/\/([^)]+)\)/g;
      const replacedPackage = rawPath.replace(pkgRegex, (match, p1, p2) => {
        const name = ((p1 ?? p2) || "").trim();
        const pkgPath = name ? rosPackagesRef.current.get(name) : undefined;
        // unknown package -> keep the expression, the result stays "not resolvable"
        return pkgPath || match;
      });

      const result: ResolveType[] = [];
      const seenPaths = new Set<string>();

      const mapped = mapRef.current.get(currentFile)?.get(rawPath);
      if (mapped) {
        result.push(mapped);
        if (mapped.path) seenPaths.add(normalizePath(mapped.path));
        if (mapped.realpath) seenPaths.add(normalizePath(mapped.realpath));
      }

      const variants = replaceAllXmlVars(replacedPackage, currentFile, getArgs(currentFile), fullTextBeforeMatch);
      for (const variant of variants) {
        if (!variant || seenPaths.has(normalizePath(variant))) continue;
        seenPaths.add(normalizePath(variant));
        result.push({ path: variant, realpath: variant, exists: looksResolved(variant), resolver: "editor" });
        addDiscoveredInclude(currentFile, rawPath, lineNumber, variant);
      }
      return result;
    },
    [getArgs, addDiscoveredInclude]
  );

  const extractIncludes = useCallback(
    (text: string, language: string, currentFile: string): IncludeMatch[] => {
      const matches: IncludeMatch[] = [];
      const lineStarts = [0];
      for (let i = 0; i < text.length; i++) if (text[i] === "\n") lineStarts.push(i + 1);

      // binary search, returns the 1-based line number
      const lineFromOffset = (offset: number): number => {
        let lo = 0;
        let hi = lineStarts.length - 1;
        while (lo <= hi) {
          const mid = (lo + hi) >> 1;
          if (lineStarts[mid] <= offset) lo = mid + 1;
          else hi = mid - 1;
        }
        return hi + 1;
      };

      if (language === "python") {
        const PY_INCLUDE_REGEX = /\bIncludeLaunchDescription\s*\(/gms;
        for (const match of text.matchAll(PY_INCLUDE_REGEX)) {
          if (match.index == null) continue;
          const startOffset = match.index;
          const lineNumber = lineFromOffset(startOffset);
          // skip commented out calls (works at line start, too)
          const prefix = text.slice(lineStarts[lineNumber - 1], startOffset);
          if (prefix.includes("#")) continue;

          const block = extractPythonInclude(text, startOffset);
          if (!block) continue;
          for (const resolved of resolve(currentFile, block, lineNumber)) {
            matches.push(...extractPythonIncludeFiles(block, startOffset, resolved));
          }
        }
        return matches;
      }

      const PATH_REGEX = new RegExp(
        [
          String.raw`(?:file|textfile|binfile)\s*=\s*"([^\n"]+)"`,
          String.raw`(\$\((?:find|find-pkg-share|dirname) [^)]+\)[^\n"]*)`,
          String.raw`((?:pkg|package):\/\/[^"]*)`,
        ].join("|"),
        "g"
      );

      for (const match of text.matchAll(PATH_REGEX)) {
        if (match.index == null) continue;
        const value = match.slice(1).find((v) => v != null);
        if (!value) continue;
        const resolves = resolve(currentFile, value, lineFromOffset(match.index), text.slice(0, match.index));
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
    },
    [resolve]
  );

  const runFetch = useCallback(
    async (generation: number, request: LaunchIncludedFilesRequest): Promise<{ result: boolean; error: string }> => {
      const prov = providerRef.current;
      try {
        const daemonFiles = await prov.launchGetIncludedFiles(request);

        // a newer request was started meanwhile -> its result wins, drop ours silently.
        // this must NOT be reported as an error, otherwise callers start a retry loop
        if (generation !== fetchGenerationRef.current) return { result: true, error: "" };

        const pending = pendingDiscoveredRef.current;
        pendingDiscoveredRef.current = [];

        if (!daemonFiles) {
          // daemon failed: release the buffered editor results as fallback
          if (pending.length > 0) {
            setIncludedFiles((prev) => {
              const merged = mergeIncludedFiles(
                prev.filter((f) => f.resolver === "daemon"),
                pending
              );
              return sameIncludes(prev, merged) ? prev : merged;
            });
          }
          return { result: false, error: `error while get included launch files from ${prov.id}` };
        }

        setIncludedFiles((prev) => {
          const merged = mergeIncludedFiles(daemonFiles as TIncludedFile[], pending);
          // identical content -> keep identity, this breaks the loadFiles/fetch loop
          return sameIncludes(prev, merged) ? prev : merged;
        });
        return { result: true, error: "" };
      } catch (error) {
        return { result: false, error: `useIncludedFiles: ${error}` };
      } finally {
        // never leave the editor path blocked, even on error or abort
        if (generation === fetchGenerationRef.current) {
          daemonLoadedRef.current = true;
          inFlightRef.current = null;
        }
      }
    },
    []
  );

  const fetchIncludedFiles = useCallback(async (): Promise<{ result: boolean; error: string }> => {
    const prov = providerRef.current;
    if (!prov) return { result: false, error: "useIncludedFiles: Provider not available" };

    const path = rootFilePathRef.current;
    if (!path) return { result: false, error: "useIncludedFiles: no root file path" };

    const launch = prov.launchFiles?.find((l) => l.path === path);
    const args =
      launch?.args?.map((t) => new LaunchArgument(t.name, t.value, t.default_value, t.description, t.choices)) || [];

    // an identical request is already running -> share the very same promise
    // (StrictMode double mount, multiple consumers, volatile effect deps)
    const key = `${prov.id}|${path}|${JSON.stringify(args.map((a) => [a.name, a.value]))}`;
    if (inFlightRef.current?.key === key) return inFlightRef.current.promise;

    const request = new LaunchIncludedFilesRequest();
    request.path = path;
    request.unique = false;
    request.recursive = true;
    request.args = args;

    const generation = ++fetchGenerationRef.current;
    // only block editor results on the very first load, not on every refresh
    if (includedFilesRef.current.length === 0) daemonLoadedRef.current = false;

    const promise = runFetch(generation, request);
    inFlightRef.current = { key, promise };
    return promise;
  }, [runFetch]);

  /**
   * Invalidates all in-flight work (pending daemon request + queued editor flush).
   * Kept as a stable callback so effect cleanups do not access `ref.current`
   * directly - that would trigger react-hooks/exhaustive-deps, although reading
   * the *latest* value is exactly what we want here.
   */
  const abortPendingWork = useCallback((): void => {
    fetchGenerationRef.current++;
    inFlightRef.current = null;
    if (flushTimerRef.current !== null) {
      clearTimeout(flushTimerRef.current);
      flushTimerRef.current = null;
    }
  }, []);

  const clearIncludedFiles = useCallback((): void => {
    abortPendingWork();
    pendingDiscoveredRef.current = [];
    discoveredQueueRef.current = [];
    daemonLoadedRef.current = false;
    mapRef.current.clear();
    cacheRef.current.clear();
    mapIncludeArgsRef.current.clear();
    updateSignatureRef.current = "";
    setIncludedFiles((prev) => (prev.length === 0 ? prev : []));
  }, [abortPendingWork]);

  // keep the top level args in sync with the props
  useEffect(() => {
    mapIncludeArgsRef.current.set(rootFilePath, {
      args: rootLaunchArgs,
      defaults: [],
      topLevel: rootLaunchArgs,
      from: "top level",
    });
  }, [rootFilePath, rootLaunchArgs]);

  useEffect(() => {
    setPackages(provider.packages);
  }, [provider, setPackages]);

  // rebuild the resolve maps only when the content really changed
  useEffect(() => {
    const signature = includesSignature(includedFiles);
    if (signature === updateSignatureRef.current) return;
    updateSignatureRef.current = signature;
    update(includedFiles);
  }, [includedFiles, update]);

  // stop pending work of an unmounted hook
  useEffect(() => abortPendingWork, [abortPendingWork]);

  useCustomEventListener(
    EVENT_PROVIDER_PACKAGES,
    (data: EventProviderRosPackages) => {
      if (data.provider.id === provider.id) setPackages(data.packages);
    },
    [provider.id, setPackages]
  );

  // stable forever: includedFiles is exposed as a getter backed by a ref
  return useMemo<IncludeResolver>(
    () => ({
      get includedFiles(): TIncludedFile[] {
        return includedFilesRef.current;
      },
      cache: cacheRef.current,
      fetchIncludedFiles,
      clearIncludedFiles,
      resolve,
      update,
      getArgs,
      extractIncludes,
    }),
    [fetchIncludedFiles, clearIncludedFiles, resolve, update, getArgs, extractIncludes]
  );
}
