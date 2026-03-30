import { TLaunchArg } from "@/types";
import { Position } from "monaco-editor";

export type ResolveType = { path: string; realpath: string; exists: boolean; resolver: "daemon" | "editor" };
export type ResolverCacheEntry = { start: Position; end: Position; match: IncludeMatch };
export type ResolverIncludeArgs = { args: TLaunchArg[]; defaults: TLaunchArg[]; topLevel: TLaunchArg[]; from: string };

export type IncludeMatch = {
  value: string;
  offset: number;
  resolved: string;
  realpath: string;
  exists: boolean;
  resolver: "daemon" | "editor";
};

function getAllVarNames(text: string) {
  const VAR_REGEX = /\$\(\s*var\s+([^)]+)\)/g;

  const varNames: string[] = [];
  for (const match of text.matchAll(VAR_REGEX)) {
    if (match[1]) {
      varNames.push(match[1].trim());
    }
  }
  return varNames;
}

/**
 * Resolves XML variables used in an include path.

 *
 * The function scans the XML text before the include statement (`fullTextBeforeMatch`)
 * for all <let ...> (or similar) definitions and creates one resolved path variant
 * for each found definition.

 *
 * Example:
 *   <let name="robot" value="a"/>
 *   <let name="robot" value="b"/>
 *   <include file="$(var robot)_launch.xml" />

 *
 *   => returns: ["a_launch.xml", "b_launch.xml"]

 *
 * @param incPath              Raw include path, possibly containing XML vars, e.g. "$(var robot)_launch.xml"
 * @param currentFile          Path of the current XML file (not used directly here, but kept for future extensions)
 * @param args                 Resolved include args (if you want to also consider <arg> etc.)
 * @param fullTextBeforeMatch  Full text of the file up to the position of the include
 * @returns                    Array of fully or partially resolved path variants

 */
export function replaceAllXmlVars(
  text: string,
  currentFile: string,
  includeArgs?: ResolverIncludeArgs,
  fullTextBeforeMatch?: string
): string[] {
  // We work with a set of current candidates (path variants).
  // Start with the original text as the only candidate.
  let currentResults = new Set<string>([text]);

  const allVarNames = getAllVarNames(text);
  if (!allVarNames || allVarNames.length === 0) {
    return [text];
  }

  // Process each variable name contained in the text
  for (const varName of allVarNames) {
    const nextResults = new Set<string>();

    // For every current variant, expand it by all possible values for this varName
    for (const candidate of currentResults) {
      // Collect all possible values for this varName
      type VarValueSource = { value: string; mIndex?: number };
      const possibleValues: VarValueSource[] = [];

      // 1) Try to resolve from include args: args -> topLevel -> defaults
      if (includeArgs) {
        const varValueFromArgs =
          includeArgs.args.find((a) => a.name === varName)?.value ??
          includeArgs.topLevel.find((a) => a.name === varName)?.value ??
          includeArgs.defaults.find((a) => a.name === varName)?.value;

        if (varValueFromArgs !== undefined) {
          possibleValues.push({ value: varValueFromArgs });
        }
      }

      // 2) If still no value (or we also want to consider <let> overrides),
      //    scan fullTextBeforeMatch for ALL <let ...> occurrences with this name.
      if (fullTextBeforeMatch) {
        const letRegex = new RegExp(`<let\\s+[^>]*name=["']${varName}["'][^>]*value=["']([^"']*)["'][^>]*/?>`, "g");

        let m: RegExpExecArray | null = letRegex.exec(fullTextBeforeMatch);
        while (m !== null) {
          possibleValues.push({ value: m[1], mIndex: m.index });
          m = letRegex.exec(fullTextBeforeMatch);
        }
      }

      // If we found no value at all, keep the candidate as-is (unresolved variable)
      if (possibleValues.length === 0) {
        nextResults.add(candidate);
        continue;
      }

      // Pattern for occurrences of this varName in the path, e.g. $(var foo)
      const varPattern = new RegExp(`\\$\\(\\s*var\\s+${varName}\\s*\\)`, "g");

      // For each possible value, create a new variant
      for (const { value: varValue, mIndex } of possibleValues) {
        console.log(`  VAR: ${varName}: value: ${varValue}`);
        const replaced = candidate.replace(varPattern, varValue);
        console.log(`  candidate: ${candidate}`);
        console.log(`  replaced: ${replaced}`);

        // If the result still contains other $(var ...) placeholders,
        // recursively resolve them. We limit fullTextBeforeMatch for nested
        // resolution to the text before the corresponding <let> (if we have mIndex).
        if (replaced.indexOf("$(var ") > -1) {
          console.log(`REC: index: ${mIndex}`);
          const nextFullTextBeforeMatch =
            mIndex !== undefined ? fullTextBeforeMatch?.slice(0, mIndex) : fullTextBeforeMatch;

          const recursiveVariants = replaceAllXmlVars(replaced, currentFile, includeArgs, nextFullTextBeforeMatch);

          for (const v of recursiveVariants) {
            nextResults.add(v);
          }
        } else {
          nextResults.add(replaced);
        }
      }
    }

    // Move to the next round with all expanded variants
    currentResults = nextResults;
  }

  return Array.from(currentResults);
}

/**
 * Extracts the complete IncludeLaunchDescription(...) block using bracket counting
 */
export function extractPythonInclude(text: string, start: number): string | undefined {
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
      resolver: resolved.resolver,
    });
  }

  return matches;
}
