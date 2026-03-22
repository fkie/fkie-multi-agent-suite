import * as MonacoReact from "@monaco-editor/react";
import { editor, IDisposable, IMarkdownString, languages, Position } from "monaco-editor";
import editorWorker from "monaco-editor/esm/vs/editor/editor.worker?worker";
import cssWorker from "monaco-editor/esm/vs/language/css/css.worker?worker";
import htmlWorker from "monaco-editor/esm/vs/language/html/html.worker?worker";
import jsonWorker from "monaco-editor/esm/vs/language/json/json.worker?worker";
import tsWorker from "monaco-editor/esm/vs/language/typescript/ts.worker?worker";

import { IMonacoContext } from "@/renderer/context/MonacoContext";
import { IRosContext } from "@/renderer/context/RosContext";
import { getFileName, RosPackage } from "@/renderer/models";
import { createUriPath, fileFromUriPath, providerIdFromEditorId, providerIdFromUriPath } from "../utils";
import { extractIncludes, ResolverCacheEntry } from "./IncludeResolver";
import { PythonLanguage } from "./languages/PythonLaunchHighlighter";
import { createPythonLaunchProposals } from "./languages/PythonLaunchProposals";
import XmlBeautify from "./languages/XmlBeautify";
import { Ros1XmlLanguage } from "./languages/XmlLaunchHighlighter";
import { Ros2XmlLanguage } from "./languages/XmlLaunchHighlighterR2";
import { createDocumentSymbols, createXMLDependencyProposals } from "./languages/XmlLaunchProposals";
import { createDocumentSymbolsR2, createXMLDependencyProposalsR2 } from "./languages/XmlLaunchProposalsR2";

export const SUPPORTED_FILES = ["ros2xml", "ros1xml", "launch", "python", "yaml"];

export type PackagesMap = Map<string, RosPackage[]>;

type RuntimeState = {
  initialized: boolean;
  disposables: IDisposable[];
};

const runtimeState: RuntimeState = {
  initialized: false,
  disposables: [],
};

function configureWorkers(m: MonacoReact.Monaco): void {
  self.MonacoEnvironment = {
    getWorker(_: string, label: string): Worker {
      switch (label) {
        case "json":
          return new jsonWorker();
        case "css":
        case "scss":
        case "less":
          return new cssWorker();
        case "html":
        case "handlebars":
        case "razor":
          return new htmlWorker();
        case "typescript":
        case "javascript":
          return new tsWorker();
        default:
          return new editorWorker();
      }
    },
  };
  MonacoReact.loader.config({ monaco: m });
}

function formatXml(xml: string, tab = 2): string {
  return new XmlBeautify().beautify(xml, tab);
}

function initThemes(m: MonacoReact.Monaco): void {
  m.editor.defineTheme("vs-ros-light", {
    base: "vs",
    inherit: true,
    colors: {},
    rules: [
      { token: "delimiter.start", foreground: "#008000", fontStyle: "bold" },
      { token: "delimiter.end", foreground: "#008000", fontStyle: "bold" },
      { token: "tag", foreground: "#008000", fontStyle: "bold" },
      { token: "attribute.name", foreground: "#7D9029" },
      { token: "attribute.value", foreground: "#BA2121" },
      { token: "subst.key", foreground: "#009000", fontStyle: "bold" },
      { token: "subst.arg", foreground: "#BA2121", fontStyle: "bold" },
      { token: "comment", foreground: "#666666", fontStyle: "italic" },
      { token: "error-token", foreground: "#ff0000ff", fontStyle: "bold underline" },
    ],
  });

  m.editor.defineTheme("vs-ros-dark", {
    base: "vs-dark",
    inherit: true,
    colors: {},
    rules: [
      { token: "delimiter.start", foreground: "#008000", fontStyle: "bold" },
      { token: "delimiter.end", foreground: "#008000", fontStyle: "bold" },
      { token: "tag", foreground: "#008000", fontStyle: "bold" },
      { token: "attribute.name", foreground: "#7D9029" },
      { token: "subst.key", foreground: "#009000", fontStyle: "bold" },
      { token: "subst.arg", foreground: "#996633", fontStyle: "bold" },
      { token: "comment", foreground: "#999999", fontStyle: "italic" },
      { token: "error-token", foreground: "#ff0000ff", fontStyle: "bold underline" },
    ],
  });
}

function initLanguages(
  monacoCtxRef: React.MutableRefObject<IMonacoContext | null>,
  rosCtxRef: React.MutableRefObject<IRosContext>
): IDisposable[] {
  // JS/TS aggressive sync
  if (!monacoCtxRef.current) return [];
  const m = monacoCtxRef.current.monaco;
  if (!m) return [];
  const newDisposables: IDisposable[] = [];
  m.languages.typescript.javascriptDefaults.setEagerModelSync(true);
  m.languages.typescript.typescriptDefaults.setEagerModelSync(true);

  // ros2xml
  m.languages.register({ id: "ros2xml" });
  newDisposables.push(m.languages.setMonarchTokensProvider("ros2xml", Ros2XmlLanguage));
  newDisposables.push(
    m.languages.setLanguageConfiguration("ros2xml", {
      comments: { blockComment: ["<!--", "-->"] },
      autoClosingPairs: [
        { open: "<", close: ">" },
        { open: '"', close: '"' },
        { open: "'", close: "'" },
      ],
      brackets: [["<", ">"]],
      onEnterRules: [{ beforeText: />/, afterText: /<\//, action: { indentAction: 2 } }],
    })
  );
  newDisposables.push(
    m.languages.registerDocumentFormattingEditProvider("ros2xml", {
      provideDocumentFormattingEdits(model) {
        return [
          {
            range: model.getFullModelRange(),
            text: formatXml(model.getValue()),
          },
        ];
      },
    })
  );
  newDisposables.push(
    m.languages.registerHoverProvider("ros2xml", {
      provideHover(model, position) {
        const wordInfo = model.getWordAtPosition(position);
        if (!wordInfo) return null;
        if (wordInfo.word === "false" || wordInfo.word === "true") {
          return {
            range: new m.Range(position.lineNumber, wordInfo.startColumn, position.lineNumber, wordInfo.endColumn),
            contents: [{ value: "Use uppercase True/False, otherwise some eval statements may fail." }],
          };
        }
        return null;
      },
    })
  );

  // ros1xml
  m.languages.register({ id: "ros1xml" });
  newDisposables.push(m.languages.setMonarchTokensProvider("ros1xml", Ros1XmlLanguage));
  newDisposables.push(
    m.languages.setLanguageConfiguration("ros1xml", {
      comments: { blockComment: ["<!--", "-->"] },
      autoClosingPairs: [
        { open: "<", close: ">" },
        { open: '"', close: '"' },
      ],
      brackets: [["<", ">"]],
      onEnterRules: [{ beforeText: />/, afterText: /<\//, action: { indentAction: 2 } }],
    })
  );
  newDisposables.push(
    m.languages.registerDocumentFormattingEditProvider("ros1xml", {
      provideDocumentFormattingEdits(model) {
        return [
          {
            range: model.getFullModelRange(),
            text: formatXml(model.getValue()),
          },
        ];
      },
    })
  );

  // python
  m.languages.register({ id: "python" });
  newDisposables.push(m.languages.setMonarchTokensProvider("python", PythonLanguage));

  // Add symbols XML and launch files
  newDisposables.push(
    m.languages.registerDocumentSymbolProvider("ros2xml", {
      displayName: "ROS Symbols",
      provideDocumentSymbols: (model: editor.ITextModel /*, token: CancellationToken */) => {
        return createDocumentSymbolsR2(model);
      },
    })
  );
  newDisposables.push(
    m.languages.registerDocumentSymbolProvider("ros1xml", {
      displayName: "ROS Symbols",
      provideDocumentSymbols: (model: editor.ITextModel /*, token: CancellationToken */) => {
        return createDocumentSymbols(model);
      },
    })
  );

  // add proposals
  for (const e of SUPPORTED_FILES) {
    // Add Completion provider for XML and launch files
    newDisposables.push(
      m.languages.registerCompletionItemProvider(e, {
        provideCompletionItems: async (model, position) => {
          const word = model.getWordUntilPosition(position);
          const range = {
            startLineNumber: position.lineNumber,
            endLineNumber: position.lineNumber,
            startColumn: word.startColumn,
            endColumn: word.endColumn,
          };

          const lineContent = model.getLineContent(position.lineNumber);
          const providerId = providerIdFromUriPath(model.uri.path);
          const provider = rosCtxRef.current.getProviderById(providerId);
          let packages: RosPackage[] = [];
          if (provider) {
            packages = provider.packages;
          }

          switch (e) {
            case "python":
              return {
                suggestions: await createPythonLaunchProposals(m, range, model.getValue(), packages),
              };
            case "ros1xml":
              return {
                suggestions: await createXMLDependencyProposals(m, range, lineContent, packages),
              };
            case "ros2xml":
              return {
                suggestions: await createXMLDependencyProposalsR2(m, range, lineContent, packages),
              };
            default:
              return {
                suggestions: [],
              };
          }
        },
      })
    );
  }
  return newDisposables;
}

export function registerLaunchLinkProvider(monacoCtxRef: React.MutableRefObject<IMonacoContext | null>): IDisposable[] {
  const monacoCtx = monacoCtxRef.current;
  if (!monacoCtx) return [];
  if (!monacoCtx.monaco) return [];

  const newDisposables: IDisposable[] = [];

  for (const e of SUPPORTED_FILES) {
    newDisposables.push(
      monacoCtx.monaco.languages.registerLinkProvider(e, {
        provideLinks(model) {
          if (!monacoCtxRef.current) return;
          const text = model.getValue();
          const currentFile = fileFromUriPath(model.uri.path);
          const links: languages.ILink[] = [];
          const editorIds = monacoCtxRef.current.modelRegistry()?.getEditorsByModels([model]);
          const providerId = providerIdFromUriPath(model.uri.path);
          for (const editorId of editorIds || []) {
            const resolver = monacoCtxRef.current.getResolver(editorId);
            if (!resolver) {
              console.log(`no resolver for ${editorId}`);
              continue;
            }
            // TODO: Should the cache be created when the resolver is created?
            const cache: ResolverCacheEntry[] = [];
            for (const match of extractIncludes(text, e, resolver, currentFile)) {
              const start = model.getPositionAt(match.offset);
              const end = model.getPositionAt(match.offset + match.value.length);
              cache.push({ start, end, match });

              if (providerId) {
                links.push({
                  range: {
                    startLineNumber: start.lineNumber,
                    startColumn: start.column,
                    endLineNumber: end.lineNumber,
                    endColumn: end.column,
                  },
                  url: createUriPath(providerId, match.resolved),
                });
              }
            }
            resolver.cache.set(currentFile, cache);
          }

          return { links };
        },
      })
    );
  }
  return newDisposables;
}

function getVarAtPosition(model: editor.ITextModel, position: Position): string | undefined {
  const lineContent = model.getLineContent(position.lineNumber);

  // Beispiel: $(var meineVar)
  const regex = /\$\(\s*var\s+([^)]+?)\s*\)/g;
  let match: RegExpExecArray | null = regex.exec(lineContent);

  while (match !== null) {
    const fullMatch = match[0];
    const varName = match[1];

    const startColumn = match.index + 1; // Monaco-Spalten sind 1-basiert
    const endColumn = startColumn + fullMatch.length; // exklusive oder inklusive ist hier egal,
    // solange du konsistent prüfst

    if (position.column >= startColumn && position.column <= endColumn) {
      return varName.trim();
    }
    match = regex.exec(lineContent);
  }

  return undefined;
}

export function registerLaunchHoverProvider(
  monacoCtxRef: React.MutableRefObject<IMonacoContext | null>
): IDisposable[] {
  const monacoCtx = monacoCtxRef.current;
  if (!monacoCtx) return [];
  if (!monacoCtx.monaco) return [];

  const newDisposables: IDisposable[] = [];

  for (const e of SUPPORTED_FILES) {
    newDisposables.push(
      monacoCtx.monaco.languages.registerHoverProvider(e, {
        provideHover(model, position) {
          if (!monacoCtxRef.current) return;
          const currentFile = fileFromUriPath(model.uri.path);
          const editorIds = monacoCtxRef.current.modelRegistry()?.getEditorsByModels([model]);
          const result = {
            contents: [] as IMarkdownString[],
          };
          let countResolved = 0;

          for (const editorId of editorIds || []) {
            const resolver = monacoCtxRef.current.getResolver(editorId);
            if (!resolver) continue;
            const poseCache = resolver.cache.get(currentFile);
            for (const cached of poseCache || []) {
              if (position.lineNumber >= cached.start.lineNumber && position.lineNumber <= cached.end.lineNumber) {
                if (position.column >= cached.start.column && position.column <= cached.end.column) {
                  if (result.contents.length === 0) {
                    result.contents.push({ value: `**${providerIdFromUriPath(model.uri.path)}**` });
                  }
                  const existsStr = cached.match.exists ? "" : "(**missing**)";
                  result.contents.push({ value: `- Resolved: ${existsStr}\`${cached.match.resolved}\`` });
                  countResolved += 1;
                  if (cached.match.realpath && cached.match.resolved !== cached.match.realpath) {
                    result.contents.push({ value: `- Realpath: \`${cached.match.realpath}\`` });
                  }
                }
              }
            }

            // 2. Wenn wir auf $(var meineVar) stehen, Args via Resolver holen
            if (resolver) {
              const hoveredVar = getVarAtPosition(model, position);
              if (!hoveredVar) continue;
              const argVar = resolver.getArgs(currentFile);
              if (!argVar) continue;
              let arg = argVar.args.find((a) => a.name === hoveredVar);
              let from = "";
              if (arg) {
                from = getFileName(argVar.from);
              }
              if (!arg) {
                arg = argVar.topLevel.find((a) => a.name === hoveredVar);
                if (arg) {
                  from = "top level";
                }
              }
              if (!arg) {
                arg = argVar.defaults.find((a) => a.name === hoveredVar);
                if (arg) {
                  from = "default";
                }
              }
              if (arg) {
                result.contents.push({
                  value: `\`${hoveredVar}\` = **\`${String(arg.value ?? "")}\`** (${from})`,
                });
                countResolved += 1;
              }
            }
          }
          return countResolved ? result : { contents: [] };
        },
      })
    );
  }
  return newDisposables;
}

export function registerLaunchDefinitionProvider(
  monacoCtxRef: React.MutableRefObject<IMonacoContext | null>
): IDisposable[] {
  const monacoCtx = monacoCtxRef.current;
  if (!monacoCtx) return [];
  if (!monacoCtx.monaco) return [];
  const newDisposables: IDisposable[] = [];

  for (const e of SUPPORTED_FILES) {
    newDisposables.push(
      monacoCtx.monaco.languages.registerDefinitionProvider(e, {
        async provideDefinition(model, position) {
          const monacoCtx = monacoCtxRef.current;
          if (!monacoCtx?.monaco) return;
          const currentFile = fileFromUriPath(model.uri.path);
          const editorIds = monacoCtx.modelRegistry()?.getEditorsByModels([model]);
          for (const editorId of editorIds || []) {
            const poseCache = monacoCtx.getResolver(editorId)?.cache.get(currentFile);
            for (const cached of poseCache || []) {
              if (position.lineNumber >= cached.start.lineNumber && position.lineNumber <= cached.end.lineNumber) {
                if (position.column >= cached.start.column && position.column <= cached.end.column) {
                  const providerId = providerIdFromEditorId(editorId);
                  if (!providerId) return;

                  const uri = createUriPath(providerId, cached.match.resolved);
                  const m = await monacoCtx.getModel(editorId, cached.match.resolved);
                  if (!m) return;
                  return {
                    uri: monacoCtx.monaco.Uri.file(uri),
                    range: new monacoCtx.monaco.Range(1, 1, 1, 1),
                  };
                }
              }
            }
          }
          return;
        },
      })
    );
  }
  return newDisposables;
}

export function initMonacoRuntime(
  monacoCtxRef: React.MutableRefObject<IMonacoContext | null>,
  rosCtxRef: React.MutableRefObject<IRosContext>
): void {
  if (runtimeState.initialized) return;
  if (!monacoCtxRef.current) return;
  if (!monacoCtxRef.current.monaco) return;

  configureWorkers(monacoCtxRef.current.monaco);

  // await MonacoReact.loader.init(); // stellt sicher, dass monaco geladen ist

  initThemes(monacoCtxRef.current.monaco);
  runtimeState.disposables.push(...initLanguages(monacoCtxRef, rosCtxRef));
  runtimeState.disposables.push(...registerLaunchLinkProvider(monacoCtxRef));
  runtimeState.disposables.push(...registerLaunchHoverProvider(monacoCtxRef));
  runtimeState.disposables.push(...registerLaunchDefinitionProvider(monacoCtxRef));

  runtimeState.initialized = true;
}

export function disposeMonacoRuntime(): void {
  // Nur globale Disposables, NICHT Models (die gehören zur Model-Schicht)
  for (const d of runtimeState.disposables) {
    d.dispose();
  }
  runtimeState.disposables = [];
}
