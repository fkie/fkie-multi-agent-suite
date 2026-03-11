import { IMonacoContext } from "@/renderer/context/MonacoContext";
import { RosPackage } from "@/renderer/models";
import { EVENT_EDITOR_SELECT_RANGE, eventEditorSelectRange } from "@/renderer/pages/NodeManager/layout/events";
import * as MonacoReact from "@monaco-editor/react";
import { editor, IDisposable, languages, Uri } from "monaco-editor";
import { emitCustomEvent } from "react-custom-events";
import { createUriPath, fileFromUriPath, providerIdFromTabId, providerIdFromUriPath } from "../utils";
import { extractIncludes, IncludeResolver } from "./IncludeResolver";
import { createPythonLaunchProposals } from "./languages/PythonLaunchProposals";
import { createDocumentSymbols, createXMLDependencyProposals } from "./languages/XmlLaunchProposals";
import { createDocumentSymbolsR2, createXMLDependencyProposalsR2 } from "./languages/XmlLaunchProposalsR2";

export const SUPPORTED_FILES = ["ros2xml", "ros1xml", "launch", "python", "yaml"];

export function configureContextMenu(
  m: MonacoReact.Monaco,
  editorRef: React.MutableRefObject<editor.IStandaloneCodeEditor | undefined>,
  saveModel: (model: editor.ITextModel) => void
): IDisposable[] {
  if (!m) return [];
  if (!editorRef.current) return [];
  const newDisposables: IDisposable[] = [];
  newDisposables.push(
    editorRef.current?.addAction({
      id: "save_action",
      label: "Save",
      keybindings: [m.KeyMod.CtrlCmd | m.KeyCode.KeyS],
      precondition: undefined,
      keybindingContext: undefined,
      contextMenuGroupId: "1_modification",
      contextMenuOrder: 1.0,
      run: async (editorInstance: editor.ICodeEditor) => {
        const model = editorInstance.getModel();
        if (model) {
          saveModel(model);
        }
      },
    } as editor.IActionDescriptor)
  );
  newDisposables.push(
    editorRef.current.addAction({
      id: "toggle line comment",
      label: "Toggle line comment",
      keybindings: [m.KeyMod.CtrlCmd | m.KeyMod.Shift | m.KeyCode.Digit7],
      precondition: undefined,
      keybindingContext: undefined,
      contextMenuGroupId: "1_modification",
      contextMenuOrder: 2.0,
      run: async (editorInstance: editor.ICodeEditor) => {
        editorInstance.trigger("toggle line comment", "editor.action.commentLine", {});
      },
    })
  );
  newDisposables.push(
    editorRef.current.addAction({
      id: "command_palette",
      label: "Command Palette",
      keybindings: [m.KeyMod.CtrlCmd | m.KeyMod.Shift | m.KeyCode.KeyP],
      precondition: undefined,
      keybindingContext: undefined,
      // contextMenuGroupId: "none",
      // contextMenuOrder: 1.0,
      run: async (editorInstance: editor.ICodeEditor) => {
        editorInstance.trigger("open command palette", "editor.action.quickCommand", {});
      },
    })
  );
  return newDisposables;
}

export function configureMonacoEditor(
  m: MonacoReact.Monaco,
  editorId: string,
  isRos2: boolean = true,
  packages: RosPackage[] = []
): IDisposable[] {
  if (!m) return [];
  const newDisposables: IDisposable[] = [];

  newDisposables.push(
    m.editor.registerLinkOpener({
      open(resource: Uri): boolean | Promise<boolean> {
        emitCustomEvent(
          EVENT_EDITOR_SELECT_RANGE,
          eventEditorSelectRange(editorId, fileFromUriPath(resource.path), null)
        );
        return true;
      },
    })
  );

  // personalize launch file objects
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

    // Add symbols XML and launch files
    if (e !== "python") {
      newDisposables.push(
        m.languages.registerDocumentSymbolProvider(e, {
          displayName: "ROS Symbols",
          provideDocumentSymbols: (model: editor.ITextModel /*, token: CancellationToken */) => {
            return isRos2 ? createDocumentSymbolsR2(model) : createDocumentSymbols(model);
          },
        })
      );
    }
  }
  return newDisposables;
}

export function registerLaunchLinkProvider(m: MonacoReact.Monaco, resolver: IncludeResolver, providerId: string) {
  const newDisposables: IDisposable[] = [];

  for (const e of SUPPORTED_FILES) {
    newDisposables.push(
      m.languages.registerLinkProvider(e, {
        provideLinks(model) {
          const text = model.getValue();
          const currentFile = fileFromUriPath(model.uri.path);
          const links: languages.ILink[] = [];
          for (const match of extractIncludes(text, e, resolver, currentFile)) {
            const start = model.getPositionAt(match.offset);
            const end = model.getPositionAt(match.offset + match.value.length);

            const cache = resolver.cache.get(currentFile);
            if (cache) {
              cache.push({ start, end, match });
            } else {
              resolver.cache.set(currentFile, [{ start, end, match }]);
            }

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

          return { links };
        },
      })
    );
  }
  return newDisposables;
}

export function registerLaunchDefinitionProvider(
  monacoCtx: IMonacoContext,
  resolver: IncludeResolver,
  editorId: string
) {
  if (!monacoCtx.monaco) return [];
  const newDisposables: IDisposable[] = [];

  for (const e of SUPPORTED_FILES) {
    newDisposables.push(
      monacoCtx.monaco.languages.registerDefinitionProvider(e, {
        async provideDefinition(model, position) {
          if (!monacoCtx.monaco) return;
          const currentFile = fileFromUriPath(model.uri.path);
          const poseCache = resolver.cache.get(currentFile);
          for (const cached of poseCache || []) {
            if (position.lineNumber >= cached.start.lineNumber && position.lineNumber <= cached.end.lineNumber) {
              if (position.column >= cached.start.column && position.column <= cached.end.column) {
                const providerId = providerIdFromTabId(editorId);
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
          return;
        },
      })
    );
  }
  return newDisposables;
}

export function registerLaunchHoverProvider(m: MonacoReact.Monaco, resolver: IncludeResolver) {
  const newDisposables: IDisposable[] = [];

  for (const e of SUPPORTED_FILES) {
    newDisposables.push(
      m.languages.registerHoverProvider(e, {
        provideHover(model, position) {
          const currentFile = fileFromUriPath(model.uri.path);
          const poseCache = resolver.cache.get(currentFile);
          for (const cached of poseCache || []) {
            if (position.lineNumber >= cached.start.lineNumber && position.lineNumber <= cached.end.lineNumber) {
              if (position.column >= cached.start.column && position.column <= cached.end.column) {
                return {
                  contents: [
                    { value: `**${providerIdFromUriPath(model.uri.path)}**` },
                    { value: `Resolved: \`${cached.match.resolved}\`` },
                  ],
                };
              }
            }
          }
          return { contents: [] };
        },
      })
    );
  }
  return newDisposables;
}
