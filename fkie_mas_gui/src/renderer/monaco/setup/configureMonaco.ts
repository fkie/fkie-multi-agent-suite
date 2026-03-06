import * as MonacoReact from "@monaco-editor/react";
import { IDisposable } from "monaco-editor";
import editorWorker from "monaco-editor/esm/vs/editor/editor.worker?worker";
import cssWorker from "monaco-editor/esm/vs/language/css/css.worker?worker";
import htmlWorker from "monaco-editor/esm/vs/language/html/html.worker?worker";
import jsonWorker from "monaco-editor/esm/vs/language/json/json.worker?worker";
import tsWorker from "monaco-editor/esm/vs/language/typescript/ts.worker?worker";

import { PythonLanguage } from "./languages/PythonLaunchHighlighter";
import XmlBeautify from "./languages/XmlBeautify";
import { Ros1XmlLanguage } from "./languages/XmlLaunchHighlighter";
import { Ros2XmlLanguage } from "./languages/XmlLaunchHighlighterR2";

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

function initLanguages(m: MonacoReact.Monaco): void {
  // JS/TS aggressive sync
  m.languages.typescript.javascriptDefaults.setEagerModelSync(true);
  m.languages.typescript.typescriptDefaults.setEagerModelSync(true);

  // ros2xml
  m.languages.register({ id: "ros2xml" });
  m.languages.setMonarchTokensProvider("ros2xml", Ros2XmlLanguage);
  m.languages.setLanguageConfiguration("ros2xml", {
    comments: { blockComment: ["<!--", "-->"] },
    autoClosingPairs: [
      { open: "<", close: ">" },
      { open: '"', close: '"' },
      { open: "'", close: "'" },
    ],
    brackets: [["<", ">"]],
    onEnterRules: [{ beforeText: />/, afterText: /<\//, action: { indentAction: 2 } }],
  });
  m.languages.registerDocumentFormattingEditProvider("ros2xml", {
    provideDocumentFormattingEdits(model) {
      return [
        {
          range: model.getFullModelRange(),
          text: formatXml(model.getValue()),
        },
      ];
    },
  });
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
  });

  // ros1xml
  m.languages.register({ id: "ros1xml" });
  m.languages.setMonarchTokensProvider("ros1xml", Ros1XmlLanguage);
  m.languages.setLanguageConfiguration("ros1xml", {
    comments: { blockComment: ["<!--", "-->"] },
    autoClosingPairs: [
      { open: "<", close: ">" },
      { open: '"', close: '"' },
    ],
    brackets: [["<", ">"]],
    onEnterRules: [{ beforeText: />/, afterText: /<\//, action: { indentAction: 2 } }],
  });
  m.languages.registerDocumentFormattingEditProvider("ros1xml", {
    provideDocumentFormattingEdits(model) {
      return [
        {
          range: model.getFullModelRange(),
          text: formatXml(model.getValue()),
        },
      ];
    },
  });

  // python
  m.languages.register({ id: "python" });
  m.languages.setMonarchTokensProvider("python", PythonLanguage);
}

export function initMonacoRuntime(monaco: MonacoReact.Monaco): void {
  if (runtimeState.initialized) return;

  configureWorkers(monaco);

  // await MonacoReact.loader.init(); // stellt sicher, dass monaco geladen ist

  initThemes(monaco);
  initLanguages(monaco);

  runtimeState.initialized = true;
}

export function disposeMonacoRuntime(): void {
  // Nur globale Disposables, NICHT Models (die gehören zur Model-Schicht)
  for (const d of runtimeState.disposables) {
    d.dispose();
  }
  runtimeState.disposables = [];
}
