/* eslint-disable no-template-curly-in-string */

import { Monaco } from "@monaco-editor/react";
import { editor, languages } from "monaco-editor/esm/vs/editor/editor.api";

import { RosPackage } from "@/renderer/models";
import { TFileRange } from "@/types";

export function createXMLDependencyProposalsR2(
  monaco: Monaco,
  range: TFileRange,
  lineContent: string,
  clipText: string,
  packages: RosPackage[]
): languages.CompletionItem[] {
  // returning a static list of proposals, valid for ROS launch and XML  files

  // TODO: Unused function. Can it be removed?
  function getParamName(defaultValue: string | undefined): string | undefined {
    const text = clipText?.replace(/(\r\n.*|\n.*|\r.*)/gm, "");
    return text || defaultValue;
  }

  // List of suggestions
  return [
    ...createAttributeSuggestionsFromTag(monaco, range, lineContent),
    ...createTagSuggestions(monaco, range),
    ...createPackageList(packages, monaco, range)
  ];
}

function createPackageList(packages: RosPackage[], monaco: Monaco, range: TFileRange): languages.CompletionItem[] {
  const result = packages?.map((item: RosPackage) => {
    console.log(`create package : ${item.name}`);
    return {
      label: `${item.name}`,
      kind: monaco.languages.CompletionItemKind.Field,
      insertText: `${item.name}`,
      insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range,
    };
  });
  return result ? result : [];
}

function createAttributeSuggestionsFromTag(monaco: Monaco, range: TFileRange, lineContent: string): languages.CompletionItem[] {
  const tags= ["launch", "include", "group", "let", "arg", "executable", "node", "param", "remap", "env", "set_env", "unset_env", "push-ros-namespace"]

  for (const tag in tags) {
    // if a tag is found, add its attributes as suggestions
    if (lineContent.includes("<" + tags[tag])) return createAttributeSuggestions(monaco, range, tags[tag])
  }
  // otherwise return nothing
  return [];
}

function createAttributeSuggestions(monaco: Monaco, range: TFileRange, tag: string): languages.CompletionItem[] {
  // return the valid attributes for each tag
  const all = [
    {
      label: "if",
      preselect: true,
      kind: monaco.languages.CompletionItemKind.Function,
      documentation: "TODO",
      insertText: 'if="${arg ${1:VALUE}}"',
      insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range,
    },
    {
      label: "unless",
      preselect: true,
      kind: monaco.languages.CompletionItemKind.Function,
      documentation: "TODO",
      insertText: 'unless="${arg ${1:VALUE}}"',
      insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range,
    },
  ];

  const name = {
    label: "name",
    preselect: true,
    kind: monaco.languages.CompletionItemKind.Function,
    documentation: "TODO",
    insertText: 'name="${1:NAME}"',
    insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
    range,
  };

  const value = {
    label: "value",
    preselect: true,
    kind: monaco.languages.CompletionItemKind.Function,
    documentation: "TODO",
    insertText: 'value="${1:VALUE}"',
    insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
    range,
  };

  const _default = {
    label: "default",
    preselect: true,
    kind: monaco.languages.CompletionItemKind.Function,
    documentation: "TODO",
    insertText: 'default="${1:DEFAULT}"',
    insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
    range,
  };

  const description = {
    label: "description",
    preselect: true,
    kind: monaco.languages.CompletionItemKind.Function,
    documentation: "TODO",
    insertText: 'description="${1:TEXT}"',
    insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
    range,
  };

  const args = {
    label: "args",
    preselect: true,
    kind: monaco.languages.CompletionItemKind.Function,
    documentation: "TODO",
    insertText: 'args="${1:ARGUMENTS}"',
    insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
    range,
  };

  const launch_prefix = {
    label: "launch-prefix",
    preselect: true,
    kind: monaco.languages.CompletionItemKind.Function,
    documentation: "TODO",
    insertText: 'launch-prefix="${1:VALUE}"',
    insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
    range,
  };

  const output = {
    label: "output",
    preselect: true,
    kind: monaco.languages.CompletionItemKind.Function,
    documentation: "TODO",
    insertText: 'output="${1:VALUE}"',
    insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
    range,
  };

  switch(tag) {
    case "launch":
      return [
        ...all,
        {
          label: "version",
          preselect: true,
          kind: monaco.languages.CompletionItemKind.Function,
          documentation: "TODO",
          insertText: 'version="${1:XML_VERSION}"',
          insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          range,
        },
      ];
    case "include":
      return [
        ...all,
        {
          label: "file",
          preselect: true,
          kind: monaco.languages.CompletionItemKind.Function,
          documentation: "Path to the launch file to be included.",
          insertText: 'file="$(find ${1:PACKAGE})${2:FILE_PATH}"',
          insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          range,
        },
      ];
    case "arg":
      return [
        ...all,
        name,
        value,
        _default,
        description,
      ]
    case "let":
      return [
        ...all,
        name,
        value,
      ];
    case "group":
      return [
        ...all,
        {
          label: "scoped",
          preselect: true,
          kind: monaco.languages.CompletionItemKind.Function,
          documentation: "TODO",
          insertText: 'scoped="${1:VALUE}"',
          insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          range,
        }
      ];
    case "env":
      return [
        ...all,
        name,
        value,
      ];
    case "set_env":
      return [
        ...all,
        name,
        value,
      ];
    case "unset_env":
      return [
        ...all,
        value,
      ];
    case "executable":
      return [
        ...all,
        name,
        args,
        launch_prefix,
        output,
        {
          label: "cwd",
          preselect: true,
          kind: monaco.languages.CompletionItemKind.Function,
          documentation: "TODO",
          insertText: 'cwd="${1:WORKING_DIRECTORY}"',
          insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          range,
        },
        {
          label: "cmd",
          preselect: true,
          kind: monaco.languages.CompletionItemKind.Function,
          documentation: "TODO",
          insertText: 'cmd="${1:VALUE}"',
          insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          range,
        },
        {
          label: "shell",
          preselect: true,
          kind: monaco.languages.CompletionItemKind.Function,
          documentation: "TODO",
          insertText: 'shell="${1:USE_SHELL}"',
          insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          range,
        },
      ];
    case "param":
      return [
        ...all,
        name,
        value,
        {
          label: "from",
          preselect: true,
          kind: monaco.languages.CompletionItemKind.Function,
          documentation: "TODO",
          insertText: 'from="${1:TOPIC_NAME}"',
          insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          range,
        },
        {
          label: "sep",
          preselect: true,
          kind: monaco.languages.CompletionItemKind.Function,
          documentation: "TODO",
          insertText: 'sep="${1:SEPARATOR}"',
          insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          range,
        },
      ];
    case "remap":
      return [
        ...all,
        {
          label: "from",
          preselect: true,
          kind: monaco.languages.CompletionItemKind.Function,
          documentation: "TODO",
          insertText: 'from="${1:TOPIC_NAME}"',
          insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          range,
        },
        {
          label: "to",
          preselect: true,
          kind: monaco.languages.CompletionItemKind.Function,
          documentation: "TODO",
          insertText: 'to="${1:TOPIC_NAME}"',
          insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          range,
        },
      ];
    case "node":
      return [
        ...all,
        name,
        args,
        launch_prefix,
        output,
        {
          label: "pkg",
          preselect: true,
          kind: monaco.languages.CompletionItemKind.Function,
          documentation: "TODO",
          insertText: 'pkg="${1:PACKAGE}"',
          insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          range,
        },
        {
          label: "exec",
          preselect: true,
          kind: monaco.languages.CompletionItemKind.Function,
          documentation: "TODO",
          insertText: 'exec="${1:NODE_EXECUTABLE_NAME}"',
          insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          range,
        },
        {
          label: "ros_args",
          preselect: true,
          kind: monaco.languages.CompletionItemKind.Function,
          documentation: "TODO",
          insertText: 'ros_args="${1:ARGUMENTS}"',
          insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          range,
        },
        {
          label: "namespace",
          preselect: true,
          kind: monaco.languages.CompletionItemKind.Function,
          documentation: "TODO",
          insertText: 'namespace="${1:NAMESPACE}"',
          insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          range,
        },
      ];
    case "push-ros-namespace":
      return [
        ...all,
        {
          label: "namespace",
          preselect: true,
          kind: monaco.languages.CompletionItemKind.Function,
          documentation: "TODO",
          insertText: 'namespace="${1:NAMESPACE}"',
          insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          range,
        },
      ];
    default: return [];
  }
}

function createTagSuggestions(monaco: Monaco, range: TFileRange) {
  return [
    {
      label: "node",
      kind: monaco.languages.CompletionItemKind.Function,
      documentation: "Add a new ROS node",
      insertText: 'node name="${1:NAME}" pkg="${2:PACKAGE}" exec="${3:TYPE}"></node>',
      insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range,
    },
    {
      label: "executable",
      kind: monaco.languages.CompletionItemKind.Function,
      documentation: "Add a new executable",
      insertText: 'node cmd="ls" cwd="/home" launch-prefix="time" output="screen"/>',
      insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range,
    },
    {
      label: "param",
      kind: monaco.languages.CompletionItemKind.Function,
      documentation: "Add a new ROS parameter",
      insertText: 'param name="\${1:${getParamName("NAME")}}" value="\${2:VALUE}" />',
      insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range,
    },
    {
      label: "arg value",
      kind: monaco.languages.CompletionItemKind.Function,
      documentation: "Add a new ROS argument with value",
      insertText: 'arg name="${1:NAME}" value="${2:VALUE}" />',
      insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range,
    },
    {
      label: "arg default",
      kind: monaco.languages.CompletionItemKind.Function,
      documentation: "Add a new ROS argument with default",
      insertText: 'arg name="${1:NAME}" default="${2:DEFAULT}" />',
      insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range,
    },
    {
      label: "include",
      kind: monaco.languages.CompletionItemKind.Function,
      documentation: "Add a new include statement",
      insertText: 'include file="$(find ${1:PACKAGE})/launch/${2:FILE}"></include>',
      insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range,
    },
    {
      label: "group",
      kind: monaco.languages.CompletionItemKind.Function,
      documentation: "Add a new group statement",
      insertText: 'group></group>',
      range,
    },
    {
      label: "set_env",
      kind: monaco.languages.CompletionItemKind.Function,
      documentation: "Allows for modifying an OS process environment",
      insertText: '<set_env name="RMW_IMPLEMENTATION" value="rmw_fastrtps_cpp"/>',
      range,
    },
    {
      label: "unset_env",
      kind: monaco.languages.CompletionItemKind.Function,
      documentation: "Allows for deleting an OS process environment variable",
      insertText: '<unset_env name="MY_ENV_VAR"/>',
      range,
    },
    {
      label: "remap",
      kind: monaco.languages.CompletionItemKind.Function,
      documentation: "Add a new remap statement",
      insertText: 'remap from="${1:FROM}" to="${2:TO}"/>',
      insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range,
    },

    {
      label: "GDB Launch Prefix",
      kind: monaco.languages.CompletionItemKind.Function,
      documentation: "GDB Launch Prefix",
      insertText: 'launch-prefix="gdb -ex run -ex bt -batch --args"',
      range,
    },

    {
      label: "nm/associations",
      kind: monaco.languages.CompletionItemKind.Function,
      documentation: "Associated ROS-Nodes are started before the node itself and stopped after the node.",
      insertText: 'param name="nm/associations" value="NODE1,NODE2" />',
      range,
    },
    {
      label: "nm/kill_on_stop",
      kind: monaco.languages.CompletionItemKind.Function,
      documentation: "Kill the node after defined time in milliseconds",
      insertText: 'param name="nm/kill_on_stop" value="300" />',
      range,
    },
    {
      label: "find-pkg-prefix",
      kind: monaco.languages.CompletionItemKind.Function,
      documentation:
        "Substituted by the install prefix path of the given package. Forward and backwards slashes will be resolved to the local filesystem convention. Substitution will fail if the package cannot be found.",
      insertText: "find-pkg-prefix ${1:<pkg-name>})",
      insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range,
    },
    {
      label: "find-pkg-share",
      kind: monaco.languages.CompletionItemKind.Function,
      documentation:
        "Substituted by the share directory path of the given package. The share directory includes the package’s name, e.g. <prefix>/share/<pkg-name>. Forward and backwards slashes will be resolved to the local filesystem convention. Substitution will fail if the package cannot be found.",
      insertText: "find-pkg-share ${1:<pkg-name>})",
      insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range,
    },
    {
      label: "find-exec",
      kind: monaco.languages.CompletionItemKind.Function,
      documentation:
        "Substituted by the path to the executable in the local filesystem. Executables are looked up in the PATH environment variable. Forward and backwards slashes will be resolved to the local filesystem convention. Substitution will fail if the executable cannot be found.",
      insertText: "find-exec ${1:<exec-name>})",
      insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range,
    },
    {
      label: "exec-in-package",
      kind: monaco.languages.CompletionItemKind.Function,
      documentation:
        "Substituted by the path to the executable in the local filesystem. Executables are looked up in the lib directory of the package. Substitution will fail if the executable cannot be found.",
      insertText: "exec-in-package ${1:<exec-name>} ${2:<package-name>})",
      insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range,
    },
    {
      label: "var",
      kind: monaco.languages.CompletionItemKind.Function,
      documentation:
        "Substituted by the value of the launch configuration variable. Substitution will fail if the named argument does not exist.",
      insertText: "var ${1:<name>})",
      insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range,
    },
    {
      label: "env",
      kind: monaco.languages.CompletionItemKind.Function,
      documentation:
        "Substituted by the value of the given environment variable Substitution will fail if the variable is not set, unless a default value is provided.",
      insertText: "env ${1:<env-var>} ${2:[default-value]})",
      insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range,
    },
    {
      label: "eval",
      kind: monaco.languages.CompletionItemKind.Function,
      documentation:
        "Substituted by the evaluated python expression. Substitution will fail if python fails to evaluate the expression.",
      insertText: "eval ${1:<python-expression>})",
      insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range,
    },
    {
      label: "dirname",
      kind: monaco.languages.CompletionItemKind.Function,
      documentation: "Substituted by the current launch file directory name. Substitution will always succeed.",
      insertText: "dirname)",
      insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range,
    },
  ];
}

export function createDocumentSymbolsR2(model: editor.ITextModel /*, token*/): languages.DocumentSymbol[] {
  const parser = new DOMParser();

  const symbolList: languages.DocumentSymbol[] = [];

  for (let lineNumber = 1; lineNumber <= model.getLineCount(); lineNumber += 1) {
    const lineContent = model.getLineContent(lineNumber);

    if (lineContent.length > 0) {
      const xml = parser.parseFromString(lineContent, "text/xml");

      // Add nodes as symbols
      const nodeElement = xml.querySelector("node");
      const nodeName = nodeElement?.getAttribute("name");
      if (nodeName) {
        const packageName = nodeElement?.getAttribute("pkg");
        const typeName = nodeElement?.getAttribute("exec");
        symbolList.push({
          range: {
            startLineNumber: lineNumber,
            startColumn: 1,
            endLineNumber: lineNumber,
            endColumn: 1,
          },
          name: nodeName,
          kind: 5,
          detail: "",
          containerName: `(${typeName}) [${packageName}]`,
          tags: [],
          selectionRange: {
            startLineNumber: lineNumber,
            startColumn: 1,
            endLineNumber: lineNumber,
            endColumn: 1,
          },
        });
      }

      // Add parameters as symbols
      const parameterElement = xml.querySelector("param");
      const parameterName = parameterElement?.getAttribute("name");
      if (parameterName) {
        const parameterValue = parameterElement?.getAttribute("value");
        symbolList.push({
          range: {
            startLineNumber: lineNumber,
            startColumn: 1,
            endLineNumber: lineNumber,
            endColumn: 1,
          },
          name: parameterName,
          kind: 6,
          detail: "",
          containerName: `[${parameterValue}]`,
          tags: [],
          selectionRange: {
            startLineNumber: lineNumber,
            startColumn: 1,
            endLineNumber: lineNumber,
            endColumn: 1,
          },
        });
      }

      // Add arguments as symbols
      const argumentElement = xml.querySelector("arg");
      const argumentName = argumentElement?.getAttribute("name");
      if (argumentName) {
        const argumentValue = argumentElement?.getAttribute("value");
        const defaultValue = argumentElement?.getAttribute("default");
        symbolList.push({
          range: {
            startLineNumber: lineNumber,
            startColumn: 1,
            endLineNumber: lineNumber,
            endColumn: 1,
          },
          name: argumentName,
          kind: 9,
          detail: "",
          containerName: argumentValue ? `[${argumentValue}]` : `[${defaultValue}]`,
          tags: [],
          selectionRange: {
            startLineNumber: lineNumber,
            startColumn: 1,
            endLineNumber: lineNumber,
            endColumn: 1,
          },
        });
      }
    }
  }

  return symbolList;
}
