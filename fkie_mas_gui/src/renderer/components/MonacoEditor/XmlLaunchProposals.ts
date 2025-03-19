/* eslint-disable no-template-curly-in-string */

import { Monaco } from "@monaco-editor/react";
import { editor, languages } from "monaco-editor/esm/vs/editor/editor.api";

import { RosPackage } from "@/renderer/models";
import { TFileRange } from "@/types";

function createXMLDependencyProposals(
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
    ...createPackageList(packages, monaco, range),
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
  const tags= ["launch", "node", "machine", "remap", "env", "param", "rosparam", "group", "test", "arg", "include"]

  for (const tag in tags) {
    // if a tag is found, add its attributes as suggestions
    if (lineContent.includes("<" + tags[tag])) return createAttributeSuggestions(monaco, range, tags[tag])
  }
  // otherwise return nothing
  return [];
}

// TODO: this is a pretty long function, maybe sort it into a different file?
function createAttributeSuggestions(monaco: Monaco, range: TFileRange, tag: string): languages.CompletionItem[] {
  // return the valid attributes for each tag
  const all = [
    {
      label: "if",
      preselect: true,
      kind: monaco.languages.CompletionItemKind.Function,
      documentation: "If value evaluates to true, include tag and its contents.",
      insertText: 'if="${arg ${1:VALUE}}"',
      insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range,
    },
    {
      label: "unless",
      preselect: true,
      kind: monaco.languages.CompletionItemKind.Function,
      documentation: "Unless value evaluates to true (which means if value evaluates to false), include tag and its contents.",
      insertText: 'unless="${arg ${1:VALUE}}"',
      insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range,
    },
  ];

  const name = {
    label: "name",
    preselect: true,
    kind: monaco.languages.CompletionItemKind.Function,
    documentation: "Assigned name",
    insertText: 'name="${1:NAME}"',
    insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
    range,
  };

  const ns = {
    label: "ns",
    preselect: true,
    kind: monaco.languages.CompletionItemKind.Function,
    documentation: "Assigned namespace",
    insertText: 'ns="${1:NAMESPACE}"',
    insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
    range,
  };

  const value = {
    label: "value",
    preselect: true,
    kind: monaco.languages.CompletionItemKind.Function,
    documentation: "Value to set. Not combineable with default",
    insertText: 'value="${1:VALUE}"',
    insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
    range,
  };

  const required = {
    label: "required",
    preselect: true,
    kind: monaco.languages.CompletionItemKind.Function,
    documentation: "If node dies, kill entire roslaunch",
    insertText: 'required="${1:VALUE}"',
    insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
    range,
  };

  const type = {
    label: "type",
    preselect: true,
    kind: monaco.languages.CompletionItemKind.Function,
    documentation: "Node type. There must be a corresponding executable with the same name",
    insertText: 'type="${1:NODETYPE}"',
    insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
    range,
  };

  const pkg = {
    label: "pkg",
    preselect: true,
    kind: monaco.languages.CompletionItemKind.Function,
    documentation: "Package of node",
    insertText: 'pkg="${1:PACKAGE}"',
    insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
    range,
  };

  const clear_params = {
    label: "clear_params",
    preselect: true,
    kind: monaco.languages.CompletionItemKind.Function,
    documentation: "Delete all parameters before launch",
    insertText: 'clear_params="${1:VALUE}"',
    insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
    range,
  };

  const command = {
    label: "command",
    preselect: true,
    kind: monaco.languages.CompletionItemKind.Function,
    documentation: "The output of the command will be read and stored as a string",
    insertText: 'command="$(find ${1:PACKAGE_NAME})${2:FILE_PATH}"',
    insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
    range,
  };

  const cwd = {
    label: "cwd",
    preselect: true,
    kind: monaco.languages.CompletionItemKind.Function,
    documentation: "If 'node', the working directory of the node will be set to the same directory as the node's executable",
    insertText: 'cwd="${1:WORKING_DIRECTORY}"',
    insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
    range,
  };

  const args = {
    label: "args",
    preselect: true,
    kind: monaco.languages.CompletionItemKind.Function,
    documentation: "Pass arguments to node",
    insertText: 'args="${1:ARGUMENTS}"',
    insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
    range,
  };

  const launch_prefix = {
    label: "launch-prefix",
    preselect: true,
    kind: monaco.languages.CompletionItemKind.Function,
    documentation: "Command/arguments to prepend to node's launch arguments",
    insertText: 'launch-prefix="${1:VALUE}"',
    insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
    range,
  };

  switch(tag) {
    case "launch":
      return [
        ...all,
        {
          label: "depricated",
          preselect: true,
          kind: monaco.languages.CompletionItemKind.Function,
          documentation: "Add a deprication message to the launch configuration",
          insertText: 'depricated="${1:MESSAGE}"',
          insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          range,
        },
      ];
    case "node": 
      return [
        ...all,
        name,
        ns,
        required,
        pkg,
        clear_params,
        cwd,
        args,
        launch_prefix,
        {
          label: "machine",
          preselect: true,
          kind: monaco.languages.CompletionItemKind.Function,
          documentation: "Launch node on designated machine",
          insertText: 'machine="${1:MACHINE_NAME}"',
          insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          range,
        },
        {
          label: "respawn",
          preselect: true,
          kind: monaco.languages.CompletionItemKind.Function,
          documentation: "Restart the node automatically if it quits",
          insertText: 'respawn="${1:SHOULD_RESPAWN}"',
          insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          range,
        },
        {
          label: "respawn_delay",
          preselect: true,
          kind: monaco.languages.CompletionItemKind.Function,
          documentation: "If respawn is true, wait respawn_delay seconds after the node failure is detected before attempting restart",
          insertText: 'respawn_delay="${1:SECONDS}"',
          insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          range,
        },
        {
          label: "output",
          preselect: true,
          kind: monaco.languages.CompletionItemKind.Function,
          documentation: "If 'screen', stdout/stderr from the node will be sent to the screen. If 'log', the stdout/stderr output will be sent to a log file in $ROS_HOME/log, and stderr will continue to be sent to screen. The default is 'log'",
          insertText: 'output="${1:VALUE}"',
          insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          range,
        },
      ];
    case "machine":
      return [
        ...all,
        name,
        {
          label: "default",
          preselect: true,
          kind: monaco.languages.CompletionItemKind.Function,
          documentation: "Sets this machine as the default to assign nodes to. The default setting only applies to nodes defined later in the same scope. NOTE: if there are no default machines, the local machine is used. You can prevent a machine from being chosen by setting default='never', in which case the machine can only be explicitly assigned",
          insertText: 'default="${1:DEFAULT}"',
          insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          range,
        },
        {
          label: "address",
          preselect: true,
          kind: monaco.languages.CompletionItemKind.Function,
          documentation: "Network address/hostname of machine",
          insertText: 'address="${1:ADDRESS}"',
          insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          range,
        },
        {
          label: "env-loader",
          preselect: true,
          kind: monaco.languages.CompletionItemKind.Function,
          documentation: "Specify environment file on remote machine. Environment file must be a shell script that sets all required environment variables, then runs exec on the provided arguments",
          insertText: 'env-loader="${1:FILEPATH}"',
          insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          range,
        },
        {
          label: "user",
          preselect: true,
          kind: monaco.languages.CompletionItemKind.Function,
          documentation: "SSH user name for logging into machine",
          insertText: 'user="${1:USERNAME}"',
          insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          range,
        },
        {
          label: "password",
          preselect: true,
          kind: monaco.languages.CompletionItemKind.Function,
          documentation: "SSH password. It is highly recommended that you configure SSH keys and SSH agent instead so you can login with certificates instead",
          insertText: 'password="${1:PASSWORD}"',
          insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          range,
        },
        {
          label: "timeout",
          preselect: true,
          kind: monaco.languages.CompletionItemKind.Function,
          documentation: "Number of seconds before a roslaunch on this machine is considered as having failed to launch",
          insertText: 'timeout="${1:SECONDS}"',
          insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          range,
        },
        {
          label: "ros-root",
          preselect: true,
          kind: monaco.languages.CompletionItemKind.Function,
          documentation: "ROS_ROOT for machine. Defaults to the ROS_ROOT set in the local environment",
          insertText: 'ros-root="${1:PATH}"',
          insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          range,
        },
        {
          label: "ros-package-path",
          preselect: true,
          kind: monaco.languages.CompletionItemKind.Function,
          documentation: "ROS_PACKAGE_PATH for machine. Defaults to the ROS_PACKAGE_PATH set in the local environment",
          insertText: 'ros-package-path="${1:PATH}"',
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
          documentation: "Remapped topic: name of the ROS topic that you are remapping FROM",
          insertText: 'from="${1:TOPIC_NAME}"',
          insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          range,
        },
        {
          label: "to",
          preselect: true,
          kind: monaco.languages.CompletionItemKind.Function,
          documentation: "Target name: name of the ROS topic that you are pointing the from topic TO",
          insertText: 'to="${1:TOPIC_NAME}"',
          insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          range,
        },
      ];
    case "env":
      return [
        ...all,
        name,
        value,
      ];
    case "param":
      return [
        ...all,
        name,
        value,
        type,
        command,
        {
          label: "textfile",
          preselect: true,
          kind: monaco.languages.CompletionItemKind.Function,
          documentation: "The contents of the file will be read and stored as a string. The file must be locally accessible",
          insertText: 'textfile="$(find ${1:PACKAGE_NAME})${2:FILE_PATH}"',
          insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          range,
        },
        {
          label: "binfile",
          preselect: true,
          kind: monaco.languages.CompletionItemKind.Function,
          documentation: "The contents of the file will be read and stored as a base64-encoded XML-RPC binary object. The file must be locally accessible",
          insertText: 'binfile="$(find ${1:PACKAGE_NAME})${2:FILE_PATH}"',
          insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          range,
        },
      ];
    case "rosparam":
      return [
        ...all,
        ns,
        command,
        {
          label: "file",
          preselect: true,
          kind: monaco.languages.CompletionItemKind.Function,
          documentation: "Name of rosparam file",
          insertText: 'file="$(find ${1:PACKAGE_NAME})${2:FILE_PATH}"',
          insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          range,
        },
        {
          label: "param",
          preselect: true,
          kind: monaco.languages.CompletionItemKind.Function,
          documentation: "Name of parameter",
          insertText: 'param="${1:PARAM_NAME}"',
          insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          range,
        },
        {
          label: "subst_value",
          preselect: true,
          kind: monaco.languages.CompletionItemKind.Function,
          documentation: "Allows use of substitution args in the YAML text",
          insertText: 'subst_value="${1:VALUE}"',
          insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          range,
        },
      ];
    case "group":
      return [
        ...all,
        ns,
        clear_params,
      ];
    case "test":
      return [
        ...all,
        name,
        ns,
        required,
        type,
        pkg,
        clear_params,
        cwd,
        args,
        launch_prefix,
        {
          label: "test-name",
          preselect: true,
          kind: monaco.languages.CompletionItemKind.Function,
          documentation: "Name of test for recording in test results",
          insertText: 'test-name="${1:TEST_NAME}"',
          insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          range,
        },
        {
          label: "retry",
          preselect: true,
          kind: monaco.languages.CompletionItemKind.Function,
          documentation: "How often the test is attempted",
          insertText: 'retry="${1:ATTEMPTS}"',
          insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          range,
        },
        {
          label: "time-limit",
          preselect: true,
          kind: monaco.languages.CompletionItemKind.Function,
          documentation: "Number of seconds before the test is considered a failure. Default is 60 seconds. The time limit is reset after every retry",
          insertText: 'time-liimit="${1:SECONDS}"',
          insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          range,
        },
      ];
    case "arg":
      return [
        ...all,
        name,
        value,
        {
          label: "default",
          preselect: true,
          kind: monaco.languages.CompletionItemKind.Function,
          documentation: "Default value of argument. Cannot be combined with value attribute",
          insertText: 'default="${1:DEFAULT}"',
          insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          range,
        },
        {
          label: "doc",
          preselect: true,
          kind: monaco.languages.CompletionItemKind.Function,
          documentation: "Description of the argument. You could get this through --ros-args argument to the roslaunch command",
          insertText: 'doc="${1:DESCRIPTION}"',
          insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          range,
        },
      ];
    case "include":
      return [
        ...all,
        ns,
        clear_params,
        {
          label: "file",
          preselect: true,
          kind: monaco.languages.CompletionItemKind.Function,
          documentation: "Name of file to include",
          insertText: 'file="$(find ${1:PACKAGE})${2:FILE_PATH}"',
          insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          range,
        },
        {
          label: "pass_all_args",
          preselect: true,
          kind: monaco.languages.CompletionItemKind.Function,
          documentation: "If true, then all args set in the current context are added to the child context that is created for processing the included file",
          insertText: 'pass_all_args="${1:VALUE}"',
          insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          range,
        },
      ]

    // or return nothing (which should never happen)
    default: return [];
  }
}

function createTagSuggestions(monaco: Monaco, range: TFileRange) {
  return [
    {
      label: "launch",
      kind: monaco.languages.CompletionItemKind.Function,
      documentation: "Create a new launch configuration",
      insertText: 'launch>\n  ${1:}\n</launch>',
      insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range,
    },
    {
      label: "node",
      kind: monaco.languages.CompletionItemKind.Function,
      documentation: "Add a new ROS node",
      insertText: 'node name="${1:NAME}" pkg="${2:PACKAGE}" type="${3:TYPE}" />\n  \n</node>',
      insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range,
    },
    {
      label: "machine",
      kind: monaco.languages.CompletionItemKind.Function,
      documentation: "Machine the ROS Node runs on",
      insertText: 'machine name="${1:NAME}" address="${2:ADDRESS}" />',
      insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range,
    },
    {
      label: "env",
      kind: monaco.languages.CompletionItemKind.Function,
      documentation: "Machine the ROS Node runs on",
      insertText: 'env name="${1:NAME}" value="${2:VALUE}" />',
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
      label: "rosparam",
      kind: monaco.languages.CompletionItemKind.Function,
      documentation: "Add a new ROS parameter",
      insertText: 'rosparam subst_value="true" />\n  ${1:}\n</rosparam>',
      insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range,
    },
    {
      label: "rosparam command",
      kind: monaco.languages.CompletionItemKind.Function,
      documentation: "Add a new ROS parameter",
      insertText:
        'rosparam command="load" file="$(find ${1:PACKAGE})/config/${2:FILE}.yaml" subst_value="true" clear_params="true" />',
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
      insertText: 'include file="$(find ${1:PACKAGE})/launch/${2:FILE}" />',
      insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range,
    },
    {
      label: "group",
      kind: monaco.languages.CompletionItemKind.Function,
      documentation: "Add a new group statement",
      insertText: "group>\n  ${1:}\n</group>",
      range,
    },
    {
      label: "group ns",
      kind: monaco.languages.CompletionItemKind.Function,
      documentation: "Add a new group statement with namespace",
      insertText: 'group ns="${1:NAMESPACE}" />\n  ${2:}\n</group>',
      insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range,
    },

    {
      label: "remap",
      kind: monaco.languages.CompletionItemKind.Function,
      documentation: "Add a new remap statement",
      insertText: 'remap from="${1:FROM}" to="${2:TO}" />',
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
  ];
}

function createDocumentSymbols(model: editor.ITextModel /*, token*/): languages.DocumentSymbol[] {
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
        const typeName = nodeElement?.getAttribute("type");
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

export { createDocumentSymbols, createXMLDependencyProposals };
