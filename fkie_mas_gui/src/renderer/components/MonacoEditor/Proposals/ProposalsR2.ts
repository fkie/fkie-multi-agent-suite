import { TFileRange } from "@/types/FileRange";
import { Monaco } from "@monaco-editor/react";
import { languages } from "monaco-editor/esm/vs/editor/editor.api";
import { TTagAttributeProposals } from "./TTagAttributeProposals";

export function getTagAttributeProposals(monaco: Monaco, range: TFileRange): TTagAttributeProposals[] {
  function createProposal(label: string, insertText: string, documentation: string): languages.CompletionItem {
    return {
      label: label,
      preselect: true,
      kind: monaco.languages.CompletionItemKind.Function,
      documentation: documentation,
      insertText: insertText,
      insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range,
    };
  }

  const all = [
    createProposal("if", 'if="${arg ${1:VALUE}}"', "If value evaluates to true, include tag and its contents."),
    createProposal(
      "unless",
      'unless="${arg ${1:VALUE}}"',
      "Unless value evaluates to true (which means if value evaluates to false), include tag and its contents."
    ),
  ];

  const name = createProposal("name", 'name="${1:NAME}"', "Assigned name");
  const value = createProposal("value", 'value="${1:VALUE}"', "Value to set. Not combinable with default");
  const args = createProposal("args", 'args="${1:ARGUMENTS}"', "Additional 'command-line' arguments");
  const launch_prefix = createProposal(
    "launch-prefix",
    'launch-prefix="${1:VALUE}"',
    "A prefix for the command-line if a shell is used to launch"
  );
  const output = createProposal(
    "output",
    'output="${1:VALUE}"',
    "If 'screen', stdout/stderr from the node will be sent to the screen. If 'log', the stdout/stderr output will be sent to a log file in $ROS_HOME/log, and stderr will continue to be sent to screen. The default is 'log'"
  );

  return [
    {
      tag: "launch",
      proposals: [...all, createProposal("version", 'version="${1:XML_VERSION}"', "Launch XML schema version in use")],
    },
    {
      tag: "include",
      proposals: [
        ...all,
        createProposal("file", 'file="$(find ${1:PACKAGE})${2:FILE_PATH}"', "Path to the launch file to be included"),
      ],
    },
    {
      tag: "arg",
      proposals: [
        ...all,
        name,
        value,
        createProposal("default", 'default="${1:VALUE}"', "Default value, if none is provided"),
        createProposal("description", 'description="${1:TEXT}"', "Brief description of the argument"),
      ],
    },
    {
      tag: "let",
      proposals: [...all, name, value],
    },
    {
      tag: "group",
      proposals: [
        ...all,
        createProposal(
          "scoped",
          'scoped="${1:VALUE}"',
          "Whether the group is a scoping one launch configuration-wise or not"
        ),
      ],
    },
    {
      tag: "env",
      proposals: [...all, name, value],
    },
    {
      tag: "set_env",
      proposals: [...all, name, value],
    },
    {
      tag: "unset_env",
      proposals: [...all, value],
    },
    {
      tag: "executable",
      proposals: [
        ...all,
        name,
        args,
        launch_prefix,
        output,
        createProposal("cwd", 'cwd="${1:WORKING_DIRECTORY}"', "The working directory for the launched process"),
        createProposal(
          "cmd",
          'cmd="${1:VALUE}"',
          "Path to the executable or a command-line if a shell is used to launch"
        ),
        createProposal("shell", 'shell="${1:true}"', "Whether to use a shell to launch or not"),
        createProposal("sigkill_timeout", 'sigkill_timeout="${1:4.0}"', ""),
        createProposal("sigterm_timeout", 'sigterm_timeout="${1:7.0}"', ""),
      ],
    },
    {
      tag: "param",
      proposals: [
        ...all,
        name,
        value,
        createProposal("from", 'from="${1:TOPIC_NAME}"', "Path to the parameters file to be loaded"),
        createProposal(
          "value-sep",
          'value-sep="${1:SEPARATOR}"',
          "Separator to use should value be a list. Examples are ',' or ';'"
        ),
      ],
    },
    {
      tag: "remap",
      proposals: [
        ...all,
        createProposal("from", 'from="${1:TOPIC_NAME}"', "Name matching expression to look for replacement candidates"),
        createProposal("to", 'to="${1:TOPIC_NAME}"', "Name replacement expression to replace candidates found"),
      ],
    },
    {
      tag: "node",
      proposals: [
        ...all,
        name,
        args,
        launch_prefix,
        output,
        createProposal("pkg", 'pkg="${1:PACKAGE}"', "Name of the package where the node is to be found"),
        createProposal("exec", 'exec="${1:NODE_EXECUTABLE_NAME}"', "Name of the node executable"),
        createProposal(
          "ros_args",
          'ros_args="${1:ARGUMENTS}"',
          "ROS-specific 'command-line' arguments for the ROS node"
        ),
        createProposal("namespace", 'namespace="${1:NAMESPACE}"', "Assigned namespace"),
      ],
    },
    {
      tag: "push_ros_namespace",
      proposals: [...all, createProposal("namespace", 'namespace="${1:NAMESPACE}"', "Assigned namespace")],
    },
  ];
}

export function getTagProposals(
  monaco: Monaco,
  range: TFileRange,
  clipText: string,
  lineContent: string
): languages.CompletionItem[] {
  function getParamName(defaultValue: string | undefined): string | undefined {
    const text = clipText?.replace(/(\r\n.*|\n.*|\r.*)/gm, "");
    return text || defaultValue;
  }

  function createProposal(label: string, insertText: string, documentation: string): languages.CompletionItem {
    return {
      label: label,
      preselect: true,
      kind: monaco.languages.CompletionItemKind.Function,
      documentation: documentation,
      insertText: insertText,
      insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range,
    };
  }

  let open = "<";
  let close = ">";

  if (lineContent.charAt(range.startColumn - 2) === "<") {
    open = "";
    close = "";
  }

  return [
    createProposal("launch", `${open}launch>\n  \${1:}\n</launch${close}`, "Create a new launch configuration"),
    createProposal(
      "node",
      `${open}node name="\${1:NODE_NAME}" pkg="\${2:PACKAGE}" exec="\${3:NODE_EXECUTABLE_NAME}">\n  \n</node${close}`,
      "Add a new ROS node"
    ),
    createProposal("executable", `${open}executable cmd="\${1:COMMAND}" /${close}`, "Add a new executable"),
    createProposal(
      "executable cwd",
      `${open}executable cmd="\${1:COMMAND}" cwd="\${2:WORKING_DIRECTORY}" /${close}`,
      "Add a new executable with working directory"
    ),
    createProposal(
      "executable args",
      `${open}executable cmd="\${1:COMMAND}" args="\${2:ARGUMENTS}" /${close}`,
      "Add a new executable with arguments"
    ),
    createProposal(
      "param",
      `${open}param name="\${1:${getParamName("NAME")}}" value="\${2:VALUE}" /${close}`,
      "Add a new ROS parameter"
    ),
    createProposal(
      "param sep",
      `${open}param name="\${1:${getParamName("NAME")}}" value="\${2:VALUE}" value-sep="\${3:SEPARATOR}" /${close}`,
      "Add a new ROS parameter with value separator"
    ),
    createProposal(
      "arg value",
      `${open}arg name="\${1:NAME}" value="\${2:VALUE}" /${close}`,
      "Add a new ROS argument with value"
    ),
    createProposal(
      "arg default",
      `${open}arg name="\${1:NAME}" default="\${2:DEFAULT}" /${close}`,
      "Add a new ROS argument with default value"
    ),
    createProposal(
      "include",
      `${open}include file="$(find \${1:PACKAGE})/launch/\${2:FILE}" /${close}`,
      "Add a new include statement"
    ),
    createProposal("group", `${open}group>\n  \n</group${close}`, "Add a new group statement"),
    createProposal(
      "group ns",
      `${open}group>\n  <push_ros_namespace namespace="\${1:NAMESPACE}"/>\n</group${close}`,
      "Add a new group statement with namespace"
    ),
    createProposal(
      "set_env",
      `${open}set_env name="\${1:NAME}" value="\${2:VALUE}" /${close}`,
      "Allows for modifying an OS process environment"
    ),
    createProposal(
      "unset_env",
      `${open}unset_env name="\${1:NAME}" /${close}`,
      "Allows for deleting an OS process environment variable"
    ),
    createProposal("remap", `${open}remap from="\${1:FROM}" to="\${2:TO}" /${close}`, "Add a new remap statement"),
    createProposal("GDB Launch Prefix", 'launch-prefix="gdb -ex run -ex bt -batch --args"', "GDB Launch Prefix"),
    createProposal(
      "nm/associations",
      `${open}param name="nm/associations" value="\${1:NODES}" /${close}`,
      "Associated ROS-Nodes are started before the node itself and stopped after the node"
    ),
    createProposal(
      "mas/kill_on_stop",
      `${open}param name="mas/kill_on_stop" value="300" /${close}`,
      "Kill the node after defined time in milliseconds"
    ),
    createProposal("timer", `${open}timer period="\${1:5}">\n   \n</timer${close}`, ""),
    createProposal(
      "find-pkg-prefix",
      "find-pkg-prefix ${1:PACKAGE})",
      "Substituted by the install prefix path of the given package. Forward and backwards slashes will be resolved to the local filesystem convention. Substitution will fail if the package cannot be found"
    ),
    createProposal(
      "find-pkg-share",
      "find-pkg-prefix ${1:PACKAGE})",
      "Substituted by the share directory path of the given package. The share directory includes the package’s name, e.g. <prefix>/share/<pkg-name>. Forward and backwards slashes will be resolved to the local filesystem convention. Substitution will fail if the package cannot be found"
    ),
    createProposal(
      "find-exec",
      "find-exec ${1:EXEC_NAME})",
      "Substituted by the path to the executable in the local filesystem. Executables are looked up in the PATH environment variable. Forward and backwards slashes will be resolved to the local filesystem convention. Substitution will fail if the executable cannot be found"
    ),
    createProposal(
      "exec-in-package",
      "exec-in-package ${1:EXEC_NAME} ${2:PACKAGE})",
      "Substituted by the path to the executable in the local filesystem. Executables are looked up in the lib directory of the package. Substitution will fail if the executable cannot be found"
    ),
    createProposal(
      "var",
      "var ${1:VALUE})",
      "Substituted by the value of the launch configuration variable. Substitution will fail if the named argument does not exist"
    ),
    createProposal(
      "env",
      "env ${1:ENV_VAR} ${2:[DEFAULT_VALUE]})",
      "Substituted by the value of the given environment variable Substitution will fail if the variable is not set, unless a default value is provided"
    ),
    createProposal(
      "eval",
      "eval ${1:PYTHON_EXPRESSION})",
      "Substituted by the evaluated python expression. Substitution will fail if python fails to evaluate the expression"
    ),
    createProposal(
      "dirname",
      "dirname)",
      "Substituted by the current launch file directory name. Substitution will always succeed"
    ),
    createProposal(
      "MAS_CAPABILITY_GROUP",
      `<set_env name="MAS_CAPABILITY_GROUP" value="\${1:NAME}" />`,
      "Sets the capability group in the MAS GUI for all nodes that have defined this environment variable."
    ),
    createProposal(
      "MAS_KILL_ON_STOP",
      `<set_env name="MAS_KILL_ON_STOP" value="\${1:300}" />`,
      "Kills all nodes after X[ms] which are affected by this environment variable."
    ),
  ];
}
