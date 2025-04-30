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
  const ns = createProposal("ns", 'ns="${1:NAMESPACE}"', "Assigned namespace");
  const value = createProposal("value", 'value="${1:VALUE}"', "Value to set. Not combineable with default");
  const required = createProposal("required", 'required="${1:VALUE}"', "If node dies, kill entire roslaunch");
  const type = createProposal(
    "type",
    'type="${1:TYPE}"',
    "Node type. There must be a corresponding executable with the same name"
  );
  const pkg = createProposal("pkg", 'pkg="${1:PACKAGE}"', "Package of node");
  const clear_params = createProposal(
    "clear_params",
    'clear_params="${1:VALUE}"',
    "Delete all parameters before launch"
  );
  const command = createProposal(
    "command",
    'command="${1:COMMAND}"',
    "The output of the command will be read and stored as a string"
  );
  const cwd = createProposal(
    "cwd",
    'cwd="${1:WORKING_DIRECTORY}"',
    "If 'node', the working directory of the node will be set to the same directory as the node's executable"
  );
  const args = createProposal("args", 'args="${1:ARGUMENTS}"', "Pass arguments to node");
  const launch_prefix = createProposal(
    "launch-prefix",
    'launch-prefix="${1:VALUE}"',
    "Command/arguments to prepend to node's launch arguments"
  );

  return [
    {
      tag: "launch",
      proposals: [
        ...all,
        createProposal(
          "depricated",
          'depricated="${1:MESSAGE}"',
          "Add a deprication message to the launch configuration"
        ),
      ],
    },
    {
      tag: "node",
      proposals: [
        ...all,
        name,
        ns,
        required,
        pkg,
        clear_params,
        cwd,
        args,
        launch_prefix,
        createProposal("machine", 'machine="${1:MACHINE_NAME}"', "Launch node on designated machine"),
        createProposal("respawn", 'respawn="${1:SHOULD_RESPAWN}"', "Restart the node automatically if it quits"),
        createProposal(
          "respawn_delay",
          'respawn_delay="${1:SECONDS}"',
          "If respawn is true, wait respawn_delay seconds after the node failure is detected before attempting restart"
        ),
        createProposal(
          "output",
          'output="${1:VALUE}"',
          "If 'screen', stdout/stderr from the node will be sent to the screen. If 'log', the stdout/stderr output will be sent to a log file in $ROS_HOME/log, and stderr will continue to be sent to screen. The default is 'log'"
        ),
      ],
    },
    {
      tag: "machine",
      proposals: [
        ...all,
        name,
        createProposal(
          "default",
          'default="${1:DEFAULT}"',
          "Sets this machine as the default to assign nodes to. The default setting only applies to nodes defined later in the same scope. NOTE: if there are no default machines, the local machine is used. You can prevent a machine from being chosen by setting default='never', in which case the machine can only be explicitly assigned"
        ),
        createProposal("address", 'address="${1:ADDRESS}"', "Network address/hostname of machine"),
        createProposal(
          "env-loader",
          'env-loader="${1:FILEPATH}"',
          "Specify environment file on remote machine. Environment file must be a shell script that sets all required environment variables, then runs exec on the provided arguments"
        ),
        createProposal("user", 'user="${1:USERNAME}"', "SSH user name for logging into machine"),
        createProposal(
          "password",
          'password="${1:PASSWORD}"',
          "SSH password. It is highly recommended that you configure SSH keys and SSH agent instead so you can login with certificates instead"
        ),
        createProposal(
          "timeout",
          'timeout="${1:SECONDS}"',
          "Number of seconds before a roslaunch on this machine is considered as having failed to launch"
        ),
        createProposal(
          "ros-root",
          'ros-root="${1:PATH}"',
          "ROS_ROOT for machine. Defaults to the ROS_ROOT set in the local environment"
        ),
        createProposal(
          "ros-package-path",
          'ros-package-path="${1:PATH}"',
          "ROS_PACKAGE_PATH for machine. Defaults to the ROS_PACKAGE_PATH set in the local environment"
        ),
      ],
    },
    {
      tag: "remap",
      proposals: [
        ...all,
        createProposal(
          "from",
          'from="${1:TOPIC_NAME}"',
          "Remapped topic: name of the ROS topic that you are remapping FROM"
        ),
        createProposal(
          "to",
          'to="${1:TOPIC_NAME}"',
          "Target name: name of the ROS topic that you are pointing the from topic TO"
        ),
      ],
    },
    {
      tag: "env",
      proposals: [...all, name, value],
    },
    {
      tag: "param",
      proposals: [
        ...all,
        name,
        value,
        type,
        command,
        createProposal(
          "textfile",
          'textfile="$(find ${1:PACKAGE_NAME})${2:FILE_PATH}"',
          "The contents of the file will be read and stored as a string. The file must be locally accessible"
        ),
        createProposal(
          "binfile",
          'binfile="$(find ${1:PACKAGE_NAME})${2:FILE_PATH}"',
          "The contents of the file will be read and stored as a base64-encoded XML-RPC binary object. The file must be locally accessible"
        ),
      ],
    },
    {
      tag: "rosparam",
      proposals: [
        ...all,
        ns,
        command,
        createProposal("file", 'file="$(find ${1:PACKAGE_NAME})${2:FILE_PATH}"', "Name of rosparam file"),
        createProposal("param", 'param="${1:PARAM_NAME}"', "Name of parameter"),
        createProposal("subst_value", 'subst_value="${1:VALUE}"', "Allows use of substitution args in the YAML text"),
      ],
    },
    {
      tag: "group",
      proposals: [...all, ns, clear_params],
    },
    {
      tag: "test",
      proposals: [
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
        createProposal("test-name", 'test-name="${1:TEST_NAME}"', "Name of test for recording in test results"),
        createProposal("retry", 'retry="${1:ATTEMPTS}"', "How often the test is attempted"),
        createProposal(
          "time-limit",
          'time-limit="${1:SECONDS}"',
          "Number of seconds before the test is considered a failure. Default is 60 seconds. The time limit is reset after every retry"
        ),
      ],
    },
    {
      tag: "arg",
      proposals: [
        ...all,
        name,
        value,
        createProposal(
          "default",
          'default="${1:VALUE}"',
          "Default value of argument. Cannot be combined with value attribute"
        ),
        createProposal(
          "doc",
          'doc="${1:DESCRIPTION}"',
          "Description of the argument. You could get this through --ros-args argument to the roslaunch command"
        ),
      ],
    },
    {
      tag: "include",
      proposals: [
        ...all,
        ns,
        clear_params,
        createProposal("file", 'file="$(find ${1:PACKAGE})${2:FILE_PATH}"', "Name of file to include"),
        createProposal(
          "pass_all_args",
          'pass_all_args="${1:VALUE}"',
          "If true, then all args set in the current context are added to the child context that is created for processing the included file"
        ),
      ],
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
      `${open}node name="\${1:NAME}" pkg="\${2:PACKAGE}" type=" \${3:TYPE}" />\n  \n</node${close}`,
      "Add a new ROS node"
    ),
    createProposal(
      "machine",
      `${open}machine name="\${1:NAME}" address="\${2:ADDRESS}" /${close}`,
      "Machine the ROS Node runs on"
    ),
    createProposal("env", `${open}env name="\${1:NAME}" value="\${2:VALUE}" /${close}`, "Machine the ROS Node runs on"),
    createProposal(
      "param",
      `${open}param name="\${1:${getParamName("NAME")}}" value="\${2:VALUE}" /${close}`,
      "Add a new ROS parameter"
    ),
    createProposal(
      "rosparam",
      `${open}rosparam subst_value="true" />\n  \${1:}\n</rosparam${close}`,
      "Add a new ROS parameter"
    ),
    createProposal(
      "rosparam command",
      `${open}rosparam command="load" file="$(find \${1:PACKAGE})/config/\${2:FILE}.yaml" subst_value="true" clear_params="true" /${close}`,
      "Add a new ROS parameter"
    ),
    createProposal(
      "arg value",
      `${open}arg name="\${1:NAME}" value="\${2:VALUE}" /${close}`,
      "Add a new ROS argument with value"
    ),
    createProposal(
      "arg default",
      `${open}arg name="\${1:NAME}" default="\${2:DEFAULT}" /${close}`,
      "Add a new ROS argument with default"
    ),
    createProposal(
      "include",
      `${open}include file="$(find \${1:PACKAGE})/launch/\${2:FILE}" /${close}`,
      "Add a new include statement"
    ),
    createProposal("group", `${open}group>\n  \${1:}\n</group${close}`, "Add a new group statement"),
    createProposal(
      "group ns",
      `${open}group ns="\${1:NAMESPACE}" />\n  \${2:}\n</group${close}`,
      "Add a new group statement with namespace"
    ),
    createProposal("remap", `${open}remap from="\${1:FROM}" to="\${2:TO}" /${close}`, "Add a new remap statement"),
    createProposal(
      "GDB Launch Prefix",
      `${open}launch-prefix="gdb -ex run -ex bt -batch --args"${close}`,
      "GDB Launch Prefix"
    ),
    createProposal(
      "mas/associations",
      `${open}mas/associations="NODE1,NODE2" /${close}`,
      "Associated ROS-Nodes are started before the node itself and stopped after the node."
    ),
    createProposal(
      "mas/kill_on_stop",
      `${open}mas/kill_on_stop="300" /${close}`,
      "Kill the node after defined time in milliseconds"
    ),
  ];
}
