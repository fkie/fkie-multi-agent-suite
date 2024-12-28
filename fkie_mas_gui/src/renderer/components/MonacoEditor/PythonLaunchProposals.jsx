/* eslint-disable no-template-curly-in-string */

const word_pattern = /(?<word>\w+)/gm;

function createWordList(text, monaco, range) {
  const words = text.match(word_pattern);
  const result = [...new Set(words)].map((word) => {
    return {
      label: `${word}`,
      kind: monaco.languages.CompletionItemKind.Text,
      insertText: `${word}`,
      insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range,
    };
  });
  return result;
}

// Method,Function,Constructor,Field,Variable,Class,Struct,Interface,Module,Property,Event,Operator,Unit,Value,Constant,Enum,EnumMember,Keyword,Text,Color,File,Reference,Customcolor,Folder,TypeParameter,User,Issue,Snippet

export const createPythonLaunchProposals = (monaco, range, clipText, text) => {
  // returning a static list of proposals, valid for ROS launch and XML  files

  const wordSuggestions = createWordList(text, monaco, range);
  return [
    ...wordSuggestions,
    {
      label: "Node",
      kind: monaco.languages.CompletionItemKind.Snippet,
      documentation: "Add a new ROS Node",
      insertText:
        "Node(name='${1:NAME}', package='${2:PACKAGE}', executable='${3:TYPE}', output='screen', respawn=use_respawn, respawn_delay=2.0, parameters=[${4:{'autostart': autostart}}], arguments=[${5:'--ros-args', '--log-level', log_level}], remappings=[${6:('topic', 'topic_test')}])",
      insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range,
    },
    {
      label: "DeclareLaunchArgument",
      kind: monaco.languages.CompletionItemKind.Snippet,
      documentation: "Add a new ROS DeclareLaunchArgument",
      insertText: `declare_\${1:${clipText || "NAME"}} = DeclareLaunchArgument('\${2:${clipText || "NAME"}}', default_value='\${3:VALUE}', description='')`,
      insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range,
    },
    {
      label: "IncludeLaunchDescription (python)",
      kind: monaco.languages.CompletionItemKind.Snippet,
      documentation: "Add a new include statement",
      insertText:
        "IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('${1:PACKAGE}'), 'launch', '${2:FILE}')), launch_arguments={${3:'namespace': namespace}}.items())",
      insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range,
    },
    {
      label: "from launch.conditions import IfCondition",
      kind: monaco.languages.CompletionItemKind.Reference,
      insertText: "from launch.conditions import IfCondition",
      insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range,
    },
    {
      label: "from launch.actions import IncludeLaunchDescription",
      kind: monaco.languages.CompletionItemKind.Reference,
      insertText: "from launch.actions import IncludeLaunchDescription",
      insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range,
    },
    {
      label: "from launch.launch_description_sources import PythonLaunchDescriptionSource",
      kind: monaco.languages.CompletionItemKind.Reference,
      insertText: "from launch.launch_description_sources import PythonLaunchDescriptionSource",
      insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range,
    },
  ];
};
