import { languages } from "monaco-editor/esm/vs/editor/editor.api";

export const Ros2XmlLanguage: languages.IMonarchLanguage = {
  defaultToken: "",
  ignoreCase: true,

  qualifiedTags:
    /launch|include|group|let|arg|executable|param|remap|env|sev_env|unset_env|push-ros-namespace|timer|node_container|node|composable_node|extra_arg|\?xml/,

  qualifiedLaunchAttrs: /if|unless/,
  qualifiedIncludeAttrs: /if|unless|file/,
  qualifiedArgAttrs: /if|unless|name|value|default|description/,
  qualifiedLetAttrs: /if|unless|name|value/,
  qualifiedExecutableAttrs: /if|unless|cmd|cwd|name|ros_args|args|namespace|launch-prefix|output|shell/,
  qualifiedNodeAttrs: /if|unless|pkg|exec|name|args|respawn|required|namespace|output|cwd|launch-prefix/,
  qualifiedNodeContainerAttrs: /if|unless|pkg|exec|name|args|respawn|required|namespace|output|cwd|launch-prefix/,
  qualifiedParamAttrs: /if|unless|name|value|sep|from/,
  qualifiedRemapAttrs: /if|unless|from|to/,
  qualifiedEnvAttrs: /if|unless|name|value/,
  qualifiedSetEnvAttrs: /if|unless|name|value/,
  qualifiedGroupAttrs: /if|unless|scoped/,
  qualifiedUnsetEnvAttrs: /if|unless|name/,
  qualifiedPushRosNamespaceAttrs: /if|unless|namespace/,
  qualifiedXmlAttrs: /version/,

  qualifiedSubs: /find-pkg-prefix|find-pkg-share|find-exec|exec-in-package|var|env|eval|dirname|command/,

  // The main tokenizer for our languages
  tokenizer: {
    root: [
      // Opening tags with their corresponding attribute options
      [
        /(<)(launch)/,
        [
          { token: "delimiter.start", bracket: "@open" },
          { token: "tag", bracket: "@open", next: "@launchtags" },
        ],
      ],
      [
        /(<)(include)/,
        [
          { token: "delimiter.start", bracket: "@open" },
          { token: "tag", bracket: "@open", next: "@includetags" },
        ],
      ],
      [
        /(<)(arg)/,
        [
          { token: "delimiter.start", bracket: "@open" },
          { token: "tag", bracket: "@open", next: "@argtags" },
        ],
      ],
      [
        /(<)(let)/,
        [
          { token: "delimiter.start", bracket: "@open" },
          { token: "tag", bracket: "@open", next: "@lettags" },
        ],
      ],
      [
        /(<)(executable)/,
        [
          { token: "delimiter.start", bracket: "@open" },
          { token: "tag", bracket: "@open", next: "@executabletags" },
        ],
      ],
      [
        /(<)(node )/,
        [
          { token: "delimiter.start", bracket: "@open" },
          { token: "tag", bracket: "@open", next: "@nodetags" },
        ],
      ],
      [
        /(<)(param)/,
        [
          { token: "delimiter.start", bracket: "@open" },
          { token: "tag", bracket: "@open", next: "@paramtags" },
        ],
      ],
      [
        /(<)(remap)/,
        [
          { token: "delimiter.start", bracket: "@open" },
          { token: "tag", bracket: "@open", next: "@remaptags" },
        ],
      ],
      [
        /(<)(env)/,
        [
          { token: "delimiter.start", bracket: "@open" },
          { token: "tag", bracket: "@open", next: "@envtags" },
        ],
      ],
      [
        /(<)(set_env)/,
        [
          { token: "delimiter.start", bracket: "@open" },
          { token: "tag", bracket: "@open", next: "@setenvtags" },
        ],
      ],
      [
        /(<)(unset_env)/,
        [
          { token: "delimiter.start", bracket: "@open" },
          { token: "tag", bracket: "@open", next: "@unsetenvtags" },
        ],
      ],
      [
        /(<)(push_ros_namespace)/,
        [
          { token: "delimiter.start", bracket: "@open" },
          { token: "tag", bracket: "@open", next: "@namespacetags" },
        ],
      ],
      [
        /(<)(group)/,
        [
          { token: "delimiter.start", bracket: "@open" },
          { token: "tag", bracket: "@open", next: "@grouptags" },
        ],
      ],
      [
        /(<)(\?xml)/,
        [
          { token: "delimiter.start", bracket: "@open" },
          { token: "tag", bracket: "@open", next: "@xmltags" },
        ],
      ],

      // Standard opening tag
      // For tags without any attributes: (none)
      [
        /(<)(@qualifiedTags)/,
        [
          { token: "delimiter.start", bracket: "@open" },
          { token: "tag", bracket: "@open", next: "@tag" },
        ],
      ],

      // Standard closing tag
      [
        /(<)(\/)(@qualifiedTags)(\s*)(>)/,
        [
          { token: "delimiter.start", bracket: "@open" },
          { token: "tag", bracket: "@close" },
          { token: "tag", bracket: "@close" },
          "",
          { token: "delimiter.end", bracket: "@close" },
        ],
      ],

      // trailing slash for single line tags
      [
        /(\/)(>)/,
        [
          { token: "tag", bracket: "@close" },
          { token: "delimiter.end", bracket: "@close", next: "@pop" },
        ],
      ],
      [/(@qualifiedTags)(\s*)(>)/, [{ token: "delimiter.end", bracket: "@close" }]],
      [/<!--/, "comment", "@comment"],
    ],

    // Standard tag
    tag: [
      [/\s+/, ""],
      [
        /(\/)(>)/,
        [
          { token: "tag", bracket: "@close" },
          { token: "delimiter.end", bracket: "@close", next: "@pop" },
        ],
      ],
      [/>/, { token: "delimiter.end", bracket: "@close", next: "@pop" }],
      [/\?>/, { token: "delimiter.end", bracket: "@close", next: "@pop" }],
    ],

    // Per tag attributes
    launchtags: [
      [/\s+/, ""],
      [/(@qualifiedLaunchAttrs)(\s*=\s*)(")/, ["attribute.name", "", { token: "", bracket: "@open", next: "@value" }]],
      [
        /(@qualifiedLaunchAttrs)(\s*=\s*)(')/,
        ["attribute.name", "attribute.name", { token: "attribute.value", bracket: "@open", next: "@value_sq" }],
      ],
      [
        /(\/)(>)/,
        [
          { token: "tag", bracket: "@close" },
          { token: "delimiter.end", bracket: "@close", next: "@pop" },
        ],
      ],
      [/>/, { token: "delimiter.end", bracket: "@close", next: "@pop" }],
      [/\?>/, { token: "delimiter.end", bracket: "@close", next: "@pop" }],
    ],
    grouptags: [
      [/\s+/, ""],
      [/(@qualifiedGroupAttrs)(\s*=\s*)(")/, ["attribute.name", "", { token: "", bracket: "@open", next: "@value" }]],
      [
        /(@qualifiedGroupAttrs)(\s*=\s*)(')/,
        ["attribute.name", "attribute.name", { token: "attribute.value", bracket: "@open", next: "@value_sq" }],
      ],
      [
        /(\/)(>)/,
        [
          { token: "tag", bracket: "@close" },
          { token: "delimiter.end", bracket: "@close", next: "@pop" },
        ],
      ],
      [/>/, { token: "delimiter.end", bracket: "@close", next: "@pop" }],
      [/\?>/, { token: "delimiter.end", bracket: "@close", next: "@pop" }],
    ],
    includetags: [
      [/\s+/, ""],
      [/(@qualifiedIncludeAttrs)(\s*=\s*)(")/, ["attribute.name", "", { token: "", bracket: "@open", next: "@value" }]],
      [
        /(@qualifiedIncludeAttrs)(\s*=\s*)(')/,
        ["attribute.name", "attribute.name", { token: "attribute.value", bracket: "@open", next: "@value_sq" }],
      ],
      [
        /(\/)(>)/,
        [
          { token: "tag", bracket: "@close" },
          { token: "delimiter.end", bracket: "@close", next: "@pop" },
        ],
      ],
      [/>/, { token: "delimiter.end", bracket: "@close", next: "@pop" }],
      [/\?>/, { token: "delimiter.end", bracket: "@close", next: "@pop" }],
    ],
    argtags: [
      [/\s+/, ""],
      [/(@qualifiedArgAttrs)(\s*=\s*)(")/, ["attribute.name", "", { token: "", bracket: "@open", next: "@value" }]],
      [
        /(@qualifiedArgAttrs)(\s*=\s*)(')/,
        ["attribute.name", "attribute.name", { token: "attribute.value", bracket: "@open", next: "@value_sq" }],
      ],
      [
        /(\/)(>)/,
        [
          { token: "tag", bracket: "@close" },
          { token: "delimiter.end", bracket: "@close", next: "@pop" },
        ],
      ],
      [/>/, { token: "delimiter.end", bracket: "@close", next: "@pop" }],
      [/\?>/, { token: "delimiter.end", bracket: "@close", next: "@pop" }],
    ],
    lettags: [
      [/\s+/, ""],
      [/(@qualifiedLetAttrs)(\s*=\s*)(")/, ["attribute.name", "", { token: "", bracket: "@open", next: "@value" }]],
      [
        /(@qualifiedLetAttrs)(\s*=\s*)(')/,
        ["attribute.name", "attribute.name", { token: "attribute.value", bracket: "@open", next: "@value_sq" }],
      ],
      [
        /(\/)(>)/,
        [
          { token: "tag", bracket: "@close" },
          { token: "delimiter.end", bracket: "@close", next: "@pop" },
        ],
      ],
      [/>/, { token: "delimiter.end", bracket: "@close", next: "@pop" }],
      [/\?>/, { token: "delimiter.end", bracket: "@close", next: "@pop" }],
    ],
    executabletags: [
      [/\s+/, ""],
      [
        /(@qualifiedExecutableAttrs)(\s*=\s*)(")/,
        ["attribute.name", "", { token: "", bracket: "@open", next: "@value" }],
      ],
      [
        /(@qualifiedExecutableAttrs)(\s*=\s*)(')/,
        ["attribute.name", "attribute.name", { token: "attribute.value", bracket: "@open", next: "@value_sq" }],
      ],
      [
        /(\/)(>)/,
        [
          { token: "tag", bracket: "@close" },
          { token: "delimiter.end", bracket: "@close", next: "@pop" },
        ],
      ],
      [/>/, { token: "delimiter.end", bracket: "@close", next: "@pop" }],
      [/\?>/, { token: "delimiter.end", bracket: "@close", next: "@pop" }],
    ],
    nodetags: [
      [/\s+/, ""],
      [/(@qualifiedNodeAttrs)(\s*=\s*)(")/, ["attribute.name", "", { token: "", bracket: "@open", next: "@value" }]],
      [
        /(@qualifiedNodeAttrs)(\s*=\s*)(')/,
        ["attribute.name", "attribute.name", { token: "attribute.value", bracket: "@open", next: "@value_sq" }],
      ],
      [
        /(\/)(>)/,
        [
          { token: "tag", bracket: "@close" },
          { token: "delimiter.end", bracket: "@close", next: "@pop" },
        ],
      ],
      [/>/, { token: "delimiter.end", bracket: "@close", next: "@pop" }],
      [/\?>/, { token: "delimiter.end", bracket: "@close", next: "@pop" }],
    ],
    paramtags: [
      [/\s+/, ""],
      [/(@qualifiedParamAttrs)(\s*=\s*)(")/, ["attribute.name", "", { token: "", bracket: "@open", next: "@value" }]],
      [
        /(@qualifiedParamAttrs)(\s*=\s*)(')/,
        ["attribute.name", "attribute.name", { token: "attribute.value", bracket: "@open", next: "@value_sq" }],
      ],
      [
        /(\/)(>)/,
        [
          { token: "tag", bracket: "@close" },
          { token: "delimiter.end", bracket: "@close", next: "@pop" },
        ],
      ],
      [/>/, { token: "delimiter.end", bracket: "@close", next: "@pop" }],
      [/\?>/, { token: "delimiter.end", bracket: "@close", next: "@pop" }],
    ],
    remaptags: [
      [/\s+/, ""],
      [/(@qualifiedRemapAttrs)(\s*=\s*)(")/, ["attribute.name", "", { token: "", bracket: "@open", next: "@value" }]],
      [
        /(@qualifiedRemapAttrs)(\s*=\s*)(')/,
        ["attribute.name", "attribute.name", { token: "attribute.value", bracket: "@open", next: "@value_sq" }],
      ],
      [
        /(\/)(>)/,
        [
          { token: "tag", bracket: "@close" },
          { token: "delimiter.end", bracket: "@close", next: "@pop" },
        ],
      ],
      [/>/, { token: "delimiter.end", bracket: "@close", next: "@pop" }],
      [/\?>/, { token: "delimiter.end", bracket: "@close", next: "@pop" }],
    ],
    envtags: [
      [/\s+/, ""],
      [/(@qualifiedEnvAttrs)(\s*=\s*)(")/, ["attribute.name", "", { token: "", bracket: "@open", next: "@value" }]],
      [
        /(@qualifiedEnvAttrs)(\s*=\s*)(')/,
        ["attribute.name", "attribute.name", { token: "attribute.value", bracket: "@open", next: "@value_sq" }],
      ],
      [
        /(\/)(>)/,
        [
          { token: "tag", bracket: "@close" },
          { token: "delimiter.end", bracket: "@close", next: "@pop" },
        ],
      ],
      [/>/, { token: "delimiter.end", bracket: "@close", next: "@pop" }],
      [/\?>/, { token: "delimiter.end", bracket: "@close", next: "@pop" }],
    ],
    setenvtags: [
      [/\s+/, ""],
      [/(@qualifiedSetEnvAttrs)(\s*=\s*)(")/, ["attribute.name", "", { token: "", bracket: "@open", next: "@value" }]],
      [
        /(@qualifiedSetEnvAttrs)(\s*=\s*)(')/,
        ["attribute.name", "attribute.name", { token: "attribute.value", bracket: "@open", next: "@value_sq" }],
      ],
      [
        /(\/)(>)/,
        [
          { token: "tag", bracket: "@close" },
          { token: "delimiter.end", bracket: "@close", next: "@pop" },
        ],
      ],
      [/>/, { token: "delimiter.end", bracket: "@close", next: "@pop" }],
      [/\?>/, { token: "delimiter.end", bracket: "@close", next: "@pop" }],
    ],
    unsetenvtags: [
      [/\s+/, ""],
      [
        /(@qualifiedUnsetEnvAttrs)(\s*=\s*)(")/,
        ["attribute.name", "", { token: "", bracket: "@open", next: "@value" }],
      ],
      [
        /(@qualifiedUnsetEnvAttrs)(\s*=\s*)(')/,
        ["attribute.name", "attribute.name", { token: "attribute.value", bracket: "@open", next: "@value_sq" }],
      ],
      [
        /(\/)(>)/,
        [
          { token: "tag", bracket: "@close" },
          { token: "delimiter.end", bracket: "@close", next: "@pop" },
        ],
      ],
      [/>/, { token: "delimiter.end", bracket: "@close", next: "@pop" }],
      [/\?>/, { token: "delimiter.end", bracket: "@close", next: "@pop" }],
    ],
    namespacetags: [
      [/\s+/, ""],
      [
        /(@qualifiedPushRosNamespaceAttrs)(\s*=\s*)(")/,
        ["attribute.name", "", { token: "", bracket: "@open", next: "@value" }],
      ],
      [
        /(@qualifiedPushRosNamespaceAttrs)(\s*=\s*)(')/,
        ["attribute.name", "attribute.name", { token: "attribute.value", bracket: "@open", next: "@value_sq" }],
      ],
      [
        /(\/)(>)/,
        [
          { token: "tag", bracket: "@close" },
          { token: "delimiter.end", bracket: "@close", next: "@pop" },
        ],
      ],
      [/>/, { token: "delimiter.end", bracket: "@close", next: "@pop" }],
      [/\?>/, { token: "delimiter.end", bracket: "@close", next: "@pop" }],
    ],
    xmltags: [
      [/\s+/, ""],
      [/(@qualifiedXmlAttrs)(\s*=\s*)(")/, ["attribute.name", "", { token: "", bracket: "@open", next: "@value" }]],
      [
        /(@qualifiedXmlAttrs)(\s*=\s*)(')/,
        ["attribute.name", "attribute.name", { token: "attribute.value", bracket: "@open", next: "@value_sq" }],
      ],
      [
        /(\/)(>)/,
        [
          { token: "tag", bracket: "@close" },
          { token: "delimiter.end", bracket: "@close", next: "@pop" },
        ],
      ],
      [/>/, { token: "delimiter.end", bracket: "@close", next: "@pop" }],
      [/\?>/, { token: "delimiter.end", bracket: "@close", next: "@pop" }],
    ],

    value: [
      [/([^"^$]*)(\$\()/, ["attribute.value", { token: "delimiter.start", bracket: "@open", next: "@subst" }]],
      [/([^"]*)(")/, ["attribute.value", { token: "", bracket: "@close", next: "@pop" }]],
    ],
    subst: [
      [/(@qualifiedSubs)(\s*)([^)]*)/, ["subst.key", "", "subst.arg"]],
      [/(\))/, { token: "delimiter.end", bracket: "@close", next: "@pop" }],
    ],
    value_sq: [
      [/([^'^$]*)(\$\()/, ["attribute.value", { token: "delimiter.start", bracket: "@open", next: "@subst" }]],
      [/([^']*)(')/, ["attribute.value", { token: "", bracket: "@close", next: "@pop" }]],
    ],
    cdata: [
      [/[^\]]+/, ""],
      [/\]\]>/, { token: "delimiter.cdata", bracket: "@close", next: "@pop" }],
      [/\]/, ""],
    ],
    whitespace: [
      [/\s+/, ""],
      [/<!--/, { token: "comment", bracket: "@open", next: "@comment" }],
    ],

    comment: [
      [/[^<-]+/, "comment.content"],
      [/-->|--!>/, { token: "comment", bracket: "@close", next: "@pop" }],
      [/<!--/, "comment.content.invalid"],
      [/[<-]/, "comment.content"],
    ],
  },
};
