import { languages } from "monaco-editor/esm/vs/editor/editor.api";

export const Ros1XmlLanguage: languages.IMonarchLanguage = {
  defaultToken: "",
  ignoreCase: true,

  qualifiedTags: /launch|node|machine|remap|env|param|rosparam|group|test|arg|\?xml/,

  qualifiedLaunchAttrs: /if|unless|depricated/,
  qualifiedMachineAttrs: /if|unless|name|address|env-loader|default|user|password|timeout|ros-root|ros-package-path/,
  qualifiedNodeAttrs: /if|unless|pkg|type|name|args|machine|respawn|respawn_delay|required|ns|clear_params|output|cwd|launch-prefix/,
  qualifiedIncludeAttrs: /if|unless|file|ns|clear_params|pass_all_args/,
  qualifiedRemapAttrs: /if|unless|from|to/,
  qualifiedEnvAttrs: /if|unless|name|value/,
  qualifiedParamAttrs: /if|unless|name|value|type|textfile|binfile|command/,
  qualifiedRosParamAttrs: /if|unless|command|file|param|ns|subst_value/,
  qualifiedGroupAttrs: /if|unless|ns|clear_params/,
  qualifiedTestAttrs: /if|unless|pkg|test-name|type|name|args|clear_params|cwd|launch-prefix|ns|retry|time-limit/,
  qualifiedArgAttrs: /if|unless|name|default|value|doc/,
  qualifiedXmlAttrs: /version/,
  
  qualifiedSubs: /env|optenv|find|anon|arg|eval|dirname/,

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
        /(<)(machine)/,
        [
          { token: "delimiter.start", bracket: "@open" },
          { token: "tag", bracket: "@open", next: "@machinetags" },
        ],
      ],
      [
        /(<)(node)/,
        [
          { token: "delimiter.start", bracket: "@open" },
          { token: "tag", bracket: "@open", next: "@nodetags" },
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
        /(<)(param)/,
        [
          { token: "delimiter.start", bracket: "@open" },
          { token: "tag", bracket: "@open", next: "@paramtags" },
        ],
      ],
      [
        /(<)(rosparam)/,
        [
          { token: "delimiter.start", bracket: "@open" },
          { token: "tag", bracket: "@open", next: "@rosparamtags" },
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
        /(<)(test)/,
        [
          { token: "delimiter.start", bracket: "@open" },
          { token: "tag", bracket: "@open", next: "@testtags" },
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
      [
        /(@qualifiedLaunchAttrs)(\s*=\s*)(")/,
        ["attribute.name", "", { token: "", bracket: "@open", next: "@value" }],
      ],
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
    machinetags: [
      [/\s+/, ""],
      [
        /(@qualifiedMachineAttrs)(\s*=\s*)(")/,
        ["attribute.name", "", { token: "", bracket: "@open", next: "@value" }],
      ],
      [
        /(@qualifiedMachineAttrs)(\s*=\s*)(')/,
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
      [
        /(@qualifiedNodeAttrs)(\s*=\s*)(")/,
        ["attribute.name", "", { token: "", bracket: "@open", next: "@value" }],
      ],
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
    includetags: [
      [/\s+/, ""],
      [
        /(@qualifiedIncludeAttrs)(\s*=\s*)(")/,
        ["attribute.name", "", { token: "", bracket: "@open", next: "@value" }],
      ],
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
    remaptags: [
      [/\s+/, ""],
      [
        /(@qualifiedRemapAttrs)(\s*=\s*)(")/,
        ["attribute.name", "", { token: "", bracket: "@open", next: "@value" }],
      ],
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
      [
        /(@qualifiedEnvAttrs)(\s*=\s*)(")/,
        ["attribute.name", "", { token: "", bracket: "@open", next: "@value" }],
      ],
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
    paramtags: [
      [/\s+/, ""],
      [
        /(@qualifiedParamAttrs)(\s*=\s*)(")/,
        ["attribute.name", "", { token: "", bracket: "@open", next: "@value" }],
      ],
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
    rosparamtags: [
      [/\s+/, ""],
      [
        /(@qualifiedRosParamAttrs)(\s*=\s*)(")/,
        ["attribute.name", "", { token: "", bracket: "@open", next: "@value" }],
      ],
      [
        /(@qualifiedRosParamAttrs)(\s*=\s*)(')/,
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
      [
        /(@qualifiedGroupAttrs)(\s*=\s*)(")/,
        ["attribute.name", "", { token: "", bracket: "@open", next: "@value" }],
      ],
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
    testtags: [
      [/\s+/, ""],
      [
        /(@qualifiedTestAttrs)(\s*=\s*)(")/,
        ["attribute.name", "", { token: "", bracket: "@open", next: "@value" }],
      ],
      [
        /(@qualifiedTestAttrs)(\s*=\s*)(')/,
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
      [
        /(@qualifiedArgAttrs)(\s*=\s*)(")/,
        ["attribute.name", "", { token: "", bracket: "@open", next: "@value" }],
      ],
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
    xmltags: [
      [/\s+/, ""],
      [
        /(@qualifiedXmlAttrs)(\s*=\s*)(")/,
        ["attribute.name", "", { token: "", bracket: "@open", next: "@value" }],
      ],
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
      [/-->/, { token: "comment", bracket: "@close", next: "@pop" }],
      [/<!--/, "comment.content.invalid"],
      [/[<-]/, "comment.content"],
    ],
  },
};
