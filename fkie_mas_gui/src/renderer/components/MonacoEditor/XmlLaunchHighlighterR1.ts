import { languages } from "monaco-editor/esm/vs/editor/editor.api";

export const Ros1XmlLanguage: languages.IMonarchLanguage = {
  defaultToken: "",
  ignoreCase: true,

  qualifiedTags: /launch|node|machine|remap|env|param|rosparam|group|test|arg/,

  qualifiedLaunchAttrs: /depricated/,
  qualifiedMachineAttrs: /name|address|env-loader|default|user|password|timeout|ros-root|ros-package-path/,
  qualifiedNodeAttrs: /pkg|type|name|args|machine|respawn|respawn_delay|required|ns|clear_params|output|cwd|launch-prefix|if/,
  qualifiedIncludeAttrs: /file|ns|clear_params|pass_all_args/,
  qualifiedRemapAttrs: /from|to/,
  qualifiedEnvAttrs: /name|value/,
  qualifiedParamAttrs: /name|value|type|textfile|binfile|command/,
  qualifiedRosParamAttrs: /command|file|param|ns|subst_value/,
  qualifiedGroupAttrs: /ns|clear_params/,
  qualifiedTestAttrs: /pkg|test-name|type|name|args|clear_params|cwd|launch-prefix|ns|retry|time-limit/,
  qualifiedArgAttrs: /name|default|value|doc/,
  qualifiedXmlAttrs: /version/,
  
  qualifiedSubs: /find-pkg-prefix|find-pkg-share|find-exec|exec-in-package|var|env|eval|dirname/,

  qualifiedName: /(?:[\w.-]+:)?[\w.-]+/,

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
        /(<\?)(xml)/,
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
      [/(>)/, [{ token: "delimiter.end", bracket: "@close" }]],
      [/<!--/, "comment", "@comment"],

      // Meta tags - instruction
      [
        /(<\?)(@qualifiedName)/,
        [
          { token: "delimiter.start", bracket: "@open" },
          { token: "metatag.instruction", next: "@tag" },
        ],
      ],

      // Meta tags - declaration
      [
        /(<!)(@qualifiedName)/,
        [
          { token: "delimiter.start", bracket: "@open" },
          { token: "metatag.declaration", next: "@tag" },
        ],
      ],
    ],
    tag: [
      [/[ \t\r\n]+/, ""],
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
    launchtags: [
      [/[ \t\r\n]+/, ""],
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
      [/[ \t\r\n]+/, ""],
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
      [/[ \t\r\n]+/, ""],
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
      [/[ \t\r\n]+/, ""],
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
      [/[ \t\r\n]+/, ""],
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
      [/[ \t\r\n]+/, ""],
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
      [/[ \t\r\n]+/, ""],
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
      [/[ \t\r\n]+/, ""],
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
      [/[ \t\r\n]+/, ""],
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
      [/[ \t\r\n]+/, ""],
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
      [/[ \t\r\n]+/, ""],
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
      [/[ \t\r\n]+/, ""],
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
      [/[ \t\r\n]+/, ""],
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
