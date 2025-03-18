import { languages } from "monaco-editor/esm/vs/editor/editor.api";

export const Ros2XmlLanguage: languages.IMonarchLanguage = {
  defaultToken: "",
  ignoreCase: true,

  qualifiedTags: /launch|include|group|let|arg|executable|node|param|remap|env|sev_env|unset_env|push_ros_namespace|xml/,

  qualifiedIncludeAttrs: /file/,
  qualifiedArgAttrs: /name|default/,
  qualifiedLetAttrs: /name|default|value|description|if|unless/,
  qualifiedExecutableAttrs: /cmd|cwd|name|launch-prefix|output|shell/,
  qualifiedNodeAttrs: /pkg|exec|name|args|respawn|required|namespace|output|cwd|launch-prefix|if/,
  qualifiedParamAttrs: /name|value|value-sep|from/,
  qualifiedRemapAttrs: /from|to/,
  qualifiedEnvAttrs: /name|value/,
  qualifiedSetEnvAttrs: /name|value|if|unless/,
  qualifiedUnsetEnvAttrs: /name|if|unless/,
  qualifiedPushRosNamespaceAttrs: /namespace/,
  qualifiedXmlAttrs: /version/,
  // TODO: find where scoped, ros_args, sep goes

  qualifiedSubs: /find-pkg-prefix|find-pkg-share|find-exec|exec-in-package|var|env|eval|dirname|command/,

  qualifiedName: /(?:[\w.-]+:)?[\w.-]+/,
  // The main tokenizer for our languages
  tokenizer: {
    root: [
      // Opening tags with defined attributes
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
        /(<)(node)/,
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
        /(<\?)(xml)/,
        [
          { token: "delimiter.start", bracket: "@open" },
          { token: "tag", bracket: "@open", next: "@xmltags" },
        ],
      ],

      // Standard opening tag
      // For tags without any attributes: launch, group
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
        /(@qualifiedLetAttrs)(\s*=\s*)(")/, // TODO: dont match any attributes
        ["attribute.name", "", { token: "", bracket: "@open", next: "@value" }],
      ],
      [
        /(@qualifiedLetAttrs)(\s*=\s*)(')/, // TODO: dont match any attributes
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
    lettags: [
      [/[ \t\r\n]+/, ""],
      [
        /(@qualifiedLetAttrs)(\s*=\s*)(")/,
        ["attribute.name", "", { token: "", bracket: "@open", next: "@value" }],
      ],
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
      [/[ \t\r\n]+/, ""],
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
    setenvtags: [
      [/[ \t\r\n]+/, ""],
      [
        /(@qualifiedSetEnvAttrs)(\s*=\s*)(")/,
        ["attribute.name", "", { token: "", bracket: "@open", next: "@value" }],
      ],
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
      [/[ \t\r\n]+/, ""],
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
      [/[ \t\r\n]+/, ""],
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
