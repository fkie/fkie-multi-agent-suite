import { languages } from "monaco-editor/esm/vs/editor/editor.api";

const tagsArray = [
  "launch",
  "include",
  "group",
  "let",
  "arg",
  "executable",
  "param",
  "remap",
  "env",
  "set_env",
  "unset_env",
  "push_ros_namespace",
  "timer",
  "node_container",
  "node",
  "load_composable_node",
  "composable_node",
  "extra_arg",
  "\\?xml",
];

export const Ros2XmlLanguage: languages.IMonarchLanguage = {
  defaultToken: "",
  ignoreCase: true,

  qualifiedTags: new RegExp(`${tagsArray.join("|")}`),

  qualified_launch_attrs: /if|unless/,
  qualified_include_attrs: /if|unless|file/,
  qualified_arg_attrs: /if|unless|name|value|default|description/,
  qualified_let_attrs: /if|unless|name|value/,
  qualified_executable_attrs: /if|unless|cmd|cwd|name|ros_args|args|namespace|launch-prefix|output|shell/,
  qualified_node_attrs: /if|unless|pkg|exec|name|args|respawn|required|namespace|output|cwd|launch-prefix/,
  qualified_node_container_attrs: /if|unless|pkg|exec|name|args|respawn|required|namespace|output|cwd|launch-prefix/,
  qualified_param_attrs: /if|unless|name|value|type|sep|from/,
  qualified_extra_arg_attrs: /if|unless|name|value/,
  qualified_remap_attrs: /if|unless|from|to/,
  qualified_env_attrs: /if|unless|name|value/,
  qualified_set_env_attrs: /if|unless|name|value/,
  qualified_group_attrs: /if|unless|scoped/,
  qualified_unset_env_attrs: /if|unless|name/,
  qualified_push_ros_namespace_attrs: /if|unless|namespace/,
  qualified_timer_attrs: /period/,
  qualified_load_composable_node_attrs: /if|unless|target/,
  qualified_composable_node_attrs: /if|unless|name|pkg|plugin/,
  qualified_xml_attrs: /version/,

  qualifiedSubs: /find-pkg-prefix|find-pkg-share|find-exec|exec-in-package|var|env|eval|dirname|command/,

  // The main tokenizer for our languages
  tokenizer: {
    root: [
      // Opening tags with their corresponding attribute options
      // for each attribute in 'tagsArray' you have to define a 'qualified_{attribute}_attrs'
      ...tagsArray.map((attr) => {
        const attr_normalized = attr.replace("\\?", "");
        return [
          new RegExp(`(<)(${attr})`),
          [
            { token: "delimiter.start", bracket: "@open" },
            { token: "tag", bracket: "@open", next: `@${attr_normalized}` },
          ],
        ] as languages.IMonarchLanguageRule;
      }),

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

    // Qualified_{attribute}_attr
    ...Object.assign(
      {},
      ...tagsArray.map((attr) => {
        const attr_normalized = attr.replace("\\?", "");
        return {
          [attr_normalized]: [
            [/\s+/, ""],
            [
              new RegExp(`(@qualified_${attr_normalized}_attrs)(\\s*=\\s*)(")`),
              ["attribute.name", "", { token: "", bracket: "@open", next: "@value" }],
            ],
            [
              new RegExp(`(@qualified_${attr_normalized}_attrs)(\\s*=\\s*)(')`),
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
        } as languages.IMonarchLanguageRule;
      })
    ),

    // special attributes
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
