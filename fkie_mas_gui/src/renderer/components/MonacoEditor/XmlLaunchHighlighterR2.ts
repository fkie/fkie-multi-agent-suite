import { languages } from "monaco-editor/esm/vs/editor/editor.api";

export const Ros2XmlLanguage: languages.IMonarchLanguage = {
  defaultToken: "",
  ignoreCase: true,

  qualifiedTags: /launch|include|group|let|arg|executable|node|param|remap|env|sev_env|unset_env/,
  qualifiedAttrs:
    /if|unless|file|scoped|name|value|default|description|env|cmd|cwd|args|shell|launch-prefix|output|pkg|exec|ros_args|namespace|sep|from|to/,
  qualifiedSubs: /find-pkg-prefix|find-pkg-share|find-exec|exec-in-package|var|env|eval|dirname/,
  qualifiedName: /(?:[\w.-]+:)?[\w.-]+/,
  // The main tokenizer for our languages
  tokenizer: {
    root: [
      // Standard opening tag
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
        /(@qualifiedAttrs)(\s*=\s*)(")/,
        ["attribute.name", "attribute.name", { token: "attribute.value", bracket: "@open", next: "@value" }],
      ],
      [
        /(@qualifiedAttrs)(\s*=\s*)(')/,
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
      [/([^"^$]*)(\$\()/, ["attribute.value", { token: "delimiter.end", bracket: "@open", next: "@subst" }]],
      [/([^"]*)(")/, ["attribute.value", { token: "attribute.value", bracket: "@close", next: "@pop" }]],
    ],
    subst: [
      [/(@qualifiedSubs)(\s*)([^)]*)/, ["subst.key", "", "subst.arg"]],
      [/(\))/, { token: "delimiter.end", bracket: "@close", next: "@pop" }],
    ],
    value_sq: [
      [/([^'^$]*)(\$\()/, ["attribute.value", { token: "delimiter.end", bracket: "@open", next: "@subst" }]],
      [/([^']*)(')/, ["attribute.value", { token: "attribute.value", bracket: "@close", next: "@pop" }]],
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
