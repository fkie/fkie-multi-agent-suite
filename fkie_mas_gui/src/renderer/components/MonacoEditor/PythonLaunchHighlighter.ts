import { languages } from "monaco-editor/esm/vs/editor/editor.api";

export const PythonLanguage: languages.IMonarchLanguage = {
  defaultToken: "",

  keywords: [
    // This section is the result of running
    // `for k in keyword.kwlist: print('  "' + k + '",')` in a Python REPL,
    // though note that the output from Python 3 is not a strict superset of the
    // output from Python 2.
    "False", // promoted to keyword.kwlist in Python 3
    "None", // promoted to keyword.kwlist in Python 3
    "True", // promoted to keyword.kwlist in Python 3
    "and",
    "as",
    "assert",
    "async", // new in Python 3
    "await", // new in Python 3
    "break",
    "class",
    "continue",
    "def",
    "del",
    "elif",
    "else",
    "except",
    "exec", // Python 2, but not 3.
    "finally",
    "for",
    "from",
    "global",
    "if",
    "import",
    "in",
    "is",
    "lambda",
    "nonlocal", // new in Python 3
    "not",
    "or",
    "pass",
    "print", // Python 2, but not 3.
    "raise",
    "return",
    "try",
    "while",
    "with",
    "yield",

    "int",
    "float",
    "long",
    "complex",
    "hex",

    "abs",
    "all",
    "any",
    "apply",
    "basestring",
    "bin",
    "bool",
    "buffer",
    "bytearray",
    "callable",
    "chr",
    "classmethod",
    "cmp",
    "coerce",
    "compile",
    "complex",
    "delattr",
    "dict",
    "dir",
    "divmod",
    "enumerate",
    "eval",
    "execfile",
    "file",
    "filter",
    "format",
    "frozenset",
    "getattr",
    "globals",
    "hasattr",
    "hash",
    "help",
    "id",
    "input",
    "intern",
    "isinstance",
    "issubclass",
    "iter",
    "len",
    "locals",
    "list",
    "map",
    "max",
    "memoryview",
    "min",
    "next",
    "object",
    "oct",
    "open",
    "ord",
    "pow",
    "print",
    "property",
    "reversed",
    "range",
    "raw_input",
    "reduce",
    "reload",
    "repr",
    "reversed",
    "round",
    "self",
    "set",
    "setattr",
    "slice",
    "sorted",
    "staticmethod",
    "str",
    "sum",
    "super",
    "tuple",
    "type",
    "unichr",
    "unicode",
    "vars",
    "xrange",
    "zip",

    "__dict__",
    "__methods__",
    "__members__",
    "__class__",
    "__bases__",
    "__name__",
    "__mro__",
    "__subclasses__",
    "__init__",
    "__import__",
  ],

  brackets: [
    { open: "{", close: "}", token: "delimiter.curly" },
    { open: "[", close: "]", token: "delimiter.bracket" },
    { open: "(", close: ")", token: "delimiter.parenthesis" },
  ],

  tokenizer: {
    root: [
      // IMPORTS
      [/(\w+)(,)/, ["attribute.name", "delimiter"]],
      [/\b(import|as|from)\b(\ \w+\.?\w+)/, ["tag", "attribute.name"]],

      // for return values => return foo
      [/(\breturn\b)(\ [a-z]\w+)/, ["tag", "attribute.name"]], // first character has to be lowercase

      // for values => : foo
      [/(\:)(\ ?[a-z]\w+)/, ["delimiter", "attribute.name"]], // first character has to be lowercase

      // for values => , foo but not , 'foo'
      [/(,)(\ ?\b[a-zA-Z_]\w*\b)/, ["delimiter", "attribute.name"]],

      { include: "@whitespace" },
      { include: "@numbers" },
      { include: "@strings" },

      [/[,:;]/, "delimiter"],
      [/[{}\[\]()]/, "@brackets"],

      // for functions => foo(<content>)
      [
        /(\w+)(\()(.*|)(\))/s,
        [
          "subst.arg",
          "delimiter",
          "attribute.name", // TODO: dont match strings (when in single line)
          "delimiter",
        ],
      ],

      // for functions => foo( | Bar(
      [/(\w+)(\()/, ["subst.arg", "delimiter"]],

      // for values => = bar
      [/(=)(\ ?\w.+)/, ["delimiter", "attribute.name"]],

      // alternative regex: /(=)(\s?[a-zA-Z_]\w*)/

      // for values => foo =
      [/(\w+)(\ ?=)/, ["attribute.name", "delimiter"]],

      // alternative regex: /([a-zA-Z_]\w*\s?)(=)/

      // for values => foo.bar.
      [/(\w+)(\.)/, ["attribute.name", "delimiter"]],

      // alternative regex: /([a-zA-Z_]\w*)(\.)/

      [
        /[a-zA-Z]\w*/,
        {
          cases: {
            "@keywords": "tag",
            "@default": "identifier",
          },
        },
      ],
    ],

    // Deal with white space, including single and multi-line comments
    whitespace: [
      [/\s+/, "white"],
      [/(^#.*$)/, "comment"],
      [/('''.*''')|(""".*""")/, "string"],
      [/'''.*$/, "string", "@endDocString"],
      [/""".*$/, "string", "@endDblDocString"],
    ],
    endDocString: [
      [/\\'/, "string"],
      [/.*'''/, "string", "@popall"],
      [/.*$/, "string"],
    ],
    endDblDocString: [
      [/\\"/, "string"],
      [/.*"""/, "string", "@popall"],
      [/.*$/, "string"],
    ],

    // Recognize hex, negatives, decimals, imaginaries, longs, and scientific notation
    numbers: [
      [/-?0x([abcdef]|[ABCDEF]|\d)+[lL]?/, "number.hex"],
      [/-?(\d*\.)?\d+([eE][+\-]?\d+)?[jJ]?[lL]?/, "number"],
    ],

    // Recognize strings, including those broken across lines with \ (but not without)
    strings: [
      [/'$/, "string.escape", "@popall"],
      [/f'{1,3}/, "string.escape", "@fStringBody"],
      [/'/, "string.escape", "@stringBody"],
      [/"$/, "string.escape", "@popall"],
      [/f"{1,3}/, "string.escape", "@fDblStringBody"],
      [/"/, "string.escape", "@dblStringBody"],
    ],
    fStringBody: [
      [/[^\\'\{\}]+$/, "string", "@popall"],
      [/[^\\'\{\}]+/, "string"],
      [/\{[^\}':!=]+/, "identifier", "@fStringDetail"],
      [/\\./, "string"],
      [/'/, "string.escape", "@popall"],
      [/\\$/, "string"],
    ],
    stringBody: [
      [/[^\\']+$/, "string", "@popall"],
      [/[^\\']+/, "string"],
      [/\\./, "string"],
      [/'/, "string.escape", "@popall"],
      [/\\$/, "string"],
    ],
    fDblStringBody: [
      [/[^\\"\{\}]+$/, "string", "@popall"],
      [/[^\\"\{\}]+/, "string"],
      [/\{[^\}':!=]+/, "identifier", "@fStringDetail"],
      [/\\./, "string"],
      [/"/, "string.escape", "@popall"],
      [/\\$/, "string"],
    ],
    dblStringBody: [
      [/[^\\"]+$/, "string", "@popall"],
      [/[^\\"]+/, "string"],
      [/\\./, "string"],
      [/"/, "string.escape", "@popall"],
      [/\\$/, "string"],
    ],
    fStringDetail: [
      [/[:][^}]+/, "string"],
      [/[!][ars]/, "string"], // only !a, !r, !s are supported by f-strings: https://docs.python.org/3/tutorial/inputoutput.html#formatted-string-literals
      [/=/, "string"],
      [/\}/, "identifier", "@pop"],
    ],
  },
};
