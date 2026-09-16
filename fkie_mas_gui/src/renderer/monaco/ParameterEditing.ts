import { TFileRange, TParameterRequest } from "@/types";
import { editor } from "monaco-editor";

export type TParameterInsert = {
  range: TFileRange; // range to replace (empty range => plain insert)
  text: string;
};

export type TParameterLookup = {
  found: boolean;
  range?: TFileRange; // location of the existing parameter definition
  insert?: TParameterInsert; // proposal if not found
  error?: string;
};

const XML_EXTENSIONS = ["launch", "xml", "xacro"];

/** offset of the node definition inside the model */
function rangeOffset(model: editor.ITextModel, request: TParameterRequest): number | null {
  const range = request.fileRange;
  if (!range || range.startLineNumber < 1 || range.startLineNumber > model.getLineCount()) return null;
  return model.getOffsetAt({ lineNumber: range.startLineNumber, column: range.startColumn });
}

/** remove node name and private prefixes from parameter name */
function shortParamName(paramName: string, nodeName: string): string {
  let name = paramName;
  if (name.startsWith(nodeName)) name = name.slice(nodeName.length);
  return name.replace(/^[~/]+/, "");
}

function escapeRegExp(value: string): string {
  return value.replace(/[.*+?^${}()|[\]\\]/g, "\\$&");
}

function toRange(model: editor.ITextModel, start: number, end: number): TFileRange {
  const s = model.getPositionAt(start);
  const e = model.getPositionAt(end);
  return {
    startLineNumber: s.lineNumber,
    startColumn: s.column,
    endLineNumber: e.lineNumber,
    endColumn: e.column,
  };
}

function emptyRange(model: editor.ITextModel, offset: number): TFileRange {
  return toRange(model, offset, offset);
}

/** indentation of the first non-empty line within a text fragment, if any */
function firstLineIndent(text: string): string | null {
  for (const line of text.split("\n")) {
    if (line.trim().length > 0) {
      return line.slice(0, line.length - line.trimStart().length);
    }
  }
  return null;
}

function indentAt(model: editor.ITextModel, offset: number): string {
  const content = model.getLineContent(model.getPositionAt(offset).lineNumber);
  return content.slice(0, content.length - content.trimStart().length);
}

/** start offset of the whitespace-only run (spaces/tabs) directly preceding the given offset */
function precedingIndentStart(text: string, offset: number): number {
  let i = offset;
  while (i > 0 && (text[i - 1] === " " || text[i - 1] === "\t")) i -= 1;
  return i;
}

/** one indentation level according to the model's editor settings */
function indentUnit(model: editor.ITextModel): string {
  const opts = model.getOptions();
  return opts.insertSpaces ? " ".repeat(opts.indentSize || opts.tabSize) : "\t";
}

/** find matching closing bracket, ignoring strings and python comments */
function findClosing(text: string, openIdx: number, open: string, close: string): number {
  let depth = 0;
  let quote = "";
  for (let i = openIdx; i < text.length; i += 1) {
    const c = text[i];
    if (quote) {
      if (c === quote && text[i - 1] !== "\\") quote = "";
      continue;
    }
    if (c === '"' || c === "'") {
      quote = c;
      continue;
    }
    if (c === "#") {
      const nl = text.indexOf("\n", i);
      if (nl < 0) return -1;
      i = nl;
      continue;
    }
    if (c === open) depth += 1;
    else if (c === close) {
      depth -= 1;
      if (depth === 0) return i;
    }
  }
  return -1;
}

function xmlValue(value: string | undefined): string {
  return (value ?? "").replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/"/g, "&quot;");
}

const PY_ESCAPES: Record<string, string> = {
  "\\": "\\\\",
  '"': '\\"',
  "\n": "\\n",
  "\r": "\\r",
  "\t": "\\t",
};

/** Renders an arbitrary string as a safe Python double-quoted literal. */
function pythonString(value: string): string {
  let escaped = "";

  // for...of iterates code points, so surrogate pairs stay intact
  for (const ch of value) {
    const mapped = PY_ESCAPES[ch];
    if (mapped !== undefined) {
      escaped += mapped;
      continue;
    }

    // charCodeAt returns number (not number | undefined) -> no assertion needed
    const code = ch.charCodeAt(0);
    // remaining C0 controls and DEL have no printable representation
    if (code < 0x20 || code === 0x7f) {
      escaped += `\\x${code.toString(16).padStart(2, "0")}`;
      continue;
    }

    escaped += ch;
  }

  return `"${escaped}"`;
}
const INT_RE = /^[+-]?\d+$/;
const FLOAT_RE = /^[+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?$/;

/** Converts a JSON value into Python literal syntax. */
function pythonLiteral(value: unknown): string {
  if (value === null) return "None";
  if (typeof value === "boolean") return value ? "True" : "False";
  if (typeof value === "number") return Number.isFinite(value) ? `${value}` : "None";
  if (typeof value === "string") return pythonString(value);
  if (Array.isArray(value)) return `[${value.map(pythonLiteral).join(", ")}]`;

  return `{${Object.entries(value as object)
    .map(([k, v]) => `${pythonString(k)}: ${pythonLiteral(v)}`)
    .join(", ")}}`;
}

function pythonValue(value: string | undefined, type: string | undefined): string {
  if (value === undefined) return '""';
  const raw = `${value}`;

  switch (type) {
    case "bool":
      return raw.trim().toLowerCase() === "true" ? "True" : "False";

    case "int":
      // validate instead of interpolating unchecked input as code
      return INT_RE.test(raw.trim()) ? raw.trim() : "0";

    case "float":
      return FLOAT_RE.test(raw.trim()) ? raw.trim() : "0.0";

    case "list":
    case "str[]":
    case "int[]":
    case "float[]":
    case "bool[]":
      try {
        const parsed = JSON.parse(`[${raw.replace(/^\[|\]$/g, "")}]`);
        return pythonLiteral(parsed);
      } catch {
        // not parseable -> treat the whole input as a single string element
        return `[${pythonString(raw)}]`;
      }

    default:
      return pythonString(raw);
  }
}

/* ------------------------------- XML launch ------------------------------- */

export type TXmlNodeBlock = {
  tag: string;        // "node" | "composable_node" | ...
  tagStart: number;   // index of "<"
  tagEnd: number;     // exclusive end of the opening tag (== bodyStart)
  bodyStart: number;
  bodyEnd: number;    // index of "</tag>" or bodyStart if self-closing
  blockEnd: number;   // exclusive end including the closing tag
  selfClosing: boolean;
};

export const NODE_TAG_NAMES = ["node", "composable_node", "node_container"] as const;

const escapeRe = (s: string) => s.replace(/[.*+?^${}()|[\]\\]/g, "\\$&");

/** Ranges that must be ignored: comments, CDATA, processing instructions. */
function ignoredRanges(text: string): Array<[number, number]> {
  const ranges: Array<[number, number]> = [];
  const re = /<!--[\s\S]*?(?:-->|$)|<!\[CDATA\[[\s\S]*?(?:\]\]>|$)|<\?[\s\S]*?(?:\?>|$)/g;
  for (let m; (m = re.exec(text)); ) ranges.push([m.index, m.index + m[0].length]);
  return ranges;
}

const inRanges = (ranges: Array<[number, number]>, i: number) =>
  ranges.some(([a, b]) => i >= a && i < b);

/** Quote-aware scan for the end of an opening tag starting at `from` ("<"). */
function openTagEnd(text: string, from: number): { end: number; selfClosing: boolean } | null {
  let quote: string | null = null;
  for (let i = from + 1; i < text.length; i++) {
    const ch = text[i];
    if (quote) {
      if (ch === quote) quote = null;
      continue;
    }
    if (ch === '"' || ch === "'") { quote = ch; continue; }
    if (ch === "<") return null;              // malformed: new tag started
    if (ch === ">") {
      let j = i - 1;
      while (j > from && /\s/.test(text[j])) j--;  // allow "/ >"
      return { end: i + 1, selfClosing: text[j] === "/" };
    }
  }
  return null;                                 // unterminated tag
}

/** Find the matching close tag with depth counting. */
function matchingClose(
  text: string,
  tag: string,
  bodyStart: number,
  ignored: Array<[number, number]>,
): { bodyEnd: number; blockEnd: number } | null {
  const re = new RegExp(`<${escapeRe(tag)}(?=[\\s/>])|</${escapeRe(tag)}\\s*>`, "g");
  re.lastIndex = bodyStart;
  let depth = 1;
  for (let m; (m = re.exec(text)); ) {
    if (inRanges(ignored, m.index)) continue;
    if (m[0][1] === "/") {
      if (--depth === 0) return { bodyEnd: m.index, blockEnd: m.index + m[0].length };
    } else {
      const open = openTagEnd(text, m.index);
      if (!open) break;                        // malformed
      if (!open.selfClosing) depth++;
      re.lastIndex = open.end;                 // don't rescan attributes
    }
  }
  return null;
}

export function xmlNodeBlockAt(
  text: string,
  offset: number,
  tagNames: readonly string[] = NODE_TAG_NAMES,
): TXmlNodeBlock | null {
  if (offset < 0 || offset > text.length || tagNames.length === 0) return null;

  const ignored = ignoredRanges(text);
  const re = new RegExp(`<(${tagNames.map(escapeRe).join("|")})(?=[\\s/>])`, "g");

  // Collect all candidate openings starting at or before `offset`.
  const candidates: Array<{ index: number; tag: string }> = [];
  for (let m; (m = re.exec(text)) && m.index <= offset; ) {
    if (!inRanges(ignored, m.index)) candidates.push({ index: m.index, tag: m[1] });
  }

  // Walk from innermost/nearest outwards and return the first block containing `offset`.
  for (let i = candidates.length - 1; i >= 0; i--) {
    const { index: tagStart, tag } = candidates[i];
    const open = openTagEnd(text, tagStart);
    if (!open) continue;

    const bodyStart = open.end;
    let bodyEnd = bodyStart;
    let blockEnd = bodyStart;

    if (!open.selfClosing) {
      const close = matchingClose(text, tag, bodyStart, ignored);
      if (!close) continue;                    // unclosed element -> not a valid block
      bodyEnd = close.bodyEnd;
      blockEnd = close.blockEnd;
    }

    if (offset >= tagStart && offset <= blockEnd) {
      return { tag, tagStart, tagEnd: bodyStart, bodyStart, bodyEnd, blockEnd, selfClosing: open.selfClosing };
    }
  }
  return null;
}

function lookupXml(model: editor.ITextModel, request: TParameterRequest, rosVersion: "1" | "2"): TParameterLookup {
  const offset = rangeOffset(model, request);
  if (offset === null) return { found: false, error: "No node location available for this file" };
  const text = model.getValue();
  const block = xmlNodeBlockAt(text, offset);
  if (!block) return { found: false, error: "No node definition found at the reported position" };

  const short = shortParamName(request.paramName, request.nodeName);
  const body = text.slice(block.bodyStart, block.bodyEnd);
  const paramRe = /<(param|rosparam)\b[^>]*?\/?>/g;
  let match: RegExpExecArray | null = paramRe.exec(body);
  while (match !== null) {
    const nameAttr = /\bname\s*=\s*["']([^"']+)["']/.exec(match[0]);
    if (nameAttr && shortParamName(nameAttr[1], request.nodeName) === short) {
      const start = block.bodyStart + match.index;
      return { found: true, range: toRange(model, start, start + match[0].length) };
    }
    match = paramRe.exec(body);
  }

  // not found -> create insert proposal
  const indent = indentAt(model, block.tagStart);
  const typeAttr = rosVersion === "1" && request.paramType ? ` type="${xmlValue(request.paramType)}"` : "";
  const paramTag = `<param name="${xmlValue(short)}" value="${xmlValue(request.paramValue)}"${typeAttr}/>`;

  if (block.selfClosing) {
    const unit = indentUnit(model);
    // convert <node ... /> into <node ...> <param/> </node>
    return {
      found: false,
      insert: {
        range: toRange(model, block.tagEnd - 2, block.tagEnd),
        text: `>\n${indent}${unit}${paramTag}\n${indent}</node>`,
      },
    };
  }
  const paramIndent = firstLineIndent(body) ?? `${indent}  `;
  const insertStart = precedingIndentStart(text, block.bodyEnd);
  return {
    found: false,
    insert: {
      range: toRange(model, insertStart, block.bodyEnd),
      text: `${paramIndent}${paramTag}\n${indent}`,
    },
  };
}

/* ----------------------------- Python launch ------------------------------ */

/** innermost Node(...) call which encloses the given offset */
function pythonNodeBlockAt(text: string, offset: number): { start: number; end: number } | null {
  const nodeRe = /(?:^|[^\w])(\w*Node)\s*\(/g;
  let block: { start: number; end: number } | null = null;
  let match: RegExpExecArray | null = nodeRe.exec(text);
  while (match !== null && match.index <= offset) {
    const nameEnd = match.index + match[0].length; // index right after "("
    const open = nameEnd - 1;
    const close = findClosing(text, open, "(", ")");
    if (close > offset) block = { start: open, end: close };
    match = nodeRe.exec(text);
  }
  return block;
}

function lookupPython(model: editor.ITextModel, request: TParameterRequest): TParameterLookup {
  const offset = rangeOffset(model, request);
  console.log(`offset: ${JSON.stringify(offset)}`);
  if (offset === null) return { found: false, error: "No node location available for this file" };
  const text = model.getValue();
  const block = pythonNodeBlockAt(text, offset);
  console.log(`block: ${JSON.stringify(block)}`);
  if (!block) return { found: false, error: "No node definition found at the reported position" };

  const short = shortParamName(request.paramName, request.nodeName);
  const nodeContent = text.slice(block.start, block.end);
  const paramsArg = /\bparameters\s*=\s*\[/.exec(nodeContent);

  if (paramsArg) {
    const listOpen = block.start + paramsArg.index + paramsArg[0].length - 1;
    const listClose = findClosing(text, listOpen, "[", "]");
    if (listClose > 0) {
      const list = text.slice(listOpen + 1, listClose);
      const keyRe = new RegExp(`["']${escapeRegExp(short)}["']\\s*:`);
      const keyMatch = keyRe.exec(list);
      if (keyMatch) {
        const start = listOpen + 1 + keyMatch.index;
        // select whole "key": value entry
        const rest = list.slice(keyMatch.index + keyMatch[0].length);
        const valueEnd = rest.search(/[,}]/);
        const end = start + keyMatch[0].length + (valueEnd < 0 ? rest.length : valueEnd);
        return { found: true, range: toRange(model, start, end) };
      }
      const indent = firstLineIndent(list) ?? `${indentAt(model, listOpen)} `;
      const separator = list.trim().length > 0 ? "," : "";
      return {
        found: false,
        insert: {
          range: emptyRange(model, listOpen + 1),
          text: `\n${indent}{"${short}": ${pythonValue(request.paramValue, request.paramType)}}${separator}`,
        },
      };
    }
  }

  // no parameters=[...] at all -> add argument before closing parenthesis
  const before = text.slice(block.start + 1, block.end).replace(/\s+$/, "");
  const insertOffset = block.start + 1 + before.length;
  const needsComma = before.length > 0 && !before.endsWith(",");
  const indent = `${indentAt(model, block.start)}    `;
  return {
    found: false,
    insert: {
      range: emptyRange(model, insertOffset),
      text: `${needsComma ? "," : ""}\n${indent}parameters=[{"${short}": ${pythonValue(
        request.paramValue,
        request.paramType
      )}}],`,
    },
  };
}

/* --------------------------------- public --------------------------------- */

export function locateNodeParameter(
  model: editor.ITextModel,
  request: TParameterRequest,
  rosVersion: "1" | "2"
): TParameterLookup {
  const extension = (model.uri.path.split(".").pop() || "").toLowerCase();
  if (extension === "py" || model.getLanguageId() === "python") {
    return lookupPython(model, request);
  }
  if (XML_EXTENSIONS.includes(extension) || model.getLanguageId() === "xml") {
    return lookupXml(model, request, rosVersion);
  }
  return { found: false, error: `Unsupported file type: [${extension}]` };
}
