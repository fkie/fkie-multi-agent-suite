import { generateUniqueId } from "../utils";

/**
 * Return the filename from a given path
 */
export function getFileName(path: string): string {
  if (!path) return path;
  return path.replace(/^.*[\\/]/, "");
}

/**
 * Return the filename without extension from a given path
 */
export function getBaseName(path: string): string {
  const fileName = getFileName(path);
  if (fileName) {
    const base = fileName
      .replace(/^.*[\\/]/, "")
      .split(".")
      .shift();
    if (base) {
      return base;
    }
  }
  return fileName;
}

/**
 * Return the first and last letter of the file base name
 */
export function getFileAbb(path: string): string {
  if (!path) return path;
  const baseSplits = path.replace(/^.*[\\/]/, "").split(".");
  let base = baseSplits.shift();
  if (!base) base = baseSplits.shift();
  if (base) {
    return `${base[0]}${base[base.length - 1]}`;
  }
  return path;
}

/**
 * Return the file extension from a given path
 */
export function getFileExtension(path: string): string {
  return `${path.split(".").pop()}`;
}

/**
 * FileItem models arguments for remote files.
 */
export default class FileItem {
  /**
   * unique ID for file
   */
  id: string;

  /**
   * path of the file.
   */
  path: string;

  /**
   * name of the file in path
   */
  fileName: string;

  realpath: string | undefined;

  /**
   * Extension
   */
  extension: string;

  /**
   * language ( C++, Python, TypeScript etc...)
   */
  language: string;

  readonly: boolean;

  /**
   * Value
   */
  value: string;

  /**
   * Value
   */
  mTime: number;

  /**
   * IP of the host machine.
   */
  host: string;

  /** Encoding ov the value */
  encoding: string = "utf-8";

  constructor(
    host: string,
    path: string,
    extension: string = "",
    language: string = "",
    readonly: boolean = false,
    value: string = "",
    mTime: number = 0
  ) {
    this.id = generateUniqueId();
    this.host = host;
    this.path = path;
    this.fileName = getFileName(path);
    this.extension = extension;
    this.language = language;
    this.readonly = readonly;
    this.value = value;
    this.mTime = mTime;
  }
}

// languages that have rich IntelliSense and validation
//   TypeScript
//   JavaScript
//   CSS
//   LESS
//   SCSS
//   JSON
//   HTML

// languages with only basic syntax colorization
//   XML
//   PHP
//   C#
//   C++
//   Razor
//   Markdown
//   Diff
//   Java
//   VB
//   CoffeeScript
//   Handlebars
//   Batch
//   Pug
//   F#
//   Lua
//   Powershell
//   Python
//   Ruby
//   SASS
//   R
//   Objective-C

export const FileLanguageAssociations: Record<string, string> = {
  xml: "ros2xml",
  world: "xml",
  launch: "xml",
  urdf: "xml",
  srdf: "xml",
  xacro: "xml",
  json: "json",
  perspective: "json",
  js: "javascript",
  jsx: "javascript",
  ts: "typescript",
  tsx: "typescript",
  cpp: "cpp",
  h: "cpp",
  hpp: "cpp",
  py: "python",
  lua: "lua",
  md: "markdown",
  html: "html",
  htm: "html",
  css: "css",
  yaml: "yaml",
  rviz: "yaml",
  iface: "yaml",
  ini: "ini",
  protobuf: "protobuf",
  bash: "shell",
  sh: "shell",
  sql: "sql",
};
