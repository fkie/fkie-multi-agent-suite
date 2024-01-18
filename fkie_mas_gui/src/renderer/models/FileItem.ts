import { generateUniqueId } from '../utils';

/**
 * Return the filename from a given path
 *
 * @param {string} path - File path
 */
const getFileName = (path: string) => {
  return path.replace(/^.*[\\/]/, '');
};

/**
 * Return the file extension from a given path
 *
 * @param {string} path - File path
 */
const getFileExtension = (path: string) => {
  return `${path.split('.').pop()}`;
};

/**
 * FileItem models arguments for remote files.
 */
class FileItem {
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

  /**
   * Extension
   */
  extension: string;

  /**
   * language ( C++, Python, TypeScript etc...)
   */
  language: string;

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
  encoding: string = 'utf-8';

  /**
   * Class Constructor
   *
   * @param {string} host - IP of the host machine.
   * @param {string} path - file path.
   * @param {string} extension - file extension.
   * @param {string} language - file type.
   * @param {string} value - file content.
   * @param {number} mTime - Date of last modification of the file.
   */
  constructor(
    host: string,
    path: string,
    extension: string = '',
    language: string = '',
    value: string = '',
    mTime: number = 0,
  ) {
    this.id = generateUniqueId();
    this.host = host;
    this.path = path;
    this.fileName = getFileName(path);
    this.extension = extension;
    this.language = language;
    this.value = value;
    this.mTime = mTime;
  }
}

export default FileItem;

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

const FileLanguageAssociations: Record<string, string> = {
  xml: 'xml',
  world: 'xml',
  launch: 'xml',
  urdf: 'xml',
  srdf: 'xml',
  xacro: 'xml',
  json: 'json',
  perspective: 'json',
  js: 'javascript',
  jsx: 'javascript',
  ts: 'typescript',
  tsx: 'typescript',
  cpp: 'cpp',
  h: 'cpp',
  hpp: 'cpp',
  py: 'python',
  lua: 'lua',
  md: 'markdown',
  html: 'html',
  htm: 'html',
  css: 'css',
  yaml: 'yaml',
  rviz: 'yaml',
  iface: 'yaml',
  ini: 'ini',
  protobuf: 'protobuf',
  bash: 'shell',
  sh: 'shell',
  sql: 'sql',
};

export { FileItem, FileLanguageAssociations, getFileExtension, getFileName };
