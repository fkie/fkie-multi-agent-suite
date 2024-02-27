import { Monaco, useMonaco } from '@monaco-editor/react';
import { Range } from 'monaco-editor/esm/vs/editor/editor.api';
import React, { createContext, useEffect, useMemo } from 'react';
import { FileItem, FileLanguageAssociations } from '../models';

export interface ISearchResult {
  file: string;
  text: string;
  lineNumber: number;
  range: Range;
}
export interface IMonacoContext {
  monaco: Monaco | null;
  existModelFromPath: (filePath: string) => boolean;
  getModelFromPath: (filePath: string) => any | null;
  createModel: (file: FileItem) => boolean;
  updateModel: (file: FileItem) => boolean;
  findAllTextMatches: (
    searchText: string,
    isRegex: boolean,
    onlyInFiles: string[],
  ) => ISearchResult[];
}

export interface ILoggingProvider {
  children: React.ReactNode;
}
export const DEFAULT_MONACO = {
  monaco: null,
  existModelFromPath: () => false,
  getModelFromPath: () => false,
  createModel: () => false,
  updateModel: () => false,
  findAllTextMatches: () => [],
};

export const MonacoContext = createContext<IMonacoContext>(DEFAULT_MONACO);

export function MonacoProvider({
  children,
}: ILoggingProvider): ReturnType<React.FC<ILoggingProvider>> {
  const monaco = useMonaco();

  useEffect(() => {
    monaco?.languages.typescript.javascriptDefaults.setEagerModelSync(true);
    monaco?.languages.typescript.typescriptDefaults.setEagerModelSync(true);
  }, [monaco]);

  /**
   * Search through all available models a given text, and return all coincidences
   *
   * @param {string} searchText - Text to search
   */
  const findAllTextMatches = (
    searchText: string,
    isRegex: boolean = false,
    onlyInFiles: string[] = [],
  ) => {
    if (!searchText) return [];
    if (searchText.length < 3) return [];

    const searchResult: ISearchResult[] = [];
    const includedText = new Set('');

    if (searchText) {
      monaco?.editor.getModels().forEach((model) => {
        if (
          onlyInFiles.length === 0 ||
          onlyInFiles.indexOf(model.uri.path) > -1
        ) {
          const matches = model.findMatches(
            searchText,
            true,
            isRegex,
            false,
            null,
            false,
          );
          matches.forEach((match) => {
            const lineNumber = match.range.startLineNumber;
            const text = model.getLineContent(match.range.startLineNumber);

            if (!includedText.has(text)) {
              searchResult.push({
                file: model.uri.path,
                text,
                lineNumber,
                range: match.range,
              });

              includedText.add(text);
            }
          });
        }
      });
    }
    return searchResult;
  };

  /**
   * Checks if a monaco model was already created for a given file path
   *
   * @param {string} filePath - file path (in the format <host>:<abs_path>)
   */
  const existModelFromPath = (filePath: string) => {
    if (!monaco) return false;

    if (monaco.editor.getModel(monaco.Uri.file(filePath))) return true;

    return false;
  };

  /**
   * Return a monaco model from a given path
   *
   * @param {string} filePath - file path (in the format <host>:<abs_path>)
   */
  const getModelFromPath = (filePath: string) => {
    if (!monaco) return null;
    if (!filePath || filePath.length === 0) return null;

    return monaco.editor.getModel(monaco.Uri.file(filePath));
  };

  /**
   * Create a new monaco model from a given file
   *
   * @param {FileItem} file - Original file
   */
  const createModel = (file: FileItem) => {
    if (!monaco) return false;
    const fullPath = `${file.host}:${file.path}`;
    // create monaco model, if it does not exists yet
    if (!existModelFromPath(fullPath)) {
      monaco.editor.createModel(
        file.value,
        FileLanguageAssociations[file.extension],
        monaco.Uri.file(fullPath),
      );
    }
    return true;
  };

  /**
   * Updates the content of an existing monaco model with a given file
   *
   * @param {FileItem} file - New file content
   */
  const updateModel = (file: FileItem) => {
    const fullPath = `${file.host}:${file.path}`;
    const model = getModelFromPath(fullPath);
    if (model) {
      console.log('updateModel', fullPath);
      model.setValue(file.value);
      return true;
    }
    return false;
  };

  const attributesMemo = useMemo(
    () => ({
      monaco,
      existModelFromPath,
      getModelFromPath,
      createModel,
      updateModel,
      findAllTextMatches,
    }),
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [monaco],
  );

  return (
    <MonacoContext.Provider value={attributesMemo}>
      {children}
    </MonacoContext.Provider>
  );
}
