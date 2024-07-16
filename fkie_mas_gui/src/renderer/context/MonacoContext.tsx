/* eslint-disable max-classes-per-file */
import * as Monaco from '@monaco-editor/react'
import React, { createContext, useCallback, useContext, useEffect, useMemo, useState } from 'react'
import { FileItem } from '../models'
import { LoggingContext } from './LoggingContext'
import { RosContext } from './RosContext'

export class ModifiedTabsInfo {
  tabId: string = ''

  providerId: string = ''

  uriPaths: string[] = []

  constructor(tabId: string, providerId: string, uriPaths: string[]) {
    this.tabId = tabId
    this.providerId = providerId
    this.uriPaths = uriPaths
  }
}

export class SaveResult {
  tabId: string = ''

  file: string = ''

  result: boolean = false

  providerId: string = ''

  message: string = ''

  constructor(tabId: string, file: string, result: boolean, providerId: string, message: string) {
    this.tabId = tabId
    this.file = file
    this.result = result
    this.providerId = providerId
    this.message = message
  }
}

export interface IMonacoContext {
  monaco: Monaco.Monaco | null
  updateModifiedFiles: (tabId: string, providerId: string, uriPaths: string[]) => void
  getModifiedTabs: () => ModifiedTabsInfo[]
  getModifiedFilesByTab: (tabId: string) => ModifiedTabsInfo | undefined
  saveModifiedFilesOfTabId: (tabId: string) => Promise<SaveResult[]>
}

export interface IMonacoProvider {
  children: React.ReactNode
}
export const DEFAULT_MONACO = {
  monaco: null,
  updateModifiedFiles: () => null,
  getModifiedTabs: () => [],
  getModifiedFilesByTab: () => undefined,
  saveModifiedFilesOfTabId: () => {
    return Promise.resolve([])
  }
}

export const MonacoContext = createContext<IMonacoContext>(DEFAULT_MONACO)

export function MonacoProvider({
  children
}: IMonacoProvider): ReturnType<React.FC<IMonacoProvider>> {
  const monaco = Monaco.useMonaco()
  const rosCtx = useContext(RosContext)
  const logCtx = useContext(LoggingContext)

  const [modifiedFiles, setModifiedFiles] = useState<ModifiedTabsInfo[]>([])

  useEffect(() => {
    monaco?.languages.typescript.javascriptDefaults.setEagerModelSync(true)
    monaco?.languages.typescript.typescriptDefaults.setEagerModelSync(true)
  }, [monaco])

  const updateModifiedFiles = useCallback(
    (tabId: string, providerId: string, uriPaths: string[]) => {
      if (uriPaths.length > 0) {
        // add to the list
        const newFilesInfo: ModifiedTabsInfo = new ModifiedTabsInfo(tabId, providerId, uriPaths)
        setModifiedFiles((prev) => {
          return [...prev.filter((item) => item.tabId !== tabId), newFilesInfo]
        })
      } else {
        // remove from the list
        setModifiedFiles((prev) => {
          return prev.filter((item) => item.tabId !== tabId)
        })
      }
    },
    [setModifiedFiles]
  )

  const getModifiedTabs: () => ModifiedTabsInfo[] = useCallback(() => {
    return modifiedFiles
  }, [modifiedFiles])

  const getModifiedFilesByTab: (tabId: string) => ModifiedTabsInfo | undefined = useCallback(
    (tabId) => {
      return modifiedFiles.filter((item) => item.tabId === tabId)[0]
    },
    [modifiedFiles]
  )

  const saveModifiedFilesOfTabId: (tabId: string) => Promise<SaveResult[]> = useCallback(
    async (tabId) => {
      if (!monaco) return Promise.resolve([])
      const result: SaveResult[] = []
      const tabInfos: ModifiedTabsInfo[] = modifiedFiles.filter((item) => item.tabId === tabId)
      if (tabInfos.length === 0) return Promise.resolve([])
      const tabInfo: ModifiedTabsInfo = tabInfos[0]
      await Promise.all(
        tabInfo.uriPaths.map(async (uriPath) => {
          const path = uriPath.split(':')[1]
          const saveItem: SaveResult = new SaveResult(tabId, path, false, tabInfo.providerId, '')
          const editorModel = monaco.editor.getModel(monaco.Uri.file(uriPath))
          if (editorModel) {
            const path = editorModel.uri.path.split(':')[1]
            // TODO change encoding if the file is encoded as HEX
            const fileToSave = new FileItem('', path, '', '', editorModel.getValue())
            const providerObj = rosCtx.getProviderById(tabInfo.providerId, false)
            if (providerObj) {
              const saveResult = await providerObj.saveFileContent(fileToSave)
              if (saveResult.bytesWritten > 0) {
                logCtx.success(`Successfully saved file`, `path: ${path}`)
                saveItem.result = true
              } else {
                saveItem.message = `Error while save file ${path}: ${saveResult.error}`
                logCtx.error(`Error while save file ${path}`, `${saveResult.error}`)
              }
            } else {
              saveItem.message = `Provider ${tabInfo.providerId} not found`
              logCtx.error(`Provider ${tabInfo.providerId} not found`, `can not save file: ${path}`)
            }
          } else {
            saveItem.message = 'Model not found'
          }
          result.push(saveItem)
        })
      )
      return Promise.resolve(result)
    },
    [logCtx, modifiedFiles, monaco, rosCtx]
  )

  const attributesMemo = useMemo(
    () => ({
      monaco,
      modifiedFiles,
      setModifiedFiles,
      getModifiedTabs,
      updateModifiedFiles,
      getModifiedFilesByTab,
      saveModifiedFilesOfTabId
    }),
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [
      monaco,
      modifiedFiles,
      setModifiedFiles,
      getModifiedTabs,
      updateModifiedFiles,
      getModifiedFilesByTab,
      saveModifiedFilesOfTabId
    ]
  )

  return <MonacoContext.Provider value={attributesMemo}>{children}</MonacoContext.Provider>
}
