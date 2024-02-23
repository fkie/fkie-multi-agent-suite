import PropTypes from 'prop-types';
import { useCallback, useContext, useEffect, useState } from 'react';

import { Alert, Stack } from '@mui/material';
import MonacoEditor from '../../../components/MonacoEditor/MonacoEditor';
import { LoggingContext } from '../../../context/LoggingContext';
import { MonacoContext } from '../../../context/MonacoContext';
import { RosContext } from '../../../context/RosContext';
import { SettingsContext } from '../../../context/SettingsContext';


import { LaunchIncludedFilesRequest } from '../../../models';

function FileEditorPanel({
  providerId,
  rootFilePath,
  currentFilePath,
  fileRange,
}) {
  const rosCtx = useContext(RosContext);
  const logCtx = useContext(LoggingContext);
  const monacoCtx = useContext(MonacoContext);
  const settingsCtx = useContext(SettingsContext);

  const [currentFile, setCurrentFile] = useState(null);
  const [includedFiles, setIncludedFiles] = useState(null);
  const [titlePanel, setTitlePanel] = useState('Loading File...');
  const [notificationDescription, setNotificationDescription] = useState('');

  const [monacoDisposables, setMonacoDisposables] = useState([]);

  // clear all component states, used when no provider/host info is available
  const clearStates = () => {
    setCurrentFile(null);
    setNotificationDescription('');
  };

  // Most important function:
  //  when the component is mounted, this callback will execute following steps:
  //  - set title of the panel
  //  - get the content of [currentFilePath]
  //  - create a monaco model (file in editor) based on [currentFilePath]
  //  - check if include files are available (for xml and launch files for example)
  //  -   if available, download all include files and create their corresponding models
  // TODO: Shall we download the models per request? the problem is then the recursive text search
  useEffect(() => {
    if (!monacoCtx.monaco) {
      // monaco is not yet available
      return;
    }

    if (!currentFilePath || currentFilePath.length === 0) {
      setNotificationDescription('[currentFilePath] Invalid file path');
      return;
    }

    if (!rootFilePath || rootFilePath.length === 0) {
      setNotificationDescription('[rootFilePath] Invalid file path');
      return;
    }

    // search host based on selected provider
    const provider = rosCtx.getProviderById(providerId);

    if (!provider) {
      clearStates();
      setNotificationDescription('No providers available');
      return;
    }

    if (provider && !provider.host()) {
      logCtx.error(
        'The provider does not have configured any host.',
        'Please check your provider configuration',
      );

      clearStates();

      setNotificationDescription(
        'The provider does not have configured any host.',
      );
      return;
    }

    setNotificationDescription('Getting file from provider...');

    const getFileAndIncludesAsync = async () => {
      const result = await provider.getFileContent(currentFilePath);
      if (result.error) {
        console.error(
          `Could not open file: [${result.file.fileName}]: ${result.error}`,
        );
        setNotificationDescription(
          `Could not open file: [${result.file.fileName}]: ${result.error}`,
        );
        return;
      }
      // save files
      setCurrentFile(result.file);

      if (!monacoCtx.createModel(result.file)) {
        console.error(`Could not create model for: [${result.file.fileName}]`);
        setNotificationDescription(
          `Could not create model for: [${result.file.fileName}]`,
        );
        return;
      }

      // Ignore "non-launch" files
      // TODO: Add parameter Here
      if (!['launch', 'xml', 'xacro'].includes(result.file.extension)) {
        setIncludedFiles([]);
        setNotificationDescription('');
        return;
      }

      if (!provider.launchGetIncludedFiles) {
        setNotificationDescription('');
        return;
      }

      // if file is a launch or XML, try to fetch included files
      const request = new LaunchIncludedFilesRequest();
      request.path = rootFilePath;
      request.unique = true;
      request.recursive = true;

      const includedFilesLocal = await provider.launchGetIncludedFiles(request);

      // get file content and create corresponding monaco models

      // filter unique file names (in case multiple imports)
      const uniqueIncludedFiles = [rootFilePath];
      includedFilesLocal.forEach((f) => {
        if (!uniqueIncludedFiles.includes(f.inc_path))
          uniqueIncludedFiles.push(f.inc_path);
      });

      // get file content and create corresponding monaco models
      uniqueIncludedFiles.forEach(async (f) => {
        const includedFullPath = `${provider.host()}:${f}`;
        if (monacoCtx.existModelFromPath(includedFullPath)) {
          //  model already exist, ignore
          return;
        }

        const { file, error } = await provider.getFileContent(f);
        if (!error) {
          if (!monacoCtx.createModel(file)) {
            logCtx.error(
              `Could not create model for included file: [${file.fileName}]`,
              `Host: ${provider.host()}, root file: ${file.path}`,
            );
          }
        } else {
          console.error(
            `Could not open included file: [${file.fileName}]: ${error}`,
          );
        }
      });

      setIncludedFiles(includedFilesLocal);
      setNotificationDescription('');
    };
    getFileAndIncludesAsync();

    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [monacoCtx.monaco, currentFilePath, providerId, rosCtx.providers]);

  /**
   * Return editor definitions, based on included files.
   *  Definitions enable the editor "Go To Definition" capability
   *  The definitions are computed based on current mouse position and model, returns [] if not found
   *
   * @param {any} model - Current editor model
   * @param {any} position - Mouse cursor position
   */
  const provideDefinitionFunction = useCallback(
    (model, position) => {
      if (!includedFiles || !monacoCtx.monaco) return [];

      // check if model file exists on included files
      const modelFiles = includedFiles.filter((f) => {
        // check file path and line
        const fUri = monacoCtx.monaco.Uri.file(`${f.host}:${f.path}`);
        return fUri.path === model.uri.path;
      });

      // no file found, means no definition found
      if (!modelFiles || modelFiles.length === 0) {
        return [];
      }

      const definitions = [];

      modelFiles.forEach((mf) => {
        // find the exact position of the include, and compare with [position.column]
        const matches = model.findMatches(mf.raw_inc_path);
        matches.forEach((match) => {
          const { range } = match;
          // only consider the range of the text for the definition
          if (
            position.lineNumber === range.startLineNumber &&
            position.column > range.startColumn &&
            position.column <= range.endColumn
          ) {
            definitions.push({
              uri: monacoCtx.monaco.Uri.file(`${mf.host}:${mf.inc_path}`),
              resource: monacoCtx.monaco.Uri.file(`${mf.host}:${mf.inc_path}`),
              range: match.range,
              options: { selection: match },
            });
          }
        });
      });

      return definitions;
    },
    [includedFiles, monacoCtx.monaco],
  );

  const addMonacoDisposable = useCallback((disposable) => {
    setMonacoDisposables((prev) => [...prev, disposable]);
  }, []);

  const onCloseComponent = useCallback(() => {
    // clear monaco disposables:
    //    disposable objects includes autocomplete, code definition and editor actions
    monacoDisposables.forEach((d) => d?.dispose());
  }, [monacoDisposables]);

  return (
    <Stack
      direction="column"
      height="100%"
      backgroundColor={settingsCtx.get('backgroundColor')}
      overflow="none"
    >
      {notificationDescription.length > 0 && (
        <Alert
          severity="warning"
          style={{ minWidth: 0 }}
          onClose={() => {
            // setNotificationDescription('');
          }}
        >
          {notificationDescription}
        </Alert>
      )}
      {includedFiles && (
        <MonacoEditor
          file={currentFile}
          rootFilePath={rootFilePath}
          fileRange={selectionRange}
          providerId={providerId}
          provideDefinitionFunction={provideDefinitionFunction}
          addMonacoDisposable={addMonacoDisposable}
          includedFiles={includedFiles}
          onCloseComponent={onCloseComponent}
          setTitlePanel={setTitlePanel}
        />
      )}
    </Stack>
  );
}

FileEditorPanel.defaultProps = {
  rootFilePath: null,
  fileRange: null,
};

FileEditorPanel.propTypes = {
  providerId: PropTypes.string.isRequired,
  rootFilePath: PropTypes.string.isRequired,
  currentFilePath: PropTypes.string.isRequired,
  fileRange: PropTypes.any,
};

export default FileEditorPanel;
