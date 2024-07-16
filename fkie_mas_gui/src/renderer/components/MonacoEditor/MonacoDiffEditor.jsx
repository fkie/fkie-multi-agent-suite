// import { DiffEditor } from '@monaco-editor/react';
// import Box from '@mui/material/Box';
// import PropTypes from 'prop-types';
// import { useContext } from 'react';

// import { SettingsContext } from '../../context/SettingsContext';
// import { FileItem } from '../../models';

// // sequence of keys pressed by the user simultaneously
// let pressedKeys = [];

// function MonacoDiffEditor({
//   referenceFile = null,
//   currentFile = null,
//   height = null,
//   onChangeFile = null,
//   onKeyCommands = null,
// }) {
//   const settingsCtx = useContext(SettingsContext);
//   // const [modifiedEditorObject, setModifiedEditorObject] = useState(null);

//   // useEffect(() => {
//   //   // TODO: Why it is required?
//   //   // it will update the text value of the modified editor
//   //   if (modifiedEditorObject) {
//   //     modifiedEditorObject.setValue(currentFile.value);
//   //   }
//   // }, [modifiedEditorObject, currentFile]);

//   const handleEditorDidMount = (editor) => {
//     // setModifiedEditorObject(editor.getModifiedEditor());
//     const modifiedEditor = editor.getModifiedEditor();
//     modifiedEditor.onDidChangeModelContent(() => {
//       currentFile.value = modifiedEditor.getValue();

//       if (onChangeFile) {
//         onChangeFile();
//       }
//     });
//   };

//   const handleEditorWillMount = (monaco) => {
//     // here is the monaco instance
//     // do something before editor is mounted
//     // monaco.languages.typescript.javascriptDefaults.setEagerModelSync(true);
//   };

//   const onKeyDown = (event) => {
//     if (!pressedKeys) pressedKeys = [];
//     // always assume lower case letters
//     pressedKeys.push(event.key.toLowerCase());
//     if (onKeyCommands) onKeyCommands(pressedKeys);
//   };

//   const onKeyUp = (event) => {
//     if (!pressedKeys) return;
//     pressedKeys = pressedKeys.filter((e) => e !== event.key.toLowerCase());
//     if (onKeyCommands) onKeyCommands(pressedKeys);
//   };

//   return (
//     <Box sx={{ height, width: '100%' }} onKeyDown={onKeyDown} onKeyUp={onKeyUp}>
//       {referenceFile && currentFile && (
//         <DiffEditor
//           theme={settingsCtx.get('useDarkMode') ? 'vs-dark' : 'light'}
//           original={referenceFile.value}
//           originalLanguage={referenceFile.language}
//           originalModelPath={referenceFile.id}
//           modified={currentFile.value}
//           modifiedLanguage={currentFile.language}
//           // modifiedModelPath={currentFile.id}
//           beforeMount={handleEditorWillMount}
//           onMount={handleEditorDidMount}
//           options={{
//             // to check the all possible options check this - https://github.com/microsoft/monaco-editor/blob/a5298e1/website/typedoc/monaco.d.ts#L3017
//             // TODO: make global config for this parameters
//             readOnly: false,
//             colorDecorators: true,
//             mouseWheelZoom: true,
//             scrollBeyondLastLine: false,
//             smoothScrolling: false,
//             wordWrap: 'off',
//             fontSize: settingsCtx.get('fontSize'),
//             minimap: { enabled: false },
//             renderSideBySide: true,
//           }}
//         />
//       )}
//     </Box>
//   );
// }

// MonacoDiffEditor.propTypes = {
//   referenceFile: PropTypes.instanceOf(FileItem),
//   currentFile: PropTypes.instanceOf(FileItem),
//   height: PropTypes.number,
//   onChangeFile: PropTypes.func,
//   onKeyCommands: PropTypes.func,
// };

// export default MonacoDiffEditor;
