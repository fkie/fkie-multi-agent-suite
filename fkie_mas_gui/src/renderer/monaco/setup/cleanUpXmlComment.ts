import { editor } from "monaco-editor";

/**
 * Fix malformed XML comment endings
 */
export function cleanUpXmlComment(changes: editor.IModelContentChange[], model: editor.ITextModel): void {
  // replace all '--' by '- - ' in XML comments
  if (!model) return;
  if (changes.length !== 2) return;
  let addedComment = false;
  if (
    changes.filter((entry) => {
      addedComment = entry.text.length > 0;
      return !["", "<!-- ", " -->"].includes(entry.text);
    }).length > 0
  )
    return;
  // get range
  const range = {
    endColumn: changes[0].range.startColumn + (addedComment ? 2 : 0),
    endLineNumber: changes[0].range.startLineNumber,
    startColumn: changes[1].range.endColumn + (addedComment ? 4 : 0),
    startLineNumber: changes[1].range.endLineNumber,
  };
  const matches = model.findMatches(
    addedComment ? "--" : "- - ", // search string
    range, // range
    false, // isRegex
    false, // matchCase
    null, // wordSeparators
    false, // captureMatches
    undefined // limitResultCount
  );
  matches.reverse().map((match) => {
    model.pushEditOperations(
      [],
      [{ forceMoveMarkers: false, range: match.range, text: addedComment ? "- - " : "--" }],
      () => null
    );
  }, true);
}
