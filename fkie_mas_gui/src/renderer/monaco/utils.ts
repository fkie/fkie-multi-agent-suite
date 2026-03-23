const EDITOR_PATH_SEP = "⏵";
const PROVIDER_SEP = "@";
const EDITOR_ID_PREFIX = "/editorId⏶";

export function isEditorEditorId(editorId: string,): boolean {
  return editorId.startsWith(EDITOR_ID_PREFIX);
}

export function createEditorId(rootPath: string, providerId: string): string {
  return `${EDITOR_ID_PREFIX}${rootPath}${PROVIDER_SEP}${providerId}`;
}

export function createUriPathFromEditorId(editorId: string, path: string): string {
  if (!path) return "";

  if (path.includes(EDITOR_PATH_SEP)) {
    return path;
  }

  const providerId = providerIdFromEditorId(editorId);
  return `/${providerId}${EDITOR_PATH_SEP}${path}`;
}

export function createUriPath(providerId: string, path: string): string {
  if (!path) return "";

  if (path.includes(EDITOR_PATH_SEP)) {
    return path;
  }
  return `/${providerId}${EDITOR_PATH_SEP}${path}`;
}

export function fileFromUriPath(uriPath: string): string {
  const sepIndex = uriPath.indexOf(EDITOR_PATH_SEP);
  return sepIndex === -1 ? uriPath : uriPath.slice(sepIndex + 1);
}

export function providerIdFromUriPath(uriPath: string): string | undefined {
  const sepIndex = uriPath.indexOf(EDITOR_PATH_SEP);
  if (sepIndex === -1) return undefined;
  const lastSlashIndex = uriPath.lastIndexOf("/", sepIndex);
  const start = lastSlashIndex + 1; // +1, um das "/" selbst zu überspringen
  const providerId = uriPath.slice(start, sepIndex);

  return providerId || undefined;
}

export function providerIdFromEditorId(editorId: string): string | undefined {
  const sepIndex = editorId.indexOf(PROVIDER_SEP);
  return sepIndex === -1 ? undefined : editorId.slice(sepIndex + 1);
}

export function pathFromEditorId(editorId: string): string | undefined {
  const sepIndex = editorId.indexOf(PROVIDER_SEP);
  return sepIndex === -1 ? undefined : editorId.slice(0, sepIndex).replace(EDITOR_ID_PREFIX, '');

}

export function isUriPath(path: string): boolean {
  const editorIndex = path.indexOf(EDITOR_PATH_SEP);

  return editorIndex !== -1;
}
