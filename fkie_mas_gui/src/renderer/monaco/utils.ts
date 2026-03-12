const EDITOR_PATH_SEP = "⏵";
const PROVIDER_SEP = "@";
const TAB_ID_PREFIX = "/tabId⏶";

export function isEditorTabId(tabId: string,): boolean {
  return tabId.startsWith(TAB_ID_PREFIX);
}

export function createEditorTabId(rootPath: string, providerId: string): string {
  return `${TAB_ID_PREFIX}${rootPath}${PROVIDER_SEP}${providerId}`;
}

export function createUriPathFromTab(tabId: string, path: string): string {
  if (!path) return "";

  if (path.includes(EDITOR_PATH_SEP)) {
    return path;
  }

  const providerId = providerIdFromTabId(tabId);
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

export function providerIdFromTabId(tabId: string): string | undefined {
  const sepIndex = tabId.indexOf(PROVIDER_SEP);
  return sepIndex === -1 ? undefined : tabId.slice(sepIndex + 1);
}

export function pathFromTabId(tabId: string): string | undefined {
  const sepIndex = tabId.indexOf(PROVIDER_SEP);
  return sepIndex === -1 ? undefined : tabId.slice(0, sepIndex);
}

export function isUriPath(path: string): boolean {
  const editorIndex = path.indexOf(EDITOR_PATH_SEP);

  return editorIndex !== -1;
}
