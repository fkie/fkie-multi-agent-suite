import { LaunchIncludedFile, LaunchIncludedFilesRequest } from "@/renderer/models";
import { useState } from "react";

import { useRosContext } from "./useRosContext";

export function useIncludedFiles(providerId: string, rootFilePath: string) {
  const rosCtx = useRosContext();
  const [includedFiles, setIncludedFiles] = useState<LaunchIncludedFile[]>([]);

  async function fetchIncludedFiles(): Promise<{ result: boolean; error: string }> {
    const provider = rosCtx.getProviderById(providerId);
    if (!provider) {
      return { result: false, error: `Provider with id ${providerId} not available` };
    }

    const request = new LaunchIncludedFilesRequest();
    request.path = rootFilePath;
    request.unique = false;
    request.recursive = true;

    const includedFilesLocal = await provider.launchGetIncludedFiles(request);

    if (!includedFilesLocal)
      return { result: false, error: `error while get included launch files from ${providerId}` };
    // // filter unique file names (in case multiple imports)
    // const uniqueIncludedFiles = [rootFilePath];
    // for (const f of includedFilesLocal) {
    //   if (!uniqueIncludedFiles.includes(f.inc_path)) uniqueIncludedFiles.push(f.inc_path);
    // }
    setIncludedFiles(includedFilesLocal);
    return { result: true, error: "" };
  }

  function clearIncludedFiles() {
    setIncludedFiles([]);
  }

  return {
    includedFiles,
    fetchIncludedFiles,
    clearIncludedFiles,
  };
}
