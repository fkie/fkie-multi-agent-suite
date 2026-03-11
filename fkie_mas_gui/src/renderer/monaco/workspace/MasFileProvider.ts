import { IRosContext } from "@/renderer/context/RosContext";
import { FileItem } from "@/renderer/models";

export class MasFileProvider {
  constructor(private rosCtxRef: React.MutableRefObject<IRosContext>) {}

  async readFile(providerId: string, path: string) {
    const provider = this.rosCtxRef.current.getProviderById(providerId, false);

    if (!provider) throw new Error(`provider '${providerId}' not found`);

    return await provider.getFileContent(path);
  }

  async writeFile(providerId: string, file: FileItem) {
    const provider = this.rosCtxRef.current.getProviderById(providerId, false);

    if (!provider) throw new Error(`provider '${providerId}' not found`);

    return provider.saveFileContent(file);
  }
}
