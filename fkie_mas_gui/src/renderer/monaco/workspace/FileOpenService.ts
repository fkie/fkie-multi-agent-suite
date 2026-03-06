import { MasFileProvider } from "./MasFileProvider";

export class FileOpenService {
  constructor(
    private fs: MasFileProvider,
  ) {}

  async open(providerId: string, path: string) {
    return await this.fs.readFile(providerId, path);
  }
}
