export type TRosInfo = {
  version: string | undefined;
  pythonVersion: string | undefined;
  etcDir: string | undefined;
  masterUri: string | undefined;
  root: string | undefined;
  distro: string | undefined;
  domainId: string | undefined;
  localhostOnly: string | undefined;

  getInfo?: () => TRosInfo;
};
