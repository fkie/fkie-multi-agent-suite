export type TResultProcess = {
  result: boolean;
  message: string;
  processes: { pid: number; cmdLine: string }[];
};
