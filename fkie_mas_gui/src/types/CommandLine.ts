export const CommandLineEvents = {
  getArgument: "commandLine:getArgument",
};

export type TCommandLine = {
  /**
   * Get the value of a registered argument
   */
  getArgument: (name: string) => Promise<string | boolean | number | undefined >;
};
