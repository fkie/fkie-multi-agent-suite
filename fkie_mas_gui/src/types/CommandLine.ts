export const CommandLineEvents = {
  getArgument: "commandLine:getArgument",
  hasArgument: "commandLine:hasArgument",
};

export type TCommandLine = {
  /**
   * Get the value of a registered argument
   */
  getArgument: (name: string) => Promise<string>;
  /**
   * For switches
   */
  hasArgument: (name: string) => Promise<boolean>;
};
