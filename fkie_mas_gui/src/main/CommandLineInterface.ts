import { app } from "electron";
import log from "electron-log";

const ARGUMENTS = {
  SHOW_OUTPUT_FROM_BACKGROUND_PROCESSES: "show-output-from-background-processes",
};

/**
 * Register one command line argument and log its value
 */
const registerArgument = (name: string, value: string): void => {
  app.commandLine.appendSwitch(name, value);
  log.info(` --${name}: ${value}`);
};

/**
 * Register command line arguments
 */
const registerArguments = (): void => {
  log.info("Program arguments: ");

  // Program arguments
  registerArgument(ARGUMENTS.SHOW_OUTPUT_FROM_BACKGROUND_PROCESSES, "true");
};

/**
 * Check if an arguments was registered
 */
const hasArgument = (name: string): boolean => {
  return app.commandLine.hasSwitch(name);
};

/**
 * Get the value of a registered argument
 */
const getArgument: (name: string) => string = (name: string) => {
  return app.commandLine.getSwitchValue(name);
};

export { ARGUMENTS, getArgument, hasArgument, registerArguments };
