import { app } from "electron";
import log from "electron-log";

export const ARGUMENTS = {
  SHOW_OUTPUT_FROM_BACKGROUND_PROCESSES: "show-output-from-background-processes",
};

/**
 * Register one command line argument and log its value
 */
function registerArgument(name: string, value: string): void {
  app.commandLine.appendSwitch(name, value);
  log.info(` --${name}: ${value}`);
}

/**
 * Register command line arguments
 */
export function registerArguments(): void {
  log.info("Program arguments: ");

  // Program arguments
  registerArgument(ARGUMENTS.SHOW_OUTPUT_FROM_BACKGROUND_PROCESSES, "true");
}

/**
 * Check if an arguments was registered
 */
export function hasArgument(name: string): boolean {
  return app.commandLine.hasSwitch(name);
}

/**
 * Get the value of a registered argument
 */
export function getArgument(name: string): string {
  return app.commandLine.getSwitchValue(name);
}
