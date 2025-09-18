import { app } from "electron";
import log from "electron-log";

export const ARGUMENTS = {
  SHOW_OUTPUT_FROM_BACKGROUND_PROCESSES: "show-output-from-background-processes",
  HEADLESS: "headless",
  UPDATE_MAS_DEBIAN_PACKAGES: "update-debs",
  UPDATE_MAS_DEBIAN_PRERELEASE_PACKAGES: "update-debs-prerelease",
};

/**
 * Register one command line argument and log its value
 */
function registerArgument(name: string, value: string | null): void {
  if (value !== null) {
    app.commandLine.appendSwitch(name, value);
    log.info(` --${name}: ${value}`);
  } else {
    app.commandLine.appendArgument(name);
    log.info(` --${name}`);
  }
}

/**
 * Register command line arguments
 */
export function registerArguments(): void {
  log.info("Program arguments: ");

  // Program arguments
  registerArgument(ARGUMENTS.SHOW_OUTPUT_FROM_BACKGROUND_PROCESSES, "true");
  registerArgument(ARGUMENTS.HEADLESS, null);
  registerArgument(ARGUMENTS.UPDATE_MAS_DEBIAN_PACKAGES, null);
  registerArgument(ARGUMENTS.UPDATE_MAS_DEBIAN_PRERELEASE_PACKAGES, null);
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
