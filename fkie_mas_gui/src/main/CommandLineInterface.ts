import { app } from "electron";
import log from "electron-log";

export const ARGUMENTS = {
  SHOW_OUTPUT_FROM_BACKGROUND_PROCESSES: "show-output-from-background-processes",
  HEADLESS: "headless",
  HEADLESS_SERVER_PORT: "headless-server-port",
  UPDATE_MAS_DEBIAN_PACKAGES: "update-debs",
  UPDATE_MAS_DEBIAN_PRERELEASE_PACKAGES: "update-debs-prerelease",
};

const defaultValues: { [key: string]: string } = {};

/**
 * Register one command line argument and log its value
 */
function registerArgument(name: string, value: string | null): void {
  app.commandLine.appendArgument(name);
  if (value !== null) {
    log.info(` --${name}: ${value}`);
    defaultValues[name] = value;
  } else {
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
  registerArgument(ARGUMENTS.HEADLESS_SERVER_PORT, "6275");
  registerArgument(ARGUMENTS.UPDATE_MAS_DEBIAN_PACKAGES, null);
  registerArgument(ARGUMENTS.UPDATE_MAS_DEBIAN_PRERELEASE_PACKAGES, null);

  log.info("Environment Variables: ");
  log.info("  VITE_JOIN_ID             Join this ROS domain ID at startup");
  log.info("  VITE_ROS_VERSION         ROS version [1,2] to join");
  log.info("  VITE_ROS_DOMAIN_ID       Specify the domain ID in advance in the connect dialog");
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
  return app.commandLine.getSwitchValue(name) || defaultValues[name] || "";
}
