import { app } from "electron";
import log from "electron-log";
import CliArgs from "../renderer/assets/cliArgs.json";

const defaultValues: { [key: string]: string } = {};

/**
 * Register one command line argument and log its value
 */
function registerArgument(name: string, value: string | null, hint: string, fromEnv: string): void {
  app.commandLine.appendArgument(name);
  log.info(` --${name}`);
  if (value !== null) {
    defaultValues[name] = value;
  }
  if (fromEnv) {
    const envValue = process.env[fromEnv];
    if (envValue) {
      defaultValues[name] = envValue;
    }
  }
  if (hint) {
    log.info(`     ${hint}`);
  }
  if (value) {
    log.info(`     Default: ${value}`);
  }
}

/**
 * Register command line arguments
 */
export function registerArguments(): void {
  log.info("Program arguments: ");

  for (const [name, value] of Object.entries(CliArgs)) {
    registerArgument(name, value.default, value.hint, value.fromEnv);
  }
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
