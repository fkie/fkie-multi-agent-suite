import { CommandLineEvents, TCommandLine } from "@/types";
import { ipcMain } from "electron";

import log from "electron-log";
import CliArgs from "../../renderer/assets/cliArgs.json";

/**
 * Handler for renderer requests
 */
export default class CommandLine implements TCommandLine {
  parsedArguments: { [key: string]: string | boolean | number | undefined } = {};

  constructor() {
    this.registerHandlers();
    this.registerArguments();
  }

  public registerHandlers: () => void = () => {
    ipcMain.handle(CommandLineEvents.getArgument, (_event, name: string) => {
      return this.getArgument(name);
    });
  };

  public registerArguments(): void {
    log.info("Program arguments: ");

    for (const [name, value] of Object.entries(CliArgs)) {
      let found = false;
      for (const arg of process.argv) {
        if (value.switch) {
          if (arg === `--${name}`) {
            this.parsedArguments[name] = !value.default;
            found = true;
            break;
          }
        } else {
          const splits = arg.split("=");
          if (splits.length === 2 && splits[0] === `--${name}`) {
            this.parsedArguments[name] = splits[1];
            if (value.toInt) {
              const num = Number.parseInt(splits[1])
              if (Number.isFinite(num)) {
                this.parsedArguments[name] = num;
              }
            }
            found = true;
            break;
          }
        }
      }
      if (!found) {
        let fromEnv = false;
        if (value.fromEnv) {
          const envValue = process.env[value.fromEnv];
          if (envValue) {
            this.parsedArguments[name] = envValue;
            fromEnv = true;
            if (value.toInt) {
              const num = Number.parseInt(envValue)
              if (Number.isFinite(num)) {
                this.parsedArguments[name] = num;
              }
            }
          }
        }
        if (!fromEnv) {
          this.parsedArguments[name] = value.default;
        }
      }
      log.info(` --${name}: ${this.parsedArguments[name]}`);
      if (value.hint) {
        log.info(`     ${value.hint}`);
      }
      if (value.default) {
        log.info(`     Default: ${value.default}`);
      }
    }
  }

  public getArg: (name: string) => string | boolean | number | undefined = (name) => {
    return this.parsedArguments[name] || undefined;
  };

  public getArgument: (name: string) => Promise<string | boolean | number | undefined> = async (name) => {
    log.info(`request: ${name}: ${this.parsedArguments[name]}`);
    return Promise.resolve(this.parsedArguments[name] || undefined);
  };
}
