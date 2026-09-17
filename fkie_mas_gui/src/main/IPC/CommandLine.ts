import { app, ipcMain } from "electron";
import log from "electron-log";
import { CommandLineEvents, TCommandLine } from "@/types";
import CliArgs from "../../renderer/assets/cliArgs.json";

type CliArgType = "boolean" | "string" | "int";

interface CliArgDefinition {
  default: string | boolean | number;
  fromEnv: string;
  switch: boolean;
  type: CliArgType;
  hint: string;
}

type CliArgsDefinitionMap = Record<string, CliArgDefinition>;
type ParsedArgumentValue = string | boolean | number | undefined;

type ArgsWithMetaObject = Record<
  string,
  {
    default: ParsedArgumentValue;
    type: CliArgType;
    switch: boolean;
    hint: string;
    fromEnv: string;
  }
>;

const typedCliArgs = CliArgs as CliArgsDefinitionMap;

/**
 * Handler for renderer requests
 */
export default class CommandLine implements TCommandLine {
  parsedArguments: Record<string, ParsedArgumentValue> = {};

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
    const showHelp = process.argv.includes("--help");

    if (showHelp) {
      log.info("Available command line arguments:");
    } else {
      log.info("Program arguments with changed values:");
    }

    for (const [name, value] of Object.entries(typedCliArgs)) {
      let found = false;
      let source: "cli" | "env" | "default" = "default";

      for (const arg of process.argv) {
        if (value.switch) {
          if (arg === `--${name}`) {
            this.parsedArguments[name] = !value.default;
            found = true;
            source = "cli";
            break;
          }
        } else {
          const splits = arg.split("=");
          if (splits.length === 2 && splits[0] === `--${name}`) {
            this.parsedArguments[name] = splits[1];
            found = true;
            source = "cli";
            break;
          }
        }
      }

      if (!found) {
        let fromEnv = false;

        if (value.fromEnv) {
          const envValue = process.env[value.fromEnv];
          if (envValue !== undefined) {
            this.parsedArguments[name] = envValue;
            fromEnv = true;
            source = "env";
          }
        }

        if (!fromEnv) {
          this.parsedArguments[name] = value.default;
          source = "default";
        }
      }

      if (value.type === "int" && typeof this.parsedArguments[name] === "string") {
        const num = Number.parseInt(this.parsedArguments[name] as string, 10);
        if (Number.isFinite(num)) {
          this.parsedArguments[name] = num;
        } else {
          log.error(`${name} was not assigned an integer: ${this.parsedArguments[name]}`);
          app.exit(1);
          return;
        }
      }

      const currentValue = this.parsedArguments[name];
      const changed = currentValue !== value.default;

      if (showHelp) {
        log.info(` --${name}${value.switch ? "" : `=[${value.type}]`}`);
        if (value.hint) {
          log.info(`     ${value.hint}`);
        }
        log.info(`     Default: ${value.default}`);
        if (value.fromEnv) {
          log.info(`     Env: ${value.fromEnv}`);
        }
      } else if (changed) {
        log.info(
          ` --${name}${value.switch ? "" : `=[${value.type}]`}: ${currentValue} (source: ${source}, default: ${value.default})`
        );
      }
    }

    if (showHelp) {
      console.log("Exiting due to help request!");
      app.exit(0);
    }
  }

  public getArg: (name: string) => ParsedArgumentValue = (name) => {
    return this.parsedArguments[name] ?? undefined;
  };

  public getArgument: (name: string) => Promise<ParsedArgumentValue> = async (name) => {
    return Promise.resolve(this.parsedArguments[name] ?? undefined);
  };

  public getArgsWithMeta(): ArgsWithMetaObject {
    const result: ArgsWithMetaObject = {};

    for (const [name, value] of Object.entries(typedCliArgs)) {
      result[name] = {
        default: this.getArg(name),
        type: value.type,
        switch: value.switch,
        hint: value.hint || "",
        fromEnv: value.fromEnv || "",
      };
    }

    return result;
  }
}
