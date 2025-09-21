import { CommandLineEvents, TCommandLine } from "@/types";
import { ipcMain } from "electron";
import { getArgument, hasArgument } from "../CommandLineInterface";

/**
 * Handler for renderer requests
 */
export default class CommandLine implements TCommandLine {
  public registerHandlers: () => void = () => {
    ipcMain.handle(CommandLineEvents.getArgument, (_event, name: string) => {
      return this.getArgument(name);
    });

    ipcMain.handle(CommandLineEvents.hasArgument, (_event, name: string) => {
      return this.hasArgument(name);
    });
  };

  public getArgument: (name: string) => Promise<string> = async (name) => {
    return getArgument(name);
  };

  public hasArgument: (name: string) => Promise<boolean> = async (name) => {
    return hasArgument(name);
  };
}
