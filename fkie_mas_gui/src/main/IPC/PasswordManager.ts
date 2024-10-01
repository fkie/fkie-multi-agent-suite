import keytar from "keytar";
import { ipcMain } from "electron";
import { PasswordManagerEvents, TPasswordManager } from "@/types";

/**
 * Class PasswordManager: Save/read passwords using the OS keychain (interfaced with keytar)
 */
class PasswordManager implements TPasswordManager {
  public registerHandlers: () => void = () => {
    // Password Manager
    ipcMain.handle(PasswordManagerEvents.setPassword, (_event, service: string, account: string, password: string) => {
      return this.setPassword(service, account, password);
    });

    ipcMain.handle(PasswordManagerEvents.deletePassword, (_event, service: string, account: string) => {
      return this.deletePassword(service, account);
    });
  };

  /**
   * Get the stored password for the service and account.
   *
   * @param {string} service - The string service name.
   * @param {string} account - The string account name.
   * @return {string} Yields the string password or null if an entry for the given service and account was not found.
   */
  public getPassword(service: string, account: string): Promise<string | null> {
    return keytar.getPassword(service, account);
  }

  /**
   * Save the password for the service and account to the keychain.
   * Adds a new entry if necessary, or updates an existing entry if one exists.
   *
   * @param {string} service - The string service name.
   * @param {string} account - The string account name.
   * @param {string} password - The string password.
   * @return {void} Yields nothing.
   */
  public setPassword(service: string, account: string, password: string): Promise<void> {
    return keytar.setPassword(service, account, password);
  }

  /**
   * Delete the stored password for the service and account.
   *
   * @param {string} service - The string service name.
   * @param {string} account - The string account name.
   * @return {boolean} Yields true if a password was deleted, or false if an entry with the given service and account was not found.
   */
  public deletePassword(service: string, account: string): Promise<boolean> {
    return keytar.deletePassword(service, account);
  }
}

export default PasswordManager;
