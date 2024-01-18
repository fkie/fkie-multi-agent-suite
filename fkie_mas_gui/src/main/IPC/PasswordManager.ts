import keytar from 'keytar';

/**
 * Class PasswordManager: Save/read passwords using the OS keychain (interfaced with keytar)
 */
class PasswordManager {
  /**
   * Get the stored password for the service and account.
   *
   * @param {string} service - The string service name.
   * @param {string} account - The string account name.
   * @return {string} Yields the string password or null if an entry for the given service and account was not found.
   */
  public getPassword(service: string, account: string) {
    return keytar.getPassword(service, account);
  }

  /**
   * Save the password for the service and account to the keychain.
   * Adds a new entry if necessary, or updates an existing entry if one exists.
   *
   * @param {string} service - The string service name.
   * @param {string} account - The string account name.
   * @param {string} password - The string password. .
   * @return {void} Yields nothing.
   */
  public setPassword(service: string, account: string, password: string) {
    return keytar.setPassword(service, account, password);
  }

  /**
   * Delete the stored password for the service and account.
   *
   * @param {string} service - The string service name.
   * @param {string} account - The string account name.
   * @return {boolean} Yields true if a password was deleted, or false if an entry with the given service and account was not found.
   */
  public deletePassword(service: string, account: string) {
    return keytar.deletePassword(service, account);
  }
}

export default PasswordManager;
