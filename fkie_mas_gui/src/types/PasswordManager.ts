export const PasswordManagerEvents = {
  setPassword: "passwordManager:setPassword",
  deletePassword: "passwordManager:deletePassword",
};

export type TPasswordManager = {
  /**
   * Save the password for the service and account to the keychain.
   * Adds a new entry if necessary, or updates an existing entry if one exists.
   *
   * @param {string} service - The string service name.
   * @param {string} account - The string account name.
   * @param {string} password - The string password.
   * @return {void} Yields nothing.
   */
  setPassword(service: string, account: string, password: string): Promise<void>;

  /**
   * Delete the stored password for the service and account.
   *
   * @param {string} service - The string service name.
   * @param {string} account - The string account name.
   * @return {boolean} Yields true if a password was deleted, or false if an entry with the given service and account was not found.
   */
  deletePassword(service: string, account: string): Promise<boolean>;
};
