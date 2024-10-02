import { TSystemInfo } from "@/types";
import React, { createContext, useContext, useMemo, useState } from "react";
import useLocalStorage from "../hooks/useLocalStorage";
import { generateUniqueId } from "../utils";
import { DEFAULT_BUG_TEXT, LoggingContext } from "./LoggingContext";
import { ConnectConfig } from "ssh2";

export interface IRosProviderContext {
  // credentials methods
  credentials: ConnectConfig[];
  systemInfo: TSystemInfo | null;
  addCredential: (credential: ConnectConfig) => void;
  deleteCredential: (credential: ConnectConfig) => void;
  getCredentialHost: (host: string) => ConnectConfig | null;

  // Password Manager methods
  setPassword: (credential: ConnectConfig) => void;
  deletePassword: (credential: ConnectConfig) => void;
  exec: (credential: ConnectConfig | null, command: string) => Promise<{ result: boolean; message: string }>;
}

export const DEFAULT = {
  credentials: [],
  systemInfo: null,
  addCredential: () => {},
  deleteCredential: () => {},
  getCredentialHost: () => null,
  setPassword: () => {},
  deletePassword: () => {},
  exec: () => {
    return Promise.resolve({ result: false, message: "" });
  },
};

interface IRosProviderComponent {
  children: React.ReactNode;
}

const DEFAULT_CREDENTIALS: ConnectConfig[] = [];
const SERVICE_NAME = "MAS_PASSWORDS";

export const SSHContext = createContext<IRosProviderContext>(DEFAULT);

export function SSHProvider({ children }: IRosProviderComponent): ReturnType<React.FC<IRosProviderComponent>> {
  const logCtx = useContext(LoggingContext);

  // uses system info to get the /etc/hosts entries
  const [systemInfo, setSystemInfo] = useState<TSystemInfo | null>(null);
  const getSystemInfo = async () => {
    // get local System Info
    if (window.systemInfo?.getInfo) {
      setSystemInfo(await window.systemInfo.getInfo());
    }
  };
  if (!systemInfo) {
    getSystemInfo();
  }

  const [credentials, setCredentials] = useLocalStorage("SSHContext:credentials", DEFAULT_CREDENTIALS);

  // PasswordManager methods
  const setPassword = async (credential: ConnectConfig) => {
    if (!window.passwordManager) {
      logCtx.error("setPassword: Invalid [PasswordManager]", "Please check Electron App (IPC handlers)");
      return null;
    }

    const account = `${credential.username}:${credential.host}`;
    let password: string = "";
    if (credential.password) {
      password = credential.password;
    } else if (credential.privateKey) {
      password = credential.privateKey;
    } else if (credential.passphrase) {
      password = credential.passphrase;
    }

    const res = await window.passwordManager.setPassword(SERVICE_NAME, account, password);
    return res;
  };

  const deletePassword = async (credential: ConnectConfig) => {
    if (!window.passwordManager) {
      logCtx.error("deletePassword: Invalid [PasswordManager]", "Please check Electron App (IPC handlers)");
      return;
    }
    const account = `${credential.username}:${credential.host}`;
    await window.passwordManager.deletePassword(SERVICE_NAME, account);
  };

  const checkPassword = (credential: ConnectConfig) => {
    if (!window.commandExecutor) {
      logCtx.error("checkPassword: Invalid [CommandExecutor]", "Please check Electron App (IPC handlers)");
      return Promise.resolve({
        result: false,
        message: "checkPassword: Invalid [CommandExecutor]; Please check Electron App (IPC handlers)",
      });
    }
    return window.commandExecutor.exec(credential, "pwd");
  };

  // credentials methods
  const addCredential = async (credential: ConnectConfig) => {
    if (!credential.host || !credential.username || !credential.password) {
      logCtx.error("Try to add an invalid credential", "Please check credential settings");
      return { result: false, message: "Try to add an invalid credential" };
    }

    // check unique host
    let foundHost = false;
    credentials.forEach((c) => {
      if (c.host === credential.host) {
        foundHost = true;
      }
    });

    if (foundHost) {
      logCtx.error(`Host [${credential.host}] have been already configured`, "Only one entry per host is allowed");
      return {
        result: false,
        message: `Host [${credential.host}] have been already configured`,
      };
    }

    try {
      // save password on systems keychain
      await setPassword(credential);

      // check if password is correct
      const valid = await checkPassword(credential);

      // remove explicit the password after saving it on system
      credential.password = "";

      // return false if the password wasn't valid
      if (!valid.result) {
        return valid;
      }

      // if successful, add new credential to app
      setCredentials((previousCredentials: ConnectConfig[]) => {
        return [...previousCredentials, credential];
      });
    } catch (error: unknown) {
      return { result: false, message: JSON.stringify(error) };
    }

    return { result: true, message: "" };
  };

  const deleteCredential = (credential: ConnectConfig) => {
    // delete associated password
    let foundCredential = false;
    credentials.forEach((c) => {
      if (c.host === credential.host && c.username === credential.username) {
        deletePassword(c);
        foundCredential = true;
      }
    });

    if (!foundCredential) return false;

    // remove credential from App
    setCredentials((previousCredentials) => {
      return previousCredentials.filter((c) => c.host !== credential.host || c.username !== credential.username);
    });

    return true;
  };

  const getCredentialHost = (host: string) => {
    if (typeof host !== "string") {
      return null;
    }

    // search the SSH credentials based on host
    let credentialHost: ConnectConfig | null = null;
    credentials.forEach((credential) => {
      if (credential.host === host) {
        credentialHost = credential;
      }
    });

    if (!credentialHost) {
      // try to get the host from hosts file
      if (systemInfo && systemInfo.hosts) {
        systemInfo.hosts.forEach((h) => {
          const ip = h[0];
          // only add valid ipv4 addresses
          const ipv4format =
            /^(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$/;
          if (ip.match(ipv4format)) {
            const lastName = h[1].split(" ").pop();
            let alternativeHost: string | undefined = "";
            if (ip === host) {
              alternativeHost = lastName;
            } else if (lastName === host) {
              alternativeHost = ip;
            }
            if (alternativeHost) {
              logCtx.debug(`Use host ${alternativeHost} instead of ${host} for credentials!`, "");
              credentials.forEach((credential) => {
                if (credential.host === alternativeHost) {
                  credentialHost = credential;
                }
              });
            }
          }
        });
      }
    }

    if (!credentialHost) {
      credentialHost = {
        username: "robot",
        host,
      } as ConnectConfig;
    }
    if (!credentialHost) {
      logCtx.error(
        `No SSH credentials have been configured for host: [${JSON.stringify(host)}]`,
        "Please check the SSH credential settings"
      );
      return null;
    }

    return credentialHost;
  };

  // CommandExecutor
  const exec = async (credential: ConnectConfig | null, command: string) => {
    if (!window.commandExecutor) {
      logCtx.error("exec: Invalid [CommandExecutor]", "Please check Electron App (IPC handlers)");
      return {
        result: false,
        message: "exec: Invalid [CommandExecutor]",
        DEFAULT_BUG_TEXT,
      };
    }

    if (command.length === 0) {
      logCtx.error("exec: Invalid empty content", DEFAULT_BUG_TEXT);
      return {
        result: false,
        message: "exec: Invalid empty content",
        DEFAULT_BUG_TEXT,
      };
    }

    const res = await window.commandExecutor.exec(credential, command);
    return res;
  };

  const attributesMemo = useMemo(
    () => ({
      credentials,
      systemInfo,
      addCredential,
      deleteCredential,
      getCredentialHost,

      setPassword,
      deletePassword,

      // exist,
      // stat,
      // get,
      // put,
      exec,
    }),
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [credentials, systemInfo]
  );

  return <SSHContext.Provider value={attributesMemo}>{children}</SSHContext.Provider>;
}

export default SSHContext;
