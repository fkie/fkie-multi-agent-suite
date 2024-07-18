import React, { createContext, useContext, useMemo, useState } from "react";
import useLocalStorage from "../hooks/useLocalStorage";
import { DEFAULT_BUG_TEXT, LoggingContext } from "./LoggingContext";
// import FileManagerWrapper from '../../main/IPC/FileManagerWrapper';
import CommandExecutor from "../../main/IPC/CommandExecutor";
import PasswordManager from "../../main/IPC/PasswordManager";
import { ISystemInfo } from "../../main/IPC/SystemInfo";
import { ICredential } from "../../main/models/ICredential";
import { generateUniqueId } from "../utils";

declare global {
  interface Window {
    PasswordManager?: PasswordManager;
    // FileManagerWrapper?: FileManagerWrapper;
    CommandExecutor?: CommandExecutor;
  }
}

export interface IRosProviderContext {
  // credentials methods
  credentials: ICredential[];
  systemInfo: ISystemInfo | null;
  addCredential: (credential: ICredential) => void;
  deleteCredential: (credentialId: string) => void;
  getCredentialHost: (host: string) => ICredential | null;

  // Password Manager methods
  setPassword: (credential: ICredential) => void;
  deletePassword: (credential: ICredential) => void;

  // // SFTP Methods
  // exist: (credential: ICredential, path: string) => Promise<boolean>;
  // stat: (credential: ICredential, path: string) => Promise<any>;
  // get: (
  //   credential: ICredential,
  //   path: string
  // ) => Promise<string | NodeJS.WritableStream | Buffer | null>;
  // put: (
  //   credential: ICredential,
  //   content: string,
  //   path: string
  // ) => Promise<string | null>;
  exec: (credential: ICredential | null, command: string) => Promise<{ result: boolean; message: string }>;
}

export const DEFAULT = {
  credentials: [],
  systemInfo: null,
  addCredential: () => {},
  deleteCredential: () => {},
  getCredentialHost: () => null,
  setPassword: () => {},
  deletePassword: () => {},
  // list: () => {
  //   return Promise.resolve([]);
  // },
  // exist: () => {
  //   return Promise.resolve(false);
  // },
  // stat: () => {
  //   return Promise.resolve({});
  // },
  // get: () => {
  //   return Promise.resolve(null);
  // },
  // put: () => {
  //   return Promise.resolve(null);
  // },
  exec: () => {
    return Promise.resolve({ result: false, message: "" });
  },
};

interface IRosProviderComponent {
  children: React.ReactNode;
}

const DEFAULT_CREDENTIALS: ICredential[] = [];

export const SSHContext = createContext<IRosProviderContext>(DEFAULT);

export function SSHProvider({ children }: IRosProviderComponent): ReturnType<React.FC<IRosProviderComponent>> {
  const logCtx = useContext(LoggingContext);

  // uses system info to get the /etc/hosts entries
  const [systemInfo, setSystemInfo] = useState<ISystemInfo | null>(null);
  const getSystemInfo = async () => {
    // get local System Info
    if (window.SystemInfo) {
      setSystemInfo(await window.SystemInfo.getInfo());
    }
  };
  if (!systemInfo) {
    getSystemInfo();
  }

  const [credentials, setCredentials] = useLocalStorage("SSHContext:credentials", DEFAULT_CREDENTIALS);

  // // FileManagerWrapper methods
  // const exist = async (credential: ICredential, path: string) => {
  //   if (!window.FileManagerWrapper) {
  //     logCtx.error(
  //       'exist: Invalid [FileManagerWrapper]',
  //       'Please check Electron App (IPC handlers)'
  //     );
  //     return false;
  //   }
  //   const res = await window.FileManagerWrapper.exist(credential, path);

  //   if (res) return true;

  //   return false;
  // };

  // const stat = async (credential: ICredential, path: string) => {
  //   if (!window.FileManagerWrapper) {
  //     logCtx.error(
  //       'stat: Invalid [FileManagerWrapper]',
  //       'Please check Electron App (IPC handlers)'
  //     );
  //     return {};
  //   }
  //   const res = await window.FileManagerWrapper.stat(credential, path);
  //   return res;
  // };

  // const get = async (credential: ICredential, path: string) => {
  //   if (!window.FileManagerWrapper) {
  //     logCtx.error(
  //       'get: Invalid [FileManagerWrapper]',
  //       'Please check Electron App (IPC handlers)'
  //     );
  //     return null;
  //   }
  //   const res = await window.FileManagerWrapper.get(credential, path);
  //   return res;
  // };

  // const put = async (
  //   credential: ICredential,
  //   content: string,
  //   path: string
  // ) => {
  //   if (!window.FileManagerWrapper) {
  //     logCtx.error(
  //       'put: Invalid [FileManagerWrapper]',
  //       'Please check Electron App (IPC handlers)'
  //     );
  //     return null;
  //   }

  //   if (content.length === 0) {
  //     logCtx.error('put: Invalid empty content', DEFAULT_BUG_TEXT);
  //     return null;
  //   }

  //   if (path.length === 0) {
  //     logCtx.error('put: Invalid empty path', DEFAULT_BUG_TEXT);
  //     return null;
  //   }

  //   const res = await window.FileManagerWrapper.put(credential, content, path);
  //   return res;
  // };

  // PasswordManager methods
  const setPassword = async (credential: ICredential) => {
    if (!window.PasswordManager) {
      logCtx.error("setPassword: Invalid [PasswordManager]", "Please check Electron App (IPC handlers)");
      return null;
    }

    credential.service = "RosNodeManager";
    credential.account = `${credential.username}:${credential.host}`;

    const res = await window.PasswordManager.setPassword(credential.service, credential.account, credential.password);
    return res;
  };

  const deletePassword = async (credential: ICredential) => {
    if (!window.PasswordManager) {
      logCtx.error("deletePassword: Invalid [PasswordManager]", "Please check Electron App (IPC handlers)");
      return;
    }

    await window.PasswordManager.deletePassword(credential.service, credential.account);
  };

  const checkPassword = (credential: ICredential) => {
    if (!window.CommandExecutor) {
      logCtx.error("checkPassword: Invalid [CommandExecutor]", "Please check Electron App (IPC handlers)");
      return Promise.resolve({
        result: false,
        message: "checkPassword: Invalid [CommandExecutor]; Please check Electron App (IPC handlers)",
      });
    }
    return window.CommandExecutor.exec(credential, "pwd");
  };

  // credentials methods
  const addCredential = async (credential: ICredential) => {
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

    // assign unique ID
    credential.id = generateUniqueId();

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
      setCredentials((previousCredentials: ICredential[]) => {
        return [...previousCredentials, credential];
      });
    } catch (error: any) {
      return { result: false, message: error.message };
    }

    return { result: true, message: "" };
  };

  const deleteCredential = (credentialId: string) => {
    if (typeof credentialId !== "string") {
      return false;
    }

    if (credentialId.length === 0) return false;

    // delete associated password
    let foundCredential = false;
    credentials.forEach((c) => {
      if (c.id === credentialId) {
        deletePassword(c);
        foundCredential = true;
      }
    });

    if (!foundCredential) return false;

    // remove credential from App
    setCredentials((previousCredentials) => {
      return previousCredentials.filter((c) => c.id !== credentialId);
    });

    return true;
  };

  const getCredentialHost = (host: string) => {
    if (typeof host !== "string") {
      return null;
    }

    // search the SSH credentials based on host
    let credentialHost = null;
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
      logCtx.error(
        `No SSH credentials have been configured for host: [${JSON.stringify(host)}]`,
        "Please check the SSH credential settings"
      );
      return null;
    }

    return credentialHost;
  };

  // CommandExecutor
  const exec = async (credential: ICredential | null, command: string) => {
    if (!window.CommandExecutor) {
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

    const res = await window.CommandExecutor.exec(credential, command);
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
