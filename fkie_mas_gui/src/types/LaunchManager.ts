import { TCredential, TResult } from "@/types";

export const LaunchManagerEvents = {
  startTerminalManager: "launchManager:startTerminalManager",
  startDaemon: "launchManager:startDaemon",
  startMasterDiscovery: "launchManager:startMasterDiscovery",
  startMasterSync: "launchManager:startMasterSync",
  startDynamicReconfigureClient: "launchManager:startDynamicReconfigureClient",
};

/**
 * Start system nodes of the multi agent suite.
 */
export type TLaunchManager = {
  /** Try to start a Terminal manager (default TTYD) */
  startTerminalManager: (
    rosVersion?: string | null,
    credential?: TCredential | null,
    port?: number
  ) => Promise<TResult>;

  /** Try to start a master discovery node */
  startMasterDiscovery: (
    rosVersion?: string | null,
    credential?: TCredential | null,
    name?: string,
    networkId?: number,
    group?: string,
    heartbeatHz?: number,
    robotHosts?: string[],
    ros1MasterUri?: string,
    forceStart?: boolean
  ) => Promise<TResult>;

  /** Try to start a master sync node */
  startMasterSync: (
    rosVersion?: string | null,
    credential?: TCredential | null,
    name?: string,
    doNotSync?: string[],
    syncTopics?: string[],
    ros1MasterUri?: string,
    forceStart?: boolean
  ) => Promise<TResult>;

  /** Try to start a Daemon Node */
  startDaemon: (
    rosVersion?: string | null,
    credential?: TCredential | null,
    name?: string,
    networkId?: number,
    ros1MasterUri?: string,
    forceStart?: boolean
  ) => Promise<TResult>;

  /** Try to start a Dynamic Reconfigure Node */
  startDynamicReconfigureClient: (
    name: string,
    rosMasterUri: string,
    credential?: TCredential | null
  ) => Promise<TResult>;
};
