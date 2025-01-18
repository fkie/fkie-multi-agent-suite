/* eslint-disable max-classes-per-file */
import { ConnectConfig } from "ssh2";
import {
  LaunchContent,
  PathEvent,
  ProviderLaunchConfiguration,
  RosNode,
  ScreensMapping,
  SubscriberEvent,
  SystemWarningGroup,
} from "../models";
import ConnectionState from "./ConnectionState";
import Provider from "./Provider";

export { ConnectionState };

export class EventProviderActivity {
  provider: Provider;

  active: boolean;

  type: string;

  constructor(provider: Provider, active: boolean, type: string = "") {
    this.provider = provider;
    this.active = active;
    this.type = type;
  }
}
export class EventProviderDiscovered {
  provider: Provider;

  reporter: Provider;

  constructor(provider: Provider, reporter: Provider) {
    this.provider = provider;
    this.reporter = reporter;
  }
}

export class EventProviderRemoved {
  provider: Provider;

  reporter: Provider;

  constructor(provider: Provider, reporter: Provider) {
    this.provider = provider;
    this.reporter = reporter;
  }
}

export class EventProviderLaunchList {
  provider: Provider;

  launches: LaunchContent[];

  constructor(provider: Provider, launches: LaunchContent[]) {
    this.provider = provider;
    this.launches = launches;
  }
}

export class EventProviderPathEvent {
  provider: Provider;

  path: PathEvent;

  constructor(provider: Provider, path: PathEvent) {
    this.provider = provider;
    this.path = path;
  }
}

export class EventProviderRosNodes {
  provider: Provider;

  nodes: RosNode[];

  constructor(provider: Provider, nodes: RosNode[]) {
    this.provider = provider;
    this.nodes = nodes;
  }
}

export class EventProviderRosServices {
  provider: Provider;

  constructor(provider: Provider) {
    this.provider = provider;
  }
}

export class EventProviderRosTopics {
  provider: Provider;

  constructor(provider: Provider) {
    this.provider = provider;
  }
}

export class EventProviderScreens {
  provider: Provider;

  screens: ScreensMapping[];

  constructor(provider: Provider, screens: ScreensMapping[]) {
    this.provider = provider;
    this.screens = screens;
  }
}

export class EventProviderState {
  provider: Provider;

  newState: ConnectionState;

  oldState: ConnectionState;

  details: string;

  constructor(provider: Provider, newState: ConnectionState, oldState: ConnectionState, details: string = "") {
    this.provider = provider;
    this.newState = newState;
    this.oldState = oldState;
    this.details = details;
  }
}

export class EventProviderSubscriberEvent {
  provider: Provider;

  event: SubscriberEvent;

  constructor(provider: Provider, event: SubscriberEvent) {
    this.provider = provider;
    this.event = event;
  }
}

export class EventProviderTimeDiff {
  provider: Provider;

  timeDiff: number;

  constructor(provider: Provider, timeDiff: number) {
    this.provider = provider;
    this.timeDiff = timeDiff;
  }
}

export class EventProviderWarnings {
  provider: Provider;

  warnings: SystemWarningGroup[];

  constructor(provider: Provider, warnings: SystemWarningGroup[]) {
    this.provider = provider;
    this.warnings = warnings;
  }
}

export class EventProviderDelay {
  provider: Provider;

  delay: number;

  constructor(provider: Provider, delay: number) {
    this.provider = provider;
    this.delay = delay;
  }
}

export class EventProviderNodeStarted {
  provider: Provider;

  node: RosNode;

  constructor(provider: Provider, node: RosNode) {
    this.provider = provider;
    this.node = node;
  }
}

export class EventProviderLaunchLoaded {
  provider: Provider;

  launchFile: string;

  constructor(provider: Provider, launchFile: string) {
    this.provider = provider;
    this.launchFile = launchFile;
  }
}

export class EventProviderRestartNodes {
  provider: Provider;

  nodes: RosNode[];

  constructor(provider: Provider, nodes: RosNode[]) {
    this.provider = provider;
    this.nodes = nodes;
  }
}

export class EventProviderAuthRequest {
  provider: Provider;

  launchConfig: ProviderLaunchConfiguration;

  connectConfig: ConnectConfig;

  constructor(provider: Provider, launchConfig: ProviderLaunchConfiguration, connectConfig: ConnectConfig) {
    this.provider = provider;
    this.launchConfig = launchConfig;
    this.connectConfig = connectConfig;
  }
}
