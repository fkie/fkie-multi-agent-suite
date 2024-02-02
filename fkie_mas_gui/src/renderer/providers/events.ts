/* eslint-disable max-classes-per-file */
import { ConnectionState } from '.';
import {
  LaunchContent,
  PathEvent,
  RosNode,
  ScreensMapping,
  SubscriberEvent,
  SystemWarningGroup,
} from '../models';
import CrossbarIOProvider from './crossbar_io/CrossbarIOProvider';

export { ConnectionState };

export const EVENT_PROVIDER_ACTIVITY = 'EVENT_PROVIDER_ACTIVITY' as const;
export const EVENT_PROVIDER_DISCOVERED = 'EVENT_PROVIDER_DISCOVERED' as const;
export const EVENT_PROVIDER_LAUNCH_LIST = 'EVENT_PROVIDER_LAUNCH_LIST' as const;
export const EVENT_PROVIDER_PATH_EVENT = 'EVENT_PROVIDER_PATH_EVENT' as const;
export const EVENT_PROVIDER_ROS_NODES = 'EVENT_PROVIDER_ROS_NODES' as const;
export const EVENT_PROVIDER_SCREENS = 'EVENT_PROVIDER_SCREENS' as const;
export const EVENT_PROVIDER_STATE = 'EVENT_PROVIDER_STATE' as const;
export const EVENT_PROVIDER_SUBSCRIBER_EVENT_PREFIX =
  'EVENT_PROVIDER_SUBSCRIBER_EVENT_PREFIX' as const;
export const EVENT_PROVIDER_TIME_DIFF = 'EVENT_PROVIDER_TIME_DIFF' as const;
export const EVENT_PROVIDER_WARNINGS = 'EVENT_PROVIDER_WARNINGS' as const;

export class EventProviderActivity {
  provider: CrossbarIOProvider;

  active: boolean;

  type: string;

  constructor(
    provider: CrossbarIOProvider,
    active: boolean,
    type: string = '',
  ) {
    this.provider = provider;
    this.active = active;
    this.type = type;
  }
}
export class EventProviderDiscovered {
  provider: CrossbarIOProvider;

  reporter: CrossbarIOProvider;

  constructor(provider: CrossbarIOProvider, reporter: CrossbarIOProvider) {
    this.provider = provider;
    this.reporter = reporter;
  }
}

export class EventProviderLaunchList {
  provider: CrossbarIOProvider;

  launches: LaunchContent[];

  constructor(provider: CrossbarIOProvider, launches: LaunchContent[]) {
    this.provider = provider;
    this.launches = launches;
  }
}

export class EventProviderPathEvent {
  provider: CrossbarIOProvider;

  path: PathEvent;

  constructor(provider: CrossbarIOProvider, path: PathEvent) {
    this.provider = provider;
    this.path = path;
  }
}

export class EventProviderRosNodes {
  provider: CrossbarIOProvider;

  nodes: RosNode[];

  constructor(provider: CrossbarIOProvider, nodes: RosNode[]) {
    this.provider = provider;
    this.nodes = nodes;
  }
}

export class EventProviderScreens {
  provider: CrossbarIOProvider;

  screens: ScreensMapping[];

  constructor(provider: CrossbarIOProvider, screens: ScreensMapping[]) {
    this.provider = provider;
    this.screens = screens;
  }
}

export class EventProviderState {
  provider: CrossbarIOProvider;

  newState: ConnectionState;

  oldState: ConnectionState;

  details: string;

  constructor(
    provider: CrossbarIOProvider,
    newState: ConnectionState,
    oldState: ConnectionState,
    details: string = '',
  ) {
    this.provider = provider;
    this.newState = newState;
    this.oldState = oldState;
    this.details = details;
  }
}

export class EventProviderSubscriberEvent {
  provider: CrossbarIOProvider;

  event: SubscriberEvent;

  constructor(provider: CrossbarIOProvider, event: SubscriberEvent) {
    this.provider = provider;
    this.event = event;
  }
}

export class EventProviderTimeDiff {
  provider: CrossbarIOProvider;

  timeDiff: number;

  constructor(provider: CrossbarIOProvider, timeDiff: number) {
    this.provider = provider;
    this.timeDiff = timeDiff;
  }
}

export class EventProviderWarnings {
  provider: CrossbarIOProvider;

  warnings: SystemWarningGroup[];

  constructor(provider: CrossbarIOProvider, warnings: SystemWarningGroup[]) {
    this.provider = provider;
    this.warnings = warnings;
  }
}
