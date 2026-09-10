import { ProviderLaunchConfiguration, RosNode } from "@/renderer/models";
import { CmdType, TEditorConfig, TEnvEntry, TPublisherConfig, TSubscriberConfig } from "@/types";
import { TServiceConfig } from "@/types/ServiceManager";
import { TTerminalConfig } from "@/types/TerminalManager";

export type TExtTerminalConfig = {
  type: CmdType;
  providerId: string;
  nodeName: string;
  topicName: string;
  screen: string;
  cmd: string;
  env: TEnvEntry[];
};

export type TParameterConfig = {
  id: string;
  nodes: RosNode[];
  providers: string[];
};

export type TNodeLoggerConfig = {
  id: string;
  node: RosNode;
};

export type TProviderLaunchConfig = {
  id: string;
  config: ProviderLaunchConfiguration;
};

export type TContentId =
  | {
      domainId: number;
      providerId?: never;
    }
  | {
      providerId: string;
      domainId?: never;
    };

export const contentToId: (contentId?: TContentId) => string | undefined = (contentId) => {
  return contentId?.domainId ? String(contentId?.domainId) : contentId?.providerId;
};

/** Compare two content ids (domain tabs use domainId, host tabs use providerId) */
export function matchesContentId(a?: TContentId, b?: TContentId): boolean {
  if (!a || !b) return false;
  if (a.providerId || b.providerId) return a.providerId === b.providerId;
  return a.domainId === b.domainId;
}

export type TLayoutTabConfig = {
  /**
   * @deprecated Instead, create a '...Config' and extend the factory() function in NodeManager.tsx
   */
  reactNode?: React.ReactNode;

  contentId?: TContentId;

  /** if true, the tab must be opened inside the domain sub-layout instead of the main layout */
  insideDomainLayout?: boolean;

  openExternal?: boolean;

  terminalType?: CmdType;

  extTerminalConfig?: TExtTerminalConfig;

  editorConfig?: TEditorConfig;

  publisherConfig?: TPublisherConfig;

  subscriberConfig?: TSubscriberConfig;

  terminalConfig?: TTerminalConfig;

  parameterConfig?: TParameterConfig;

  serviceCallerConfig?: TServiceConfig;

  serviceIntrospectionConfig?: TServiceConfig;

  actionConfig?: TServiceConfig;

  actionIntrospectionConfig?: TServiceConfig;

  nodeLoggerConfig?: TNodeLoggerConfig;

  providerLaunchConfig?: TProviderLaunchConfig;

  filterText?: string;
};
