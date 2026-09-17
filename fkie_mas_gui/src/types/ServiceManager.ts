export const ServiceManagerEvents = {
  has: "service:has",
  start: "service:start",
  close: "service:close",
  onClose: "service:onClose",
};

export type ServiceCloseCallback = (id: string) => void;

export type TServiceConfig = {
  id: string;
  providerId: string;
  host: string;
  port: number;
  serviceName: string;
  serviceType: string;
  htmlName: "serviceCaller" | "serviceIntrospection" | "actionSendGoal" | "actionIntrospection";
};

export type TServiceManager = {
  start: (props: TServiceConfig) => Promise<string | null>;
  close: (id: string) => Promise<boolean>;
  has: (id: string) => Promise<boolean>;
  onClose: (callback: ServiceCloseCallback) => void;
};
