const LAYOUT_TAB_SETS = {
  CENTER: "main",
  HOSTS: "hosts",
  LEFT_TOP: "left_top",
  LEFT_BOTTOM: "left_bottom",
  BORDER_TOP: "border_top",
  BORDER_RIGHT: "border_right",
  BORDER_LEFT: "border_left",
  BORDER_BOTTOM: "border_bottom",
};

const LAYOUT_TABS = {
  ABOUT: "about-tab",
  APPS: "apps-tab",
  NODES: "nodes-tab",
  PACKAGES: "packages-tab",
  HOSTS: "hosts-tab",
  PARAMETER: "parameter-tab",
  DETAILS: "details-tab",
  TOPICS: "topics-tab",
  SERVICES: "services-tab",
  ACTIONS: "actions-tab",
  ACTION_SEND_GOAL: "action-send-goal-tab",
  ACTION_INTROSPECTION: "action-introspection-tab",
  SETTINGS: "settings-tab",
  LOGGING: "logging-tab",
  DOMAIN: "domain-tab",
  EDITOR: "editor-tab",
  NO_RUNNING_DAEMONS: "no-running-daemon-info-tab",
  NODE_LOGGER: "node-logger-tab",
  PROVIDER_LAUNCH_CONTROL: "provider-launch-control-tab",
  SERVICE_CALLER: "service-caller-tab",
  SERVICE_INTROSPECTION: "service-introspection-tab",
  TERMINAL: "terminal-tab",
  TOPIC_ECHO: "topic-echo-tab",
  TOPIC_PUBLISHER: "topic-publisher-tab",
};

const LAYOUT_TAB_LIST = Object.keys(LAYOUT_TABS).map((key) => {
  return LAYOUT_TABS[key];
});

export { LAYOUT_TAB_LIST, LAYOUT_TAB_SETS, LAYOUT_TABS };

/** Components which live inside a domain sub-layout and must never leave it. */
export const DOMAIN_LAYOUT_COMPONENTS: string[] = [
  LAYOUT_TABS.NODES,
  LAYOUT_TABS.TOPICS,
  LAYOUT_TABS.SERVICES,
  LAYOUT_TABS.ACTIONS,
  LAYOUT_TABS.APPS,
];

/** Panels whose internal state must survive tab moves and hide/show. */
export const PERSISTENT_COMPONENTS: string[] = [
  LAYOUT_TABS.TERMINAL,
  LAYOUT_TABS.EDITOR,
  LAYOUT_TABS.TOPIC_ECHO,
  LAYOUT_TABS.TOPIC_PUBLISHER,
  LAYOUT_TABS.SERVICE_CALLER,
  LAYOUT_TABS.SERVICE_INTROSPECTION,
  LAYOUT_TABS.ACTION_SEND_GOAL,
  LAYOUT_TABS.ACTION_INTROSPECTION,
];
