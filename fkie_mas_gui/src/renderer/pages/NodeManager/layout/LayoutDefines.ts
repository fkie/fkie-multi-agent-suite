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
  ABOUT: "about_tab",
  NODES: "nodes_tab",
  PACKAGES: "packages_tab",
  HOSTS: "hosts_tab",
  PARAMETER: "parameter_tab",
  DETAILS: "details_tab",
  TOPICS: "topics_tab",
  SERVICES: "services_tab",
  SETTINGS: "settings-tab",
  LOGGING: "logging_tab",
};

const LAYOUT_TAB_LIST = Object.keys(LAYOUT_TABS).map((key) => {
  return LAYOUT_TABS[key];
});

export { LAYOUT_TAB_LIST, LAYOUT_TAB_SETS, LAYOUT_TABS };
