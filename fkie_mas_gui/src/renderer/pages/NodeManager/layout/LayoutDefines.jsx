const LAYOUT_TAB_SETS = {
  CENTER: 'main',
  PROVIDER: 'provider',
  LEFT_TOP: 'left_top',
  LEFT_BOTTOM: 'left_bottom',
  BORDER_TOP: 'border_top',
  BORDER_RIGHT: 'border_right',
  BORDER_BOTTOM: 'border_bottom',
};

const LAYOUT_TABS = {
  HOSTS: 'hosts_tab',
  PACKAGES: 'packages_tab',
  PROVIDER: 'provider_tab',
  PARAMETER: 'parameter_tab',
  NODE_DETAILS: 'node_details_tab',
  TOPICS: 'topics_tab',
  SERVICES: 'services_tab',
  LOGGING: 'logging_tab',
};

const LAYOUT_TAB_LIST = Object.keys(LAYOUT_TABS).map((key) => {
  return LAYOUT_TABS[key];
});

export { LAYOUT_TABS, LAYOUT_TAB_LIST, LAYOUT_TAB_SETS };
