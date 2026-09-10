import { IJsonBorderNode, IJsonModel } from "flexlayout-react";

import { LAYOUT_TAB_SETS, LAYOUT_TABS } from "./LayoutDefines";

export interface IJsonBorderNodeExt extends IJsonBorderNode {
  id: string;
}

console.log(`window.location.href: ${window.location.href}`);
const enablePopout = false;

export const LAYOUT_NO_RUNNING_DAEMONS = {
  id: LAYOUT_TABS.NO_RUNNING_DAEMONS,
  type: "tab",
  name: "Info",
  component: LAYOUT_TABS.NO_RUNNING_DAEMONS,
  enableClose: false,
  enablePopout: enablePopout,
};

export const LAYOUT_DOMAIN_TAB_SET = {
  id: LAYOUT_TAB_SETS.CENTER,
  type: "tabset",
  weight: 75,
  tabLocation: "top",
  enableDeleteWhenEmpty: false,
  enableClose: true,
  enableDivide: true,
  children: [structuredClone(LAYOUT_NO_RUNNING_DAEMONS)],
};

export const DEFAULT_LAYOUT = {
  global: {
    splitterSize: 5,
    tabEnablePopout: true,
    tabSetMinWidth: 100,
    tabSetMinHeight: 100,
    borderMinSize: 100,
    tabEnableRename: false,
    enableRotateBorderIcons: false,
    tabSetEnableDeleteWhenEmpty: true,
    tabEnableRenderOnDemand: false,
    tabSetEnableSingleTabStretch: true,
    tabSetEnableTabStrip: true,
  },
  borders: [
    {
      id: LAYOUT_TAB_SETS.BORDER_BOTTOM,
      type: "border",
      // selected: 0,
      size: 350,
      location: "bottom",
      enableAutoHide: true,
      children: [
        {
          type: "tab",
          id: LAYOUT_TABS.LOGGING,
          name: "Logging",
          component: LAYOUT_TABS.LOGGING,
          enableClose: false,
          enablePopout: enablePopout,
        },
      ],
    },
    {
      id: LAYOUT_TAB_SETS.BORDER_LEFT,
      type: "border",
      // selected: 0,
      size: 420,
      location: "left",
      enableClose: false,
      enableAutoHide: true,
      children: [],
    },
    {
      id: LAYOUT_TAB_SETS.BORDER_RIGHT,
      type: "border",
      // selected: 0,
      size: 540,
      location: "right",
      enableClose: false,
      enableAutoHide: true,
      children: [],
    },
    {
      id: LAYOUT_TAB_SETS.BORDER_TOP,
      type: "border",
      // selected: 0,
      size: 640,
      location: "top",
      enableClose: false,
      enableAutoHide: true,
      children: [],
    } as IJsonBorderNodeExt,
  ],
  layout: {
    id: "rootRow",
    type: "row",
    weight: 100,
    children: [
      {
        type: "row",
        weight: 25,
        children: [
          {
            id: LAYOUT_TAB_SETS.HOSTS,
            type: "tabset",
            weight: 35,
            tabSetEnableClose: false,
            children: [
              {
                id: LAYOUT_TABS.HOSTS,
                type: "tab",
                name: "Hosts",
                component: LAYOUT_TABS.HOSTS,
                enableClose: false,
                enablePopout: enablePopout,
              },
            ],
          },
          {
            id: "details-set",
            type: "tabset",
            weight: 65,
            selected: 0,
            tabSetEnableClose: false,
            children: [
              {
                id: LAYOUT_TABS.PACKAGES,
                type: "tab",
                name: "Packages",
                component: LAYOUT_TABS.PACKAGES,
                enableClose: false,
                enablePopout: enablePopout,
              },
              {
                id: LAYOUT_TABS.DETAILS,
                type: "tab",
                name: "Details",
                component: LAYOUT_TABS.DETAILS,
                enableClose: false,
                enablePopout: enablePopout,
              },
            ],
          },
        ],
      },
      LAYOUT_DOMAIN_TAB_SET,
    ],
  },
} as IJsonModel;
