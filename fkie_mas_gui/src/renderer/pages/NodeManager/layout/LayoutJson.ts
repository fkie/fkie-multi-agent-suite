import { IJsonBorderNode, IJsonModel } from "flexlayout-react";

import { LAYOUT_TAB_SETS, LAYOUT_TABS } from "./LayoutDefines";

interface IJsonBorderNodeExt extends IJsonBorderNode {
  id: string;
}

const enablePopout = !window.commandExecutor && window.location.href.indexOf(":6275") === -1;

const DEFAULT_LAYOUT = {
  global: {
    splitterSize: 5,
    tabenablePopout: true,
    tabSetMinWidth: 100,
    tabSetMinHeight: 100,
    borderMinSize: 100,
    tabEnableRename: false,
    tabSetEnableClose: false,
    enableRotateBorderIcons: false,
    tabSetEnableDeleteWhenEmpty: true,
    tabEnableRenderOnDemand: false,
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
    type: "row",
    weight: 100,
    children: [
      {
        type: "row",
        weight: 30,
        children: [
          {
            id: LAYOUT_TAB_SETS.HOSTS,
            type: "tabset",
            weight: 30,
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
            type: "tabset",
            weight: 30,
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
      {
        id: LAYOUT_TAB_SETS.CENTER,
        type: "tabset",
        weight: 70,
        children: [
          {
            id: LAYOUT_TABS.NODES,
            type: "tab",
            name: "Nodes",
            component: LAYOUT_TABS.NODES,
            enableClose: false,
            enablePopout: enablePopout,
          },
        ],
      },
    ],
  },
} as IJsonModel;
export { DEFAULT_LAYOUT };
