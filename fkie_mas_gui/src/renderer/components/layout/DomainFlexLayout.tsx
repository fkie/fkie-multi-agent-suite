import AccountTreeIcon from "@mui/icons-material/AccountTree";
import AppsIcon from "@mui/icons-material/Apps";
import FeaturedPlayListIcon from "@mui/icons-material/FeaturedPlayList";
import TopicIcon from "@mui/icons-material/Topic";
import { Box } from "@mui/material";
import * as FlexLayout from "flexlayout-react";
import { useCallback, useEffect, useMemo, useReducer } from "react";
import { useCustomEventListener } from "react-custom-events";

import {
  EVENT_CLOSE_COMPONENT,
  EVENT_OPEN_COMPONENT,
  EVENT_SELECT_TAB,
  EVENT_TOGGLE_COMPONENT,
  TEventId,
  TEventOpenComponent,
  TEventSelectTab,
} from "@/renderer/components/layout/events";
import { pAddTabStickyButton } from "@/renderer/components/layout/helpers";
import { contentToId, matchesContentId, TContentId } from "@/renderer/components/layout/LayoutTabConfig";
import { usePersistentLayout } from "@/renderer/hooks/usePersistentLayout";
import { useRosContext } from "@/renderer/hooks/useRosContext";
import { LAYOUT_TABS } from "./LayoutDefines";
import { hasJsonNode, TJsonNode } from "./LayoutPersistance";
import { collapseBorderOnLastTab, ensureBorderTabVisible, resolveDockTarget } from "./LayoutTargets";

/** components which are persisted inside a domain layout */
const DOMAIN_LAYOUT_COMPONENTS: string[] = [
  LAYOUT_TABS.NODES,
  LAYOUT_TABS.TOPICS,
  LAYOUT_TABS.SERVICES,
  LAYOUT_TABS.ACTIONS,
  LAYOUT_TABS.APPS,
];

/** Default layout of a domain sub-layout: one tabset plus bottom/right borders. */
function createDefaultDomainLayout(contentId: TContentId): FlexLayout.IJsonModel {
  return {
    global: {
      tabEnableRename: false,
      tabSetEnableSingleTabStretch: false,
      tabSetEnableTabStrip: true,
      tabSetEnableTabWrap: false,
      tabSetEnableMaximize: true,
    },
    borders: [
      { type: "border", location: "bottom", size: 300, selected: -1, enableAutoHide: true, children: [] },
      { type: "border", location: "right", size: 400, selected: -1, enableAutoHide: true, children: [] },
    ],
    layout: {
      type: "row",
      weight: 100,
      children: [
        {
          type: "tabset",
          weight: 100,
          children: [
            {
              id: `${LAYOUT_TABS.NODES}-${contentToId(contentId)}`,
              type: "tab",
              name: "Nodes",
              enableClose: false,
              component: LAYOUT_TABS.NODES,
              config: { contentId: contentId },
            },
          ],
        },
      ],
    },
  };
}

type DomainFlexLayoutProps = {
  storageKey: string;
  insideTabId: string;
  contentId: TContentId;
  factory: (tabNode: FlexLayout.TabNode, contentId: TContentId) => JSX.Element;
  onRenderTab: (node: FlexLayout.TabNode, renderValues: FlexLayout.ITabRenderValues) => void;
  onCloseTab: (id: string) => void;
};

export function DomainFlexLayout(props: DomainFlexLayoutProps): JSX.Element | null {
  const { contentId, storageKey, insideTabId, factory, onRenderTab, onCloseTab } = props;

  const rosCtx = useRosContext();
  const [forceUpdate, setForceUpdate] = useReducer((x) => x + 1, 0);

  const nodesTabId = `${LAYOUT_TABS.NODES}-${contentToId(contentId)}`;
  const defaultLayout = useMemo(() => createDefaultDomainLayout(contentId), [contentId]);

  const keepDomainTab = useCallback((tab: TJsonNode): boolean => {
    return DOMAIN_LAYOUT_COMPONENTS.includes(tab.component ?? "");
  }, []);

  // the nodes tab is mandatory: fall back to the default layout if it got lost
  const repairDomainLayout = useCallback(
    (json: FlexLayout.IJsonModel): FlexLayout.IJsonModel => {
      return hasJsonNode(json.layout as TJsonNode, nodesTabId, "tab") ? json : structuredClone(defaultLayout);
    },
    [nodesTabId, defaultLayout]
  );

  const { model, saveLayout } = usePersistentLayout({
    stateKey: `${storageKey}-${contentToId(contentId)}`,
    defaultLayout,
    version: 1,
    migrateFromLocalStorageKey: `${storageKey}-${contentToId(contentId)}`,
    keepTab: keepDomainTab,
    repairLayout: repairDomainLayout,
  });

  const closeThisTab = useCallback(() => {
    console.log(`DomainFlexLayout: all providers for ${contentToId(contentId)} removed. Close this tab!`);
    onCloseTab(`${LAYOUT_TABS.DOMAIN}-${contentToId(contentId)}`);
  }, [contentId, onCloseTab]);

  useEffect(() => {
    // if no provider is available for this domain, close this tab
    for (const p of rosCtx.providers) {
      if (p.connection.domainId === contentId.domainId || p.id === contentId.providerId) return;
    }
    closeThisTab();
  }, [contentId, rosCtx.providers, closeThisTab]);

  // select the nodes tab when the contentId changes
  useEffect(() => {
    model?.doAction(FlexLayout.Actions.selectTab(nodesTabId));
  }, [model, nodesTabId]);

  const nodeFactory = useCallback(
    (tabNode: FlexLayout.TabNode): JSX.Element => factory(tabNode, contentId),
    [factory, contentId]
  );

  /** Adds a tab to the resolved tabset/border of this sub-layout. */
  const addTab = useCallback(
    (data: TEventOpenComponent): void => {
      if (!model) return;
      const target = resolveDockTarget(model, data.toNodeId, nodesTabId);
      if (!target) {
        console.warn(`DomainFlexLayout: no target found to add tab ${data.id}`);
        return;
      }
      const tab: FlexLayout.ITabAttributes = {
        id: data.id,
        type: "tab",
        name: data.title,
        component: data.component,
        enableClose: data.closable,
        enablePopout: false,
        config: data.config,
      };
      model.doAction(FlexLayout.Actions.addTab(tab, target.id, FlexLayout.DockLocation.CENTER, -1));
      if (target.isBorder) {
        ensureBorderTabVisible(model, target.location);
      }
    },
    [model, nodesTabId]
  );

  const deleteTab = useCallback(
    (tabId: string): void => {
      if (!model?.getNodeById(tabId)) return;
      model.doAction(FlexLayout.Actions.deleteTab(tabId));
    },
    [model]
  );

  function onRenderTabSet(
    node: FlexLayout.TabSetNode | FlexLayout.BorderNode,
    renderValues: FlexLayout.ITabSetRenderValues
  ): void {
    if (!model) return;
    const hasNodesTab = node.getChildren().some((child) => child.getId() === nodesTabId);
    if (!hasNodesTab) return;

    const buttons: { id: string; title: string; component: string; icon: JSX.Element }[] = [
      {
        id: LAYOUT_TABS.TOPICS,
        title: "Topics",
        component: LAYOUT_TABS.TOPICS,
        icon: <TopicIcon sx={{ fontSize: "inherit" }} />,
      },
      {
        id: LAYOUT_TABS.SERVICES,
        title: "Services",
        component: LAYOUT_TABS.SERVICES,
        icon: <FeaturedPlayListIcon sx={{ fontSize: "inherit" }} />,
      },
      {
        id: LAYOUT_TABS.ACTIONS,
        title: "Actions",
        component: LAYOUT_TABS.ACTIONS,
        icon: <AccountTreeIcon sx={{ fontSize: "inherit" }} />,
      },
      {
        id: LAYOUT_TABS.APPS,
        title: "ROS Apps",
        component: LAYOUT_TABS.APPS,
        icon: <AppsIcon sx={{ fontSize: "inherit" }} />,
      },
    ];

    for (const btn of buttons) {
      pAddTabStickyButton({
        model: model,
        container: renderValues.stickyButtons,
        id: `${btn.id}-${contentToId(contentId)}`,
        title: btn.title,
        component: btn.component,
        setId: node.getId(),
        icon: btn.icon,
        config: { contentId: contentId, insideDomainLayout: true },
      });
    }
  }

  useCustomEventListener(EVENT_SELECT_TAB, (data: TEventSelectTab) => {
    // when the surrounding domain tab becomes active again, force a resize
    if (data.tabId === insideTabId) {
      setForceUpdate();
    }
    model?.doAction(FlexLayout.Actions.selectTab(data.tabId));
  });

  useCustomEventListener(
    EVENT_OPEN_COMPONENT,
    (data: TEventOpenComponent) => {
      if (!model || !data.config?.insideDomainLayout) return;
      if (!matchesContentId(data.config.contentId, contentId)) return;

      if (model.getNodeById(data.id)) {
        model.doAction(FlexLayout.Actions.selectTab(data.id));
        return;
      }
      addTab(data);
    },
    [model, contentId, addTab]
  );

  useCustomEventListener(
    EVENT_TOGGLE_COMPONENT,
    (data: TEventOpenComponent) => {
      if (!model || !data.config?.insideDomainLayout) return;
      if (!matchesContentId(data.config.contentId, contentId)) return;

      const tab = model.getNodeById(data.id) as FlexLayout.TabNode | undefined;
      if (!tab) {
        addTab(data);
        return;
      }
      if (!tab.isVisible()) {
        model.doAction(FlexLayout.Actions.selectTab(data.id));
        return;
      }
      deleteTab(data.id);
    },
    [model, contentId, addTab, deleteTab]
  );

  /** Close tabs on signals from the tab itself (e.g. ctrl+d) */
  useCustomEventListener(
    EVENT_CLOSE_COMPONENT,
    (data: TEventId) => {
      deleteTab(data.id);
    },
    [deleteTab]
  );

  useEffect(() => {
    window.dispatchEvent(new Event("resize"));
  }, [forceUpdate]);

  if (!model) return null;

  return (
    <Box sx={{ flex: 1, height: "100%", width: "100%", overflow: "hidden", position: "relative" }}>
      <FlexLayout.Layout
        model={model}
        factory={nodeFactory}
        onAction={(action: FlexLayout.Action) => {
          if (action.type === FlexLayout.Actions.DELETE_TAB) {
            const node = model.getNodeById(action.data.node);
            collapseBorderOnLastTab(model, action.data.node);
            // select the "Nodes" tab if it lives in the same tabset as the closed tab
            for (const tab of node?.getParent()?.getChildren() ?? []) {
              if (tab.getType() === "tab" && (tab as FlexLayout.TabNode).getComponent() === LAYOUT_TABS.NODES) {
                model.doAction(FlexLayout.Actions.selectTab(tab.getId()));
              }
            }
          }
          return action;
        }}
        onModelChange={(_model, _action) => {
          if (![FlexLayout.Actions.SELECT_TAB, FlexLayout.Actions.SET_ACTIVE_TABSET].includes(_action.type)) {
            saveLayout(_model);
          }
        }}
        onRenderTab={onRenderTab}
        onRenderTabSet={onRenderTabSet}
      />
    </Box>
  );
}
