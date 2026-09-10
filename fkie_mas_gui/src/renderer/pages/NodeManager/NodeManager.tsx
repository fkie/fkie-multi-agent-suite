import AccountTreeIcon from "@mui/icons-material/AccountTree";
import AppsIcon from "@mui/icons-material/Apps";
import BorderColorIcon from "@mui/icons-material/BorderColor";
import ChatBubbleOutlineIcon from "@mui/icons-material/ChatBubbleOutline";
import CloseIcon from "@mui/icons-material/Close";
import DesktopWindowsIcon from "@mui/icons-material/DesktopWindows";
import DesktopWindowsOutlinedIcon from "@mui/icons-material/DesktopWindowsOutlined";
import DomainIcon from "@mui/icons-material/Domain";
import DvrIcon from "@mui/icons-material/Dvr";
import FeaturedPlayListIcon from "@mui/icons-material/FeaturedPlayList";
import InfoOutlinedIcon from "@mui/icons-material/InfoOutlined";
import LaunchIcon from "@mui/icons-material/Launch";
import PlayCircleOutlineIcon from "@mui/icons-material/PlayCircleOutline";
import SettingsIcon from "@mui/icons-material/Settings";
import SettingsInputCompositeOutlinedIcon from "@mui/icons-material/SettingsInputCompositeOutlined";
import StartIcon from "@mui/icons-material/Start";
import SyncAltOutlinedIcon from "@mui/icons-material/SyncAltOutlined";
import TerminalIcon from "@mui/icons-material/Terminal";
import TopicIcon from "@mui/icons-material/Topic";
import TroubleshootIcon from "@mui/icons-material/Troubleshoot";
import TuneIcon from "@mui/icons-material/Tune";
import WysiwygIcon from "@mui/icons-material/Wysiwyg";

import {
  Badge,
  Button,
  Dialog,
  DialogActions,
  DialogContent,
  DialogContentText,
  DialogTitle,
  IconButton,
  Stack,
  Tooltip,
  Typography,
} from "@mui/material";
import { useDebounceCallback } from "@react-hook/debounce";
import {
  Action,
  Actions,
  BorderNode,
  DockLocation,
  // IJsonBorderNode,
  IJsonModel,
  IJsonRowNode,
  // IJsonTabSetNode,
  ITabAttributes,
  ITabRenderValues,
  ITabSetRenderValues,
  Layout,
  Model,
  TabNode,
  TabSetNode,
} from "flexlayout-react";
import React, { useCallback, useContext, useEffect, useRef, useState } from "react";
import { useCustomEventListener } from "react-custom-events";

// import ExternalAppsModal from "@/renderer/components/ExternalAppsModal/ExternalAppsModal";
import { DomainFlexLayout } from "@/renderer/components/layout/DomainFlexLayout";
import PasswordDialog from "@/renderer/components/PasswordModal/PasswordDialog";
import ProviderSelectionModal from "@/renderer/components/SelectionModal/ProviderSelectionModal";
import { getInfoStateColor } from "@/renderer/components/UI/Colors";
import DraggablePaper from "@/renderer/components/UI/DraggablePaper";
import { useAutoUpdateContext } from "@/renderer/context/AutoUpdateContext";
import { ElectronContext } from "@/renderer/context/ElectronContext";
import { useAppState } from "@/renderer/hooks/useAppState";
import { useLoggingContext } from "@/renderer/hooks/useLoggingContext";
import { useMonacoContext } from "@/renderer/hooks/useMonacoContext";
import { useNavigationContext } from "@/renderer/hooks/useNavigationContext";
import { useRosContext } from "@/renderer/hooks/useRosContext";
import { useSetting } from "@/renderer/hooks/useSetting";
import { getBaseName, getFileName } from "@/renderer/models";
import { SaveResult } from "@/renderer/monaco/types";
import { isEditorEditorId } from "@/renderer/monaco/utils";
import { Provider } from "@/renderer/providers";
import { EventProviderAuthRequest } from "@/renderer/providers/events";
import { EVENT_PROVIDER_AUTH_REQUEST } from "@/renderer/providers/eventTypes";
import { basename } from "@/renderer/utils";
import { isElectron, openBrowserSite } from "@/renderer/utils/popout";
import { CmdTypes, InfoStateLevel, TInfoState } from "@/types";
import { DEFAULT_LAYOUT, LAYOUT_TAB_LIST, LAYOUT_TAB_SETS, LAYOUT_TABS } from "./layout";
import {
  emitCloseComponent,
  emitSelectTab,
  emitToggleComponent,
  EVENT_CLOSE_COMPONENT,
  EVENT_INFO_STATE,
  EVENT_OPEN_COMPONENT,
  EVENT_SELECT_TAB,
  EVENT_TOGGLE_COMPONENT,
  TEventId,
  TEventInfoState,
  TEventOpenComponent,
  TEventSelectTab,
} from "./layout/events";
import { pAddTabStickyButton } from "./layout/helpers";
import { LAYOUT_DOMAIN_TAB_SET, LAYOUT_NO_RUNNING_DAEMONS } from "./layout/LayoutJson";
import { contentToId, TContentId, TExtTerminalConfig, TLayoutTabConfig } from "./layout/LayoutTabConfig";
import "./NodeManager.css";
import AboutPanel from "./panels/AboutPanel";
import ActionIntrospectionPanel from "./panels/ActionIntrospectionPanel";
import ActionPanel from "./panels/ActionPanel";
import ActionsPanel from "./panels/ActionsPanel";
import DetailsPanel from "./panels/DetailsPanel";
import ExternalAppsPanel from "./panels/ExternalAppsPanel";
import FileEditorPanel from "./panels/FileEditorPanel";
import HostTreeViewPanel from "./panels/HostTreeViewPanel";
import InfoNoRunningDaemons from "./panels/InfoNoRunningDaemons";
import LoggingPanel from "./panels/LoggingPanel";
import NodeLoggerPanel from "./panels/NodeLoggerPanel";
import PackageExplorerPanel from "./panels/PackageExplorerPanel";
import ParameterPanel from "./panels/ParameterPanel";
import ProviderLaunchConfigPanel from "./panels/ProviderLaunchConfigPanel";
import ProviderPanel from "./panels/ProviderPanel";
import ServiceCallerPanel from "./panels/ServiceCallerPanel";
import ServiceIntrospectionPanel from "./panels/ServiceIntrospectionPanel";
import ServicesPanel from "./panels/ServicesPanel";
import SettingsPanel from "./panels/SettingsPanel";
import SingleTerminalPanel from "./panels/SingleTerminalPanel";
import TopicEchoPanel from "./panels/TopicEchoPanel";
import TopicPublishPanel from "./panels/TopicPublishPanel";
import TopicsPanel from "./panels/TopicsPanel";

type TPanelId = {
  id: string;
  isBorder: boolean;
  location: DockLocation;
};

interface ITabAttributesExt extends ITabAttributes {
  toNodeId: string;
}

export default function NodeManager(): JSX.Element {
  const auCtx = useAutoUpdateContext();
  const electronCtx = useContext(ElectronContext);
  const rosCtx = useRosContext();
  const logCtx = useLoggingContext();
  const monacoCtx = useMonacoContext();
  const navCtx = useNavigationContext();

  const [tabFullName] = useSetting<boolean>("tabFullName");
  const [useDarkMode] = useSetting<boolean>("useDarkMode");
  const [openAsPopout] = useSetting<boolean>("openAsPopout");
  const [resetLayout, setResetLayout] = useSetting<boolean>("resetLayout");
  const [dedicatedTabsFor, setDedicatedTabsFor] = useSetting<string>("dedicatedTabsFor");
  const [fontSize, setFontSize] = useSetting<number>("fontSize");
  const [ignoreProcessesOnShutdown] = useSetting<string>("ignoreProcessesOnShutdown");

  const { value: layoutJson, set: setLayoutJson } = useAppState<IJsonModel>("layouts", "main", DEFAULT_LAYOUT, {
    version: 2,
    migrateFrom: {
      localStorageKey: "layout",
    },
  });
  const [model, setModel] = useState<Model>(() => Model.fromJson(sanitizeLayout(layoutJson)));
  const layoutRef = useRef<React.ComponentRef<typeof Layout> | null>(null);

  // const [layoutComponents] = useState<Record<string, React.ReactNode>>({});
  const [addToLayout, setAddToLayout] = useState<ITabAttributesExt[]>([]);
  const [dirtyTabs, setDirtyTabs] = useState<string[]>([]);
  const [passwordRequests, setPasswordRequests] = useState<React.ReactNode[]>([]);

  const [infoStates, setInfoStates] = useState<TInfoState[]>([]);
  const [infoStateTimer, setInfoStateTimer] = useState<NodeJS.Timeout | undefined>();
  const [currentInfoState, setCurrentInfoState] = useState<TInfoState | undefined>();

  // const [enablePopout, setEnablePopout] = useState<boolean>(!window.commandExecutor);

  const modelRef = useRef<Model>(model);
  useEffect(() => {
    modelRef.current = model;
  }, [model]);

  /**
   * @deprecated will be removed with TLayoutTabConfig.reactNode
   */
  const layoutComponentsRef = useRef<Record<string, React.ReactNode>>({});

  // enable/disable popout depending on environment
  // useEffect(() => {
  //   setEnablePopout(!window.commandExecutor);
  // }, []);

  // info state queue handling
  useEffect(() => {
    if (!infoStateTimer && infoStates.length > 0) {
      setCurrentInfoState(infoStates[0]);
      setInfoStateTimer(
        setTimeout(() => {
          setInfoStates((prev) => prev.slice(1));
          setInfoStateTimer(undefined);
        }, 1500)
      );
    } else if (infoStates.length === 0) {
      setCurrentInfoState(undefined);
    }
  }, [infoStates, infoStateTimer]);

  useCustomEventListener(
    EVENT_INFO_STATE,
    (data: TEventInfoState) => {
      setInfoStates((prev) => {
        const exists = prev.some((item) => item.level === data.level && item.message === data.message);
        if (exists) return prev;
        return [...prev, { level: data.level, message: data.message } as TInfoState];
      });
    },
    []
  );

  const hasTab = useCallback((layout: IJsonRowNode, editorId: string): boolean => {
    if (!layout.children) return false;
    const found = layout.children.filter((item: IJsonRowNode) => {
      if (item.type === "tab" && item.id === editorId) {
        return true;
      }
      if (item.children) {
        return hasTab(item, editorId);
      }
      return false;
    });
    return found.length > 0;
  }, []);

  // /** Disable float button if the GUI is not running in a browser */
  // const updateFloatButton = useCallback(
  //   (layout: IJsonRowNode | IJsonBorderNode | IJsonTabSetNode): boolean => {
  //     if (!layout.children) return false;
  //     let result = false;

  //     // biome-ignore lint/complexity/noForEach: <explanation>
  //     layout.children.forEach((item) => {
  //       if (item.type === "tab") {
  //         if (item.enablePopout !== enablePopout) {
  //           item.enablePopout = enablePopout;
  //           result = true;
  //         }
  //         if (item.children) {
  //           if (updateFloatButton(item)) {
  //             result = true;
  //           }
  //         }
  //       }
  //     });
  //     return result;
  //   },
  //   [enablePopout]
  // );

  useEffect(() => {
    navCtx.setLayoutModel(model);
  }, [model, navCtx]);

  // useEffect(() => {
  //   // update float button for all tabs on load or when layoutJson changes
  //   let changed = updateFloatButton(layoutJson.layout);

  //   for (const border of layoutJson.borders || []) {
  //     if (updateFloatButton(border)) {
  //       changed = true;
  //     }
  //   }
  //   for (const layout of layoutJson.layout.children || []) {
  //     if (updateFloatButton(layout)) {
  //       changed = true;
  //     }
  //   }

  //   if (changed) {
  //     setLayoutJson(layoutJson);
  //     setModel(Model.fromJson(layoutJson));
  //   }
  // }, [layoutJson]);

  useEffect(() => {
    const needsReset = resetLayout || !hasTab(layoutJson.layout, LAYOUT_TABS.DETAILS);

    if (needsReset) {
      setLayoutJson(structuredClone(DEFAULT_LAYOUT));
      setModel(Model.fromJson(DEFAULT_LAYOUT));
      setResetLayout(false);
      logCtx.success("Layout reset!", "", "layout reset");
    }
  }, [resetLayout, layoutJson, hasTab, logCtx, setLayoutJson]);

  /** Hide bottom panel when last terminal is closed and handle editor tabs with unsaved changes */
  const deleteTab = useCallback(
    (tabId: string, fromEvent: boolean = false): void => {
      // handle editor tabs with modified files
      if (isEditorEditorId(tabId)) {
        const modified = monacoCtx.getModifiedFilesByEditor(tabId);
        if (modified.length > 0) {
          const editorTab = model.getNodeById(tabId);
          if (editorTab) {
            model.doAction(Actions.selectTab(editorTab.getId()));
            setDirtyTabs([tabId]);
            return;
          }
        }
      }

      const nodeBId = model.getNodeById(tabId);
      if (!nodeBId) {
        if (!fromEvent) {
          // does the tab exists in domain flex layout? If yes, let it handle the close event
          emitCloseComponent({ id: tabId });
        }
        return;
      }
      const parentNode = nodeBId.getParent();
      if (!parentNode) {
        // delete tab
        model.doAction(Actions.deleteTab(tabId));
        // Cleanup React node reference
        delete layoutComponentsRef.current[tabId];
        return;
      }

      // handle tabs in bottom border
      if (parentNode.getType() === "border") {
        const borderNode = parentNode as BorderNode;
        if (borderNode.getLocation().getName() === DockLocation.BOTTOM.getName()) {
          // if closing last visible bottom tab, select it first to hide border
          const shouldSelectNewTab = parentNode.getChildren().length === 2 && borderNode.getSelectedNode()?.isVisible();
          if (shouldSelectNewTab) {
            model.doAction(Actions.selectTab(tabId));
          }
        }
      }

      // if closing last domain/hosts tab, add info tab first to hide border
      if (
        parentNode.getId() === LAYOUT_TAB_SETS.CENTER &&
        parentNode.getChildren().length === 1 &&
        parentNode.getChildren()[0].getId() !== LAYOUT_TABS.NO_RUNNING_DAEMONS
      ) {
        model.doAction(
          Actions.addTab(structuredClone(LAYOUT_NO_RUNNING_DAEMONS), LAYOUT_TAB_SETS.CENTER, DockLocation.CENTER, 0)
        );
      }

      // inform domain flex layout to re-render the content to avoid a delay before the content becomes visible
      if (parentNode && parentNode.getType() === "tabset") {
        const selectedNode = (parentNode as TabSetNode).getSelectedNode();
        if (selectedNode) {
          console.log(`After delete, emit select for: ${selectedNode.getId()}`);
          emitSelectTab({ tabId: selectedNode.getId(), forSubLayoutOnly: true });
        }
      }

      // delete tab
      model.doAction(Actions.deleteTab(tabId));
      // Cleanup React node reference
      delete layoutComponentsRef.current[tabId];
    },
    [model, monacoCtx]
  );

  useCustomEventListener(
    EVENT_OPEN_COMPONENT,
    (data: TEventOpenComponent) => {
      console.log(`open component: ${data.id}`);
      // tabs with insideDomainLayout are handled by DomainFlexLayout
      if (data.config?.insideDomainLayout) return;
      const node = modelRef.current.getNodeById(data.id);
      if (node) {
        if (node.getParent()?.getType() === "border") {
          const selectedNode = (node.getParent() as BorderNode)?.getSelectedNode();
          if (selectedNode?.getId() === node.getId()) {
            // already selected -> nothing to do
          } else if (selectedNode?.getId() === LAYOUT_TABS.HOSTS) {
            // HOSTS tab is selected -> keep it
          } else if (node.getId() === LAYOUT_TABS.LOGGING) {
            // activate logging tab if not visible
            if (!selectedNode?.isVisible()) {
              modelRef.current.doAction(Actions.selectTab(data.id));
            }
          } else if (selectedNode?.isVisible()) {
            // activate existing tab if border is visible
            modelRef.current.doAction(Actions.selectTab(data.id));
          } else {
            modelRef.current.doAction(Actions.selectTab(data.id));
          }
        } else if (!node.getId().startsWith(`${LAYOUT_TABS.DOMAIN}-`)) {
          // normal tab: just select it
          modelRef.current.doAction(Actions.selectTab(data.id));
        }
        if (data.toNodeId === LAYOUT_TAB_SETS.CENTER && data.id.startsWith(LAYOUT_TABS.DOMAIN)) {
          // hide info tab if domain tab was added
          deleteTab(LAYOUT_TABS.NO_RUNNING_DAEMONS);
        }
      } else {
        console.log(` -> create component: ${data.component} with id: ${data.id}`);
        // create a new tab
        const tab: ITabAttributesExt = {
          id: data.id,
          type: "tab",
          name: data.title,
          component: data.component,
          toNodeId: data.toNodeId,
          enableClose: data.closable,
          enablePopout: false,
          config: data.config,
        };
        // store react node and return it in factory()
        if (data.config?.reactNode) {
          layoutComponentsRef.current[data.id] = data.config.reactNode;
        }
        // store tab in state; will be added in a later effect
        setAddToLayout((prev) => [tab, ...prev]);
      }
    },
    []
  );

  useCustomEventListener(EVENT_SELECT_TAB, (data: TEventSelectTab) => {
    if (!data.forSubLayoutOnly) {
      model?.doAction(Actions.selectTab(data.tabId));
    }
  });

  /** Close tabs on signals from the tab itself (e.g. ctrl+d) */
  useCustomEventListener(
    EVENT_CLOSE_COMPONENT,
    (data: TEventId) => {
      deleteTab(data.id, true);
    },
    [deleteTab]
  );

  useCustomEventListener(
    EVENT_TOGGLE_COMPONENT,
    (data: TEventOpenComponent) => {
      if (data.config?.insideDomainLayout) {
        return;
      }
      console.log(`toggle component: ${data.component} with id: ${data.id}`);
      const tab = modelRef.current.getNodeById(data.id);
      const createTab = tab === undefined;
      if (tab && !(tab as TabNode)?.isVisible()) {
        modelRef.current.doAction(Actions.selectTab(data.id));
        return;
      }
      deleteTab(data.id);
      if (createTab) {
        // create a new tab
        const tab: ITabAttributesExt = {
          id: data.id,
          type: "tab",
          name: data.title,
          component: data.component,
          toNodeId: data.toNodeId,
          enableClose: data.closable,
          enablePopout: false,
          config: data.config,
        };
        if (data.config?.reactNode) {
          layoutComponentsRef.current[data.id] = data.config.reactNode;
        }
        // store new tabs using useEffect so dockMove() can create panels if events comes to fast
        setAddToLayout((oldValue) => [tab, ...oldValue]);
      }
    },
    [deleteTab]
  );

  useCustomEventListener(
    EVENT_PROVIDER_AUTH_REQUEST,
    (data: EventProviderAuthRequest) => {
      setPasswordRequests((prev) => [
        ...prev,
        <PasswordDialog
          key={data.provider.id}
          provider={data.provider}
          connectConfig={data.connectConfig}
          launchConfig={data.launchConfig}
          onClose={(prov) => {
            setPasswordRequests((prevInner) => prevInner.filter((item) => prov.id !== (item as { key: string })?.key));
          }}
        />,
      ]);
    },
    []
  );

  /** Resolves the BorderNode at the given dock location via its conventional id. */
  function findBorderByLocation(model: Model, location: DockLocation): BorderNode | undefined {
    // Border nodes use fixed ids of the form "border_<location>"
    const node = model.getNodeById(`border_${location.getName()}`);
    return node instanceof BorderNode ? node : undefined;
  }

  function getPanelId(id: string, toNodeId: string): TPanelId {
    const result: TPanelId = {
      id: toNodeId,
      isBorder: false,
      location: DockLocation.CENTER,
    };

    switch (toNodeId) {
      case LAYOUT_TAB_SETS.CENTER:
        result.isBorder = false;
        break;
      case LAYOUT_TAB_SETS.BORDER_TOP:
        result.isBorder = true;
        result.location = DockLocation.TOP;
        break;
      case LAYOUT_TAB_SETS.BORDER_BOTTOM:
        result.isBorder = true;
        result.location = DockLocation.BOTTOM;
        break;
      case LAYOUT_TAB_SETS.BORDER_RIGHT:
        result.isBorder = true;
        result.location = DockLocation.RIGHT;
        break;
      default:
        result.isBorder = false;
        break;
    }

    if (result.isBorder) {
      result.id = findBorderByLocation(modelRef.current, result.location)?.getId() || id;
    } else {
      const nodeBId = modelRef.current.getNodeById(toNodeId);
      if (toNodeId === LAYOUT_TAB_SETS.CENTER && !nodeBId) {
        // no center panel group found, reset layout
        setLayoutJson(structuredClone(DEFAULT_LAYOUT));
      }
      if (nodeBId && LAYOUT_TAB_LIST.includes(nodeBId.getId())) {
        result.id = nodeBId.getParent()?.getId() || id;
      }
    }

    return result;
  }

  // Add tabs to layout after EVENT_OPEN_COMPONENT was received
  useEffect(() => {
    if (addToLayout.length > 0) {
      const newAddToLayout = [...addToLayout];
      const tab = newAddToLayout.pop();
      if (tab?.id) {
        const node = modelRef.current.getNodeById(tab.id);
        if (node) {
          return;
        }
        const panelId = getPanelId(tab.id || "", tab.toNodeId);

        // store current selected tab in CENTER
        const isDomainCenterTab = tab.component === LAYOUT_TABS.DOMAIN && panelId.id === LAYOUT_TAB_SETS.CENTER;
        let previouslySelectedTabId: string | undefined;
        if (isDomainCenterTab) {
          const ts = modelRef.current.getNodeById(LAYOUT_TAB_SETS.CENTER) as TabSetNode | undefined;
          if (ts) {
            // const children = ts.getChildren();
            previouslySelectedTabId = ts.getSelectedNode()?.getId();
            if (previouslySelectedTabId === LAYOUT_TABS.NO_RUNNING_DAEMONS) previouslySelectedTabId = undefined;
          }
        }

        console.log(`add tab: ${tab.id}, to panel: ${panelId.id}`);
        const action = Actions.addTab(tab, panelId.id, DockLocation.CENTER, -1);
        modelRef.current.doAction(action);

        if (panelId.isBorder) {
          // If any tab in same border is visible, selecting the new tab can hide it
          const border = findBorderByLocation(modelRef.current, panelId.location);

          const hasVisible = border?.getChildren().some((c) => (c as TabNode).isVisible());

          if (!hasVisible && border?.getChildren().length) {
            const editorId = border.getChildren().slice(-1)[0].getId();
            modelRef.current.doAction(Actions.selectTab(editorId));
          }
        }
        // select previously selected
        if (isDomainCenterTab && previouslySelectedTabId) {
          modelRef.current.doAction(Actions.selectTab(previouslySelectedTabId));
        }
        if (tab.toNodeId === LAYOUT_TAB_SETS.CENTER) {
          // hide info tab if domain tab was added
          deleteTab(LAYOUT_TABS.NO_RUNNING_DAEMONS);
        }
      }
      setAddToLayout((prev) => prev.filter((t) => t.id !== tab?.id));
    }
  }, [addToLayout]);

  function factory(node: TabNode, contentId?: TContentId): JSX.Element {
    const component = node.getComponent();
    const config: TLayoutTabConfig = node.getConfig();
    const custom = layoutComponentsRef.current[node.getId()];
    if (custom) {
      return custom as React.ReactElement;
    }
    const flexId = contentId?.domainId || contentId?.providerId;

    switch (component) {
      case LAYOUT_TABS.NODES:
        return <HostTreeViewPanel key={`nodes-panel-${flexId}`} contentId={contentId} />;
      case LAYOUT_TABS.HOSTS:
        return <ProviderPanel key="hosts-panel" />;
      case LAYOUT_TABS.PACKAGES:
        return <PackageExplorerPanel key="pkg-panel" />;
      case LAYOUT_TABS.DETAILS:
        return <DetailsPanel key="node-details-panel" />;
      case LAYOUT_TABS.LOGGING:
        return <LoggingPanel key="logging-panel" />;
      case LAYOUT_TABS.TOPICS:
        return <TopicsPanel key={`topics-panel-${flexId}`} contentId={contentId} />;
      case LAYOUT_TABS.SERVICES:
        return <ServicesPanel key={`services-panel-${flexId}`} contentId={contentId} />;
      case LAYOUT_TABS.ACTIONS:
        return <ActionsPanel key={`actions-panel-${flexId}`} contentId={contentId} />;
      case LAYOUT_TABS.SETTINGS:
        return <SettingsPanel key="settings-panel" />;
      case LAYOUT_TABS.EDITOR: {
        if (!config.editorConfig) {
          return <Typography>Invalid editor configuration {JSON.stringify(config.editorConfig)}</Typography>;
        }
        const prov = rosCtx.getProviderById(config.editorConfig.providerId);
        if (prov)
          return (
            <FileEditorPanel
              key={config.editorConfig.id}
              editorId={config.editorConfig.id}
              provider={prov}
              currentFilePath={config.editorConfig.path}
              rootFilePath={config.editorConfig.rootLaunch}
              fileRange={config.editorConfig.fileRange}
              launchArgs={config.editorConfig.launchArgs}
              topLevelLaunchArgs={config.editorConfig.topLevelLaunchArgs}
              selectParameter={config.editorConfig.selectParameter}
            />
          );
        return <Typography>Provider with ID {config.editorConfig.providerId} not found</Typography>;
      }
      case LAYOUT_TABS.TERMINAL: {
        if (!config.terminalConfig) {
          return <Typography>Invalid terminal configuration {JSON.stringify(config.terminalConfig)}</Typography>;
        }
        const prov = rosCtx.getProviderById(config.terminalConfig.providerId);
        if (prov)
          return (
            <SingleTerminalPanel
              key={config.terminalConfig.id}
              id={config.terminalConfig.id}
              type={config.terminalConfig.cmdType}
              provider={prov}
              nodeName={config.terminalConfig.node}
              screen={config.terminalConfig.screen}
              cmd={config.terminalConfig.cmd}
              env={config.terminalConfig.env}
            />
          );
        return <Typography>Provider with ID {config.terminalConfig.providerId} not found</Typography>;
      }
      case LAYOUT_TABS.TOPIC_ECHO: {
        if (!config.subscriberConfig) {
          return <Typography>Invalid subscriber configuration {JSON.stringify(config.subscriberConfig)}</Typography>;
        }
        const prov = rosCtx.getProviderById(config.subscriberConfig.providerId);
        if (prov)
          return (
            <TopicEchoPanel
              key={config.subscriberConfig.id}
              provider={prov}
              showOptions={config.subscriberConfig.showOptions}
              defaultTopic={config.subscriberConfig.topic}
              defaultNoData={config.subscriberConfig.noData}
            />
          );
        return <Typography>Provider with ID {config.subscriberConfig.providerId} not found</Typography>;
      }
      case LAYOUT_TABS.TOPIC_PUBLISHER: {
        if (!config.publisherConfig) {
          return <Typography>Invalid publisher configuration {JSON.stringify(config.publisherConfig)}</Typography>;
        }
        return (
          <TopicPublishPanel
            key={config.publisherConfig.id}
            providerId={config.publisherConfig.providerId}
            topicName={config.publisherConfig.topicName}
            topicType={config.publisherConfig.topicType}
          />
        );
      }
      case LAYOUT_TABS.SERVICE_CALLER: {
        if (!config.serviceCallerConfig) {
          return (
            <Typography>Invalid service caller configuration {JSON.stringify(config.serviceCallerConfig)}</Typography>
          );
        }
        return (
          <ServiceCallerPanel
            key={config.serviceCallerConfig.id}
            providerId={config.serviceCallerConfig.providerId}
            serviceName={config.serviceCallerConfig.serviceName}
            serviceType={config.serviceCallerConfig.serviceType}
          />
        );
      }
      case LAYOUT_TABS.SERVICE_INTROSPECTION: {
        if (!config.serviceIntrospectionConfig) {
          return (
            <Typography>
              Invalid service introspection configuration {JSON.stringify(config.serviceIntrospectionConfig)}
            </Typography>
          );
        }
        return (
          <ServiceIntrospectionPanel
            key={config.serviceIntrospectionConfig.id}
            providerId={config.serviceIntrospectionConfig.providerId}
            serviceName={config.serviceIntrospectionConfig.serviceName}
            serviceType={config.serviceIntrospectionConfig.serviceType}
          />
        );
      }
      case LAYOUT_TABS.ACTION_SEND_GOAL: {
        if (!config.actionConfig) {
          return <Typography>Invalid action configuration {JSON.stringify(config.actionConfig)}</Typography>;
        }
        return (
          <ActionPanel
            key={config.actionConfig.id}
            showOptions={true}
            providerId={config.actionConfig.providerId}
            actionName={config.actionConfig.serviceName}
            actionType={config.actionConfig.serviceType}
          />
        );
      }
      case LAYOUT_TABS.ACTION_INTROSPECTION: {
        if (!config.actionIntrospectionConfig) {
          return (
            <Typography>
              Invalid action introspection configuration {JSON.stringify(config.actionIntrospectionConfig)}
            </Typography>
          );
        }
        return (
          <ActionIntrospectionPanel
            key={config.actionIntrospectionConfig.id}
            providerId={config.actionIntrospectionConfig.providerId}
            actionName={config.actionIntrospectionConfig.serviceName}
            actionType={config.actionIntrospectionConfig.serviceType}
          />
        );
      }
      case LAYOUT_TABS.NODE_LOGGER:
        if (!config.nodeLoggerConfig) {
          return <Typography>Invalid node logger configuration {JSON.stringify(config.nodeLoggerConfig)}</Typography>;
        }
        return <NodeLoggerPanel key={config.nodeLoggerConfig.id} node={config.nodeLoggerConfig.node} />;
      case LAYOUT_TABS.ABOUT:
        return <AboutPanel key="about-panel" />;
      case LAYOUT_TABS.PARAMETER:
        return (
          <ParameterPanel
            key="parameter-panel"
            nodes={config.parameterConfig?.nodes || []}
            providers={config.parameterConfig?.providers || []}
          />
        );
      case LAYOUT_TABS.APPS:
        return <ExternalAppsPanel key={`apps-panel-${flexId}`} contentId={contentId} />;

      case LAYOUT_TABS.PROVIDER_LAUNCH_CONTROL:
        if (!config.providerLaunchConfig) {
          return (
            <Typography>Invalid provider launch configuration {JSON.stringify(config.providerLaunchConfig)}</Typography>
          );
        }
        return (
          <ProviderLaunchConfigPanel key={config.providerLaunchConfig.id} config={config.providerLaunchConfig.config} />
        );
      case LAYOUT_TABS.NO_RUNNING_DAEMONS:
        return <InfoNoRunningDaemons key="info-no-running-daemons" />;
      case LAYOUT_TABS.DOMAIN:
        if (config?.contentId === undefined) {
          return <InfoNoRunningDaemons key="info-no-running-daemons" />;
        }
        return (
          <DomainFlexLayout
            key={`domain-flex-layout-${contentToId(config.contentId)}`}
            storageKey="layout-domain"
            contentId={config.contentId}
            insideTabId={node.getId()}
            factory={(node, contentId) => {
              return factory(node, contentId);
            }}
            onRenderTab={(node, renderValues) => {
              onRenderTab(node, renderValues);
            }}
            onCloseTab={(id: string) => deleteTab(id)}
          />
        );
      default:
        return <Typography>unknown component: {component}</Typography>;
    }
  }

  async function openExternalTerminal(config: TExtTerminalConfig, tabNodeId: string): Promise<void> {
    // create terminal command
    const provider = rosCtx.getProviderById(config.providerId);
    if (!provider) return;

    const terminalCmd = await provider.cmdForType(
      config.type,
      config.nodeName,
      config.topicName,
      config.screen,
      config.cmd,
      config.env
    );

    if (!isElectron()) {
      openBrowserSite("terminal", tabNodeId, terminalCmd, openAsPopout);
    } else {
      try {
        window.commandExecutor?.execTerminal(
          provider.isLocalHost ? null : { host: provider.host() },
          `"${config.type} ${config.nodeName}@${provider.host()}"`,
          terminalCmd.cmd
        );
        deleteTab(tabNodeId);
      } catch (error) {
        logCtx.error(
          `Can't open external terminal for ${config.nodeName}`,
          JSON.stringify(error),
          "not external terminal"
        );
      }
    }
  }

  function onRenderTab(node: TabNode, renderValues: ITabRenderValues): void {
    const renderNameValues = renderValues as ITabRenderValues & { name: string };
    // add tooltip to abbreviations
    if (
      ![
        "Hosts",
        "Details",
        "Packages",
        "Nodes",
        "Topics",
        "Services",
        "Parameter",
        "Logging",
        "Settings",
        "About",
      ].includes(renderNameValues.name)
    ) {
      renderNameValues.content = (
        <Tooltip title={renderNameValues.name} placement="bottom" disableInteractive>
          <Typography>{tabFullName ? renderNameValues.name : basename(renderNameValues.name)}</Typography>
        </Tooltip>
      );
    }

    switch (node.getId()) {
      case LAYOUT_TABS.LOGGING:
        renderNameValues.content = "";
        renderNameValues.leading = (
          <Tooltip title="Logging (mas gui)" placement="top" disableInteractive>
            <Badge
              color="info"
              badgeContent={`${logCtx.countErrors}`}
              invisible
              sx={{
                "& .MuiBadge-badge": {
                  fontSize: "inherit",
                  height: 12,
                  minWidth: 12,
                },
              }}
            >
              <DesktopWindowsOutlinedIcon sx={{ fontSize: "inherit" }} />
            </Badge>
          </Tooltip>
        );
        renderNameValues.name = "Option";
        break;
      default:
        // add leading icons depending on tab type
        switch (node.getComponent()) {
          case LAYOUT_TABS.SETTINGS:
            renderNameValues.leading = <SettingsIcon sx={{ fontSize: (theme) => theme.typography.fontSize }} />;
            break;
          case LAYOUT_TABS.TERMINAL:
            switch (node.getConfig()?.terminalType) {
              case CmdTypes.LOG:
                renderNameValues.leading = <WysiwygIcon sx={{ fontSize: (theme) => theme.typography.fontSize }} />;
                break;
              case CmdTypes.SCREEN:
                renderNameValues.leading = <DvrIcon sx={{ fontSize: (theme) => theme.typography.fontSize }} />;
                break;
              case CmdTypes.TERMINAL:
                renderNameValues.leading = <TerminalIcon sx={{ fontSize: (theme) => theme.typography.fontSize }} />;
                break;
            }
            break;
          case LAYOUT_TABS.TOPIC_ECHO:
            renderNameValues.leading = (
              <ChatBubbleOutlineIcon sx={{ fontSize: (theme) => theme.typography.fontSize }} />
            );
            break;
          case LAYOUT_TABS.TOPIC_PUBLISHER:
            renderNameValues.leading = (
              <PlayCircleOutlineIcon sx={{ fontSize: (theme) => theme.typography.fontSize }} />
            );
            break;
          case LAYOUT_TABS.ACTION_SEND_GOAL:
            renderNameValues.leading = <StartIcon sx={{ fontSize: (theme) => theme.typography.fontSize }} />;
            break;
          case LAYOUT_TABS.SERVICE_CALLER:
            renderNameValues.leading = <SyncAltOutlinedIcon sx={{ fontSize: (theme) => theme.typography.fontSize }} />;
            break;
          case LAYOUT_TABS.SERVICE_INTROSPECTION:
          case LAYOUT_TABS.ACTION_INTROSPECTION:
            renderNameValues.leading = <TroubleshootIcon sx={{ fontSize: (theme) => theme.typography.fontSize }} />;
            break;
          case LAYOUT_TABS.ABOUT:
            renderNameValues.leading = <InfoOutlinedIcon sx={{ fontSize: (theme) => theme.typography.fontSize }} />;
            break;
          case LAYOUT_TABS.PARAMETER:
            renderNameValues.leading = <TuneIcon sx={{ fontSize: (theme) => theme.typography.fontSize }} />;
            break;
          case LAYOUT_TABS.EDITOR:
            renderNameValues.leading = <BorderColorIcon sx={{ fontSize: (theme) => theme.typography.fontSize }} />;
            break;
          case LAYOUT_TABS.NODE_LOGGER:
            renderNameValues.leading = (
              <SettingsInputCompositeOutlinedIcon
                sx={{ fontSize: (theme) => theme.typography.fontSize, rotate: "90deg" }}
              />
            );
            break;
          case LAYOUT_TABS.TOPICS:
            renderNameValues.leading = <TopicIcon sx={{ fontSize: (theme) => theme.typography.fontSize }} />;
            break;
          case LAYOUT_TABS.SERVICES:
            renderNameValues.leading = <FeaturedPlayListIcon sx={{ fontSize: (theme) => theme.typography.fontSize }} />;
            break;
          case LAYOUT_TABS.ACTIONS:
            renderNameValues.leading = <AccountTreeIcon sx={{ fontSize: (theme) => theme.typography.fontSize }} />;
            break;
          case LAYOUT_TABS.APPS:
            renderNameValues.leading = <AppsIcon sx={{ fontSize: (theme) => theme.typography.fontSize }} />;
            break;
          default:
            break;
        }

        // add "open externally" button if supported
        if (node.getConfig()?.openExternal) {
          renderNameValues.buttons.push(
            <Tooltip
              key={`button-close-${node.getId()}`}
              title="Open in external window"
              placement="bottom"
              disableInteractive
            >
              <IconButton
                sx={{ padding: "1px" }}
                onMouseDown={(event) => {
                  if (event?.button === 1) return;

                  const cfg: TLayoutTabConfig = node.getConfig();

                  if (cfg.extTerminalConfig) {
                    openExternalTerminal(cfg.extTerminalConfig, node.getId());
                  }
                  if (cfg.editorConfig) {
                    if (!isElectron()) {
                      openBrowserSite("editor", node.getId(), cfg.editorConfig, openAsPopout);
                    } else {
                      window.editorManager?.open(cfg.editorConfig);
                    }
                    deleteTab(node.getId());
                  }
                  if (cfg.publisherConfig) {
                    if (!isElectron()) {
                      openBrowserSite("publisher", node.getId(), cfg.publisherConfig, openAsPopout);
                    } else {
                      window.publishManager?.start(cfg.publisherConfig);
                    }
                    deleteTab(node.getId());
                  }
                  if (cfg.subscriberConfig) {
                    if (!isElectron()) {
                      openBrowserSite("publisher", node.getId(), cfg.subscriberConfig, openAsPopout);
                    } else {
                      window.subscriberManager?.open(cfg.subscriberConfig);
                    }
                    deleteTab(node.getId());
                  }
                  if (cfg.serviceCallerConfig) {
                    if (!isElectron()) {
                      openBrowserSite(
                        cfg.serviceCallerConfig.htmlName,
                        node.getId(),
                        cfg.serviceCallerConfig,
                        openAsPopout
                      );
                    } else {
                      window.serviceManager?.start(cfg.serviceCallerConfig);
                    }
                    deleteTab(node.getId());
                  }
                  if (cfg.serviceIntrospectionConfig) {
                    if (!isElectron()) {
                      openBrowserSite(
                        cfg.serviceIntrospectionConfig.htmlName,
                        node.getId(),
                        cfg.serviceIntrospectionConfig,
                        openAsPopout
                      );
                    } else {
                      window.serviceManager?.start(cfg.serviceIntrospectionConfig);
                    }
                    deleteTab(node.getId());
                  }
                  if (cfg.actionConfig) {
                    if (!isElectron()) {
                      openBrowserSite(cfg.actionConfig.htmlName, node.getId(), cfg.actionConfig, openAsPopout);
                    } else {
                      window.serviceManager?.start(cfg.actionConfig);
                    }
                    deleteTab(node.getId());
                  }
                  if (cfg.actionIntrospectionConfig) {
                    if (!isElectron()) {
                      openBrowserSite(
                        cfg.actionIntrospectionConfig.htmlName,
                        node.getId(),
                        cfg.actionIntrospectionConfig,
                        openAsPopout
                      );
                    } else {
                      window.serviceManager?.start(cfg.actionIntrospectionConfig);
                    }
                    deleteTab(node.getId());
                  }
                  if (cfg.terminalConfig) {
                    if (!isElectron()) {
                      openBrowserSite("terminal", node.getId(), cfg.terminalConfig, openAsPopout);
                    } else {
                      window.terminalManager?.open(cfg.terminalConfig);
                    }
                    deleteTab(node.getId());
                  }

                  event.stopPropagation();
                }}
              >
                <LaunchIcon sx={{ fontSize: (theme) => theme.typography.fontSize }} />
              </IconButton>
            </Tooltip>
          );
        }
        break;
    }
  }

  function onRenderTabSet(node: TabSetNode | BorderNode, renderValues: ITabSetRenderValues): void {
    if (node.getId() === LAYOUT_TAB_SETS.CENTER) {
      renderValues.leading =
        dedicatedTabsFor === "HOSTS" ? (
          <Tooltip
            key="tooltip-log"
            title="Use a dedicated tab for each host. Click to switch to domains."
            disableInteractive
          >
            <IconButton
              sx={{
                padding: "0em",
                color: useDarkMode ? "#fff" : "rgba(0, 0, 0, 0.54)",
              }}
              onClick={() => {
                setDedicatedTabsFor("DOMAINS");
              }}
            >
              <DesktopWindowsIcon sx={{ fontSize: "inherit" }} />
            </IconButton>
          </Tooltip>
        ) : (
          <Tooltip
            key="tooltip-log"
            title="Use a dedicated tab for each domain. Click to switch to hosts."
            disableInteractive
          >
            <IconButton
              sx={{
                padding: "0em",
                color: useDarkMode ? "#fff" : "rgba(0, 0, 0, 0.54)",
              }}
              onClick={() => {
                setDedicatedTabsFor("HOSTS");
              }}
            >
              <DomainIcon sx={{ fontSize: "inherit" }} />
            </IconButton>
          </Tooltip>
        );
    }

    // Workaround: When tabSetEnableSingleTabStretch is enabled, the close button
    // of a single closable tab is hidden. This manually adds a close button to
    // the tabset toolbar to restore that functionality.
    if (node.getId() === LAYOUT_TAB_SETS.CENTER && node.getChildren().length === 1) {
      const tab = node.getChildren()[0] as TabNode;
      if (tab.isEnableClose()) {
        renderValues.buttons.push(
          <IconButton
            key="close"
            size="small"
            // className="flexlayout__tab_button_trailing"
            onClick={() => deleteTab(tab.getId())}
          >
            <CloseIcon fontSize="inherit" />
          </IconButton>
        );
      }
    }

    if (node.getId() === LAYOUT_TAB_SETS.BORDER_BOTTOM) {
      if (currentInfoState) {
        renderValues.buttons.push(
          <Tooltip key="tooltip-log" title="" disableInteractive>
            <Typography style={{ color: getInfoStateColor(currentInfoState.level, useDarkMode) }}>
              {currentInfoState.message}
            </Typography>
          </Tooltip>
        );
      }

      // add settings tab button in bottom border
      pAddTabStickyButton({
        model: model,
        container: renderValues.buttons,
        id: LAYOUT_TABS.SETTINGS,
        title: "Settings",
        component: LAYOUT_TABS.SETTINGS,
        setId: LAYOUT_TAB_SETS.CENTER,
        icon: <SettingsIcon sx={{ fontSize: "inherit" }} />,
        force: true,
      });

      // add about tab button in bottom border
      pAddTabStickyButton({
        model: model,
        container: renderValues.buttons,
        id: LAYOUT_TABS.ABOUT,
        title: "About",
        component: LAYOUT_TABS.ABOUT,
        setId: LAYOUT_TAB_SETS.CENTER,
        icon: <InfoOutlinedIcon sx={{ fontSize: "inherit" }} />,
        force: true,
      });

      // add update button in bottom border
      if (auCtx.updateAvailable) {
        renderValues.buttons.push(
          <Tooltip
            key="update-available"
            title={`new version ${auCtx.updateAvailable.version} available`}
            placement="top"
          >
            <Button
              style={{ textTransform: "none" }}
              onClick={() => {
                emitToggleComponent({
                  id: LAYOUT_TABS.ABOUT,
                  title: "About",
                  component: LAYOUT_TABS.ABOUT,
                  closable: true,
                  toNodeId: LAYOUT_TAB_SETS.CENTER,
                });
              }}
              variant="text"
              color="info"
              size="small"
            >
              <Typography noWrap variant="body2">
                update available
              </Typography>
            </Button>
          </Tooltip>
        );
      }
    }
  }

  function removeGenericTabs(parent: { children?: IJsonRowNode[]; selected?: number }): IJsonRowNode {
    if (!parent.children) return parent;

    // if tabs are removed, selection index may become invalid
    if (parent.selected !== undefined) {
      parent.selected = undefined;
    }

    parent.children = parent.children.filter(
      (item: { children?: IJsonRowNode[]; selected?: number; type?: string; id?: string }) => {
        // do not store Settings, About and Parameter tabs
        if (
          item.type === "tab" &&
          item.id !== LAYOUT_TABS.ABOUT &&
          item.id !== LAYOUT_TABS.SETTINGS &&
          item.id !== LAYOUT_TABS.PARAMETER &&
          item.id !== LAYOUT_TABS.NODES &&
          LAYOUT_TAB_LIST.includes(item.id)
          // (LAYOUT_TAB_LIST.includes(item.id) || item.id?.startsWith(LAYOUT_TABS.DOMAIN))
        ) {
          return true;
        }
        if (item.children) {
          removeGenericTabs(item);
          if (item.id === LAYOUT_TAB_SETS.CENTER && (item.children?.length || 0) === 0) {
            item.children?.push(structuredClone(LAYOUT_NO_RUNNING_DAEMONS));
          }
          return true;
        }
        return false;
      }
    );

    return parent;
  }

  /** Recursively checks whether a tabset with the given id exists anywhere in the tree */
  function hasTabSetWithId(node: { id?: string; type?: string; children?: IJsonRowNode[] }, id: string): boolean {
    if (node.type === "tabset" && node.id === id) return true;
    if (!node.children) return false;
    return node.children.some((child) => hasTabSetWithId(child, id));
  }

  /** Remove generic tabs */
  const cleanAndSaveLayout = useDebounceCallback(() => {
    const modelJson = modelRef.current.toJson();

    for (const item of modelJson.borders || []) {
      item.selected = -1;
      removeGenericTabs(item);
    }

    modelJson.layout = removeGenericTabs(modelJson.layout);

    // search recursively instead of only flat
    const foundDomainSet = hasTabSetWithId(modelJson.layout, LAYOUT_TAB_SETS.CENTER);

    if (!foundDomainSet) {
      modelJson.layout.children?.push(structuredClone(LAYOUT_DOMAIN_TAB_SET));
    }

    setLayoutJson(modelJson);
  }, 500);

  /** Removes/repairs duplicate ids in the layout JSON, keeping the first "main" */
  function sanitizeLayout(json: IJsonModel): IJsonModel {
    // deep clone first, so shared object references can no longer collide
    const cloned: IJsonModel = structuredClone(json);
    const seen = new Set<string>();

    const walk = (node: { id?: string; type?: string; children?: IJsonRowNode[] }): void => {
      if (node.id) {
        if (seen.has(node.id)) {
          // duplicate: assign a new unique id
          node.id = `#dup-${crypto.randomUUID?.() ?? Math.random().toString(36).slice(2)}`;
          seen.add(node.id); // register the new id as well, to be safe
        } else {
          seen.add(node.id);
        }
      }
      if (node.children) {
        for (const child of node.children) {
          walk(child);
        }
      }
    };

    for (const border of cloned.borders || []) {
      walk(border);
    }
    walk(cloned.layout);
    return cloned;
  }

  const isInstallUpdateRequested = useCallback(() => {
    return auCtx.requestedInstallUpdate;
  }, [auCtx.requestedInstallUpdate]);

  useEffect(() => {
    // do not ask for shutdown in some situations
    if (electronCtx.terminateSubprocesses) {
      if (isInstallUpdateRequested()) {
        electronCtx.shutdownManager?.quitGui();
      }
      if (rosCtx.providers.length <= 0) {
        electronCtx.shutdownManager?.quitGui();
      }
      const dirtyModels = monacoCtx.dirtyManager()?.getDirtyModels();
      if (!dirtyModels) return;
      setDirtyTabs(monacoCtx.modelRegistry()?.getEditorsByModels(dirtyModels) || []);
    }
  }, [
    electronCtx.shutdownManager,
    electronCtx.terminateSubprocesses,
    isInstallUpdateRequested,
    monacoCtx,
    rosCtx.providers.length,
  ]);

  const shutdownProviders = useCallback(
    async (providers: Provider[]) => {
      if (providers && providers.length > 0) {
        await Promise.all(
          providers.map(async (prov) => {
            console.log(`shutdown ${prov.id}`);
            const result = await prov.shutdown(true, ignoreProcessesOnShutdown.split(","));
            console.log(`finished shutdown ${prov.id} ${JSON.stringify(result)}`);
          })
        );
      }
      console.log("Quit app");
      electronCtx.shutdownManager?.quitGui();
    },
    [electronCtx]
  );

  function onKeyDown(event: React.KeyboardEvent): void {
    if (event.ctrlKey && event.key === "+") {
      setFontSize(fontSize + 1);
    }
    if (event.ctrlKey && event.key === "-") {
      setFontSize(fontSize - 1);
    }
    if (event.ctrlKey && event.key === "0") {
      setFontSize(14);
    }
  }

  const saveAllDirty = useCallback(async () => {
    // save all modified files
    const editorModels = monacoCtx.modelRegistry()?.getByEditorIds(dirtyTabs) || [];
    const dirtyModels = monacoCtx.dirtyManager()?.reduceToDirty(Array.from(editorModels)) || [];
    const results: SaveResult[] = await Promise.all(dirtyModels.map((model) => monacoCtx.saveFile(model)));

    const allTabs = new Set<string>();
    const failedTabs = new Set<string>();

    // collect all tabs and track which ones failed to save
    for (const { editorIds = [], result } of results) {
      for (const editorId of editorIds) {
        allTabs.add(editorId);
        if (!result) {
          failedTabs.add(editorId);
        }
      }
    }

    // close tabs that were successfully saved
    for (const editorId of allTabs) {
      if (!failedTabs.has(editorId)) {
        modelRef.current.doAction(Actions.deleteTab(editorId));
      }
    }

    // cancel app close if any tab failed to save
    if (failedTabs.size > 0) {
      electronCtx.cancelCloseApp();
    }

    setDirtyTabs([]);
  }, [dirtyTabs, monacoCtx, electronCtx]);

  return (
    <Stack
      onKeyDown={onKeyDown}
      tabIndex={0} // required for onKeyDown
      style={{
        position: "absolute",
        left: 2,
        top: 2,
        right: 2,
        bottom: 2,
      }}
    >
      <Layout
        key="node-manager-layout"
        ref={layoutRef}
        model={model}
        factory={factory}
        onAction={(action: Action) => {
          // hide bottom panel when last terminal is closed
          if (action.type === Actions.DELETE_TAB) {
            deleteTab(action.data.node);
            return undefined;
          }
          if (action.type === Actions.SELECT_TAB) {
            const tabId = action.data.tabNode as string;
            emitSelectTab({ tabId: tabId, forSubLayoutOnly: true });
          }
          return action;
        }}
        onRenderTab={onRenderTab}
        onRenderTabSet={onRenderTabSet}
        onModelChange={(_model, _action) => {
          if (![Actions.SELECT_TAB, Actions.SET_ACTIVE_TABSET].includes(_action.type)) {
            cleanAndSaveLayout();
          }
        }}
        onContextMenu={(node) => {
          console.log(`NO context for ${node.getId()}`);
        }}
        onAuxMouseClick={(node, event) => {
          // close tabs with middle mouse click
          if (event?.button === 1 && node.getType() === "tab" && (node as TabSetNode | TabNode).isEnableClose()) {
            deleteTab(node.getId());
          }
        }}
      />

      {electronCtx.terminateSubprocesses && dirtyTabs.length === 0 && rosCtx.providers.length > 0 && (
        // ask for provider shutdown before quitting GUI
        <ProviderSelectionModal
          title="Select providers to shut down"
          providers={rosCtx.providers}
          onCloseCallback={() => {
            electronCtx.cancelCloseApp();
          }}
          onConfirmCallback={(providers) => {
            shutdownProviders(providers);
          }}
          onForceCloseCallback={() => electronCtx.shutdownManager?.quitGui()}
          onToggle={() => electronCtx.cancelCloseTimer()}
        />
      )}

      {dirtyTabs.length > 0 && (
        <Dialog
          open={dirtyTabs.length > 0}
          onClose={() => {
            setDirtyTabs([]);
            electronCtx.cancelCloseApp();
          }}
          onAbort={() => {
            setDirtyTabs([]);
            electronCtx.cancelCloseApp();
          }}
          onFocus={() => {
            electronCtx.cancelCloseTimer();
          }}
          fullWidth
          scroll="paper"
          maxWidth="sm"
          PaperComponent={DraggablePaper}
          aria-labelledby="draggable-dialog-title"
        >
          <DialogTitle className="draggable-dialog-title" style={{ cursor: "move" }} id="draggable-dialog-title">
            Changed Files
          </DialogTitle>

          <DialogContent aria-label="list">
            {dirtyTabs.map((editorId) => {
              const editorModels = monacoCtx.modelRegistry()?.getByEditorIds([editorId]) || [];
              const dirtyModels = monacoCtx.dirtyManager()?.reduceToDirty(Array.from(editorModels)) || [];
              const files = dirtyModels.map((m) => getFileName(m.uri.path));
              return (
                <DialogContentText key={editorId} id="alert-dialog-description">
                  {`Modified files in "${getBaseName(editorId)}" tab: ${files}`}
                </DialogContentText>
              );
            })}
          </DialogContent>

          <DialogActions>
            <Button
              color="warning"
              onClick={() => {
                for (const editorId of dirtyTabs) {
                  model.doAction(Actions.deleteTab(editorId));
                }
                setDirtyTabs([]);
              }}
            >
              Don&apos;t save
            </Button>
            <Button
              color="primary"
              onClick={() => {
                setDirtyTabs([]);
                electronCtx.cancelCloseApp();
              }}
            >
              Cancel
            </Button>
            <Button
              autoFocus
              color="primary"
              onClick={() => {
                saveAllDirty();
              }}
            >
              Save all
            </Button>
          </DialogActions>
        </Dialog>
      )}

      {passwordRequests.map((item) => item)}

      {currentInfoState?.level === InfoStateLevel.ERROR && (
        <div
          style={{
            position: "fixed",
            inset: 0,
            display: "flex",
            alignItems: "center",
            justifyContent: "center",
            pointerEvents: "none",
            zIndex: 9999,
          }}
        >
          <div
            style={{
              pointerEvents: "auto",
              padding: "8px 14px",
              borderRadius: 8,
              background: "rgba(0, 0, 0, 0.75)",
              color: "#fff",
              fontSize: 13,
              maxWidth: 400,
              textAlign: "center",
              boxShadow: "0 4px 12px rgba(0,0,0,0.3)",
            }}
          >
            {currentInfoState?.message}
          </div>
        </div>
      )}
    </Stack>
  );
}
