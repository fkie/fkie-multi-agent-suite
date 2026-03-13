import BorderColorIcon from "@mui/icons-material/BorderColor";
import ChatBubbleOutlineIcon from "@mui/icons-material/ChatBubbleOutline";
import DesktopWindowsOutlinedIcon from "@mui/icons-material/DesktopWindowsOutlined";
import DvrIcon from "@mui/icons-material/Dvr";
import FeaturedPlayListIcon from "@mui/icons-material/FeaturedPlayList";
import InfoOutlinedIcon from "@mui/icons-material/InfoOutlined";
import LaunchIcon from "@mui/icons-material/Launch";
import PlayCircleOutlineIcon from "@mui/icons-material/PlayCircleOutline";
import SettingsIcon from "@mui/icons-material/Settings";
import SettingsInputCompositeOutlinedIcon from "@mui/icons-material/SettingsInputCompositeOutlined";
import SyncAltOutlinedIcon from "@mui/icons-material/SyncAltOutlined";
import TerminalIcon from "@mui/icons-material/Terminal";
import TopicIcon from "@mui/icons-material/Topic";
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
  IJsonBorderNode,
  IJsonModel,
  IJsonRowNode,
  IJsonTabSetNode,
  ITabAttributes,
  ITabSetRenderValues,
  Layout,
  Model,
  TabNode,
  TabSetNode,
} from "flexlayout-react";
import React, { useCallback, useContext, useEffect, useRef, useState } from "react";
import { emitCustomEvent, useCustomEventListener } from "react-custom-events";

import ExternalAppsModal from "@/renderer/components/ExternalAppsModal/ExternalAppsModal";
import PasswordDialog from "@/renderer/components/PasswordModal/PasswordDialog";
import ProviderSelectionModal from "@/renderer/components/SelectionModal/ProviderSelectionModal";
import { getInfoStateColor } from "@/renderer/components/UI/Colors";
import DraggablePaper from "@/renderer/components/UI/DraggablePaper";
import { useAutoUpdateContext } from "@/renderer/context/AutoUpdateContext";
import { ElectronContext } from "@/renderer/context/ElectronContext";
import useLocalStorage from "@/renderer/hooks/useLocalStorage";
import { useLoggingContext } from "@/renderer/hooks/useLoggingContext";
import { useMonacoInitContext } from "@/renderer/hooks/useMonacoInitContext";
import { useNavigationContext } from "@/renderer/hooks/useNavigationContext";
import { useRosContext } from "@/renderer/hooks/useRosContext";
import { useSettingsContext } from "@/renderer/hooks/useSettingsContext";
import { getBaseName, getFileName } from "@/renderer/models";
import { SaveResult } from "@/renderer/monaco/types";
import { isEditorEditorId } from "@/renderer/monaco/utils";
import { CmdType, Provider } from "@/renderer/providers";
import { EventProviderAuthRequest } from "@/renderer/providers/events";
import { EVENT_PROVIDER_AUTH_REQUEST } from "@/renderer/providers/eventTypes";
import { basename } from "@/renderer/utils";
import { TInfoState } from "@/types";

import { DEFAULT_LAYOUT, LAYOUT_TAB_LIST, LAYOUT_TAB_SETS, LAYOUT_TABS } from "./layout";
import {
  EVENT_CLOSE_COMPONENT,
  EVENT_INFO_STATE,
  EVENT_OPEN_COMPONENT,
  eventOpenComponent,
  TEventId,
  TEventInfoState,
  TEventOpenComponent,
} from "./layout/events";
import { IExtTerminalConfig } from "./layout/LayoutTabConfig";
import "./NodeManager.css";
import AboutPanel from "./panels/AboutPanel";
import DetailsPanel from "./panels/DetailsPanel";
import HostTreeViewPanel from "./panels/HostTreeViewPanel";
import LoggingPanel from "./panels/LoggingPanel";
// import OverflowMenuNodeDetails from "./panels/OverflowMenuNodeDetails";
import PackageExplorerPanel from "./panels/PackageExplorerPanel";
import ParameterPanel from "./panels/ParameterPanel";
import ProviderPanel from "./panels/ProviderPanel";
import ServicesPanel from "./panels/ServicesPanel";
import SettingsPanel from "./panels/SettingsPanel";
import TopicsPanel from "./panels/TopicsPanel";

type TPanelId = {
  id: string;
  isBorder: boolean;
  location: DockLocation;
};

interface ITabAttributesExt extends ITabAttributes {
  panelGroup: string;
}

export default function NodeManager(): JSX.Element {
  const auCtx = useAutoUpdateContext();
  const electronCtx = useContext(ElectronContext);
  const rosCtx = useRosContext();
  const logCtx = useLoggingContext();
  const monacoInitCtx = useMonacoInitContext();
  const monacoCtx = monacoInitCtx.monacoCtx;
  const navCtx = useNavigationContext();
  const settingsCtx = useSettingsContext();

  const [layoutJson, setLayoutJson] = useLocalStorage<IJsonModel>("layout", DEFAULT_LAYOUT);
  const [model, setModel] = useState<Model>(() => Model.fromJson(layoutJson));
  const layoutRef = useRef<Layout | null>(null);

  const [layoutComponents] = useState<Record<string, React.ReactNode>>({});
  const [addToLayout, setAddToLayout] = useState<ITabAttributesExt[]>([]);
  const [dirtyTabs, setDirtyTabs] = useState<string[]>([]);
  const [passwordRequests, setPasswordRequests] = useState<React.ReactNode[]>([]);

  const [tooltipDelay, setTooltipDelay] = useState<number>(settingsCtx.get("tooltipEnterDelay") as number);
  const [tabFullName, setTabFullName] = useState<boolean>(settingsCtx.get("tabFullName") as boolean);
  const [isDarkMode, setIsDarkMode] = useState<boolean>(settingsCtx.get("useDarkMode") as boolean);

  const [infoStates, setInfoStates] = useState<TInfoState[]>([]);
  const [infoStateTimer, setInfoStateTimer] = useState<NodeJS.Timeout | undefined>();
  const [currentInfoState, setCurrentInfoState] = useState<TInfoState | undefined>();

  const [enablePopout, setEnablePopout] = useState<boolean>(
    !window.commandExecutor && window.location.href.indexOf(":6275") === -1
  );

  // sync settings dependent state
  useEffect(() => {
    setTooltipDelay(settingsCtx.get("tooltipEnterDelay") as number);
    setTabFullName(settingsCtx.get("tabFullName") as boolean);
    setIsDarkMode(settingsCtx.get("useDarkMode") as boolean);
  }, [settingsCtx.changed]);

  // enable/disable popout depending on environment
  useEffect(() => {
    setEnablePopout(!window.commandExecutor && window.location.href.indexOf(":6275") === -1);
  }, []);

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

  const hasTab = useCallback((layout: any, editorId: string): boolean => {
    if (!layout.children) return false;
    const found = layout.children.filter((item: any) => {
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

  /** Disable float button if the GUI is not running in a browser */
  const updateFloatButton = useCallback(
    (layout: IJsonRowNode | IJsonBorderNode | IJsonTabSetNode): boolean => {
      if (!layout.children) return false;
      let result = false;

      // biome-ignore lint/complexity/noForEach: <explanation>
      layout.children.forEach((item) => {
        if (item.type === "tab") {
          if (item.enablePopout !== enablePopout) {
            item.enablePopout = enablePopout;
            result = true;
          }
          if (item.children) {
            if (updateFloatButton(item)) {
              result = true;
            }
          }
        }
      });
      return result;
    },
    [enablePopout]
  );

  useEffect(() => {
    navCtx.setLayoutModel(model);
  }, [model, navCtx]);

  useEffect(() => {
    // update float button for all tabs on load or when layoutJson changes
    let changed = updateFloatButton(layoutJson.layout);

    for (const border of layoutJson.borders || []) {
      if (updateFloatButton(border)) {
        changed = true;
      }
    }
    for (const layout of layoutJson.layout.children || []) {
      if (updateFloatButton(layout)) {
        changed = true;
      }
    }

    if (changed) {
      setLayoutJson(layoutJson);
      setModel(Model.fromJson(layoutJson));
    }
  }, [layoutJson]);

  useEffect(() => {
    const needsReset =
      settingsCtx.get("resetLayout") ||
      !hasTab(layoutJson.layout, LAYOUT_TABS.NODES) ||
      !hasTab(layoutJson.layout, LAYOUT_TABS.DETAILS);

    if (needsReset) {
      setLayoutJson(DEFAULT_LAYOUT);
      setModel(Model.fromJson(DEFAULT_LAYOUT));
      settingsCtx.set("resetLayout", false);
      logCtx.success("Layout reset!", "", "layout reset");
    }
  }, [settingsCtx.changed, layoutJson, hasTab, logCtx, setLayoutJson]);

  /** Hide bottom panel when last terminal is closed and handle editor tabs with unsaved changes */
  const deleteTab = useCallback(
    (editorId: string): void => {
      // handle editor tabs with modified files
      if (isEditorEditorId(editorId)) {
        const modified = monacoCtx.getModifiedFilesByEditor(editorId);
        if (modified.length > 0) {
          const editorTab = model.getNodeById(editorId);
          if (editorTab) {
            model.doAction(Actions.selectTab(editorTab.getId()));
            setDirtyTabs([editorId]);
            return;
          }
        } else {
          model.doAction(Actions.deleteTab(editorId));
        }
      }

      // handle tabs in bottom border
      const nodeBId = model.getNodeById(editorId);
      if (
        nodeBId &&
        nodeBId.getParent()?.getType() === "border" &&
        (nodeBId.getParent() as BorderNode)?.getLocation().getName() === DockLocation.BOTTOM.getName()
      ) {
        // if closing last visible bottom tab, select it first to hide border
        const shouldSelectNewTab =
          nodeBId.getParent()?.getChildren().length === 2 &&
          (nodeBId.getParent() as BorderNode)?.getSelectedNode()?.isVisible();
        if (shouldSelectNewTab) {
          model.doAction(Actions.selectTab(editorId));
        }
      } else {
        // select "Nodes" tab if it is in the same tabset as the closed tab
        const tabNode = model.getNodeById(editorId);
        if (tabNode) {
          for (const tab of tabNode.getParent()?.getChildren() || []) {
            if (tab.getId() === LAYOUT_TABS.NODES) {
              model.doAction(Actions.selectTab(tab.getId()));
            }
          }
        }
      }

      model.doAction(Actions.deleteTab(editorId));
    },
    [model, monacoCtx, setDirtyTabs]
  );

  useCustomEventListener(
    EVENT_OPEN_COMPONENT,
    (data: TEventOpenComponent) => {
      const node = model.getNodeById(data.id);
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
              model.doAction(Actions.selectTab(data.id));
            }
          } else if (selectedNode?.isVisible()) {
            // activate existing tab if border is visible
            model.doAction(Actions.selectTab(data.id));
          } else {
            model.doAction(Actions.selectTab(data.id));
          }
        } else {
          // normal tab: just select it
          model.doAction(Actions.selectTab(data.id));
        }
      } else {
        // create a new tab
        const tab: ITabAttributesExt = {
          id: data.id,
          type: "tab",
          name: data.title,
          component: data.id,
          panelGroup: data.panelGroup,
          enableClose: data.closable,
          enablePopout,
          config: data.config,
        };
        layoutComponents[data.id] = data.component;
        // store tab in state; will be added in a later effect
        setAddToLayout((prev) => [tab, ...prev]);
      }
    },
    [layoutComponents, model, enablePopout]
  );

  /** Close tabs on signals from the tab itself (e.g. ctrl+d) */
  useCustomEventListener(
    EVENT_CLOSE_COMPONENT,
    (data: TEventId) => {
      deleteTab(data.id);
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

  function getPanelId(id: string, panelGroup: string): TPanelId {
    const result: TPanelId = {
      id: panelGroup,
      isBorder: false,
      location: DockLocation.CENTER,
    };

    switch (panelGroup) {
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
      result.id =
        model
          .getBorderSet()
          .getBorders()
          .find((b) => b.getLocation() === result.location)
          ?.getId() || id;
    } else {
      const nodeBId = model.getNodeById(panelGroup);
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
      if (tab) {
        const panelId = getPanelId(tab.id || "", tab.panelGroup);
        const action = Actions.addNode(tab, panelId.id, DockLocation.CENTER, -1);
        model.doAction(action);

        if (panelId.isBorder) {
          // If any tab in same border is visible, selecting the new tab can hide it
          const border = model
            .getBorderSet()
            .getBorders()
            .find((b) => b.getLocation() === panelId.location);

          const hasVisible = border?.getChildren().some((c) => (c as TabNode).isVisible());

          if (!hasVisible && border?.getChildren().length) {
            const editorId = border.getChildren().slice(-1)[0].getId();
            model.doAction(Actions.selectTab(editorId));
          }
        }
      }
      setAddToLayout(newAddToLayout);
    }
  }, [addToLayout, model]);

  function factory(node: TabNode): JSX.Element {
    const component = node.getComponent();
    if (component && layoutComponents[component]) {
      return layoutComponents[component] as React.ReactElement;
    }

    switch (component) {
      case LAYOUT_TABS.NODES:
        return <HostTreeViewPanel key="nodes-panel" />;
      case LAYOUT_TABS.HOSTS:
        return <ProviderPanel key="hosts-panel" />;
      case LAYOUT_TABS.PACKAGES:
        return <PackageExplorerPanel key="pkg-panel" />;
      case LAYOUT_TABS.DETAILS:
        return <DetailsPanel key="node-details-panel" />;
      case LAYOUT_TABS.LOGGING:
        return <LoggingPanel key="logging-panel" />;
      case LAYOUT_TABS.TOPICS:
        return <TopicsPanel key="topics-panel" />;
      case LAYOUT_TABS.SERVICES:
        return <ServicesPanel key="services-panel" />;
      case LAYOUT_TABS.SETTINGS:
        return <SettingsPanel key="settings-panel" />;
      case LAYOUT_TABS.ABOUT:
        return <AboutPanel key="about-panel" />;
      case LAYOUT_TABS.PARAMETER:
        return <ParameterPanel key="parameter-panel" nodes={[]} providers={[]} />;
      default:
        return <></>;
    }
  }

  async function openExternalTerminal(config: IExtTerminalConfig, tabNodeId: string): Promise<void> {
    // create terminal command
    const provider = rosCtx.getProviderById(config.providerId);
    if (!provider) return;

    const terminalCmd = await provider.cmdForType(
      config.type,
      config.nodeName,
      config.topicName,
      config.screen,
      config.cmd
    );

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

  function onRenderTab(node: TabNode, renderValues: any): void {
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
      ].includes(renderValues.name)
    ) {
      renderValues.content = (
        <Tooltip title={renderValues.name} placement="bottom" disableInteractive>
          <Typography>{tabFullName ? renderValues.name : basename(renderValues.name)}</Typography>
        </Tooltip>
      );
    }

    switch (node.getId()) {
      case LAYOUT_TABS.LOGGING:
        renderValues.content = "";
        renderValues.leading = (
          <Tooltip title="Logging (mas gui)" placement="top" enterDelay={tooltipDelay} disableInteractive>
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
        renderValues.name = "Option";
        break;

      default:
        // add leading icons depending on tab type
        switch (node.getConfig()?.tabType) {
          case CmdType.LOG:
            renderValues.leading = <WysiwygIcon sx={{ fontSize: (theme) => theme.typography.fontSize }} />;
            break;
          case CmdType.SCREEN:
            renderValues.leading = <DvrIcon sx={{ fontSize: (theme) => theme.typography.fontSize }} />;
            break;
          case CmdType.TERMINAL:
            renderValues.leading = <TerminalIcon sx={{ fontSize: (theme) => theme.typography.fontSize }} />;
            break;
          case "echo":
          case CmdType.ECHO:
            renderValues.leading = <ChatBubbleOutlineIcon sx={{ fontSize: (theme) => theme.typography.fontSize }} />;
            break;
          case "publish":
            renderValues.leading = <PlayCircleOutlineIcon sx={{ fontSize: (theme) => theme.typography.fontSize }} />;
            break;
          case LAYOUT_TABS.SERVICES:
            renderValues.leading = <SyncAltOutlinedIcon sx={{ fontSize: (theme) => theme.typography.fontSize }} />;
            break;
          case "info":
            renderValues.leading = <InfoOutlinedIcon sx={{ fontSize: (theme) => theme.typography.fontSize }} />;
            break;
          case "parameter":
            renderValues.leading = <TuneIcon sx={{ fontSize: (theme) => theme.typography.fontSize }} />;
            break;
          case "editor":
            renderValues.leading = <BorderColorIcon sx={{ fontSize: (theme) => theme.typography.fontSize }} />;
            break;
          case "node-logger":
            renderValues.leading = (
              <SettingsInputCompositeOutlinedIcon
                sx={{ fontSize: (theme) => theme.typography.fontSize, rotate: "90deg" }}
              />
            );
            break;
          default:
            break;
        }

        // add "open externally" button if supported
        if (node.getConfig()?.openExternal && window.commandExecutor) {
          renderValues.buttons.push(
            <Tooltip
              key={`button-close-${node.getId()}`}
              title="Open in external window"
              placement="bottom"
              enterDelay={tooltipDelay}
              disableInteractive
            >
              <IconButton
                sx={{ padding: "1px" }}
                onMouseDown={(event) => {
                  if (event?.button === 1) return;

                  const cfg = node.getConfig();

                  if (cfg.extTerminalConfig) {
                    openExternalTerminal(cfg.extTerminalConfig, node.getId());
                  }
                  if (cfg.editorConfig) {
                    const ecfg = cfg.editorConfig;
                    window.editorManager?.open(
                      ecfg.id,
                      ecfg.host,
                      ecfg.port,
                      ecfg.path,
                      ecfg.rootLaunch,
                      ecfg.fileRange,
                      ecfg.launchArgs
                    );
                    deleteTab(node.getId());
                  }
                  if (cfg.publisherConfig) {
                    const pcfg = cfg.publisherConfig;
                    window.publishManager?.start(pcfg.id, pcfg.host, pcfg.port, pcfg.topicName, pcfg.topicType);
                    deleteTab(node.getId());
                  }
                  if (cfg.subscriberConfig) {
                    const scfg = cfg.subscriberConfig;
                    window.subscriberManager?.open(
                      scfg.id,
                      scfg.host,
                      scfg.port,
                      scfg.topic,
                      scfg.showOptions,
                      scfg.noData
                    );
                    deleteTab(node.getId());
                  }
                  if (cfg.terminalConfig) {
                    const tcfg = cfg.terminalConfig;
                    window.terminalManager?.open(
                      tcfg.id,
                      tcfg.host,
                      tcfg.port,
                      tcfg.cmdType,
                      tcfg.node,
                      tcfg.screen,
                      tcfg.cmd
                    );
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

  function pAddTabStickyButton(
    container: React.ReactNode[],
    id: string,
    title: string,
    component: React.ReactNode,
    setId: string,
    icon: React.ReactNode,
    tooltipLoc:
      | "right"
      | "top"
      | "bottom"
      | "bottom-end"
      | "bottom-start"
      | "left-end"
      | "left-start"
      | "left"
      | "right-end"
      | "right-start"
      | "top-end"
      | "top-start"
      | undefined = "right",
    force = false
  ): void {
    if (force || !model.getNodeById(id)) {
      container.push(
        <Tooltip
          key={`tooltip-${id}`}
          title={title}
          placement={tooltipLoc}
          enterDelay={tooltipDelay}
          enterNextDelay={tooltipDelay}
          disableInteractive
        >
          <span>
            <IconButton
              onClick={() =>
                emitCustomEvent(EVENT_OPEN_COMPONENT, eventOpenComponent(id, title, component, true, setId))
              }
            >
              {icon}
            </IconButton>
          </span>
        </Tooltip>
      );
    }
  }

  function onRenderTabSet(node: TabSetNode | BorderNode, renderValues: ITabSetRenderValues): void {
    const children = node.getChildren();

    for (const child of children) {
      if (child.getId() === LAYOUT_TABS.NODES) {
        pAddTabStickyButton(
          renderValues.stickyButtons,
          LAYOUT_TABS.TOPICS,
          "Topics",
          <TopicsPanel />,
          node.getId(),
          <TopicIcon sx={{ fontSize: "inherit" }} />
        );
        pAddTabStickyButton(
          renderValues.stickyButtons,
          LAYOUT_TABS.SERVICES,
          "Services",
          <ServicesPanel />,
          node.getId(),
          <FeaturedPlayListIcon sx={{ fontSize: "inherit" }} />
        );
        pAddTabStickyButton(
          renderValues.stickyButtons,
          LAYOUT_TABS.PARAMETER,
          "Parameter",
          <ParameterPanel nodes={[]} providers={[]} />,
          node.getId(),
          <TuneIcon sx={{ fontSize: "inherit" }} />
        );
        if (window.commandExecutor) {
          renderValues.stickyButtons.push(<ExternalAppsModal key="external-apps-dialog" />);
        }
      }
      // if (child.getId() === LAYOUT_TABS.HOSTS) {
      //   renderValues.buttons.push(<SettingsModal key="settings-dialog" />);
      // }
    }

    if (node.getId() === LAYOUT_TAB_SETS.BORDER_BOTTOM) {
      if (currentInfoState) {
        renderValues.buttons.push(
          <Tooltip key="tooltip-log" title="" disableInteractive>
            <Typography style={{ color: getInfoStateColor(currentInfoState.level, isDarkMode) }}>
              {currentInfoState.message}
            </Typography>
          </Tooltip>
        );
      }

      // add settings tab button in bottom border
      pAddTabStickyButton(
        renderValues.buttons,
        LAYOUT_TABS.SETTINGS,
        "Settings",
        <SettingsPanel />,
        LAYOUT_TABS.NODES,
        <SettingsIcon sx={{ fontSize: "inherit" }} />,
        "top",
        true
      );

      // add about tab button in bottom border
      pAddTabStickyButton(
        renderValues.buttons,
        LAYOUT_TABS.ABOUT,
        "About",
        <AboutPanel />,
        LAYOUT_TABS.NODES,
        <InfoOutlinedIcon sx={{ fontSize: "inherit" }} />,
        "top",
        true
      );

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
                emitCustomEvent(
                  EVENT_OPEN_COMPONENT,
                  eventOpenComponent(LAYOUT_TABS.ABOUT, "About", <AboutPanel />, true, LAYOUT_TABS.NODES)
                );
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

  function removeGenericTabs(parent: any): IJsonRowNode {
    if (!parent.children) return parent;

    // if tabs are removed, selection index may become invalid
    if (parent.selected !== undefined) {
      parent.selected = undefined;
    }

    parent.children = parent.children.filter((item: any) => {
      // do not store Settings, About and Parameter tabs
      if (
        item.type === "tab" &&
        item.id !== LAYOUT_TABS.ABOUT &&
        item.id !== LAYOUT_TABS.SETTINGS &&
        item.id !== LAYOUT_TABS.PARAMETER &&
        LAYOUT_TAB_LIST.includes(item.id)
      ) {
        return true;
      }
      if (item.children) {
        removeGenericTabs(item);
        return true;
      }
      return false;
    });

    return parent;
  }

  /** Remove all tabs from layout that are not in LAYOUT_TAB_LIST */
  const cleanAndSaveLayout = useDebounceCallback(() => {
    const modelJson = model.toJson();

    for (const item of modelJson.borders || []) {
      item.selected = -1;
      removeGenericTabs(item);
    }

    modelJson.layout = removeGenericTabs(modelJson.layout);
    setLayoutJson(modelJson);
  }, 500);

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
            const result = await prov.shutdown();
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
    const currFontSize = settingsCtx.get("fontSize") as number;
    if (event.ctrlKey && event.key === "+") {
      settingsCtx.set("fontSize", currFontSize + 1);
    }
    if (event.ctrlKey && event.key === "-") {
      settingsCtx.set("fontSize", currFontSize - 1);
    }
    if (event.ctrlKey && event.key === "0") {
      settingsCtx.set("fontSize", 14);
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
        model.doAction(Actions.deleteTab(editorId));
      }
    }

    // cancel app close if any tab failed to save
    if (failedTabs.size > 0) {
      electronCtx.cancelCloseApp();
    }

    setDirtyTabs([]);
  }, [dirtyTabs, monacoCtx, model, electronCtx]);

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
    </Stack>
  );
}
