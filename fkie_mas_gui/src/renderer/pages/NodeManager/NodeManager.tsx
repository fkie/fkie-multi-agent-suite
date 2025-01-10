import PasswordDialog from "@/renderer/components/PasswordModal/PasswordDialog";
import { EventProviderAuthRequest } from "@/renderer/providers/events";
import { EVENT_PROVIDER_AUTH_REQUEST } from "@/renderer/providers/eventTypes";
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
import { useCallback, useContext, useEffect, useRef, useState } from "react";
import { emitCustomEvent, useCustomEventListener } from "react-custom-events";
import ExternalAppsModal from "../../components/ExternalAppsModal/ExternalAppsModal";
import ProviderSelectionModal from "../../components/SelectionModal/ProviderSelectionModal";
import DraggablePaper from "../../components/UI/DraggablePaper";
import { AutoUpdateContext } from "../../context/AutoUpdateContext";
import { ElectronContext } from "../../context/ElectronContext";
import { LoggingContext } from "../../context/LoggingContext";
import { ModifiedTabsInfo, MonacoContext } from "../../context/MonacoContext";
import { RosContext } from "../../context/RosContext";
import { SettingsContext } from "../../context/SettingsContext";
import useLocalStorage from "../../hooks/useLocalStorage";
import { getBaseName } from "../../models";
import { CmdType, Provider } from "../../providers";
import { getRosNameAbb } from "../../utils";
import { DEFAULT_LAYOUT, LAYOUT_TAB_LIST, LAYOUT_TAB_SETS, LAYOUT_TABS } from "./layout";
import {
  EVENT_CLOSE_COMPONENT,
  EVENT_OPEN_COMPONENT,
  eventOpenComponent,
  TEventId,
  TEventOpenComponent,
} from "./layout/events";
import { IExtTerminalConfig } from "./layout/LayoutTabConfig";
import "./NodeManager.css";
import AboutPanel from "./panels/AboutPanel";
import HostTreeViewPanel from "./panels/HostTreeViewPanel";
import LoggingPanel from "./panels/LoggingPanel";
import NodesDetailsPanel from "./panels/NodesDetailsPanel";
import OverflowMenuNodeDetails from "./panels/OverflowMenuNodeDetails";
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
  const auCtx = useContext(AutoUpdateContext);
  const electronCtx = useContext(ElectronContext);
  const rosCtx = useContext(RosContext);
  const logCtx = useContext(LoggingContext);
  const monacoCtx = useContext(MonacoContext);
  // const navCtx = useContext(NavigationContext);
  const settingsCtx = useContext(SettingsContext);
  const [layoutJson, setLayoutJson] = useLocalStorage<IJsonModel>("layout", DEFAULT_LAYOUT);
  const [model, setModel] = useState<Model>(Model.fromJson(layoutJson));
  const layoutRef = useRef(null);
  const [layoutComponents] = useState({});
  const [addToLayout, setAddToLayout] = useState<ITabAttributesExt[]>([]);
  const [modifiedEditorTabs, setModifiedEditorTabs] = useState<ModifiedTabsInfo[]>([]);
  const [passwordRequests, setPasswordRequests] = useState<React.ReactNode[]>([]);
  const [tooltipDelay, setTooltipDelay] = useState<number>(settingsCtx.get("tooltipEnterDelay") as number);

  useEffect(() => {
    setTooltipDelay(settingsCtx.get("tooltipEnterDelay") as number);
  }, [settingsCtx.changed]);

  function hasTab(layout, tabId): boolean {
    if (!layout.children) return false;
    const tabs = layout.children.filter((item) => {
      if (item.type === "tab" && item.id === tabId) {
        return true;
      }
      if (item.children) {
        return hasTab(item, tabId);
      }
      return false;
    });
    return tabs.length > 0;
  }

  // disable float button if the gui is not running in the browser
  function updateFloatButton(layout: IJsonRowNode | IJsonBorderNode | IJsonTabSetNode): boolean {
    if (!layout.children) return false;
    let result = false;
    layout.children.forEach((item) => {
      if (item.type === "tab") {
        if (item.enableFloat !== !window.commandExecutor) {
          item.enableFloat = !window.commandExecutor;
          result = true;
        }
      }
      if (item.children) {
        if (updateFloatButton(item)) {
          result = true;
        }
      }
    });
    return result;
  }

  useEffect(() => {
    rosCtx.setLayoutModel(model);
  }, [model]);

  useEffect(() => {
    // check on load if float button should be enabled or not
    let changed: boolean = updateFloatButton(layoutJson.layout);
    layoutJson.borders?.forEach((border) => {
      if (updateFloatButton(border)) {
        changed = true;
      }
    });
    layoutJson.layout.children?.forEach((layout) => {
      if (updateFloatButton(layout)) {
        changed = true;
      }
    });
    if (changed) {
      setLayoutJson(layoutJson);
      setModel(Model.fromJson(layoutJson));
    }
  }, [layoutJson]);

  useEffect(() => {
    if (settingsCtx.get("resetLayout") || !hasTab(layoutJson.layout, LAYOUT_TABS.NODES)) {
      setLayoutJson(DEFAULT_LAYOUT);
      setModel(Model.fromJson(DEFAULT_LAYOUT));
      settingsCtx.set("resetLayout", false);
      logCtx.success(`Layout reset!`);
    }
  }, [settingsCtx.changed, layoutJson, setLayoutJson, setModel, settingsCtx, hasTab, logCtx]);

  /** Hide bottom panel on close of last terminal */
  function deleteTab(tabId: string): void {
    // check if it is an editor with modified files
    if (tabId.startsWith("editor")) {
      const mTab = monacoCtx.getModifiedFilesByTab(tabId);
      if (mTab) {
        const editorTab = model.getNodeById(tabId);
        if (editorTab) {
          model.doAction(Actions.selectTab(editorTab.getId()));
          setModifiedEditorTabs([mTab]);
          return;
        }
      }
    }
    // handle tabs in bottom border
    const nodeBId = model.getNodeById(tabId);
    if (
      nodeBId &&
      nodeBId.getParent()?.getType() === "border" &&
      (nodeBId.getParent() as BorderNode)?.getLocation().getName() === DockLocation.BOTTOM.getName()
    ) {
      // on close of last bottom tab hide the border if it currently visible
      const shouldSelectNewTab =
        nodeBId.getParent()?.getChildren().length === 2 &&
        (nodeBId.getParent() as BorderNode)?.getSelectedNode()?.isVisible();
      if (shouldSelectNewTab) {
        model.doAction(Actions.selectTab(tabId));
      }
    } else {
      // select nodes tab it is in the same set as closed tab
      const nodeBId = model.getNodeById(tabId);
      if (nodeBId) {
        nodeBId
          .getParent()
          ?.getChildren()
          .filter((tab) => {
            if (tab.getId() === LAYOUT_TABS.NODES) {
              model.doAction(Actions.selectTab(tab.getId()));
            }
          });
      }
    }
    model.doAction(Actions.deleteTab(tabId));
  }

  useCustomEventListener(
    EVENT_OPEN_COMPONENT,
    (data: TEventOpenComponent) => {
      const node = model.getNodeById(data.id);
      if (node) {
        if (node.getParent()?.getType() === "border") {
          const selectedNode = (node.getParent() as BorderNode)?.getSelectedNode();
          if (selectedNode?.getId() === node.getId()) {
            // it is border and current tab is selected => nothing to do
          } else if (selectedNode?.getId() === LAYOUT_TABS.HOSTS) {
            // it is border and current tab is HOSTS => nothing to do
          } else if (node.getId() === LAYOUT_TABS.LOGGING) {
            if (!selectedNode?.isVisible()) {
              // activate logging tab if not visible.
              model.doAction(Actions.selectTab(data.id));
            }
          } else if (selectedNode?.isVisible()) {
            // activate already existing tab if the border is enabled.
            model.doAction(Actions.selectTab(data.id));
          } else {
            model.doAction(Actions.selectTab(data.id));
          }
        } else {
          // activate already existing tab.
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
          enableFloat: !window.commandExecutor,
          config: data.config,
        };
        layoutComponents[data.id] = data.component;
        // store new tabs using useEffect so dockMove() can create panels if events comes to fast
        setAddToLayout((oldValue) => [tab, ...oldValue]);
      }
    },
    [layoutComponents, setAddToLayout]
  );

  /** close tabs on signals from tab itself (e.g. ctrl+d) */
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
            setPasswordRequests((prev) =>
              prev.filter((item) => {
                // hack to remove closed dialog from our list
                return prov.id !== (item as { key: string })?.key;
              })
            );
          }}
        />,
      ]);
    },
    [setPasswordRequests]
  );

  function getPanelId(id: string, panelGroup: string): TPanelId {
    const result: TPanelId = {
      id: panelGroup,
      isBorder: false,
      location: DockLocation.CENTER,
    };
    if (panelGroup.startsWith("border")) {
      // search first for a tab with same start prefix
    }
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
        // it could be a tabSet id
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

  // adds tabs to layout after event to create new tab was received and the 'addToLayout' was updated.
  useEffect(() => {
    if (addToLayout.length > 0) {
      const tab = addToLayout.pop();
      if (tab) {
        const panelId = getPanelId(tab.id || "", tab.panelGroup);
        const action = Actions.addNode(tab, panelId.id, DockLocation.CENTER, 1);
        model.doAction(action);
        if (model && panelId.isBorder && panelId.id) {
          // If a tab in the same border is visible,
          // selectTab causes the new tab to hide
          const shouldSelectNewTab = !model
            .getBorderSet()
            .getBorders()
            .find((b) => b.getLocation() === panelId.location)
            ?.getChildren()
            .map((c) => c.isVisible())
            .includes(true);

          if (shouldSelectNewTab) {
            const tabId: string | undefined = model
              .getBorderSet()
              .getBorders()
              .find((b) => b.getLocation() === panelId.location)
              ?.getChildren()
              .slice(-1)[0] // Get last children === new tab
              .getId();
            if (tabId) {
              model.doAction(Actions.selectTab(tabId));
            }
          }
        }
      }
      setAddToLayout([...addToLayout]);
    }
  }, [addToLayout, getPanelId, model]);

  function factory(node: TabNode): JSX.Element {
    const component = node.getComponent();
    switch (component) {
      case LAYOUT_TABS.NODES:
        return <HostTreeViewPanel key="nodes-panel" />;
      case LAYOUT_TABS.HOSTS:
        return <ProviderPanel key="hosts-panel" />;
      case LAYOUT_TABS.PACKAGES:
        return <PackageExplorerPanel key="pkg-panel" />;
      case LAYOUT_TABS.NODE_DETAILS:
        return <NodesDetailsPanel key="node-details-panel" />;
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
        if (component) {
          return layoutComponents[component];
        }
    }
    return <></>;
  }

  async function openExternalTerminal(config: IExtTerminalConfig, tabNodeId: string): Promise<void> {
    // create a terminal command
    const provider = rosCtx.getProviderById(config.providerId);
    if (!provider) return;
    const terminalCmd = await provider.cmdForType(
      config.type,
      config.nodeName,
      config.topicName,
      config.screen,
      config.cmd
    );
    // open screen in a new terminal
    try {
      window.commandExecutor?.execTerminal(
        provider.isLocalHost ? null : { host: provider.host() },
        `"${config.type} ${config.nodeName}@${provider.host()}"`,
        terminalCmd.cmd
      );
      deleteTab(tabNodeId);
    } catch (error) {
      logCtx.error(`Can't open external terminal for ${config.nodeName}`, JSON.stringify(error), true);
    }
  }

  function onRenderTab(node /* TabNode */, renderValues /* ITabRenderValues */): void {
    // add tooltip to the abbreviations
    if (
      ![
        "Hosts",
        "Node Details",
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
          <Typography>{getRosNameAbb(renderValues.name)}</Typography>
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
              // variant="standard"
              // anchorOrigin="top"
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
      case LAYOUT_TABS.NODE_DETAILS:
        renderValues.buttons.push(<OverflowMenuNodeDetails key="overflow-node-details" />);
        break;
      default:
        // add leading icons to the tabs
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
        if (node.getConfig()?.openExternal && window.commandExecutor) {
          renderValues.buttons.push(
            <Tooltip
              key={`button-close-${node.getId()}`}
              title="open new in external window"
              placement="bottom"
              enterDelay={tooltipDelay}
              disableInteractive
            >
              <IconButton
                sx={{ padding: "1px" }}
                onMouseDown={(event) => {
                  if (event?.button === 1) return;
                  if (node.getConfig().extTerminalConfig) {
                    openExternalTerminal(node.getConfig().extTerminalConfig, node.getId());
                  }
                  if (node.getConfig().editorConfig) {
                    const cfg = node.getConfig().editorConfig;
                    window.editorManager?.open(
                      cfg.id,
                      cfg.host,
                      cfg.port,
                      cfg.path,
                      cfg.rootLaunch,
                      cfg.fileRange,
                      cfg.launchArgs
                    );
                    deleteTab(node.getId());
                  }
                  if (node.getConfig().subscriberConfig) {
                    const cfg = node.getConfig().subscriberConfig;
                    window.subscriberManager?.open(cfg.id, cfg.host, cfg.port, cfg.topic, cfg.showOptions, cfg.noData);
                    deleteTab(node.getId());
                  }
                  if (node.getConfig().terminalConfig) {
                    const cfg = node.getConfig().terminalConfig;
                    window.terminalManager?.open(
                      cfg.id,
                      cfg.host,
                      cfg.port,
                      cfg.cmdType,
                      cfg.node,
                      cfg.screen,
                      cfg.cmd
                    );
                    deleteTab(node.getId());
                  }
                  event.stopPropagation();
                }}
              >
                <LaunchIcon
                  sx={{
                    fontSize: (theme) => theme.typography.fontSize,
                  }}
                />
              </IconButton>
            </Tooltip>
          );
        }
        break;
    }
    // renderValues.content = (<div>hello</div>);
    // renderValues.content += " *";
    // renderValues.leading = <img style={{width:"1em", height:"1em"}}src="images/folder.svg"/>;
    // renderValues.name = "tab " + node.getId(); // name used in overflow menu
    // renderValues.buttons.push(<div style={{flexGrow:1}}></div>);
    // renderValues.buttons.push(<img style={{width:"1em", height:"1em"}} src="images/folder.svg"/>);
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
    force: boolean = false
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
    children.forEach((child) => {
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
    });
    if (node.getId() === LAYOUT_TAB_SETS.BORDER_BOTTOM) {
      // add settings to bottom border
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
      // add about to bottom border
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
      // add update button in the bottom border
      if (auCtx.updateAvailable) {
        renderValues.buttons.push(
          <Tooltip title={`new version ${auCtx.updateAvailable.version} available`} placement="top">
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
        // } else {
        //   renderValues.buttons.push(
        //     <Typography noWrap variant="body2">
        //       v{packageJson.version}
        //     </Typography>,
        //   );
      }
    }
  }

  function removeGenericTabs(parent): IJsonRowNode {
    if (!parent.children) return parent;
    if (parent.selected) {
      // if tabs are removed we have to disable the selection of them in the panel
      parent.selected = undefined;
    }
    parent.children = parent.children.filter((item) => {
      // do not store Setting and About tabs
      if (
        item.type === "tab" &&
        item.id !== LAYOUT_TABS.ABOUT &&
        item.id !== LAYOUT_TABS.SETTINGS &&
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

  /** removes all tab from layout not listed in LAYOUT_TAB_LIST */
  const cleanAndSaveLayout = useDebounceCallback(
    (/* model */) => {
      const modelJson = model.toJson();
      modelJson.borders?.forEach((item) => {
        item.selected = -1;
        removeGenericTabs(item);
      });
      modelJson.layout = removeGenericTabs(modelJson.layout);
      setLayoutJson(modelJson);
    },
    500
  );

  const isInstallUpdateRequested = useCallback(() => {
    return auCtx.requestedInstallUpdate;
  }, [auCtx.requestedInstallUpdate]);

  useEffect(() => {
    // do not ask for shutdown on some reasons
    if (electronCtx.terminateSubprocesses) {
      if (isInstallUpdateRequested()) {
        electronCtx.shutdownManager?.quitGui();
      }
      if (rosCtx.providers.length <= 0) {
        electronCtx.shutdownManager?.quitGui();
      }
      setModifiedEditorTabs(monacoCtx.getModifiedTabs());
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
      console.log(`Quit app`);
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
      settingsCtx.set("fontSize", currFontSize);
    }
  }

  const dialogRef = useRef(null);

  return (
    <Stack
      onKeyDown={(event) => onKeyDown(event)}
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
          // hide bottom panel on close of last terminal
          if (action.type === Actions.DELETE_TAB) {
            deleteTab(action.data.node);
            return undefined;
          }
          return action;
        }}
        onRenderTab={(node, renderValues) => onRenderTab(node, renderValues)}
        onRenderTabSet={(node, renderValues) => onRenderTabSet(node, renderValues)}
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
      {electronCtx.terminateSubprocesses && monacoCtx.getModifiedTabs().length === 0 && rosCtx.providers.length > 0 && (
        // check for unsaved files before quit gui
        <ProviderSelectionModal
          title="Select providers to shut down"
          providers={rosCtx.providers}
          onCloseCallback={() => electronCtx.cancelCloseApp()}
          onConfirmCallback={(providers) => shutdownProviders(providers)}
          onForceCloseCallback={() => electronCtx.shutdownManager?.quitGui()}
        />
      )}
      {modifiedEditorTabs.length > 0 && (
        <Dialog
          open={modifiedEditorTabs.length > 0}
          onClose={() => setModifiedEditorTabs([])}
          fullWidth
          scroll="paper"
          maxWidth="sm"
          ref={dialogRef}
          PaperProps={{
            component: DraggablePaper,
            dialogRef: dialogRef,
          }}
          aria-labelledby="draggable-dialog-title"
        >
          <DialogTitle className="draggable-dialog-title" style={{ cursor: "move" }} id="draggable-dialog-title">
            Changed Files
          </DialogTitle>

          <DialogContent aria-label="list">
            {modifiedEditorTabs.map((tab) => {
              return (
                <DialogContentText key={tab.tabId} id="alert-dialog-description">
                  {`There are ${tab.uriPaths.length} unsaved files in "${getBaseName(tab.tabId)}" tab.`}
                </DialogContentText>
              );
            })}
          </DialogContent>

          <DialogActions>
            <Button
              color="warning"
              onClick={() => {
                modifiedEditorTabs.forEach((tab) => {
                  model.doAction(Actions.deleteTab(tab.tabId));
                });
                setModifiedEditorTabs([]);
              }}
            >
              Don&apos;t save
            </Button>
            <Button
              color="primary"
              onClick={() => {
                setModifiedEditorTabs([]);
                electronCtx.cancelCloseApp();
              }}
            >
              Cancel
            </Button>

            <Button
              autoFocus
              color="primary"
              onClick={async () => {
                // save all files
                const result = await Promise.all(
                  modifiedEditorTabs.map(async (tab) => {
                    const tabResult = await monacoCtx.saveModifiedFilesOfTabId(tab.tabId);
                    return tabResult;
                  })
                );
                // TODO inform about error on failed save
                let failed = false;
                result.forEach((item) => {
                  if (item[0].result) {
                    model.doAction(Actions.deleteTab(item[0].tabId));
                  } else {
                    failed = true;
                  }
                });
                if (failed) {
                  electronCtx.cancelCloseApp();
                }
                setModifiedEditorTabs([]);
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
