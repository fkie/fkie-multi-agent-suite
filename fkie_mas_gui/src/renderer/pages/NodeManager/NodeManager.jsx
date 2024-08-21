/* eslint-disable jsx-a11y/click-events-have-key-events */
/* eslint-disable jsx-a11y/no-static-element-interactions */
import BorderColorIcon from "@mui/icons-material/BorderColor";
import CallMadeIcon from "@mui/icons-material/CallMade";
import ChatBubbleOutlineIcon from "@mui/icons-material/ChatBubbleOutline";
import DesktopWindowsOutlinedIcon from "@mui/icons-material/DesktopWindowsOutlined";
import DvrIcon from "@mui/icons-material/Dvr";
import FeaturedPlayListIcon from "@mui/icons-material/FeaturedPlayList";
import InfoOutlinedIcon from "@mui/icons-material/InfoOutlined";
import PlayCircleOutlineIcon from "@mui/icons-material/PlayCircleOutline";
import SettingsInputCompositeOutlinedIcon from "@mui/icons-material/SettingsInputCompositeOutlined";
import SubjectIcon from "@mui/icons-material/Subject";
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
import { Actions, DockLocation, Layout, Model } from "flexlayout-react";
import { useCallback, useContext, useEffect, useRef, useState } from "react";
import { emitCustomEvent, useCustomEventListener } from "react-custom-events";
import ExternalAppsModal from "../../components/ExternalAppsModal/ExternalAppsModal";
import ProviderSelectionModal from "../../components/SelectionModal/ProviderSelectionModal";
import SettingsModal from "../../components/SettingsModal/SettingsModal";
import DraggablePaper from "../../components/UI/DraggablePaper";
import { ElectronContext } from "../../context/ElectronContext";
import { LoggingContext } from "../../context/LoggingContext";
import { MonacoContext } from "../../context/MonacoContext";
import { NavigationContext } from "../../context/NavigationContext";
import { RosContext } from "../../context/RosContext";
import { SettingsContext } from "../../context/SettingsContext";
import { SSHContext } from "../../context/SSHContext";
import useLocalStorage from "../../hooks/useLocalStorage";
import { getBaseName } from "../../models";
import { CmdType } from "../../providers";
import { getRosNameAbb } from "../../utils";
import {
  EVENT_CLOSE_COMPONENT,
  EVENT_OPEN_COMPONENT,
  EVENT_OPEN_SETTINGS,
  eventOpenComponent,
  eventOpenSettings,
  SETTING,
} from "../../utils/events";
import { DEFAULT_LAYOUT, LAYOUT_TAB_LIST, LAYOUT_TAB_SETS, LAYOUT_TABS } from "./layout";
import "./NodeManager.css";
import HostTreeViewPanel from "./panels/HostTreeViewPanel";
import LoggingPanel from "./panels/LoggingPanel";
import NodesDetailsPanel from "./panels/NodesDetailsPanel";
import OverflowMenuNodeDetails from "./panels/OverflowMenuNodeDetails";
import PackageExplorerPanel from "./panels/PackageExplorerPanel";
import ParameterPanel from "./panels/ParameterPanel";
import ProviderPanel from "./panels/ProviderPanel";
import ServicesPanel from "./panels/ServicesPanel";
import TopicsPanel from "./panels/TopicsPanel";

function NodeManager() {
  const electronCtx = useContext(ElectronContext);
  const rosCtx = useContext(RosContext);
  const logCtx = useContext(LoggingContext);
  const monacoCtx = useContext(MonacoContext);
  const navCtx = useContext(NavigationContext);
  const settingsCtx = useContext(SettingsContext);
  const SSHCtx = useContext(SSHContext);
  const [layoutJson, setLayoutJson] = useLocalStorage("layout", DEFAULT_LAYOUT);
  const [model, setModel] = useState(Model.fromJson(layoutJson));
  const layoutRef = useRef(null);
  const [layoutComponents] = useState({});
  const [addToLayout, setAddToLayout] = useState([]);
  const [modifiedEditorTabs, setModifiedEditorTabs] = useState([]);

  const tooltipDelay = settingsCtx.get("tooltipEnterDelay");

  const hasTab = useCallback((layout, tabId) => {
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
  }, []);

  // disable float button if the gui is not running in the browser
  const updateFloatButton = useCallback((layout) => {
    if (!layout.children) return false;
    let result = false;
    const tabs = layout.children.forEach((item) => {
      if (item.type === "tab") {
        if (item.enableFloat !== !window.CommandExecutor) {
          item.enableFloat = !window.CommandExecutor;
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
  }, []);

  useEffect(() => {
    // check on load if float button should be enabled or not
    let changed = updateFloatButton(layoutJson.layout);
    layoutJson.border?.forEach((border) => {
      if (updateFloatButton(border.layout)) {
        changed = true;
      }
    });
    if (changed) {
      setLayoutJson(layoutJson);
      setModel(Model.fromJson(layoutJson));
    }
  }, [layoutJson, setLayoutJson, setModel, updateFloatButton]);

  useEffect(() => {
    if (settingsCtx.get("resetLayout") || !hasTab(layoutJson.layout, LAYOUT_TABS.NODES)) {
      setLayoutJson(DEFAULT_LAYOUT);
      setModel(Model.fromJson(DEFAULT_LAYOUT));
      settingsCtx.set("resetLayout", false);
      logCtx.success(`Layout reset!`);
    }
  }, [settingsCtx.changed, layoutJson, setLayoutJson, setModel, settingsCtx, hasTab, logCtx]);

  /** Hide bottom panel on close of last terminal */
  const deleteTab = useCallback(
    (tabId) => {
      // check if it is an editor with modified files
      if (tabId.startsWith("editor")) {
        const mTab = monacoCtx.getModifiedFilesByTab(tabId);
        if (mTab) {
          const editorTab = model.getNodeById(tabId);
          model.doAction(Actions.selectTab(editorTab));
          setModifiedEditorTabs([mTab]);
          return;
        }
        // select nodes tab it is in the same set as close editor
        const nodeBId = model.getNodeById(tabId);
        nodeBId
          ?.getParent()
          .getChildren()
          .filter((tab) => {
            if (tab.getId() === LAYOUT_TABS.NODES) {
              model.doAction(Actions.selectTab(tab.getId()));
            }
          });
      }
      // handle tabs in bottom border
      const nodeBId = model.getNodeById(tabId);
      if (
        nodeBId.getParent().getType() === "border" &&
        nodeBId.getParent()?.getLocation().name === DockLocation.BOTTOM.name
      ) {
        // on close of last bottom tab hide the border if it currently visible
        const shouldSelectNewTab =
          nodeBId.getParent().getChildren().length === 2 && nodeBId.getParent().getSelectedNode()?.isVisible();
        if (shouldSelectNewTab) {
          model.doAction(Actions.selectTab(tabId));
        }
      }
      model.doAction(Actions.deleteTab(tabId));
    },
    [model, monacoCtx]
  );

  useCustomEventListener(
    EVENT_OPEN_COMPONENT,
    (data) => {
      const node = model.getNodeById(data.id);
      if (node) {
        if (node.getParent().getType() === "border") {
          if (node.getParent().getSelectedNode()?.getId() === node.getId()) {
            // it is border and current tab is selected => nothing to do
          } else if (node.getParent().getSelectedNode()?.getId() === LAYOUT_TABS.HOSTS) {
            // it is border and current tab is HOSTS => nothing to do
          } else if (node.getId() === LAYOUT_TABS.LOGGING) {
            if (!node.getParent().getSelectedNode()?.isVisible()) {
              // activate logging tab if not visible.
              model.doAction(Actions.selectTab(data.id));
            }
          } else if (node.getParent().getSelectedNode()?.isVisible()) {
            // activate already existing tab if the border is enabled.
            model.doAction(Actions.selectTab(data.id));
          }
        } else {
          // activate already existing tab.
          model.doAction(Actions.selectTab(data.id));
        }
      } else {
        // create a new tab
        const tab = {
          id: data.id,
          type: "tab",
          name: data.title,
          content: `${data.title}bal`,
          component: data.id,
          panelGroup: data.panelGroup,
          enableClose: data.closable,
          enableFloat: !window.CommandExecutor,
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
    (data) => {
      deleteTab(data.id);
    },
    [deleteTab]
  );

  const getPanelId = useCallback(
    (id, panelGroup) => {
      const result = {
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
          // it could be a tabset id
          result.isBorder = false;
          break;
      }
      if (result.isBorder) {
        result.id = model
          .getBorderSet()
          .getBorders()
          .find((b) => b.getLocation() === result.location)
          ?.getId();
      } else {
        const nodeBId = model.getNodeById(panelGroup);
        if (nodeBId && LAYOUT_TAB_LIST.includes(nodeBId.getId())) {
          result.id = nodeBId.getParent().getId();
        }
      }
      return result;
    },
    [model]
  );

  // adds tabs to layout after event to create new tab was received and the 'addToLayout' was updated.
  useEffect(() => {
    if (addToLayout.length > 0) {
      const tab = addToLayout.pop();
      const panelId = getPanelId(tab.id, tab.panelGroup);
      const action = Actions.addNode(tab, panelId.id, DockLocation.CENTER, -1);
      model.doAction(action);
      if (panelId.isBorder && panelId.id) {
        // If a tab in the same border is visible,
        // selectTab causes the new tab to hide
        const shouldSelectNewTab = !model
          .getBorderSet()
          .getBorders()
          .find((b) => b.getLocation() === panelId.location)
          .getChildren()
          .map((c) => c.isVisible())
          .includes(true);

        if (shouldSelectNewTab) {
          model.doAction(
            Actions.selectTab(
              model
                .getBorderSet()
                .getBorders()
                .find((b) => b.getLocation() === panelId.location)
                .getChildren()
                .slice(-1)[0] // Get last children === new tab
                .getId()
            )
          );
        }
      }
      setAddToLayout([...addToLayout]);
    }
  }, [addToLayout, getPanelId, model]);

  const factory = (node) => {
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
      case LAYOUT_TABS.PARAMETER:
        return <ParameterPanel key="parameter-panel" />;
      default:
        return layoutComponents[component];
    }
  };

  function onRenderTab(node /* TabNode */, renderValues /* ITabRenderValues */) {
    // add tooltip to the abbreviations
    if (
      !["Hosts", "Node Details", "Packages", "Nodes", "Topics", "Services", "Parameter", "Logging"].includes(
        renderValues.name
      )
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
            renderValues.leading = <WysiwygIcon sx={{ fontSize: "inherit" }} />;
            break;
          case CmdType.SCREEN:
            renderValues.leading = <DvrIcon sx={{ fontSize: "inherit" }} />;
            break;
          case CmdType.TERMINAL:
            renderValues.leading = <TerminalIcon sx={{ fontSize: "inherit" }} />;
            break;
          case "echo":
          case CmdType.ECHO:
            renderValues.leading = <ChatBubbleOutlineIcon sx={{ fontSize: "inherit" }} />;
            break;
          case "publish":
            renderValues.leading = <PlayCircleOutlineIcon sx={{ fontSize: "inherit" }} />;
            break;
          case LAYOUT_TABS.SERVICES:
            renderValues.leading = <SyncAltOutlinedIcon sx={{ fontSize: "inherit" }} />;
            break;
          case "info":
            renderValues.leading = <InfoOutlinedIcon sx={{ fontSize: "inherit" }} />;
            break;
          case "parameter":
            renderValues.leading = <TuneIcon sx={{ fontSize: "inherit" }} />;
            break;
          case "editor":
            renderValues.leading = <BorderColorIcon sx={{ fontSize: "inherit" }} />;
            break;
          case "node-logger":
            renderValues.leading = <SettingsInputCompositeOutlinedIcon sx={{ fontSize: "inherit", rotate: "90deg" }} />;
            break;
          default:
            break;
        }
        if (node.getConfig()?.openExternal && window.CommandExecutor) {
          renderValues.buttons.push(
            <Tooltip
              key={`button-close-${node.getId()}`}
              title="open in external terminal"
              placement="bottom"
              enterDelay={tooltipDelay}
              disableInteractive
            >
              <IconButton
                onMouseDown={(event) => {
                  if (node.getConfig().terminalConfig) {
                    const openExternalTerminal = async (config, tabNodeId) => {
                      // create a terminal command
                      const provider = rosCtx.getProviderById(config.providerId);
                      const terminalCmd = await provider.cmdForType(
                        config.type,
                        config.nodeName,
                        config.topicName,
                        config.screen,
                        config.cmd
                      );
                      // open screen in a new terminal
                      try {
                        window.CommandExecutor?.execTerminal(
                          provider.isLocalHost ? null : SSHCtx.getCredentialHost(provider.host()),
                          `"${config.type.toLocaleUpperCase()} ${config.nodeName}@${provider.host()}"`,
                          terminalCmd.cmd
                        );
                        deleteTab(tabNodeId);
                      } catch (error) {
                        logCtx.error(`Can't open external terminal for ${config.nodeName}`, error, true);
                      }
                    };
                    openExternalTerminal(node.getConfig().terminalConfig, node.getId());
                  }
                  if (node.getConfig().editorConfig) {
                    const cfg = node.getConfig().editorConfig;
                    window.electronAPI.openEditor(cfg.id, cfg.host, cfg.port, cfg.rootLaunch, cfg.path, cfg.fileRange);
                    deleteTab(node.getId());
                  }
                  event.stopPropagation();
                }}
              >
                <CallMadeIcon sx={{ fontSize: "inherit" }} />
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

  function pAddTabStickyButton(container, id, title, component, setId, icon) {
    if (!model.getNodeById(id)) {
      container.push(
        <Tooltip
          key={`tooltip-${id}`}
          title={title}
          placement="right"
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

  function onRenderTabSet(node /* TabSetNode */, renderValues /* ITabSetRenderValues */) {
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
          <ParameterPanel nodes={null} providers={null} />,
          node.getId(),
          <TuneIcon sx={{ fontSize: "inherit" }} />
        );
        if (window.CommandExecutor) {
          renderValues.stickyButtons.push(<ExternalAppsModal key="external-apps-dialog" />);
        }
      }
      if (child.getId() === LAYOUT_TABS.HOSTS) {
        renderValues.buttons.push(<SettingsModal key="settings-dialog" />);
      }
    });
    if (node.getId() === LAYOUT_TAB_SETS.BORDER_BOTTOM) {
      // add update button in the bottom border
      if (electronCtx.updateAvailable) {
        renderValues.buttons.push(
          <Tooltip title={`new version ${electronCtx.updateAvailable} available`} placement="top">
            <Button
              style={{ textTransform: "none" }}
              onClick={() => {
                emitCustomEvent(EVENT_OPEN_SETTINGS, eventOpenSettings(SETTING.IDS.ABOUT));
              }}
              variant="text"
              color="info"
              size="small"
            >
              <Typography noWrap variant="body2">
                update
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

  function removeGenericTabs(parent) {
    if (!parent.children) return parent;
    if (parent.selected) {
      // if tabs are removed we have to disable the selection of them in the panel
      parent.selected = undefined;
    }
    parent.children = parent.children.filter((item) => {
      if (item.type === "tab" && LAYOUT_TAB_LIST.includes(item.id)) {
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
      modelJson.borders.forEach((item) => {
        item.selected = -1;
        removeGenericTabs(item);
      });
      modelJson.layout = removeGenericTabs(modelJson.layout);
      setLayoutJson(modelJson);
    },
    500
  );

  useEffect(() => {
    if (navCtx.selectedNodes.length > 0) {
      // inform details panel tab about selected nodes by user
      emitCustomEvent(EVENT_OPEN_COMPONENT, eventOpenComponent(LAYOUT_TABS.NODE_DETAILS, "default", {}));
    } else {
      // select package explorer if no nodes are selected
      emitCustomEvent(EVENT_OPEN_COMPONENT, eventOpenComponent(LAYOUT_TABS.PACKAGES, "default", {}));
    }
  }, [navCtx.selectedNodes]);

  const isInstallUpdateRequested = useCallback(() => {
    return electronCtx.requestedInstallUpdate;
  }, [electronCtx.requestedInstallUpdate]);

  useEffect(() => {
    // do not ask for shutdown on some reasons
    if (electronCtx.terminateSubprocesses) {
      if (isInstallUpdateRequested()) {
        electronCtx.shutdownInterface.quitGui();
      }
      if (rosCtx.providers.length <= 0) {
        electronCtx.shutdownInterface.quitGui();
      }
      setModifiedEditorTabs(monacoCtx.getModifiedTabs());
    }
  }, [
    electronCtx.shutdownInterface,
    electronCtx.terminateSubprocesses,
    isInstallUpdateRequested,
    monacoCtx,
    rosCtx.providers.length,
  ]);

  const shutdownProviders = useCallback(
    async (providers) => {
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
      electronCtx.shutdownInterface.quitGui();
    },
    [electronCtx]
  );

  const onKeyDown = (event) => {
    if (event.ctrlKey && event.key === "+") {
      settingsCtx.set("fontSize", settingsCtx.get("fontSize") + 1);
    }
    if (event.ctrlKey && event.key === "-") {
      settingsCtx.set("fontSize", settingsCtx.get("fontSize") - 1);
    }
  };

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
        margin="1em"
        key="node-manager-layout"
        ref={layoutRef}
        model={model}
        factory={factory}
        onAction={(action) => {
          // hide bottom panel on close of last terminal
          if (action.type === Actions.DELETE_TAB) {
            deleteTab(action.data.node);
          } else {
            model.doAction(action);
          }
        }}
        onRenderTab={(node, renderValues) => onRenderTab(node, renderValues)}
        onRenderTabSet={(node, renderValues) => onRenderTabSet(node, renderValues)}
        onModelChange={(_model, _action) => {
          if (![Actions.SELECT_TAB, Actions.SET_ACTIVE_TABSET].includes(_action.type)) {
            cleanAndSaveLayout(_model);
          }
        }}
        onContextMenu={(node) => {
          console.log(`NO context for ${node.getId()}`);
        }}
        onAuxMouseClick={(node, event) => {
          // close tabs with middle mouse click
          if (event?.button === 1 && node.getType() === "tab" && node.isEnableClose()) {
            deleteTab(node.getId());
          }
        }}
      />
      {electronCtx.terminateSubprocesses && monacoCtx.getModifiedTabs().length === 0 && rosCtx.providers.length > 0 && (
        // check for unsaved files before quit gui
        <ProviderSelectionModal
          title="Select providers to shut down"
          providers={rosCtx.providers}
          onCloseCallback={() => electronCtx.setTerminateSubprocesses(false)}
          onConfirmCallback={(providers) => shutdownProviders(providers)}
          onForceCloseCallback={() => electronCtx.shutdownInterface.quitGui()}
        />
      )}
      {modifiedEditorTabs.length > 0 && (
        <Dialog
          open={modifiedEditorTabs.length > 0}
          onClose={() => setModifiedEditorTabs([])}
          fullWidth
          maxWidth="sm"
          ref={dialogRef}
          PaperProps={{
            component: DraggablePaper,
            dialogRef: dialogRef,
          }}
          aria-labelledby="draggable-dialog-title"
        >
          <DialogTitle className="handle" style={{ cursor: "move" }} id="draggable-dialog-title">
            Changed Files
          </DialogTitle>

          <DialogContent scroll="paper" aria-label="list">
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
                electronCtx.setTerminateSubprocesses(false);
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
                  electronCtx.setTerminateSubprocesses(false);
                }
                setModifiedEditorTabs([]);
              }}
            >
              Save all
            </Button>
          </DialogActions>
        </Dialog>
      )}
    </Stack>
  );
}

export default NodeManager;
