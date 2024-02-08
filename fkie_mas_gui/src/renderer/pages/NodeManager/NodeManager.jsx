/* eslint-disable jsx-a11y/click-events-have-key-events */
/* eslint-disable jsx-a11y/no-static-element-interactions */
import BorderColorIcon from '@mui/icons-material/BorderColor';
import CallMadeIcon from '@mui/icons-material/CallMade';
import ChatBubbleOutlineIcon from '@mui/icons-material/ChatBubbleOutline';
import DesktopWindowsOutlinedIcon from '@mui/icons-material/DesktopWindowsOutlined';
import FeaturedPlayListIcon from '@mui/icons-material/FeaturedPlayList';
import InfoOutlinedIcon from '@mui/icons-material/InfoOutlined';
import PlayCircleOutlineIcon from '@mui/icons-material/PlayCircleOutline';
import SubjectIcon from '@mui/icons-material/Subject';
import TerminalIcon from '@mui/icons-material/Terminal';
import TopicIcon from '@mui/icons-material/Topic';
import TuneIcon from '@mui/icons-material/Tune';
import WysiwygIcon from '@mui/icons-material/Wysiwyg';
import { Badge, IconButton, Stack, Tooltip } from '@mui/material';
import { useDebounceCallback } from '@react-hook/debounce';
import { Actions, DockLocation, Layout, Model } from 'flexlayout-react';
import { useCallback, useContext, useEffect, useRef, useState } from 'react';
import { emitCustomEvent, useCustomEventListener } from 'react-custom-events';

import { CmdType } from 'renderer/providers';
import ExternalAppsModal from '../../components/ExternalAppsModal/ExternalAppsModal';
import ProviderSelectionModal from '../../components/SelectionModal/ProviderSelectionModal';
import SettingsModal from '../../components/SettingsModal/SettingsModal';
import { ElectronContext } from '../../context/ElectronContext';
import { LoggingContext } from '../../context/LoggingContext';
import { RosContext } from '../../context/RosContext';
import { SettingsContext } from '../../context/SettingsContext';
import { SSHContext } from '../../context/SSHContext';
import {
  EVENT_CLOSE_COMPONENT,
  EVENT_OPEN_COMPONENT,
  eventOpenComponent,
} from '../../utils/events';
import HostTreeViewPanel from './panels/HostTreeViewPanel';
import LoggingPanel from './panels/LoggingPanel';
import NodesDetailsPanel from './panels/NodesDetailsPanel';
import OverflowMenuNodeDetails from './panels/OverflowMenuNodeDetails';
import PackageExplorerPanel from './panels/PackageExplorerPanel';
import ParameterPanel from './panels/ParameterPanel';
import ProviderPanel from './panels/ProviderPanel';
import ServicesPanel from './panels/ServicesPanel';
import TopicsPanel from './panels/TopicsPanel';

import useLocalStorage from '../../hooks/useLocalStorage';

import {
  DEFAULT_LAYOUT,
  LAYOUT_TAB_LIST,
  LAYOUT_TAB_SETS,
  LAYOUT_TABS,
} from './layout';
import './NodeManager.css';

function NodeManager() {
  const electronCtx = useContext(ElectronContext);
  const rosCtx = useContext(RosContext);
  const logCtx = useContext(LoggingContext);
  const settingsCtx = useContext(SettingsContext);
  const SSHCtx = useContext(SSHContext);
  const [layoutJson, setLayoutJson] = useLocalStorage('layout', DEFAULT_LAYOUT);
  const [model, setModel] = useState(Model.fromJson(layoutJson));
  const layoutRef = useRef(null);
  const [layoutComponents] = useState({});
  const [addToLayout, setAddToLayout] = useState([]);

  const tooltipDelay = settingsCtx.get('tooltipEnterDelay');

  useEffect(() => {
    if (settingsCtx.get('resetLayout')) {
      setLayoutJson(DEFAULT_LAYOUT);
      setModel(Model.fromJson(DEFAULT_LAYOUT));
      settingsCtx.set('resetLayout', false);
    }
  }, [settingsCtx, setLayoutJson, setModel]);

  useCustomEventListener(EVENT_OPEN_COMPONENT, (data) => {
    const node = model.getNodeById(data.id);
    if (node) {
      // activate already existing tab.
      model.doAction(Actions.selectTab(data.id));
    } else {
      // create a new tab
      const tab = {
        id: data.id,
        type: 'tab',
        name: data.title,
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
  });

  useCustomEventListener(EVENT_CLOSE_COMPONENT, (data) => {
    model.doAction(Actions.deleteTab(data.id));
  });

  const getPanelId = useCallback(
    (id, panelGroup) => {
      const result = {
        id: panelGroup,
        isBorder: false,
        location: DockLocation.CENTER,
      };
      if (panelGroup.startsWith('border')) {
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
    [model],
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
                .getId(),
            ),
          );
        }
      }
      setAddToLayout([...addToLayout]);
    }
  }, [addToLayout, getPanelId, model]);

  const factory = (node) => {
    const component = node.getComponent();
    switch (component) {
      case LAYOUT_TABS.HOSTS:
        return <HostTreeViewPanel key="host-panel" />;
      case LAYOUT_TABS.PROVIDER:
        return <ProviderPanel key="providers-panel" />;
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
      default:
        return layoutComponents[component];
    }
  };

  function onRenderTab(
    node /* TabNode */,
    renderValues /* ITabRenderValues */,
  ) {
    switch (node.getId()) {
      case LAYOUT_TABS.LOGGING:
        renderValues.content = '';
        renderValues.leading = (
          <Tooltip
            title="Logging (mas gui)"
            placement="top"
            enterDelay={tooltipDelay}
          >
            <Badge
              color="info"
              badgeContent={`${logCtx.countErrors}`}
              invisible
              // variant="standard"
              // anchorOrigin="top"
              sx={{
                '& .MuiBadge-badge': { fontSize: 9, height: 12, minWidth: 12 },
              }}
            >
              <DesktopWindowsOutlinedIcon sx={{ fontSize: 'inherit' }} />
            </Badge>
          </Tooltip>
        );
        renderValues.name = 'Option';
        break;
      case LAYOUT_TABS.NODE_DETAILS:
        renderValues.buttons.push(
          <OverflowMenuNodeDetails key="overflow-node-details" />,
        );
        break;
      default:
        // add leading icons to the tabs
        switch (node.getConfig()?.tabType) {
          case CmdType.LOG:
            renderValues.leading = <SubjectIcon sx={{ fontSize: 'inherit' }} />;
            break;
          case CmdType.SCREEN:
            renderValues.leading = <WysiwygIcon sx={{ fontSize: 'inherit' }} />;
            break;
          case CmdType.TERMINAL:
            renderValues.leading = (
              <TerminalIcon sx={{ fontSize: 'inherit' }} />
            );
            break;
          case 'echo':
          case CmdType.ECHO:
            renderValues.leading = (
              <ChatBubbleOutlineIcon sx={{ fontSize: 'inherit' }} />
            );
            break;
          case 'publish':
            renderValues.leading = (
              <PlayCircleOutlineIcon sx={{ fontSize: 'inherit' }} />
            );
            break;
          case 'info':
            renderValues.leading = (
              <InfoOutlinedIcon sx={{ fontSize: 'inherit' }} />
            );
            break;
          case 'parameter':
            renderValues.leading = <TuneIcon sx={{ fontSize: 'inherit' }} />;
            break;
          case 'editor':
            renderValues.leading = (
              <BorderColorIcon sx={{ fontSize: 'inherit' }} />
            );
            break;
          default:
            break;
        }
        if (node.getConfig()?.openExternal && window.CommandExecutor) {
          renderValues.buttons.push(
            <IconButton
              key={`button-close-${node.getId()}`}
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
                      config.cmd,
                    );
                    // open screen in a new terminal
                    try {
                      window.CommandExecutor?.execTerminal(
                        provider.isLocalHost
                          ? null
                          : SSHCtx.getCredentialHost(provider.host()),
                        `"${config.type.toLocaleUpperCase()} ${
                          config.nodeName
                        }@${provider.host()}"`,
                        terminalCmd.cmd,
                      );
                      model.doAction(Actions.deleteTab(tabNodeId));
                    } catch (error) {
                      logCtx.error(
                        `Can't open external terminal for ${config.nodeName}`,
                        error,
                        true,
                      );
                    }
                  };
                  openExternalTerminal(
                    node.getConfig().terminalConfig,
                    node.getId(),
                  );
                }
                event.stopPropagation();
              }}
            >
              <CallMadeIcon sx={{ fontSize: 'inherit' }} />
            </IconButton>,
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
        >
          <span>
            <IconButton
              onClick={() =>
                emitCustomEvent(
                  EVENT_OPEN_COMPONENT,
                  eventOpenComponent(id, title, component, true, setId),
                )
              }
            >
              {icon}
            </IconButton>
          </span>
        </Tooltip>,
      );
    }
  }

  function onRenderTabSet(
    node /* TabSetNode */,
    renderValues /* ITabSetRenderValues */,
  ) {
    const children = node.getChildren();
    children.forEach((child) => {
      if (child.getId() === LAYOUT_TABS.HOSTS) {
        pAddTabStickyButton(
          renderValues.stickyButtons,
          LAYOUT_TABS.TOPICS,
          'Topics',
          <TopicsPanel />,
          node.getId(),
          <TopicIcon sx={{ fontSize: 'inherit' }} />,
        );
        pAddTabStickyButton(
          renderValues.stickyButtons,
          LAYOUT_TABS.SERVICES,
          'Services',
          <ServicesPanel />,
          node.getId(),
          <FeaturedPlayListIcon sx={{ fontSize: 'inherit' }} />,
        );
        pAddTabStickyButton(
          renderValues.stickyButtons,
          LAYOUT_TABS.PARAMETER,
          'Parameter',
          <ParameterPanel nodes={null} />,
          node.getId(),
          <TuneIcon sx={{ fontSize: 'inherit' }} />,
        );
        if (window.CommandExecutor) {
          renderValues.buttons.push(
            <ExternalAppsModal key="external-apps-dialog" />,
          );
        }
      }
      if (child.getId() === LAYOUT_TABS.PROVIDER) {
        renderValues.buttons.push(<SettingsModal key="settings-dialog" />);
      }
    });
  }

  function removeGenericTabs(parent) {
    if (!parent.children) return parent;
    parent.children = parent.children.filter((item) => {
      if (item.type === 'tab' && LAYOUT_TAB_LIST.includes(item.id)) {
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
    500,
  );

  return (
    <Stack
      style={{
        position: 'absolute',
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
        onRenderTab={(node, renderValues) => onRenderTab(node, renderValues)}
        onRenderTabSet={(node, renderValues) =>
          onRenderTabSet(node, renderValues)
        }
        onModelChange={(_model /* _action */) => cleanAndSaveLayout(_model)}
        onContextMenu={(node) => {
          console.log(`NO context for ${node.getId()}`);
        }}
      />
      {electronCtx.terminateSubprocesses && (
        <ProviderSelectionModal
          title="Select providers to shut down"
          providers={rosCtx.providers}
          onCloseCallback={() => electronCtx.setTerminateSubprocesses(false)}
          onConfirmCallback={async (providers) => {
            if (providers && providers.length > 0) {
              await Promise.all(
                providers.map(async (prov) => {
                  await prov.shutdown();
                }),
              );
            }
            electronCtx.shutdownInterface.quitGui();
          }}
        />
      )}
    </Stack>
  );
}

export default NodeManager;
