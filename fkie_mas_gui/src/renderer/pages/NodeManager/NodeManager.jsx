/* eslint-disable jsx-a11y/click-events-have-key-events */
/* eslint-disable jsx-a11y/no-static-element-interactions */
import { Stack } from '@mui/material';
import DockLayout from 'rc-dock';
import { useCallback, useContext, useEffect, useRef, useState } from 'react';
import { useCustomEventListener } from 'react-custom-events';

import { SettingsContext } from '../../context/SettingsContext';
import { EVENT_OPEN_COMPONENT } from '../../utils/events';
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

import './NodeManager.css';

// add close button to panel if only closable tabs are inside
const PANEL_GROUPS = {
  'close-all': {
    floatable: true,
    closable: true,
    panelExtra: (panelData, context) => {
      const buttons = [];
      if (panelData.parent.mode !== 'window') {
        buttons.push(
          <div
            className={
              panelData.parent.mode === 'maximize'
                ? 'dock-panel-min-btn'
                : 'dock-panel-max-btn'
            }
            key="maximize"
            title={
              panelData.parent.mode === 'maximize' ? 'Restore' : 'Maximize'
            }
            onClick={() => context.dockMove(panelData, null, 'maximize')}
          >
            {/* {panelData.parent.mode === 'maximize' ? '▬' : '▣'} */}
          </div>,
        );
      }
      buttons.push(
        <div
          className="dock-panel-close-btn"
          key="close"
          title="Close Panel"
          onClick={() => context.dockMove(panelData, null, 'remove')}
        />,
      );
      return <div className="dock-extra-content">{buttons}</div>;
    },
  },
};

const SAVEABLE_TABS = {
  HOSTS: 'hosts-tab',
  PACKAGES: 'packages-tab',
  PROVIDER: 'provider-tab',
  NODE_DETAILS: 'node-details-tab',
  PARAMETER: 'parameter-tab',
  TOPICS: 'topics-tab',
  SERVICES: 'services-tab',
  LOGGING: 'logging-tab',
};
var SAVEABLE_TABS_LIST = Object.keys(SAVEABLE_TABS).map(function (key) {
  return SAVEABLE_TABS[key];
});

const DEFAULT_LAYOUT = {
  dockbox: {
    mode: 'vertical',
    children: [
      {
        mode: 'horizontal',
        children: [
          {
            mode: 'vertical',
            children: [
              {
                tabs: [{ id: SAVEABLE_TABS.PROVIDER, title: 'Providers' }],
              },
              {
                tabs: [{ id: SAVEABLE_TABS.PACKAGES, title: 'Packages' }],
              },
              {
                tabs: [
                  { id: SAVEABLE_TABS.NODE_DETAILS, title: 'Node Details' },
                ],
              },
            ],
          },
          {
            tabs: [{ id: SAVEABLE_TABS.HOSTS, title: 'Hosts' }],
          },
        ],
      },
    ],
  },
};

const PANEL_SIZES = {
  parameter: { w: 720, h: 500 },
  screen: { w: 820, h: 450 },
  log: { w: 800, h: 550 },
  terminal: { w: 720, h: 480 },
  editor: { w: 1240, h: 720 },
  providers: { w: 800, h: 550 },
  'echo-topic': { w: 750, h: 550 },
};

function NodeManager() {
  // const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const dockLayoutRef = useRef(null);
  const [layoutComponents] = useState({});
  const [addToLayout, setAddToLayout] = useState([]);
  const [layoutSaved, setLayoutSaved] = useLocalStorage(
    'NodeManager:layout',
    DEFAULT_LAYOUT,
  );
  const [layout, setLayout] = useState(layoutSaved);
  const [layoutSizesAssigned] = useState([]);

  // rc-dock method to load a tab
  const loadTab = (data) => {
    const { id } = data;
    if (data.id in layoutComponents) {
      return layoutComponents[data.id];
    }
    let tab = null;
    switch (id) {
      case SAVEABLE_TABS.HOSTS:
        tab = {
          id: SAVEABLE_TABS.HOSTS,
          title: 'Hosts',
          closable: false,
          content: <HostTreeViewPanel key="host-panel" />,
          panelGroup: 'main',
        };
        break;
      case SAVEABLE_TABS.PACKAGES:
        tab = {
          id: SAVEABLE_TABS.PACKAGES,
          title: 'Packages',
          closable: false,
          content: <PackageExplorerPanel key="pkg-panel" />,
          panelGroup: 'explorer',
        };
        break;
      case SAVEABLE_TABS.PROVIDER:
        tab = {
          id: SAVEABLE_TABS.PROVIDER,
          title: 'Providers',
          closable: false,
          content: <ProviderPanel key="providers-panel" />,
          panelGroup: 'providers',
        };
        break;
      case SAVEABLE_TABS.NODE_DETAILS:
        tab = {
          id: SAVEABLE_TABS.NODE_DETAILS,
          title: (
            <div>
              Node Details <OverflowMenuNodeDetails />
            </div>
          ),
          closable: false,
          content: <NodesDetailsPanel key="details-panel" />,
          panelGroup: 'details',
        };
        break;
      case SAVEABLE_TABS.PARAMETER:
        tab = {
          id: SAVEABLE_TABS.PARAMETER,
          title: 'Parameter',
          closable: true,
          content: <ParameterPanel nodes={null} providers={null} />,
          panelGroup: 'main',
        };
        break;
      case SAVEABLE_TABS.TOPICS:
        tab = {
          id: SAVEABLE_TABS.TOPICS,
          title: 'Topics',
          closable: true,
          content: <TopicsPanel />,
          panelGroup: 'main',
        };
        break;
      case SAVEABLE_TABS.SERVICES:
        tab = {
          id: SAVEABLE_TABS.SERVICES,
          title: 'Services',
          closable: true,
          content: <ServicesPanel />,
          panelGroup: 'main',
        };
        break;
      case SAVEABLE_TABS.LOGGING:
        tab = {
          id: SAVEABLE_TABS.LOGGING,
          title: 'Logging',
          closable: true,
          content: <LoggingPanel />,
          panelGroup: 'main',
        };
        break;
      default:
        tab = null;
    }
    if (tab !== null) {
      layoutComponents[data.id] = tab;
    }
    return tab;
  };

  // searches for a panel which contains tabs which id starting with given prefix id
  const getPanelFromLayout = useCallback((idPrefix, subLayout) => {
    if (subLayout.children && idPrefix) {
      for (let i = 0; i < subLayout.children.length; i += 1) {
        const child = subLayout.children[i];
        if (child.children?.length > 0) {
          const found = getPanelFromLayout(idPrefix, child.children);
          if (found) {
            return found;
          }
        } else if (child.tabs?.length > 0) {
          for (let c = 0; c < child.tabs.length; c += 1) {
            const tab = child.tabs[c];
            if (tab.panelGroup?.localeCompare(idPrefix) === 0) {
              return child;
            }
          }
        }
      }
    }
    return null;
  }, []);

  const getSaveableTabs = useCallback((children) => {
    return children.filter((child) => {
      const newChild = {};
      Object.keys(child).forEach((key) => {
        if (key === 'children') {
          newChild[key] = getSaveableTabs(child.children);
        } else if (key === 'tabs') {
          newChild[key] = child.tabs.filter((tab) => {
            return Object.values(SAVEABLE_TABS).includes(tab.id);
          });
        } else {
          newChild[key] = child[key];
        }
      });
      return newChild.tabs?.length > 0 || newChild.children?.length > 0;
    });
  }, []);

  useEffect(() => {
    // save layout
    // copy the origin layout
    const newLayout = {};
    // remove all empty panels and not saveable tabs
    Object.keys(layout).forEach((boxKey) => {
      const newBox = {};
      Object.keys(layout[boxKey]).forEach((key) => {
        newBox[key] =
          key === 'children'
            ? getSaveableTabs(layout[boxKey][key])
            : layout[boxKey][key];
      });
      newLayout[boxKey] = newBox;
    });
    setLayoutSaved(newLayout);
  }, [getSaveableTabs, layout, setLayoutSaved]);

  useEffect(() => {
    if (settingsCtx.get('resetLayout')) {
      setLayoutSaved(DEFAULT_LAYOUT);
      setLayout(DEFAULT_LAYOUT);
      settingsCtx.set('resetLayout', false);
    }
  }, [settingsCtx, setLayoutSaved]);

  // checks if the panel contains only closable tabs
  const hasOnlyCloseableTabs = (panel) => {
    for (let c = 0; c < panel.tabs.length; c += 1) {
      if (!panel.tabs[c].closable) {
        return false;
      }
    }
    return true;
  };

  // searches for tab with given id in the given RC-dock layout.
  const findTab = (id, subLayout) => {
    if (subLayout.children && id) {
      for (let i = 0; i < subLayout.children.length; i += 1) {
        const child = subLayout.children[i];
        if (child.children?.length > 0) {
          for (let c = 0; c < child.children.length; c += 1) {
            const subChild = child.children[c];
            const tab = findTab(id, subChild);
            if (tab) {
              return tab;
            }
          }
        } else if (child.tabs?.length > 0) {
          const tab = findTab(id, child);
          if (tab) {
            return tab;
          }
        }
      }
    } else if (subLayout.tabs?.length > 0) {
      for (let c = 0; c < subLayout.tabs.length; c += 1) {
        const tab = subLayout.tabs[c];
        if (tab.id.localeCompare(id) === 0) {
          return tab;
        }
      }
    } else if (subLayout.dockbox) {
      let tab = null;
      const layoutKeys = Object.keys(subLayout); // boxKey: [dockbox, floatbox, maxbox, windowbox]
      for (let k = 0; k < layoutKeys.length; k += 1) {
        tab = findTab(id, subLayout[layoutKeys[k]]);
        if (tab) {
          return tab;
        }
      }
    }
    return null;
  };

  useCustomEventListener(EVENT_OPEN_COMPONENT, (data) => {
    if (data.id in SAVEABLE_TABS_LIST) {
      // activate already existing tab.
      if (dockLayoutRef.current) {
        dockLayoutRef.current.updateTab(data.id, loadTab(data));
      }
    } else if (data.id in layoutComponents) {
      // activate already existing tab.
      if (dockLayoutRef.current) {
        dockLayoutRef.current.updateTab(data.id, layoutComponents[data.id]);
      }
    } else {
      // create a new tab
      const tab = {
        id: data.id,
        title: data.title,
        closable: data.closable,
        content: data.component,
        panelGroup: data.panelGroup,
      };
      layoutComponents[data.id] = tab;
      // store new tabs using useEffect so dockMove() can create panels if events comes to fast
      setAddToLayout((oldValue) => [tab, ...oldValue]);
    }
  });

  // adds tabs to layout after event to create new tab was received and the 'addToLayout' was updated.
  useEffect(() => {
    if (addToLayout.length > 0) {
      const tab = addToLayout.pop();
      setAddToLayout([...addToLayout]);
      let panel = null;
      if (tab.panelGroup && dockLayoutRef.current) {
        panel = getPanelFromLayout(
          tab.panelGroup,
          dockLayoutRef.current.state.layout.dockbox,
        );
        if (!panel) {
          panel = getPanelFromLayout(
            tab.panelGroup,
            dockLayoutRef.current.state.layout.floatbox,
          );
        }
      }
      if (panel && hasOnlyCloseableTabs(panel)) {
        // add close button to panel, so we can close multiple tabs at once.
        // after this the tabs of this panel can not be added to 'main', 'details' or other panels.
        panel.group = 'close-all';
      }
      dockLayoutRef.current.dockMove(tab, panel, panel ? 'middle' : 'float');
    }
  }, [addToLayout, getPanelFromLayout]);

  // remove stored tabs on layout change
  const onLayoutChange = (newLayout, currentTabId, direction) => {
    if (direction === 'remove') {
      // remove saved tab from saved layout map
      delete layoutComponents[currentTabId];
      // if panel with multiple tabs is closed, we have to check and remove other tabs.
      Object.keys(layoutComponents).forEach((key) => {
        const tab = findTab(key, newLayout);
        if (!tab) {
          delete layoutComponents[key];
        }
      });
    }
    if (direction === 'move') {
      // TODO save size
    }
    // save layout
    setLayout(newLayout);
  };

  // TODO: change the size and position of the new float panels
  const afterPanelLoaded = (savedPanel, panelData) => {
    const panelGroup = panelData?.tabs[0]?.panelGroup;
    const panelId = `${panelGroup}-${panelData.id}`;
    if (!layoutSizesAssigned.includes(panelId)) {
      if (panelGroup in PANEL_SIZES) {
        panelData.w = PANEL_SIZES[panelGroup].w;
        panelData.h = PANEL_SIZES[panelGroup].h;
      }
      // TODO: set origin X/Y position of panel
      layoutSizesAssigned.push(panelId);
    }
    // console.log('panelData:', Object.getOwnPropertyNames(panelData));
    // console.log('panelData. id:', panelData.id);
    // console.log('panelData. w:', panelData.w);
    // console.log('panelData. h:', panelData.h);
    // console.log('panelData. size:', panelData.size);
    // console.log('panelData. group:', panelData.group);
    // console.log('panelData panelGroup:', panelData.tabs[0].panelGroup);
  };

  return (
    <div>
      <Stack height="100%" direction="row">
        <DockLayout
          key="node-manager-layout"
          // defaultLayout={layout}
          layout={layout}
          loadTab={loadTab}
          onLayoutChange={onLayoutChange}
          afterPanelLoaded={afterPanelLoaded}
          groups={PANEL_GROUPS}
          style={{
            position: 'absolute',
            left: 60,
            top: 2,
            right: 2,
            bottom: 2,
          }}
          ref={dockLayoutRef}
        />
      </Stack>
    </div>
  );
}

export default NodeManager;
