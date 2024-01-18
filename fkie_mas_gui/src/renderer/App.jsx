import { useWindowHeight } from '@react-hook/window-size/throttled';
import { useContext, useEffect } from 'react';
import {
  unstable_HistoryRouter as HistoryRouter,
  Route,
  Routes,
} from 'react-router-dom';

import DesktopWindowsOutlinedIcon from '@mui/icons-material/DesktopWindowsOutlined';
import FeaturedPlayListIcon from '@mui/icons-material/FeaturedPlayList';
import TopicIcon from '@mui/icons-material/Topic';
import TuneIcon from '@mui/icons-material/Tune';
import { emitCustomEvent } from 'react-custom-events';
// https://github.com/azouaoui-med/react-pro-sidebar/blob/master/storybook/Playground.tsx
import { Badge, Box, CssBaseline, Divider, Tooltip } from '@mui/material';
import { Menu, MenuItem, Sidebar } from 'react-pro-sidebar';

import { ThemeProvider } from '@mui/material/styles';

import { createHashHistory } from 'history';
import ExternalAppsModal from './components/ExternalAppsModal/ExternalAppsModal';
import SettingsModal from './components/SettingsModal/SettingsModal';
import { LoggingContext } from './context/LoggingContext';
import { RosContext } from './context/RosContext';
import { SettingsContext } from './context/SettingsContext';
import About from './pages/About/About';
import NodeManager from './pages/NodeManager/NodeManager';
import LoggingPanel from './pages/NodeManager/panels/LoggingPanel';
import ParameterPanel from './pages/NodeManager/panels/ParameterPanel';
import ServicesPanel from './pages/NodeManager/panels/ServicesPanel';
import TopicsPanel from './pages/NodeManager/panels/TopicsPanel';

// load default style for rc-dock. Dark/Light theme changes are in ./themes
import 'rc-dock/dist/rc-dock.css';
import './App.scss';
import { darkTheme, lightTheme } from './themes';
import { EVENT_OPEN_COMPONENT, eventOpenComponent } from './utils/events';

export const history = createHashHistory();

export default function App() {
  const settingsCtx = useContext(SettingsContext);
  const logCtx = useContext(LoggingContext);
  const rosCtx = useContext(RosContext);
  const windowHeight = useWindowHeight();

  const handleWindowError = (e) => {
    // fix "ResizeObserver loop limit exceeded" while change size of the editor
    if (e.message === 'ResizeObserver loop limit exceeded') {
      const resizeObserverErrDiv = document.getElementById(
        'webpack-dev-server-client-overlay-div',
      );
      const resizeObserverErr = document.getElementById(
        'webpack-dev-server-client-overlay',
      );
      if (resizeObserverErr) {
        resizeObserverErr.setAttribute('style', 'display: none');
      }
      if (resizeObserverErrDiv) {
        resizeObserverErrDiv.setAttribute('style', 'display: none');
      }
    }
  };

  useEffect(() => {
    // Anything in here is fired on component mount.
    window.addEventListener('error', handleWindowError);
    return () => {
      // Anything in here is fired on component unmount.
      window.removeEventListener('error', handleWindowError);
    };
  }, []);

  return (
    <ThemeProvider
      theme={settingsCtx.get('useDarkMode') ? darkTheme : lightTheme}
    >
      <HistoryRouter history={history}>
        <Box
          style={{
            height: windowHeight,
            width: '100%',
          }}
        >
          <CssBaseline />
          <div style={{ display: 'flex', height: '100%' }}>
            <Routes>
              <Route path="/" element={<NodeManager />} />
              {/* <Route path="/editor" element={<NodeManager />} /> */}
              <Route path="/about" element={<About />} />
            </Routes>

            <Sidebar
              collapsed
              collapsedWidth="58px"
              backgroundColor={
                settingsCtx.get('useDarkMode')
                  ? darkTheme.backgroundColor
                  : lightTheme.backgroundColor
              }
              style={{ height: '100%', borderRightWidth: 0 }}
            >
              <Menu
                menuItemStyles={{
                  button: {
                    '&:hover': {
                      backgroundColor: settingsCtx.get('useDarkMode')
                        ? '#ffffff14'
                        : '#0000000A',
                    },
                    padding: '0.7em',
                    color: settingsCtx.get('useDarkMode')
                      ? '#fff'
                      : '#0000008A',
                  },
                }}
              >
                <Tooltip title="Topics" placement="right">
                  <span>
                    <MenuItem
                      onClick={() =>
                        emitCustomEvent(
                          EVENT_OPEN_COMPONENT,
                          eventOpenComponent(
                            'topics-tab',
                            `Topics`,
                            <TopicsPanel />,
                            false,
                            true,
                            'main',
                          ),
                        )
                      }
                      icon={<TopicIcon sx={{ fontSize: 30 }} />}
                    >
                      Topics
                    </MenuItem>
                  </span>
                </Tooltip>
                <Tooltip title="Services" placement="right">
                  <span>
                    <MenuItem
                      onClick={() =>
                        emitCustomEvent(
                          EVENT_OPEN_COMPONENT,
                          eventOpenComponent(
                            'services-tab',
                            `Services`,
                            <ServicesPanel />,
                            false,
                            true,
                            'main',
                          ),
                        )
                      }
                      icon={<FeaturedPlayListIcon sx={{ fontSize: 28 }} />}
                    >
                      Services
                    </MenuItem>
                  </span>
                </Tooltip>
                <Tooltip title="Parameter" placement="right">
                  <span>
                    <MenuItem
                      onClick={() =>
                        emitCustomEvent(
                          EVENT_OPEN_COMPONENT,
                          eventOpenComponent(
                            'parameter-tab',
                            `Parameter`,
                            <ParameterPanel nodes={null} />,
                            false,
                            true,
                            'main',
                          ),
                        )
                      }
                      icon={<TuneIcon sx={{ fontSize: 28 }} />}
                    >
                      Parameter
                    </MenuItem>
                  </span>
                </Tooltip>
                <Tooltip title="Logging" placement="right">
                  <span>
                    <MenuItem
                      onClick={() =>
                        emitCustomEvent(
                          EVENT_OPEN_COMPONENT,
                          eventOpenComponent(
                            'logging-tab',
                            `Logging`,
                            <LoggingPanel />,
                            false,
                            true,
                            'main',
                          ),
                        )
                      }
                      icon={
                        <Badge
                          color="info"
                          badgeContent={`${logCtx.countErrors}`}
                          invisible={logCtx.countErrors === 0}
                          // variant="standard"
                          // anchorOrigin="top"
                        >
                          <DesktopWindowsOutlinedIcon sx={{ fontSize: 28 }} />
                        </Badge>
                      }
                    >
                      Logging
                    </MenuItem>
                  </span>
                </Tooltip>
                <Divider orientation="horizontal" />

                {/* Only show External apps modal, if IPC back-end is available */}
                {window.CommandExecutor && <ExternalAppsModal />}

                <SettingsModal />
              </Menu>
            </Sidebar>
          </div>
        </Box>
      </HistoryRouter>
    </ThemeProvider>
  );
}
