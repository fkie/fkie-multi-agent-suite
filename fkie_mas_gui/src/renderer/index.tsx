import 'core-js/modules/es.array.fill';
import 'core-js/modules/es.array.includes';
import 'core-js/modules/es.object.values';
import 'core-js/modules/es.string.includes';
import 'core-js/modules/es.string.trim';

// Add polyfills for backward compatibility with older browsers
import 'react-app-polyfill/ie11';
import 'react-app-polyfill/stable';

import './index.scss';

import { SnackbarProvider } from 'notistack';
import { createRoot } from 'react-dom/client';
import { BrowserRouter } from 'react-router-dom';
import App from './App';

import { LoggingProvider } from './context/LoggingContext';
import { MonacoProvider } from './context/MonacoContext';
import { NavigationProvider } from './context/NavigationContext';
import { RosProviderReact } from './context/RosContext';
import { SSHProvider } from './context/SSHContext';
import { SettingsProvider } from './context/SettingsContext';

const container = document.getElementById('root');
if (container) {
  const root = createRoot(container);
  root.render(
    <SettingsProvider>
      <SnackbarProvider
        maxSnack={4}
        autoHideDuration={4000}
        anchorOrigin={{
          vertical: 'bottom',
          horizontal: 'right',
        }}
        dense
        preventDuplicate
      >
        <LoggingProvider>
          <SSHProvider>
            <RosProviderReact>
              <NavigationProvider>
                <BrowserRouter>
                  {/* TODO: Monaco is not required to be global */}
                  <MonacoProvider>
                    <App />
                  </MonacoProvider>
                </BrowserRouter>
              </NavigationProvider>
            </RosProviderReact>
          </SSHProvider>
        </LoggingProvider>
      </SnackbarProvider>
    </SettingsProvider>,
  );
}
