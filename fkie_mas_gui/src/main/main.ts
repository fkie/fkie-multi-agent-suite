/* eslint global-require: off, no-console: off, promise/always-return: off */

/**
 * This module executes inside of electron's main process. You can start
 * electron renderer process from here and communicate with the other processes
 * through IPC.
 *
 * When running `npm run build` or `npm run build:main`, this file is compiled to
 * `./src/main.js` using webpack. This gives us some performance wins.
 */
import { app, BrowserWindow, shell } from 'electron';
import path from 'path';
import { registerArguments } from './CommandLineInterface';
import { AutoUpdateManager, registerHandlers } from './IPC';
import MenuBuilder from './menu';
import { resolveHtmlPath } from './util';
import windowStateKeeper from './windowStateKeeper';

// Disable security warnings and set react app path on dev env
process.env.ELECTRON_DISABLE_SECURITY_WARNINGS = 'true';

let mainWindow: BrowserWindow | null = null;
let autoUpdateManager: AutoUpdateManager | null = null;

if (process.env.NODE_ENV === 'production') {
  const sourceMapSupport = require('source-map-support');
  sourceMapSupport.install();
}

const isDebug =
  process.env.NODE_ENV === 'development' || process.env.DEBUG_PROD === 'true';

if (isDebug) {
  require('electron-debug')();
}

const installExtensions = async () => {
  const installer = require('electron-devtools-installer');
  const forceDownload = !!process.env.UPGRADE_EXTENSIONS;
  const extensions = ['REACT_DEVELOPER_TOOLS'];

  return installer
    .default(
      extensions.map((name) => installer[name]),
      forceDownload,
    )
    .catch(console.log);
};

const createWindow = async () => {
  if (isDebug) {
    await installExtensions();
  }

  const RESOURCES_PATH = app.isPackaged
    ? path.join(process.resourcesPath, 'assets')
    : path.join(__dirname, '../../assets');

  const getAssetPath = (...paths: string[]): string => {
    return path.join(RESOURCES_PATH, ...paths);
  };

  const mainWindowStateKeeper = await windowStateKeeper('main');

  mainWindow = new BrowserWindow({
    show: false,
    frame: true,
    x: mainWindowStateKeeper.x,
    y: mainWindowStateKeeper.y,
    width: mainWindowStateKeeper.width,
    height: mainWindowStateKeeper.height,
    icon: getAssetPath('icon.png'),
    webPreferences: {
      sandbox: false,
      nodeIntegration: true,
      preload: app.isPackaged
        ? path.join(__dirname, 'preload.js')
        : path.join(__dirname, '../../.erb/dll/preload.js'),
    },
  });
  // Track window state
  mainWindowStateKeeper.track(mainWindow);

  mainWindow.loadURL(resolveHtmlPath('index.html'));

  mainWindow.on('ready-to-show', () => {
    if (!mainWindow) {
      throw new Error('"mainWindow" is not defined');
    }
    if (process.env.START_MINIMIZED) {
      mainWindow.minimize();
    } else {
      if (mainWindowStateKeeper.isMaximized) mainWindow.maximize();
      mainWindow.show();
    }
  });

  mainWindow.on('closed', () => {
    mainWindow = null;
  });

  const menuBuilder = new MenuBuilder(mainWindow);
  menuBuilder.buildMenu();

  // Open urls in the user's browser
  mainWindow.webContents.setWindowOpenHandler((data) => {
    shell.openExternal(data.url);
    return { action: 'deny' };
  });

  // Remove this if your app does not use auto updates
  autoUpdateManager = new AutoUpdateManager(mainWindow);
};

/**
 * Event listeners
 */

app.on('window-all-closed', () => {
  // Respect the OSX convention of having the application in memory even
  // after all windows have been closed
  if (process.platform !== 'darwin') {
    app.quit();
  }
});

// register command line options
registerArguments();

// start app
app
  .whenReady()
  .then(() => {
    // register IPC callbacks
    registerHandlers();

    createWindow();
    app.on('activate', () => {
      // On macOS it's common to re-create a window in the app when the
      // dock icon is clicked and there are no other windows open.
      if (mainWindow === null) createWindow();
    });
  })
  .catch(console.log);
