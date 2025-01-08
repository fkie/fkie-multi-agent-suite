/**
 * This module executes inside of electron's main process. You can start
 * electron renderer process from here and communicate with the other processes
 * through IPC.
 *
 * When running `npm run build` or `npm run build:main`, this file is compiled to
 * `./src/main.js` using webpack. This gives us some performance wins.
 */
import { electronApp, is, optimizer } from "@electron-toolkit/utils";
import { BrowserWindow, app, shell } from "electron";
import { join } from "path";
import { registerArguments } from "./CommandLineInterface";
import { AutoUpdateManager, DialogManager, ShutdownManager, registerHandlers } from "./IPC";
import MenuBuilder from "./menu";
import windowStateKeeper from "./windowStateKeeper";
// import installer from 'electron-devtools-installer'
// import electrondebug from 'electron-debug'

// Disable security warnings and set react app path on dev env
process.env.ELECTRON_DISABLE_SECURITY_WARNINGS = "true";

let mainWindow: BrowserWindow | null = null;
let autoUpdateManager: AutoUpdateManager | null = null;
let dialogManager: DialogManager | null = null;
let shutdownManager: ShutdownManager | null = null;

import * as sourceMap from "source-map-support";

if (process.env.NODE_ENV === "production") {
  sourceMap.install();
}

// app.disableHardwareAcceleration();
// app.disableDomainBlockingFor3DAPIs();

// const isDebug = process.env.NODE_ENV === 'development' || process.env.DEBUG_PROD === 'true'

// if (isDebug) {
//   // electrondebug()
// }

// const installExtensions = async (): Promise<string | void> => {
//   // const installer = require('electron-devtools-installer')
//   const forceDownload = !!process.env.UPGRADE_EXTENSIONS
//   const extensions = ['REACT_DEVELOPER_TOOLS']

//   return installer(
//     extensions.map((name) => installer[name]),
//     forceDownload
//   ).catch(console.log)
// }

const createWindow = async (): Promise<void> => {
  // if (isDebug) {
  //   await installExtensions()
  // }

  const mainWindowStateKeeper = await windowStateKeeper("main");

  mainWindow = new BrowserWindow({
    autoHideMenuBar: true,
    show: false,
    frame: true,
    x: mainWindowStateKeeper.x,
    y: mainWindowStateKeeper.y,
    width: mainWindowStateKeeper.width,
    height: mainWindowStateKeeper.height,
    icon: join(__dirname, "../../icon/mas.png"),
    webPreferences: {
      sandbox: false,
      nodeIntegration: true,
      preload: join(__dirname, "../preload/index.js"),
    },
  });
  // mainWindow.webContents.openDevTools();
  // Track window state
  mainWindowStateKeeper.track(mainWindow);
  dialogManager = new DialogManager(mainWindow);

  // Remove this if your app does not use auto updates
  autoUpdateManager = new AutoUpdateManager(mainWindow);

  // Handle app shutdown.
  shutdownManager = new ShutdownManager(mainWindow);

  mainWindow.on("ready-to-show", () => {
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

  mainWindow.on("close", (e) => {
    e.preventDefault();
    shutdownManager?.emitCloseAppRequest();
  });

  mainWindow.on("closed", () => {
    mainWindow = null;
  });

  const menuBuilder = new MenuBuilder(mainWindow);
  menuBuilder.buildMenu();

  // Open urls in the user's browser
  mainWindow.webContents.setWindowOpenHandler((data) => {
    if (data.url === "about:blank") {
      return {
        action: "allow",
        overrideBrowserWindowOptions: {
          frame: false,
          fullscreenable: true,
          backgroundColor: "black",
          webPreferences: {
            preload: join(__dirname, "../preload/index.js"),
          },
        },
      };
    }
    shell.openExternal(data.url);
    return { action: "deny" };
  });

  // HMR for renderer base on electron-vite cli.
  // Load the remote URL for development or the local html file for production.
  if (is.dev && process.env.ELECTRON_RENDERER_URL) {
    mainWindow.loadURL(process.env.ELECTRON_RENDERER_URL);
  } else {
    mainWindow.loadFile(join(__dirname, "../renderer/index.html"));
  }
};

/**
 * Event listeners
 */

app.on("window-all-closed", () => {
  // Respect the OSX convention of having the application in memory even
  // after all windows have been closed
  if (process.platform !== "darwin") {
    autoUpdateManager?.quit();
    dialogManager?.quit();
    app.quit();
  }
});

// register command line options
registerArguments();

// start app
app
  .whenReady()
  .then(() => {
    // Set app user model id for windows
    electronApp.setAppUserModelId("fkie.cms");

    // Default open or close DevTools by F12 in development
    // and ignore CommandOrControl + R in production.
    // see https://github.com/alex8088/electron-toolkit/tree/master/packages/utils
    app.on("browser-window-created", (_, window) => {
      optimizer.watchWindowShortcuts(window);
    });

    // register IPC callbacks
    registerHandlers();

    createWindow();

    app.on("activate", () => {
      // On macOS it's common to re-create a window in the app when the
      // dock icon is clicked and there are no other windows open.
      if (mainWindow === null) createWindow();
    });
  })
  .catch(console.log);
