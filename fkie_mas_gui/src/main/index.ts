/**
 * This module executes inside of electron's main process. You can start
 * electron renderer process from here and communicate with the other processes
 * through IPC.
 *
 * When running `npm run build` or `npm run build:main`, this file is compiled to
 * `./src/main.js` using webpack. This gives us some performance wins.
 */
import { electronApp, is, optimizer } from "@electron-toolkit/utils";
import { BrowserWindow, app, dialog, ipcMain, shell } from "electron";
import { join } from "path";
import { registerArguments } from "./CommandLineInterface";
import { AutoUpdateManager, DialogManager, ShutdownInterface, registerHandlers } from "./IPC";
import MenuBuilder from "./menu";
import windowStateKeeper from "./windowStateKeeper";
// import installer from 'electron-devtools-installer'
// import electrondebug from 'electron-debug'

// Disable security warnings and set react app path on dev env
process.env.ELECTRON_DISABLE_SECURITY_WARNINGS = "true";

let mainWindow: BrowserWindow | null = null;
let autoUpdateManager: AutoUpdateManager | null = null;
let dialogManager: DialogManager | null = null;
let shutdownInterface: ShutdownInterface | null = null;

console.log(`process.env.NODE_ENV: ${process.env.NODE_ENV}`);

if (process.env.NODE_ENV === "production") {
  const sourceMapSupport = require("source-map-support");
  sourceMapSupport.install();
}

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

interface IEditor {
  window: BrowserWindow;
  changed: string[];
}

const editors = {};

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
    icon: join(__dirname, "../../icon/crystal_clear_app_clicknrun_256x256.png"),
    webPreferences: {
      sandbox: false,
      nodeIntegration: true,
      preload: join(__dirname, "../preload/index.js"),
    },
  });
  // Track window state
  mainWindowStateKeeper.track(mainWindow);

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
    mainWindow?.webContents.send("ShutdownInterface:terminateSubprocesses");
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

  dialogManager = new DialogManager(mainWindow);

  // Remove this if your app does not use auto updates
  autoUpdateManager = new AutoUpdateManager(mainWindow);

  // Handle app shutdown.
  shutdownInterface = new ShutdownInterface(mainWindow);
  // ShutdownInterface
  ipcMain.handle("ShutdownInterface:quitGui", () => {
    return shutdownInterface?.quitGui();
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

ipcMain.handle("editor:has", handleHasEditor);
ipcMain.handle("editor:open", handleEditorOpen);
ipcMain.handle("editor:changed", handleEditorChanged);
ipcMain.handle("editor:emitFileRange", handleEditorFileRange);

async function handleHasEditor(
  _event: Electron.IpcMainInvokeEvent,
  host: string,
  port: number,
  rootLaunch: string
): Promise<boolean> {
  const id = `${host}-${port}-${rootLaunch}`;
  if (editors[id]) {
    return Promise.resolve(true);
  }
  return Promise.resolve(false);
}

async function handleEditorFileRange(
  _event: Electron.IpcMainInvokeEvent,
  launchFile: string,
  host: string,
  port: number,
  rootLaunch: string,
  fileRange: { startLineNumber: number; endLineNumber: number; startColumn: number; endColumn: number }
): Promise<null> {
  const id = `${host}-${port}-${rootLaunch}`;
  if (editors[id]) {
    editors[id].window.focus();
    editors[id].window.webContents.send("editor-file-range", id, launchFile, fileRange);
    return Promise.resolve(null);
  }
}

async function handleEditorChanged(
  _event: Electron.IpcMainInvokeEvent,
  host: string,
  port: number,
  rootLaunch: string,
  launchFile: string,
  changed: boolean
): Promise<boolean> {
  const id = `${host}-${port}-${rootLaunch}`;
  if (editors[id]) {
    if (editors[id].changed.includes(launchFile)) {
      if (!changed) {
        editors[id].changed = editors[id].changed.filter((item) => item !== rootLaunch);
      }
    } else if (changed) {
      editors[id].changed.push(rootLaunch);
    }
    return Promise.resolve(true);
  }
  return Promise.resolve(false);
}

async function handleEditorOpen(
  _event: Electron.IpcMainInvokeEvent,
  launchFile: string,
  host: string,
  port: number,
  rootLaunch: string,
  fileRange: { startLineNumber: number; endLineNumber: number; startColumn: number; endColumn: number }
): Promise<string | null> {
  // if (isDebug) {
  //   await installExtensions()
  // }
  const id = `${host}-${port}-${rootLaunch}`;
  if (editors[id]) {
    editors[id].window.focus();
    editors[id].window.webContents.send("editor-file-range", id, launchFile, fileRange);
    return Promise.resolve(null);
  }

  const mainWindowStateKeeper = await windowStateKeeper("editor");

  const newWindow = new BrowserWindow({
    autoHideMenuBar: true,
    show: false,
    frame: true,
    x: mainWindowStateKeeper.x,
    y: mainWindowStateKeeper.y,
    width: mainWindowStateKeeper.width,
    height: mainWindowStateKeeper.height,
    icon: join(__dirname, "../../icon/crystal_clear_edit_launch.png"),
    webPreferences: {
      sandbox: false,
      nodeIntegration: true,
      preload: join(__dirname, "../preload/index.js"),
    },
  });
  editors[id] = { window: newWindow, changed: [] };
  // Track window state
  mainWindowStateKeeper.track(newWindow);

  newWindow.on("ready-to-show", () => {
    if (!newWindow) {
      throw new Error('"mainWindow" is not defined');
    }
    if (process.env.START_MINIMIZED) {
      newWindow.minimize();
    } else {
      if (mainWindowStateKeeper.isMaximized) newWindow.maximize();
      newWindow.show();
    }
  });

  newWindow.on("close", (e) => {
    if (editors[id]?.changed?.length > 0) {
      e.preventDefault();
      const doFocus = dialog
        .showMessageBox(mainWindow, {
          type: "question",
          title: "Save changes",
          message: `Changed files: ${JSON.stringify(editors[id].changed)}`,
          buttons: ["Don't save", "Cancel", "TODO: Save All"],
        })
        .then((result) => {
          if (result.response === 0) {
            editors[id].window.destroy();
            delete editors[id];
            return false;
          } else if (result.response === 2) {
            console.log(`todo: save`);
          } else {
          }
          return true;
        })
        .catch((err) => {
          console.log(err);
          return false;
        });
      if (doFocus) {
        editors[id].window.show();
        editors[id].window.focus();
      }
    }
  });

  newWindow.on("closed", () => {
    delete editors[id];
  });

  dialogManager = new DialogManager(newWindow);

  // HMR for renderer base on electron-vite cli.
  // Load the remote URL for development or the local html file for production.
  if (is.dev && process.env.ELECTRON_RENDERER_URL) {
    newWindow.loadURL(
      `${process.env.ELECTRON_RENDERER_URL}/editor.html?path=${launchFile}&host=${host}&port=${port}&root=${rootLaunch}&sl=${fileRange.startLineNumber}&el=${fileRange.endLineNumber}&sc=${fileRange.startColumn}&ec=${fileRange.endColumn}`
    );
  } else {
    newWindow.loadFile(join(__dirname, `../renderer/editor.html`), {
      query: {
        path: launchFile,
        host: host,
        port: `${port}`,
        root: rootLaunch,
        sl: `${fileRange.startLineNumber}`,
        el: `${fileRange.endLineNumber}`,
        sc: `${fileRange.startColumn}`,
        ec: `${fileRange.endColumn}`,
      },
    });
  }
  return Promise.resolve(null);
}
