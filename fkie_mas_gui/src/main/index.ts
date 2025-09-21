/**
 * This module executes inside of electron's main process. You can start
 * electron renderer process from here and communicate with the other processes
 * through IPC.
 *
 * When running `npm run build` or `npm run build:main`, this file is compiled to
 * `./src/main.js` using webpack. This gives us some performance wins.
 */
import { electronApp, is, optimizer } from "@electron-toolkit/utils";
import appIcon from "@public/mas.png?asset";
import { BrowserWindow, app, shell } from "electron";
import log from "electron-log";
import express from "express";
import path, { join } from "node:path";
import * as sourceMap from "source-map-support";
import { AutoUpdateManager, DialogManager, ShutdownManager, registerHandlers } from "./IPC";
import CommandExecutor, { updateDebianPackages } from "./IPC/CommandExecutor";
import CommandLine from "./IPC/CommandLine";
import MenuBuilder from "./menu";
import windowStateKeeper from "./windowStateKeeper";

let mainWindow: BrowserWindow | null = null;
let autoUpdateManager: AutoUpdateManager | null = null;
let dialogManager: DialogManager | null = null;
let shutdownManager: ShutdownManager | null = null;
const commandLine: CommandLine = new CommandLine();
const commandExecutor = new CommandExecutor(commandLine);
commandExecutor.registerHandlers();


const installUpdates = commandLine.getArg("update-debs") as boolean;
const installPrerelease = commandLine.getArg("update-debs-prerelease") as boolean;
const headless = commandLine.getArg("headless") as boolean;
const headlessServerPort = commandLine.getArg("headless-server-port") as number;

// Disable security warnings and set react app path on dev env
process.env.ELECTRON_DISABLE_SECURITY_WARNINGS = "true";

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

const startServer = async (): Promise<void> => {
  let dirPrefix = "/";

  if (__dirname.indexOf("/app.asar/") > 0) {
    dirPrefix = path.join(__dirname, "../renderer");
  } else {
    dirPrefix = path.join(__dirname, "../../dist/linux-unpacked/resources/app.asar/out/renderer");
  }

  // fs.readFileSync('/opt/mas-gui/resources/app.asar/file.txt')
  const serverApp = express();

  log.info("");
  log.info(`Listening on port: ${headlessServerPort}`);
  log.info("");

  serverApp.listen(headlessServerPort);

  serverApp.get("/", async (_req, res) => {
    res.sendFile(`${dirPrefix}/index.html`);
  });

  serverApp.use((req, res, next) => {
    const url = req.path;
    if (url.includes("/assets/cliArgs")) {
      // instead of env.js we send our desired env file
      res.type("text/javascript");
      res.send(`const CliArgs = {
  "hide-output-from-background-processes": { "default": ${commandLine.getArg("hide-output-from-background-processes")} },
  "headless": { "default": ${commandLine.getArg("headless")} },
  "headless-server-port": { "default": ${commandLine.getArg("headless-server-port")} },
  "update-debs": { "default": "", "fromEnv": "", "hint": "" },
  "update-debs-prerelease": { "default": "", "fromEnv": "", "hint": "" },
  "ros-version": { "default": "${commandLine.getArg("ros-version") || ""}" },
  "ros-domain-id": { "default": ${commandLine.getArg("ros-domain-id")} },
  "rmw-implementation": { "default": "${commandLine.getArg("rmw-implementation") || ""}" },
  "join": { "default": ${commandLine.getArg("join")} },
  "start": { "default": ${commandLine.getArg("start")} }
};
export {
  CliArgs as C
};
      `);
      return;
    }
    next();
  });
  serverApp.use(async (req, res) => {
    res.sendFile(`${dirPrefix}/${req.path}`);
  });
};

if (headless) {
  log.info("");
  log.info("run in headless mode!");
  startServer();
}

const createWindow = async (): Promise<void> => {
  if (installUpdates || installPrerelease) {
    log.info("update debian packages from github");
    if (installPrerelease) log.info("  -> use prerelease channel");
    await updateDebianPackages(installPrerelease);
    app.exit();
  }

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
    icon: appIcon,
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
  shutdownManager = new ShutdownManager(mainWindow, commandLine);

  mainWindow.on("ready-to-show", () => {
    if (!mainWindow) {
      throw new Error('"mainWindow" is not defined');
    }
    if (!headless) {
      if (process.env.START_MINIMIZED) {
        mainWindow.minimize();
      } else {
        if (mainWindowStateKeeper.isMaximized) mainWindow.maximize();
        mainWindow.show();
      }
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
