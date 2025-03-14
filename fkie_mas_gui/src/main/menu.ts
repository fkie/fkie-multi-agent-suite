import { BrowserWindow, Menu, MenuItemConstructorOptions, app, shell } from "electron";

// const path = require('path')

interface DarwinMenuItemConstructorOptions extends MenuItemConstructorOptions {
  selector?: string;
  submenu?: DarwinMenuItemConstructorOptions[] | Menu;
}

export default class MenuBuilder {
  mainWindow: BrowserWindow;

  constructor(mainWindow: BrowserWindow) {
    this.mainWindow = mainWindow;
  }

  buildMenu(): Menu {
    if (process.env.NODE_ENV === "development" || process.env.DEBUG_PROD === "true") {
      this.setupDevelopmentEnvironment();
    }

    const template = process.platform === "darwin" ? this.buildDarwinTemplate() : this.buildDefaultTemplate();

    const menu = Menu.buildFromTemplate(template);
    Menu.setApplicationMenu(menu);

    return menu;
  }

  setupDevelopmentEnvironment(): void {
    this.mainWindow.webContents.on("context-menu", (_, props) => {
      const { x, y } = props;

      Menu.buildFromTemplate([
        {
          label: "Inspect element",
          click: (): void => {
            this.mainWindow.webContents.inspectElement(x, y);
          },
        },
      ]).popup({ window: this.mainWindow });
    });
  }

  buildDarwinTemplate(): MenuItemConstructorOptions[] {
    const subMenuAbout: DarwinMenuItemConstructorOptions = {
      label: "Electron",
      submenu: [
        {
          label: "About ElectronReact",
          selector: "orderFrontStandardAboutPanel:",
        },
        { type: "separator" },
        { label: "Services", submenu: [] },
        { type: "separator" },
        {
          label: "Hide ElectronReact",
          accelerator: "Command+H",
          selector: "hide:",
        },
        {
          label: "Hide Others",
          accelerator: "Command+Shift+H",
          selector: "hideOtherApplications:",
        },
        { label: "Show All", selector: "unhideAllApplications:" },
        { type: "separator" },
        {
          label: "Quit",
          accelerator: "Command+Q",
          click: (): void => {
            app.quit();
          },
        },
      ],
    };
    const subMenuEdit: DarwinMenuItemConstructorOptions = {
      label: "Edit",
      submenu: [
        { label: "Undo", accelerator: "Command+Z", selector: "undo:" },
        { label: "Redo", accelerator: "Shift+Command+Z", selector: "redo:" },
        { type: "separator" },
        { label: "Cut", accelerator: "Command+X", selector: "cut:" },
        { label: "Copy", accelerator: "Command+C", selector: "copy:" },
        { label: "Paste", accelerator: "Command+V", selector: "paste:" },
        {
          label: "Select All",
          accelerator: "Command+A",
          selector: "selectAll:",
        },
      ],
    };
    const subMenuViewDev: MenuItemConstructorOptions = {
      label: "View",
      submenu: [
        {
          label: "Reload",
          accelerator: "Command+R",
          click: (): void => {
            this.mainWindow.webContents.reload();
          },
        },
        {
          label: "Toggle Full Screen",
          accelerator: "Ctrl+Command+F",
          click: (): void => {
            this.mainWindow.setFullScreen(!this.mainWindow.isFullScreen());
          },
        },
        {
          label: "Toggle Developer Tools",
          accelerator: "Alt+Command+I",
          click: (): void => {
            this.mainWindow.webContents.toggleDevTools();
          },
        },
      ],
    };
    const subMenuViewProd: MenuItemConstructorOptions = {
      label: "View",
      submenu: [
        {
          label: "Toggle Full Screen",
          accelerator: "Ctrl+Command+F",
          click: (): void => {
            this.mainWindow.setFullScreen(!this.mainWindow.isFullScreen());
          },
        },
      ],
    };
    const subMenuWindow: DarwinMenuItemConstructorOptions = {
      label: "Window",
      submenu: [
        {
          label: "Minimize",
          accelerator: "Command+M",
          selector: "performMiniaturize:",
        },
        { label: "Close", accelerator: "Command+W", selector: "performClose:" },
        { type: "separator" },
        { label: "Bring All to Front", selector: "arrangeInFront:" },
      ],
    };
    const subMenuHelp: MenuItemConstructorOptions = {
      label: "Help",
      submenu: [
        {
          label: "Learn More",
          click(): void {
            shell.openExternal("https://electronjs.org");
          },
        },
        {
          label: "Documentation",
          click(): void {
            shell.openExternal("https://github.com/electron/electron/tree/main/docs#readme");
          },
        },
        {
          label: "Community Discussions",
          click(): void {
            shell.openExternal("https://www.electronjs.org/community");
          },
        },
        {
          label: "Search Issues",
          click(): void {
            shell.openExternal("https://github.com/electron/electron/issues");
          },
        },
      ],
    };

    const subMenuView =
      process.env.NODE_ENV === "development" || process.env.DEBUG_PROD === "true" ? subMenuViewDev : subMenuViewProd;

    return [subMenuAbout, subMenuEdit, subMenuView, subMenuWindow, subMenuHelp];
  }

  buildDefaultTemplate(): MenuItemConstructorOptions[] {
    const templateDefault = [
      {
        label: "&File",
        submenu: [
          {
            label: "&Close",
            accelerator: "Ctrl+W",
            click: (): void => {
              this.mainWindow.close();
            },
          },
        ],
      },
      {
        label: "&View",
        submenu: [
          {
            label: "&Reload",
            accelerator: "F5",
            click: (): void => {
              this.mainWindow.webContents.reload();
            },
          },
          {
            label: "Toggle &Full Screen",
            accelerator: "F11",
            click: (): void => {
              this.mainWindow.setFullScreen(!this.mainWindow.isFullScreen());
            },
          },
          {
            label: "Toggle &Developer Tools",
            accelerator: "F12",
            click: (): void => {
              this.mainWindow.webContents.toggleDevTools();
            },
          },
        ],
      },
      {
        label: "Help",
        submenu: [
          {
            label: "Documentation \u{1F855}",
            click: (): void => {
              shell.openExternal("https://fkie.github.io/multimaster_fkie/");
            },
          },
          {
            label: "Search Issues \u{1F855}",
            click: (): void => {
              shell.openExternal("https://github.com/fkie/fkie-multi-agent-suite/issues");
            },
          },
          {
            label: "About \u{1F855}",
            click: (): void => {
              shell.openExternal("https://github.com/fkie/fkie-multi-agent-suite");
            },
          },
        ],
      },
    ];

    return templateDefault;
  }
}
