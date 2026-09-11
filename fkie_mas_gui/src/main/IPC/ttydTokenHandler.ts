import { ipcMain, net } from "electron";

export function registerTtydTokenHandler(): void {
  ipcMain.handle("ttyd:fetchToken", async (_event, url: string): Promise<string> => {
    try {
      const response = await net.fetch(url, { headers: { Accept: "application/json" } });
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      const json = (await response.json()) as { token?: string };
      return json.token ?? "";
    } catch (error) {
      console.warn("[ttyd] main process token request failed:", error);
      return "";
    }
  });
}
