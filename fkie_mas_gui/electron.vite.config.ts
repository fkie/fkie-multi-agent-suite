import { resolve } from "node:path";
import react from "@vitejs/plugin-react";
import { defineConfig, externalizeDepsPlugin } from "electron-vite";

import cliArgs from "./src/renderer/assets/cliArgs.json";

function getDefaultHeadlessPort(): number {
  return Number(cliArgs["headless-server-port"]?.default ?? 6275);
}

function getHeadlessPort(): number {
  const defaultPort = getDefaultHeadlessPort();
  const idx = process.argv.findIndex((a) => a.startsWith("--headless-server-port"));
  if (idx !== -1) {
    // Format: --headless-server-port=1234 or --headless-server-port 1234
    const val = process.argv[idx].includes("=") ? process.argv[idx].split("=")[1] : process.argv[idx + 1];
    return Number.parseInt(val) || defaultPort;
  }
  return defaultPort; // default
}

export default defineConfig({
  main: {
    resolve: {
      alias: {
        "@": resolve(__dirname, "src"),
        "@public": resolve("./src/renderer/assets"),
      },
    },
    plugins: [externalizeDepsPlugin()],
  },
  preload: {
    resolve: {
      alias: {
        "@": resolve(__dirname, "src"),
      },
    },
    plugins: [externalizeDepsPlugin()],
  },
  renderer: {
    resolve: {
      alias: {
        "@": resolve(__dirname, "src"),
      },
    },
    optimizeDeps: {
      include: ["@emotion/react", "@emotion/styled", "@mui/material"],
    },
    plugins: [
      react({
        // jsxImportSource: "@emotion/react",
        include: "**/*.tsx",
      }),
    ],
    server: {
      port: 6274,
      host: true,
      proxy: {
        "/api": {
          target: `http://localhost:${getHeadlessPort()}`,
          changeOrigin: true,
        },
      },
    },
    build: {
      rollupOptions: {
        input: {
          app: resolve("src/renderer/index.html"),
          editor: resolve("src/renderer/editor.html"),
          actionIntrospection: resolve("src/renderer/actionIntrospection.html"),
          actionSendGoal: resolve("src/renderer/actionSendGoal.html"),
          publisher: resolve("src/renderer/publisher.html"),
          serviceCaller: resolve("src/renderer/serviceCaller.html"),
          serviceIntrospection: resolve("src/renderer/serviceIntrospection.html"),
          subscriber: resolve("src/renderer/subscriber.html"),
          terminal: resolve("src/renderer/terminal.html"),
          cliArgs: resolve("src/renderer/assets/cliArgs.json"),
        },
      },
    },
  },
});
