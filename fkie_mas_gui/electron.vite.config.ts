import react from "@vitejs/plugin-react";
import { defineConfig, externalizeDepsPlugin } from "electron-vite";
import { resolve } from "node:path";

export default defineConfig({
  main: {
    resolve: {
      alias: {
        "@": resolve("./src"),
        "@public": resolve("./src/renderer/assets")
      },
    },
    plugins: [externalizeDepsPlugin()],
  },
  preload: {
    resolve: {
      alias: {
        "@": resolve("./src"),
      },
    },
    plugins: [externalizeDepsPlugin()],
  },
  renderer: {
    resolve: {
      alias: {
        "@": resolve("./src"),
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
    },
    build: {
      rollupOptions: {
        input: {
          app: resolve("src/renderer/index.html"),
          editor: resolve("src/renderer/editor.html"),
          popout: resolve("src/renderer/popout.html"),
          publisher: resolve("src/renderer/publisher.html"),
          subscriber: resolve("src/renderer/subscriber.html"),
          terminal: resolve("src/renderer/terminal.html"),
        },
      },
    },
  },
});
