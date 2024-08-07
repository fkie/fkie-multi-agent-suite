import react from "@vitejs/plugin-react";
import { defineConfig, externalizeDepsPlugin } from "electron-vite";
import { resolve } from "path";

export default defineConfig({
  main: {
    plugins: [externalizeDepsPlugin()],
  },
  preload: {
    plugins: [externalizeDepsPlugin()],
  },
  renderer: {
    resolve: {
      alias: {
        "@renderer": resolve("src/renderer/"),
      },
    },
    optimizeDeps: {
      include: ["@emotion/react", "@emotion/styled"],
    },
    plugins: [
      react({
        jsxImportSource: "@emotion/react",
      }),
    ],
    server: {
      port: 6274,
      host: true,
    },
    build: {
      rollupOptions: {
        input: {
          app: resolve(__dirname, "src/renderer/index.html"),
          editor: resolve(__dirname, "src/renderer/editor.html"),
        },
      },
    },
  },
});
