import react from "@vitejs/plugin-react";
import { resolve } from "path";
import { defineConfig } from "vite";

// https://vitejs.dev/config/
export default defineConfig({
  root: resolve(__dirname, "src/renderer/"),
  resolve: {
    alias: {
      "@": resolve(__dirname, "src/"),
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
});
