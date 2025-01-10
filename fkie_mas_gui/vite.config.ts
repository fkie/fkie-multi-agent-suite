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
  server: {
    port: 6274,
    host: true,
  },
});
