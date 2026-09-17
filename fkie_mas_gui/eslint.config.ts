import js from "@eslint/js";
import { defineConfig, globalIgnores } from "eslint/config";
import pluginReact from "eslint-plugin-react";
import reactHooks from "eslint-plugin-react-hooks";
import globals from "globals";
import tseslint from "typescript-eslint";

export default defineConfig([
  // --- Global ignores (replaces the standalone "ignores" object) ---
  globalIgnores(["out/**", "dist/**", "build/**", "resources/**"]),

  // --- Base rules for all sources we own ---
  {
    files: ["src/**/*.{ts,tsx,js,jsx}", "*.config.ts"],
    extends: [js.configs.recommended, tseslint.configs.recommended],
    rules: {
      // Formatting is owned by Biome - ESLint stays on correctness only
      "@typescript-eslint/no-unused-vars": [
        "error",
        { argsIgnorePattern: "^_", varsIgnorePattern: "^_", caughtErrorsIgnorePattern: "^_" },
      ],
      "@typescript-eslint/no-explicit-any": "warn",
    },
  },

  // --- Electron main + preload: Node environment ---
  {
    files: ["src/main/**/*.{ts,tsx}", "src/preload/**/*.{ts,tsx}", "*.config.ts"],
    languageOptions: { globals: globals.node },
  },

  // --- Renderer: browser environment + React ---
  {
    files: ["src/renderer/**/*.{ts,tsx}"],
    extends: [pluginReact.configs.flat.recommended, pluginReact.configs.flat["jsx-runtime"]],
    languageOptions: { globals: globals.browser },
    settings: { react: { version: "detect" } },
    plugins: { "react-hooks": reactHooks },
    rules: {
      // THE missing piece: actually enable the hook rules
      ...reactHooks.configs.recommended.rules,
      // TypeScript already covers prop validation
      "react/prop-types": "off",
    },
  },
]);
