// Add polyfills for backward compatibility with older browsers
import "react-app-polyfill/ie11";
import "react-app-polyfill/stable";
// imports
import ProviderStack from "@/renderer/ProviderStack";
import { MonacoProvider } from "@/renderer/context/MonacoContext";
import { SettingsProvider } from "@/renderer/context/SettingsContext";
import { createRoot } from "react-dom/client";
import EditorApp from "./App";

const container = document.getElementById("root");
if (container) {
  const root = createRoot(container);
  root.render(
    <SettingsProvider>
      <ProviderStack>
        <MonacoProvider>
          <EditorApp />
        </MonacoProvider>
      </ProviderStack>
    </SettingsProvider>
  );
}
