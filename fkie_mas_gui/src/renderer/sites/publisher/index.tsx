// Add polyfills for backward compatibility with older browsers
import "react-app-polyfill/ie11";
import "react-app-polyfill/stable";
// imports
import ProviderStack from "@/renderer/ProviderStack";
import { MsgHistoryProvider } from "@/renderer/context/MsgHistoryContext";
import { SettingsProvider } from "@/renderer/context/SettingsContext";
import { createRoot } from "react-dom/client";
import PublisherApp from "./App";

const container = document.getElementById("root");
if (container) {
  const root = createRoot(container);
  root.render(
    <SettingsProvider>
      <ProviderStack>
        <MsgHistoryProvider>
          <PublisherApp />
        </MsgHistoryProvider>
      </ProviderStack>
    </SettingsProvider>
  );
}
