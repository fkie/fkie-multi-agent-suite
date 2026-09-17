// Add polyfills for backward compatibility with older browsers
import "react-app-polyfill/ie11";
import "react-app-polyfill/stable";
import { createRoot } from "react-dom/client";
import { LoadingScreen } from "@/renderer/components/loading/LoadingScreen";
import { PersistenceGate } from "@/renderer/components/loading/PersistenceGate";
import { AppStateProvider } from "@/renderer/context/AppStateContext";
import { SettingsProvider } from "@/renderer/context/SettingsContext";
// imports
import ProviderStack from "@/renderer/ProviderStack";
import ServiceCallerApp from "./App";

const container = document.getElementById("root");
if (container) {
  const root = createRoot(container);
  root.render(
    <SettingsProvider>
      <AppStateProvider>
        <ProviderStack>
          <PersistenceGate fallback={<LoadingScreen message="Loading settings..." />}>
            <ServiceCallerApp />
          </PersistenceGate>
        </ProviderStack>
      </AppStateProvider>
    </SettingsProvider>
  );
}
