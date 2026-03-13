import { useContext } from "react";

import { ISettingsContext, SettingsContext } from "@/renderer/context/SettingsContext";

export function useSettingsContext(): ISettingsContext {
  const context = useContext(SettingsContext);

  if (!context) {
    throw new Error("useSettingsContext must be used inside SettingsProvider");
  }

  return context;
}
