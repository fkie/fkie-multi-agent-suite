import { useContext } from "react";

import { IMonacoContext, MonacoContext } from "@/renderer/context/MonacoContext";

export function useMonacoContext(): IMonacoContext {
  const context = useContext(MonacoContext);

  if (!context) {
    throw new Error("useMonacoContext must be used inside MonacoProvider");
  }

  return context;
}
