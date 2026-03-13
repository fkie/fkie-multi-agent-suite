import { useContext } from "react";

import { IMonacoInitContext, MonacoInitContext } from "../context/MonacoInitContext";

export function useMonacoInitContext(): IMonacoInitContext {
  const context = useContext(MonacoInitContext);

  if (!context) {
    throw new Error("useMonacoInitContext must be used inside MonacoInitProvider");
  }

  return context;
}
