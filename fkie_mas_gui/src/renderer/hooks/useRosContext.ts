import { useContext } from "react";

import { IRosContext, RosContext } from "@/renderer/context/RosContext";

export function useRosContext(): IRosContext {
  const context = useContext(RosContext);

  if (!context) {
    throw new Error("useRosContext must be used inside RosProvider");
  }

  return context;
}
