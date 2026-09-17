import { useContext } from "react";
import { CliArgsContext, ICliArgsContext } from "@/renderer/context/CliArgsContext";

export function useCliArgs(): ICliArgsContext {
  const context = useContext(CliArgsContext);

  if (!context) {
    throw new Error("useCliArgs must be used inside CliArgsProvider");
  }

  return context;
}
