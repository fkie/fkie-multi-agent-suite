import { useContext } from "react";

import NavigationContext, { INavigationContext } from "../context/NavigationContext";

export function useNavigationContext(): INavigationContext {
  const context = useContext(NavigationContext);

  if (!context) {
    throw new Error("useNavigationContext must be used inside NavigationProvider");
  }

  return context;
}
