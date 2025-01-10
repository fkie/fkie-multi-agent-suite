import React, { createContext, useMemo, useState } from "react";

export interface INavigationContext {
  selectedNodes: string[];
  setSelectedNodes: (nodes: string[]) => void;
  selectedProviders: string[];
  setSelectedProviders: (providers: string[]) => void;
}

export const DEFAULT = {
  selectedNodes: [],
  selectedProviders: [],
  modifiedFiles: [],
  setSelectedNodes: (): void => {},
  setSelectedProviders: (): void => {},
};

interface INavigationProvider {
  children: React.ReactNode;
}

export const NavigationContext = createContext<INavigationContext>(DEFAULT);

export function NavigationProvider({ children }: INavigationProvider): ReturnType<React.FC<INavigationProvider>> {
  const [selectedNodes, setSelectedNodes] = useState<string[]>(DEFAULT.selectedNodes);
  const [selectedProviders, setSelectedProviders] = useState<string[]>(DEFAULT.selectedProviders);

  const attributesMemo = useMemo(
    () => ({
      selectedNodes,
      setSelectedNodes,
      selectedProviders,
      setSelectedProviders,
    }),
    [selectedNodes, selectedProviders]
  );

  return <NavigationContext.Provider value={attributesMemo}>{children}</NavigationContext.Provider>;
}

export default NavigationContext;
