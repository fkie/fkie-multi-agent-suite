import React, { createContext, useMemo, useState } from 'react';

export interface INavigationContext {
  selectedNodes: any[];
  setSelectedNodes?: (nodes: any[]) => void;
  selectedProviders: any[];
  setSelectedProviders?: (providers: any[]) => void;
}

export const DEFAULT = {
  selectedNodes: [],
  selectedProviders: [],
};

interface INavigationProvider {
  children: React.ReactNode;
}

export const NavigationContext = createContext<INavigationContext>(DEFAULT);

export function NavigationProvider({
  children,
}: INavigationProvider): ReturnType<React.FC<INavigationProvider>> {
  const [selectedNodes, setSelectedNodes] = useState<any[]>(
    DEFAULT.selectedNodes,
  );
  const [selectedProviders, setSelectedProviders] = useState<any[]>(
    DEFAULT.selectedProviders,
  );

  const attributesMemo = useMemo(
    () => ({
      selectedNodes,
      setSelectedNodes,
      selectedProviders,
      setSelectedProviders,
    }),
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [selectedNodes, selectedProviders],
  );

  return (
    <NavigationContext.Provider value={attributesMemo}>
      {children}
    </NavigationContext.Provider>
  );
}

export default NavigationContext;
