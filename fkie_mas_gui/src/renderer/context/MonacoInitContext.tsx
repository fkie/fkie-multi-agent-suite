import { createContext, useEffect, useMemo, useState } from "react";

import { useAlwaysCurrentRef } from "../hooks/useAlwaysCurrentRef";
import { useMonacoContext } from "../hooks/useMonacoContext";
import { useRosContext } from "../hooks/useRosContext";
import { disposeMonacoRuntime, initMonacoRuntime } from "../monaco/setup";
import { IMonacoContext } from "./MonacoContext";
import { IRosContext } from "./RosContext";

export interface IMonacoInitContext {
  monacoCtx: IMonacoContext;
  initialized: boolean;
}

export const MonacoInitContext = createContext<IMonacoInitContext | null>(null);

// -------------------- MonacoProvider --------------------
export function MonacoInitProvider({ children }: { children: React.ReactNode }) {
  const monacoCtx = useMonacoContext();
  const rosCtx = useRosContext();

  const [initialized, setInitialized] = useState(false);
  const rosCtxRef = useAlwaysCurrentRef<IRosContext>(rosCtx);
  const monacoCtxRef = useAlwaysCurrentRef<IMonacoContext>(monacoCtx);

  useEffect(() => {

    setInitialized(initMonacoRuntime(monacoCtxRef, rosCtxRef));
  }, [monacoCtxRef.current, rosCtxRef.current]);

  useEffect(() => {
    return () => {
      console.log("Destroy global monaco disposables");
      // remove all disposables if MonacoInitProvider will be destroyed
      disposeMonacoRuntime();
    };
  }, []);

  // -------------------- Context Value --------------------
  const contextValue: IMonacoInitContext = useMemo(
    () => ({
      monacoCtx,
      initialized,
    }),
    [monacoCtx, initialized]
  );

  return <MonacoInitContext.Provider value={contextValue}>{children}</MonacoInitContext.Provider>;
}
