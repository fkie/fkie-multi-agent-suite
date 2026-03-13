import { createContext, useEffect, useMemo } from "react";

import { useMonacoContext } from "../hooks/useMonacoContext";
import { useRefContext } from "../hooks/useRefContext";
import { useRosContext } from "../hooks/useRosContext";
import { disposeMonacoRuntime, initMonacoRuntime } from "../monaco/setup";
import { IMonacoContext } from "./MonacoContext";
import { IRosContext } from "./RosContext";

export interface IMonacoInitContext {
  monacoCtx: IMonacoContext;
}

export const MonacoInitContext = createContext<IMonacoInitContext | null>(null);

// -------------------- MonacoProvider --------------------
export function MonacoInitProvider({ children }: { children: React.ReactNode }) {
  const monacoCtx = useMonacoContext();
  const rosCtx = useRosContext();

  const rosCtxRef = useRefContext<IRosContext>(rosCtx);
  const monacoCtxRef = useRefContext<IMonacoContext>(monacoCtx);

  useEffect(() => {
    initMonacoRuntime(monacoCtxRef, rosCtxRef);
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
    }),
    [monacoCtx]
  );

  return <MonacoInitContext.Provider value={contextValue}>{children}</MonacoInitContext.Provider>;
}
