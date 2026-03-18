import { RefObject, useEffect, useRef, useState } from "react";

import useLocalStorage from "../useLocalStorage";
import { useSettingsContext } from "../useSettingsContext";

interface UseEditorSidebarProps {
  panelRef: RefObject<HTMLDivElement>;
  sideBarWidth: number;
  enableExplorer: boolean;
  enableGlobalSearch: boolean;
}

export function useEditorSidebarLayout(props: UseEditorSidebarProps) {
  const { panelRef, sideBarWidth, enableExplorer, enableGlobalSearch } = props;

  const settingsCtx = useSettingsContext();

  const resizeObserver = useRef<ResizeObserver>();

  const [panelSize, setPanelSize] = useState<DOMRect>();
  const [fontSize, setFontSize] = useState<number>(settingsCtx.get("fontSize") as number);
  const [savedExplorerBarHight, setSavedExplorerBarHight] = useLocalStorage<number>(
    "Editor:explorerBarHight",
    fontSize * 20
  );
  const [panelHeight, setPanelHeight] = useState<number>(panelSize?.height || fontSize * 2);
  const [explorerBarMinSize, setExplorerBarMinSize] = useState<number>(fontSize * 2);
  const [explorerBarHeight, _setExplorerBarHeight] = useState<number>(fontSize * 2);
  const [globalSearchHeight, setGlobalSearchHeight] = useState<number>(100);
  const [showExplorerName, setShowExplorerName] = useState<boolean>(false);

  useEffect(() => {
    setFontSize(settingsCtx.get("fontSize") as number);
  }, [settingsCtx.changed]);

  useEffect(() => {
    setExplorerBarMinSize(fontSize * 2 + 2);
  }, [fontSize]);

  useEffect(() => {
    setShowExplorerName(enableExplorer && sideBarWidth > fontSize * 7);
  }, [enableExplorer, sideBarWidth, fontSize]);

  const setExplorerBarHeight: (size: number) => void = (size) => {
    if (size !== explorerBarHeight && size >= explorerBarMinSize) {
      if (enableExplorer && enableGlobalSearch) {
        setSavedExplorerBarHight(size);
      }
    }
    _setExplorerBarHeight(size);
  };

  useEffect(() => {
    // update height and width of the split panel on the left side
    if (enableExplorer) {
      if (enableGlobalSearch) {
        setExplorerBarHeight(savedExplorerBarHight);
      } else {
        setExplorerBarHeight(panelHeight - explorerBarMinSize);
      }
    } else {
      setExplorerBarHeight(explorerBarMinSize);
    }
  }, [enableExplorer, enableGlobalSearch, panelHeight]);

  useEffect(() => {
    setGlobalSearchHeight(panelHeight - explorerBarHeight - fontSize * 2 - 8);
  }, [panelHeight, explorerBarHeight, fontSize]);

  useEffect(() => {
    if (!panelSize) return;
    setPanelHeight(panelSize.height);
  }, [panelSize]);

  useEffect(() => {
    if (!panelRef.current) return;

    resizeObserver.current = new ResizeObserver(() => {
      const rect = panelRef.current?.getBoundingClientRect();
      if (!rect) return;

      setPanelSize(rect);
    });

    resizeObserver.current.observe(panelRef.current);

    return () => resizeObserver.current?.disconnect();
  }, []);

  return {
    fontSize,

    explorerBarHeight,
    explorerBarMinSize,

    setExplorerBarHeight,

    globalSearchHeight,
    showExplorerName,
  };
}
