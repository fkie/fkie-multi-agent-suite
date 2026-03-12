import { useEffect, useRef, useState } from "react";

import useLocalStorage from "../useLocalStorage";
import { useSettingsContext } from "../useSettingsContext";

export function useEditorLayout() {
  const settingsCtx = useSettingsContext();
  const panelRef = useRef<HTMLDivElement>(null);
  const resizeObserver = useRef<ResizeObserver>();

  const [fontSize, setFontSize] = useState<number>(settingsCtx.get("fontSize") as number);
  const [panelSize, setPanelSize] = useState<DOMRect>();

  const [sideBarWidth, setSideBarWidth] = useState(fontSize * 20);
  const [sideBarMinSize, setSideBarMinSize] = useState(fontSize * 2);

  const [editorWidth, setEditorWidth] = useState(0);
  const [editorHeight, setEditorHeight] = useState(0);

  const toolbarRef = useRef<HTMLDivElement>();
  const alertRef = useRef<HTMLDivElement>();
  const [savedSideBarUserWidth, setSavedSideBarUserWidth] = useLocalStorage<number>(
    "Editor:sideBarWidth",
    fontSize * 20
  );

  useEffect(() => {
    setSideBarMinSize(fontSize * 2 + 2);
    setSideBarWidth(fontSize * 2 + 2);
  }, [fontSize]);

  useEffect(() => {
    setFontSize(settingsCtx.get("fontSize") as number);
  }, [settingsCtx.changed]);

  useEffect(() => {
    if (!panelRef.current) return;

    resizeObserver.current = new ResizeObserver(() => {
      const rect = panelRef.current?.getBoundingClientRect();
      if (!rect) return;

      setPanelSize(rect);

      setEditorWidth(rect.width - sideBarWidth);
      setEditorHeight(rect.height);
    });

    resizeObserver.current.observe(panelRef.current);

    return () => resizeObserver.current?.disconnect();
  }, [sideBarWidth]);

  useEffect(() => {
    if (!panelSize) return;
    const infoHeight: number = toolbarRef.current ? toolbarRef.current?.getBoundingClientRect().height : 0;
    const alertHeight: number = alertRef.current ? alertRef.current?.getBoundingClientRect().height : 0;
    setEditorHeight(panelSize.height - infoHeight - alertHeight);
    setEditorWidth(panelSize.width - sideBarWidth);
  }, [sideBarWidth, panelSize]);

  return {
    panelRef,
    toolbarRef,
    alertRef,

    panelSize,
    fontSize,

    sideBarWidth,
    setSideBarWidth,
    sideBarMinSize,
    setSideBarMinSize,

    editorWidth,
    setEditorWidth,
    editorHeight,

    savedSideBarUserWidth,
    setSavedSideBarUserWidth,
  };
}
