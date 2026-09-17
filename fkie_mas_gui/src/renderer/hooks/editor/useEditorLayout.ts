import { useEffect, useRef, useState } from "react";

import { useAppState } from "../useAppState";
import { useSetting } from "../useSetting";

export function useEditorLayout() {
  const panelRef = useRef<HTMLDivElement>(null);
  const resizeObserver = useRef<ResizeObserver>();

  const [fontSize] = useSetting<number>("fontSize");
  const [panelSize, setPanelSize] = useState<DOMRect>();

  const [sideBarWidth, setSideBarWidth] = useState(fontSize * 20);
  const [sideBarMinSize, setSideBarMinSize] = useState(fontSize * 2);

  const [editorWidth, setEditorWidth] = useState(0);
  const [editorHeight, setEditorHeight] = useState(0);

  const toolbarRef = useRef<HTMLDivElement>();
  const alertRef = useRef<HTMLDivElement>();
  const { value: savedSideBarUserWidth, set: setSavedSideBarUserWidth } = useAppState<number>(
    "editor",
    "sidebar-width",
    fontSize * 20,
    {
      version: 1,
      migrateFrom: {
        localStorageKey: "Editor:sideBarWidth",
      },
    }
  );

  useEffect(() => {
    setSideBarMinSize(fontSize * 2 + 2);
    setSideBarWidth(fontSize * 2 + 2);
  }, [fontSize]);

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
