import { useLayoutEffect, useRef, useSyncExternalStore } from "react";
import { createPortal } from "react-dom";

type TPanelEntry = { id: string; element: React.ReactNode; host: HTMLDivElement };

/**
 * Keeps panel react trees mounted independently of the layout that displays them.
 * Only the DOM position of the host element changes when a tab is moved.
 */
class PersistentPanelStore {
  private panels = new Map<string, TPanelEntry>();
  private listeners = new Set<() => void>();
  private snapshot: TPanelEntry[] = [];
  private notifyScheduled = false;

  private notify(): void {
    // never notify during the render phase of the calling component
    if (this.notifyScheduled) return;
    this.notifyScheduled = true;
    queueMicrotask(() => {
      this.notifyScheduled = false;
      this.snapshot = [...this.panels.values()];
      for (const listener of this.listeners) listener();
    });
  }

  public subscribe = (listener: () => void): (() => void) => {
    this.listeners.add(listener);
    return () => this.listeners.delete(listener);
  };

  public getSnapshot = (): TPanelEntry[] => this.snapshot;

  /** Create the panel once; later calls return the existing instance. */
  public ensure(id: string, create: () => React.ReactNode): void {
    if (this.panels.has(id)) return;
    const host = document.createElement("div");
    host.style.cssText = "width:100%;height:100%;position:relative;overflow:hidden;";
    host.dataset.panelId = id;
    this.panels.set(id, { id, element: create(), host });
    this.notify();
  }

  public getHost(id: string): HTMLDivElement | undefined {
    return this.panels.get(id)?.host;
  }

  /** Detach from DOM but keep the react tree alive (tab hidden or moved). */
  public park(id: string): void {
    this.panels.get(id)?.host.remove();
  }

  /** Destroy the panel for good (tab really closed). */
  public destroy(id: string): void {
    const entry = this.panels.get(id);
    if (!entry) return;
    entry.host.remove();
    this.panels.delete(id);
    this.notify();
  }
}

export const persistentPanelStore = new PersistentPanelStore();

/** Renders all persistent panels once. Must be mounted above every layout. */
export function PersistentPanelPortals(): JSX.Element {
  const entries = useSyncExternalStore(persistentPanelStore.subscribe, persistentPanelStore.getSnapshot);
  // container per entry is stable -> react does not remount the children
  return <>{entries.map((entry) => createPortal(entry.element, entry.host, entry.id))}</>;
}

/** Placeholder inside a layout tab; adopts the persistent host element. */
export function PanelMount({ id }: { id: string }): JSX.Element {
  const ref = useRef<HTMLDivElement | null>(null);

  useLayoutEffect(() => {
    const host = persistentPanelStore.getHost(id);
    const container = ref.current;
    if (!host || !container) return;
    if (host.parentElement !== container) {
      container.appendChild(host);
      // xterm/monaco need a resize after being moved in the DOM
      requestAnimationFrame(() => window.dispatchEvent(new Event("resize")));
    }
    return () => persistentPanelStore.park(id);
  }, [id]);

  return <div ref={ref} style={{ width: "100%", height: "100%" }} />;
}
