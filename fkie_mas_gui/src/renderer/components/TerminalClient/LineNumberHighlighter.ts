import { IDecoration, IDisposable, IMarker, Terminal as XTerminal } from "@xterm/xterm";

/** Matches line numbers such as 'file.py:534:' or 'foo:534' */
const LINE_NUMBER_REGEX = /:(\d+)(?=:|\s|$)/g;

const MAX_LINES_PER_SCAN = 2000;

/**
 * Highlights line numbers using own markers/decorations.
 * Independent from SearchAddon, so search highlighting stays usable.
 */
export class LineNumberHighlighter {
  private terminal: XTerminal | null;

  private decorations: IDecoration[] = [];

  private markers: IMarker[] = [];

  private renderListeners: IDisposable[] = [];

  private nextLineToScan = 0;

  private backgroundColor: string;

  private disposed = false;

  constructor(terminal: XTerminal, backgroundColor = "#004C99") {
    this.terminal = terminal;
    this.backgroundColor = backgroundColor;
  }

  /** Scans all buffer lines that were not processed yet */
  public scan(): void {
    if (this.disposed || !this.terminal) return;

    const buffer = this.terminal.buffer.active;
    const cursorLine = buffer.baseY + buffer.cursorY;

    // terminal was cleared or scrollback trimmed -> restart
    if (this.nextLineToScan > cursorLine + 1) {
      this.nextLineToScan = 0;
    }
    // do not scan an unbounded amount of lines at once
    if (cursorLine - this.nextLineToScan > MAX_LINES_PER_SCAN) {
      this.nextLineToScan = cursorLine - MAX_LINES_PER_SCAN;
    }

    let added = false;
    for (let line = this.nextLineToScan; line < cursorLine; line += 1) {
      if (this.decorateLine(line, cursorLine)) added = true;
    }
    // keep the cursor line for re-scan, it may still be incomplete
    this.nextLineToScan = cursorLine;

    this.pruneDisposed();

    // decoration colors are resolved while a row is rendered, so already
    // painted rows must be refreshed explicitly
    if (added) {
      try {
        this.terminal.refresh(0, this.terminal.rows - 1);
      } catch (error) {
        console.warn("[ttyd] refresh after decoration failed:", error);
      }
    }
  }

  private decorateLine(line: number, cursorLine: number): boolean {
    if (!this.terminal) return false;

    const bufferLine = this.terminal.buffer.active.getLine(line);
    if (!bufferLine) return false;

    const text = bufferLine.translateToString(false);
    LINE_NUMBER_REGEX.lastIndex = 0;

    let added = false;
    let match = LINE_NUMBER_REGEX.exec(text);
    while (match !== null) {
      // skip the leading ':' of the match
      const x = match.index + 1;
      const width = match[1].length;

      // markers are relative to the current cursor line
      const marker = this.terminal.registerMarker(line - cursorLine);
      if (marker) {
        const decoration = this.terminal.registerDecoration({
          marker,
          x,
          width,
          backgroundColor: this.backgroundColor,
          overviewRulerOptions: { color: this.backgroundColor, position: "left" },
        });
        if (decoration) {
          // style the overlay element as well: this stays visible regardless of
          // the active renderer and survives partial refreshes
          const listener = decoration.onRender((element: HTMLElement) => {
            element.style.backgroundColor = this.backgroundColor;
            element.style.pointerEvents = "none";
            element.style.zIndex = "5";
          });
          this.renderListeners.push(listener);
          this.decorations.push(decoration);
          this.markers.push(marker);
          added = true;
        } else {
          marker.dispose();
        }
      }
      match = LINE_NUMBER_REGEX.exec(text);
    }
    return added;
  }

  /** Drops references of decorations/markers already disposed by xterm */
  private pruneDisposed(): void {
    this.decorations = this.decorations.filter((d) => !d.isDisposed);
    this.markers = this.markers.filter((m) => !m.isDisposed);
  }

  /** Removes all decorations, e.g. on terminal.clear() */
  public clear(): void {
    for (const listener of this.renderListeners) {
      try {
        listener.dispose();
      } catch (error) {
        console.warn("[ttyd] render listener dispose failed:", error);
      }
    }
    for (const decoration of this.decorations) {
      try {
        decoration.dispose();
      } catch (error) {
        console.warn("[ttyd] decoration dispose failed:", error);
      }
    }
    for (const marker of this.markers) {
      try {
        marker.dispose();
      } catch (error) {
        console.warn("[ttyd] marker dispose failed:", error);
      }
    }
    this.renderListeners = [];
    this.decorations = [];
    this.markers = [];
    this.nextLineToScan = 0;
  }

  /** Must be called BEFORE terminal.dispose() */
  public dispose(): void {
    if (this.disposed) return;
    this.disposed = true;
    this.clear();
    this.terminal = null;
  }
}
