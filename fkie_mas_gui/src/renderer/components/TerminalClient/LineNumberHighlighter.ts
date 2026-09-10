/*
 * Highlights line numbers such as 'file.py:534:' or 'foo:534' inside an xterm
 * buffer using own markers and decorations, independent from the SearchAddon.
 */

import { IDecoration, IMarker, Terminal as XTerminal } from "@xterm/xterm";

/** Matches line numbers such as 'file.py:534:' or 'foo:534', never a padding space */
const LINE_NUMBER_REGEX = /:(\d+)(?=:|\s|$)/g;

/** Number of buffer lines above the cursor that are re-scanned on every run */
const SCAN_WINDOW_LINES = 500;

/** Upper bound of live decorations, oldest entries are dropped first */
const MAX_ENTRIES = 2000;

/** One buffer row mapped to real terminal columns */
type Row = { text: string; columns: number[] };

type Entry = {
  marker: IMarker;
  decoration: IDecoration;
  /** column of the first digit */
  x: number;
  /** the digits themselves, used to revalidate the row content */
  text: string;
};

export class LineNumberHighlighter {
  private terminal: XTerminal | null;

  private color: string;

  private entries: Entry[] = [];

  private disposed = false;

  constructor(terminal: XTerminal, color: string) {
    this.terminal = terminal;
    this.color = color;
  }

  /**
   * Reads a row cell by cell. translateToString() is not column accurate:
   * the trailing cell of a wide character is skipped and combining marks add
   * extra code units, so string indices must not be used as columns.
   */
  private readRow(line: number): Row | null {
    const bufferLine = this.terminal?.buffer.active.getLine(line);
    if (!bufferLine) return null;

    const chars: string[] = [];
    const columns: number[] = [];

    for (let x = 0; x < bufferLine.length; x += 1) {
      const cell = bufferLine.getCell(x);
      if (!cell) continue;
      // width 0 marks the second cell of a wide character
      if (cell.getWidth() === 0) continue;

      // empty cells must become a space so digits can never merge across gaps
      const value = cell.getChars() || " ";
      for (let i = 0; i < value.length; i += 1) {
        chars.push(value[i]);
        columns.push(x);
      }
    }

    return { text: chars.join(""), columns };
  }

  /** column of the first digit -> digits, computed once per row and scan */
  private rowMatches(line: number): Map<number, string> {
    const row = this.readRow(line);
    const result = new Map<number, string>();
    if (!row) return result;

    LINE_NUMBER_REGEX.lastIndex = 0;
    let match = LINE_NUMBER_REGEX.exec(row.text);
    while (match !== null) {
      const digits = match[1];
      // skip the leading ':' of the match, then map to real columns
      const start = match.index + 1;
      const column = row.columns[start];
      const lastColumn = row.columns[start + digits.length - 1];
      if (column !== undefined && lastColumn !== undefined) {
        result.set(column, digits);
      }
      match = LINE_NUMBER_REGEX.exec(row.text);
    }
    return result;
  }

  /**
   * Removes entries disposed by xterm (trim, reset) and entries whose row no
   * longer holds the same number at the same column (reflow, overwrite).
   */
  private dropInvalid(matches: (line: number) => Map<number, string>): void {
    const alive: Entry[] = [];
    for (const entry of this.entries) {
      if (entry.marker.isDisposed || entry.decoration.isDisposed) {
        this.disposeEntry(entry, false);
        continue;
      }
      if (matches(entry.marker.line).get(entry.x) === entry.text) {
        alive.push(entry);
      } else {
        this.disposeEntry(entry, true);
      }
    }
    this.entries = alive;
  }

  /** Creates marker and decoration for one number, returns true on success */
  private addDecoration(line: number, cursorLine: number, column: number, digits: string): boolean {
    if (this.disposed || !this.terminal) return false;

    let marker: IMarker | undefined;
    try {
      // registerMarker() takes an offset relative to the current cursor line
      marker = this.terminal.registerMarker(line - cursorLine);
    } catch (error) {
      console.warn("[ttyd] registerMarker failed:", error);
      return false;
    }
    if (!marker) return false;

    let decoration: IDecoration | undefined;
    try {
      decoration = this.terminal.registerDecoration({
        marker,
        x: column,
        width: digits.length,
        height: 1,
        backgroundColor: this.color,
        layer: "bottom",
      });
    } catch (error) {
      console.warn("[ttyd] registerDecoration failed:", error);
    }

    if (!decoration) {
      marker.dispose();
      return false;
    }

    // the canvas/webgl renderer ignores backgroundColor, style the element too
    decoration.onRender((element) => {
      element.style.backgroundColor = this.color;
      element.style.pointerEvents = "none";
      element.style.zIndex = "-1";
    });

    // xterm may drop the marker on trim, keep the entry list consistent
    marker.onDispose(() => {
      try {
        decoration?.dispose();
      } catch (error) {
        console.warn("[ttyd] decoration dispose failed:", error);
      }
    });

    this.entries.push({ marker, decoration, x: column, text: digits });
    return true;
  }

  /** Keeps the number of live decorations bounded, oldest first */
  private enforceLimit(): void {
    if (this.entries.length <= MAX_ENTRIES) return;
    const removed = this.entries.splice(0, this.entries.length - MAX_ENTRIES);
    for (const entry of removed) {
      this.disposeEntry(entry, true);
    }
  }

  /** Disposes decoration and marker of one entry, 'alive' skips disposed objects */
  private disposeEntry(entry: Entry, alive: boolean): void {
    if (!alive) return;
    try {
      if (!entry.decoration.isDisposed) entry.decoration.dispose();
    } catch (error) {
      console.warn("[ttyd] decoration dispose failed:", error);
    }
    try {
      if (!entry.marker.isDisposed) entry.marker.dispose();
    } catch (error) {
      console.warn("[ttyd] marker dispose failed:", error);
    }
  }

  /** Scans the tail window and the viewport for line numbers */
  public scan(): void {
    if (this.disposed || !this.terminal) return;

    const buffer = this.terminal.buffer.active;
    const cursorLine = buffer.baseY + buffer.cursorY;

    // matches are needed for validation and for new decorations, cache per scan
    const cache = new Map<number, Map<number, string>>();
    const matches = (line: number): Map<number, string> => {
      const cached = cache.get(line);
      if (cached) return cached;
      const computed = this.rowMatches(line);
      cache.set(line, computed);
      return computed;
    };

    this.dropInvalid(matches);

    const known = new Set(this.entries.map((e) => `${e.marker.line}:${e.x}`));

    const ranges: Array<[number, number]> = [
      [Math.max(0, cursorLine - SCAN_WINDOW_LINES + 1), cursorLine],
      [buffer.viewportY, Math.min(buffer.viewportY + this.terminal.rows - 1, cursorLine)],
    ];

    let added = false;
    const visited = new Set<number>();
    for (const [from, to] of ranges) {
      for (let line = from; line <= to; line += 1) {
        if (visited.has(line)) continue;
        visited.add(line);

        for (const [column, digits] of matches(line)) {
          // the cursor line may be incomplete: ':37' of ':3794' would get a wrong
          // width, so wait until the number is followed by more characters
          if (line === cursorLine && column + digits.length >= buffer.cursorX) continue;
          if (known.has(`${line}:${column}`)) continue;
          if (this.addDecoration(line, cursorLine, column, digits)) {
            known.add(`${line}:${column}`);
            added = true;
          }
        }
      }
    }

    this.enforceLimit();

    if (added) {
      try {
        this.terminal.refresh(0, this.terminal.rows - 1);
      } catch (error) {
        console.warn("[ttyd] refresh after decoration failed:", error);
      }
    }
  }

  /** Drops all decorations, the terminal itself stays usable */
  public clear(): void {
    for (const entry of this.entries) {
      this.disposeEntry(entry, true);
    }
    this.entries = [];
  }

  /** Must be called while the terminal is still alive */
  public dispose(): void {
    if (this.disposed) return;
    this.clear();
    this.disposed = true;
    this.terminal = null;
  }
}
