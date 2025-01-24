import DeleteForeverIcon from "@mui/icons-material/DeleteForever";
import FileDownloadOutlinedIcon from "@mui/icons-material/FileDownloadOutlined";
import SubtitlesOutlinedIcon from "@mui/icons-material/SubtitlesOutlined";
import {
  Box,
  IconButton,
  MenuItem,
  Select,
  Stack,
  Table,
  TableCell,
  TableRow,
  ToggleButton,
  Tooltip,
  Typography,
} from "@mui/material";
import { useDebounceCallback } from "@react-hook/debounce";
import React, { createRef, useContext, useEffect, useRef, useState } from "react";
import { TableVirtuoso } from "react-virtuoso";

import { levelColors } from "@/renderer/components/UI/Colors";
import SearchBar from "@/renderer/components/UI/SearchBar";
import { LoggingContext } from "@/renderer/context/LoggingContext";
import { SettingsContext } from "@/renderer/context/SettingsContext";
import useLocalStorage from "@/renderer/hooks/useLocalStorage";
import { LogEvent, LoggingLevel } from "@/renderer/models";
import "./TableResizable.css";

// const VtScroller = forwardRef<HTMLDivElement, TableContainerProps>(function VtScroller(props, ref) {
//   return <TableContainer component={Paper} {...props} ref={ref} />;
// });

// const VtTableBody = forwardRef<HTMLTableSectionElement, TableBodyProps>(function VtTableBody(props, ref) {
//   return <TableBody component={Paper} {...props} ref={ref} />;
// });

const VirtuosoTableComponents = {
  // Scroller: VtScroller,
  Table: Table,
  // TableHead: TableHead,
  // TableRow: TableRow,
  // TableBody: TableBody,
};

type TColumnData = {
  key: string;
  header: string;
  width: number | string;
  minWidth?: number;
  maxWidth?: number;
  ref?: React.RefObject<HTMLDivElement>;
};

type TColumnDataSaved = {
  index: number;
  width: number | string;
};

function exportLogs(logs: LogEvent[]): void {
  const filename = "logs.json";
  const jsonStr = JSON.stringify(logs, null, 2);

  const element = document.createElement("a");
  element.setAttribute("href", `data:text/plain;charset=utf-8,${encodeURIComponent(jsonStr)}`);
  element.setAttribute("download", filename);

  element.style.display = "none";
  document.body.appendChild(element);
  element.click();
  document.body.removeChild(element);
}

const DEFAULT_MIN_WIDTH_CELL = 70;
const DEFAULT_MAX_WIDTH_CELL = 2048;

export default function LoggingPanel(): JSX.Element {
  const [headers] = useState<TColumnData[]>([
    {
      key: "datum",
      header: "Datum",
      width: 200,
      ref: createRef(),
    },
    {
      key: "level",
      header: "Level",
      width: 100,
      ref: createRef(),
    },
    {
      key: "description",
      header: "Description",
      width: "auto",
      ref: createRef(),
    },
  ]);
  const logCtx = useContext(LoggingContext);
  const settingsCtx = useContext(SettingsContext);
  const [logLevel, setLogLevel] = useLocalStorage<LoggingLevel>("LoggingPanel:level", LoggingLevel.INFO);
  const [showDetails, setShowDetails] = useState(true);
  const [searchTerm, setSearchTerm] = useState("");
  const [loggerColumnWidths, setLoggerColumnWidths] = useLocalStorage<TColumnDataSaved[]>("LoggingPanel:columnWidths", [
    { index: 0, width: 200 },
    { index: 1, width: 100 },
    { index: 2, width: "auto" },
  ]);

  const isResizing = useRef(-1);
  const resizingStart = useRef(-1);

  function showLogLevel(level: string): boolean {
    if (level === LoggingLevel.DEBUG) {
      return logLevel === LoggingLevel.DEBUG;
    }
    if (level === LoggingLevel.INFO) {
      return [LoggingLevel.DEBUG, LoggingLevel.INFO].includes(logLevel);
    }
    if (level === LoggingLevel.WARN) {
      return [LoggingLevel.DEBUG, LoggingLevel.INFO, LoggingLevel.WARN].includes(logLevel);
    }
    return true;
  }

  function saveColumnInfoLocalStorage(): void {
    const columnsInfo: TColumnDataSaved[] = [];
    headers.forEach((col, index) => {
      const wi = headers[index].ref?.current?.parentElement?.style.width;
      columnsInfo.push({
        index,
        width: wi ? wi : col.width,
      });
    });
    setLoggerColumnWidths(columnsInfo);
  }

  function adjustWidthColumn(index: number, width: number): void {
    const minWidth = headers[index].minWidth ?? DEFAULT_MIN_WIDTH_CELL;
    const maxWidth = headers[index].maxWidth ?? DEFAULT_MAX_WIDTH_CELL;
    let newWidth = width;
    if (width > maxWidth) {
      newWidth = maxWidth;
    } else if (width < minWidth) {
      newWidth = minWidth;
    }
    if (headers[index].ref?.current?.parentElement) {
      headers[index].ref.current.parentElement!.style.width = `${newWidth}px`;
    }
    headers[index].width = newWidth;
  }

  function setCursorDocument(isColResizing: boolean): void {
    document.body.style.cursor = isColResizing ? "col-resize" : "auto";
  }

  const handleOnMouseMove = useDebounceCallback((e) => {
    if (isResizing.current >= 0) {
      const left = headers[isResizing.current].ref?.current?.parentElement?.getBoundingClientRect().left;
      if (left) {
        // const newWidth = e.clientX - left;
        const newWidth = resizingStart.current + e.clientX;
        adjustWidthColumn(isResizing.current, newWidth);
      }
    }
  }, 1);

  function handleOnMouseUp(): void {
    if (isResizing.current >= 0) {
      isResizing.current = -1;
      saveColumnInfoLocalStorage();
      setCursorDocument(false);
    }
  }

  function onClickResizeColumn(event: React.MouseEvent, index: number): void {
    isResizing.current = index;
    if (typeof headers[index].width === "number") {
      resizingStart.current = headers[index].width - event.clientX;
    } else {
      resizingStart.current = parseInt(headers[index].width.replace("px", "")) - event.clientX;
    }
    setCursorDocument(true);
  }

  /** create header row with an empty column used width='auto' to avoid width change of resizable columns */
  function fixedHeaderContent(): JSX.Element {
    return (
      <TableRow key="header">
        {headers.map((column, index) => {
          return (
            <TableCell
              className="tableCell resizable"
              key={column.key}
              variant="head"
              sx={{
                backgroundColor: "background.paper",
              }}
              style={{ width: column.width }}
            >
              <Stack
                direction="row"
                spacing={1}
                sx={{
                  backgroundColor: "background.paper",
                }}
              >
                <Typography>{column.header}</Typography>
                {column.key === "description" && (
                  <Tooltip
                    title={showDetails ? "Hide Details" : "Show Details"}
                    placement="bottom"
                    sx={{ paddingRight: 1 }}
                  >
                    <ToggleButton
                      size="small"
                      value="showDetails"
                      sx={{ height: "1.8em", padding: 0, width: "1.8em" }}
                      selected={showDetails}
                      onChange={() => {
                        setShowDetails(!showDetails);
                      }}
                    >
                      <SubtitlesOutlinedIcon sx={{ fontSize: "inherit" }} />
                    </ToggleButton>
                  </Tooltip>
                )}
                <Box
                  id={`${column.key}`}
                  onMouseDown={(event: React.MouseEvent) => onClickResizeColumn(event, index)}
                  ref={column.ref}
                  className="resizeLine"
                />
              </Stack>
            </TableCell>
          );
        })}
        <TableCell
          className="tableCell resizable"
          key="empty"
          variant="head"
          sx={{
            backgroundColor: "background.paper",
          }}
          style={{ width: "auto" }}
        />
      </TableRow>
    );
  }

  function rowContent(_index: number, row: LogEvent): JSX.Element {
    const color = levelColors[row.level.toLowerCase()];
    return (
      <>
        <TableCell className="tableCell" key={`datum-${row.id}`} sx={{ color }}>
          {row.datum}
        </TableCell>
        <TableCell className="tableCell" key={`level-${row.id}`} sx={{ color }}>
          {row.level}
        </TableCell>
        <TableCell className="tableCell" key={`description-${row.id}`} sx={{ color }}>
          <div>{row.description}</div>
          {showDetails && row.details && <div>{JSON.stringify(row.details)}</div>}
        </TableCell>
        <TableCell className="tableCell" key="empty" sx={{ color }} />
      </>
    );
  }

  useEffect(() => {
    document.onmousemove = handleOnMouseMove;
    document.onmouseup = handleOnMouseUp;
    return (): void => {
      document.onmousemove = null;
      document.onmouseup = null;
    };
  }, []);

  useEffect(() => {
    loggerColumnWidths.forEach((item) => {
      headers[item.index].width = item.width;
      if (headers[item.index] && headers[item.index].ref?.current) {
        if (headers[item.index].ref) {
          const obj = headers[item.index].ref?.current;
          if (obj) {
            obj.parentElement!.style.width = `${item.width}`;
          }
        }
      }
    });
  }, [headers, loggerColumnWidths]);

  return (
    <Stack
      direction="column"
      spacing={1}
      height="100%"
      sx={{ backgroundColor: settingsCtx.get("backgroundColor") as string }}
    >
      <Stack direction="row" spacing={0.5}>
        <SearchBar
          onSearch={(value) => {
            setSearchTerm(value);
          }}
          placeholder="Filter logs"
          defaultValue={searchTerm}
          // fullWidth={true}
        />

        <Select
          id="log-level-select"
          value={logLevel}
          sx={{ m: 1, minHeight: 10 }}
          size="small"
          // autoWidth={true}
          variant="standard"
          onChange={(event) => {
            setLogLevel(event.target.value as LoggingLevel);
          }}
        >
          {[LoggingLevel.DEBUG, LoggingLevel.INFO, LoggingLevel.WARN, LoggingLevel.SUCCESS, LoggingLevel.ERROR].map(
            (lvl) => {
              return (
                <MenuItem key={lvl} value={lvl}>
                  {lvl}
                </MenuItem>
              );
            }
          )}
        </Select>
        <Tooltip title="Export to JSON" placement="bottom">
          <IconButton
            edge="start"
            aria-label="Export to JSON"
            onClick={() =>
              exportLogs(logCtx.logs.filter((log) => showLogLevel(log.level) && log.description.includes(searchTerm)))
            }
          >
            <FileDownloadOutlinedIcon sx={{ fontSize: "inherit" }} />
          </IconButton>
        </Tooltip>
        <Tooltip title="Delete All" placement="bottom">
          <IconButton edge="start" aria-label="Delete All" onClick={() => logCtx.clearLogs()}>
            <DeleteForeverIcon sx={{ fontSize: "inherit" }} />
          </IconButton>
        </Tooltip>
      </Stack>
      <TableVirtuoso
        key="logging-panel"
        // useWindowScroll
        style={{
          backgroundColor: settingsCtx.get("backgroundColor") as string,
        }}
        data={logCtx.logs.filter((log) => showLogLevel(log.level) && log.description.includes(searchTerm))}
        components={VirtuosoTableComponents}
        fixedHeaderContent={() => fixedHeaderContent()}
        itemContent={(index: number, row: LogEvent) => rowContent(index, row)}
      />
    </Stack>
  );
}
