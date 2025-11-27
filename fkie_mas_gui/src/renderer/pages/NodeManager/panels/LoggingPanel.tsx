import DeleteForeverIcon from "@mui/icons-material/DeleteForever";
import FileDownloadOutlinedIcon from "@mui/icons-material/FileDownloadOutlined";
import SubtitlesOutlinedIcon from "@mui/icons-material/SubtitlesOutlined";
import {
  IconButton,
  MenuItem,
  Select,
  Stack,
  Table,
  TableCell,
  ToggleButton,
  Tooltip,
  Typography,
} from "@mui/material";
import { useContext, useState } from "react";
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

export default function LoggingPanel(): JSX.Element {
  const logCtx = useContext(LoggingContext);
  const settingsCtx = useContext(SettingsContext);
  const [logLevel, setLogLevel] = useLocalStorage<LoggingLevel>("LoggingPanel:level", LoggingLevel.INFO);
  const [showDetails, setShowDetails] = useState(true);
  const [searchTerm, setSearchTerm] = useState("");

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

  function convertBytes(bytes: number): string {
    if (bytes < 1000) {
      return `${bytes} B`;
    }
    const kilobytes = bytes / 1000;
    const fmt = (n) =>
      new Intl.NumberFormat("de-DE", {
        minimumFractionDigits: 2,
        maximumFractionDigits: 2,
      }).format(n);

    if (kilobytes < 1000) {
      return `${fmt(kilobytes)} kB`;
    }
    const megabytes = bytes / 1_000_000;
    return `${fmt(megabytes)} MB`;
  }

  function rowContent(_index: number, row: LogEvent): JSX.Element {
    const color = levelColors[row.level.toLowerCase()];
    const details = JSON.stringify(row.details).replaceAll("\\n", "\n").replaceAll('\\"', '"').replace(/^"|"$/g, "");
    const detailsSize = details.length || 0;
    const detailsSizeStr = convertBytes(detailsSize);
    return (
      <TableCell className="tableCell" key={`datum-${row.id}`} sx={{ color, padding: 0.1, margin: 0 }}>
        <Stack direction="row" spacing="1em">
          <Typography
            variant="body1"
            sx={{
              fontFamily: "sans-serif",
              fontSize: "0.9em",
              whiteSpace: "nowrap",
              textOverflow: "ellipsis",
            }}
          >
            {row.timestamp}
          </Typography>
          <Typography
            variant="body1"
            sx={{
              fontFamily: "sans-serif",
              fontSize: "0.9em",
              minWidth: "60px",
              whiteSpace: "nowrap",
              textOverflow: "ellipsis",
            }}
          >
            {row.level}
          </Typography>
          <Stack direction="column">
            <Typography
              variant="body1"
              sx={{
                fontFamily: "sans-serif",
                fontSize: "0.9em",
                whiteSpace: "normal",
                overflowWrap: "anywhere",
                wordBreak: "break-word",
                textOverflow: "ellipsis",
              }}
            >
              {row.description}
              {detailsSize >= 300 && (
                <IconButton
                  sx={{
                    fontSize: "0.9em",
                    color: (theme) => theme.palette.text.disabled,
                    paddingTop: 0,
                    paddingBottom: 0,
                  }}
                  size="small"
                  component="span"
                  onClick={() => {
                    navigator.clipboard.writeText(details);
                    logCtx.info(`Text with ${detailsSizeStr} copied!`, "", "copied to clipboard");
                  }}
                >
                  copy {detailsSizeStr}
                </IconButton>
              )}
            </Typography>
            {showDetails && row.details && (
              <Typography
                variant="body1"
                sx={{
                  fontFamily: "monospace",
                  fontSize: "0.8em",
                  overflow: "hidden",
                  whiteSpace: "normal",
                  overflowWrap: "anywhere",
                  wordBreak: "break-word",
                  textOverflow: "ellipsis",
                  // whiteSpace: "pre-line",
                }}
              >
                {detailsSize > 300 ? `${details.slice(0, 300)}...` : details}
              </Typography>
            )}
          </Stack>
        </Stack>
      </TableCell>
    );
  }

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
          sx={{ fontSize: "0.8em" }}
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
                <MenuItem key={lvl} value={lvl} sx={{ fontSize: "0.8em" }}>
                  {lvl}
                </MenuItem>
              );
            }
          )}
        </Select>
        <Tooltip title={showDetails ? "Hide Details" : "Show Details"} placement="bottom" sx={{ paddingRight: 1 }}>
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
          overflow: "auto",
          backgroundColor: settingsCtx.get("backgroundColor") as string,
        }}
        data={logCtx.logs.filter((log) => showLogLevel(log.level) && log.description.includes(searchTerm))}
        components={VirtuosoTableComponents}
        itemContent={(index: number, row: LogEvent) => rowContent(index, row)}
      />
    </Stack>
  );
}
