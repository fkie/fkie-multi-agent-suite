import {
  Box,
  IconButton,
  MenuItem,
  Paper,
  Select,
  Stack,
  Table,
  TableBody,
  TableCell,
  TableContainer,
  TableHead,
  TableRow,
  ToggleButton,
  Tooltip,
  Typography,
} from '@mui/material';
import React, {
  createRef,
  useContext,
  useEffect,
  useRef,
  useState,
} from 'react';
import { TableVirtuoso } from 'react-virtuoso';
import './TableResizable.css';

import DeleteForeverIcon from '@mui/icons-material/DeleteForever';
import FileDownloadOutlinedIcon from '@mui/icons-material/FileDownloadOutlined';
import SubtitlesOutlinedIcon from '@mui/icons-material/SubtitlesOutlined';

import { levelColors, SearchBar } from '../../../components';
import { LoggingLevel } from '../../../models';

import { LoggingContext } from '../../../context/LoggingContext';
import { SettingsContext } from '../../../context/SettingsContext';
import useLocalStorage from '../../../hooks/useLocalStorage';

const VirtuosoTableComponents = {
  Scroller: React.forwardRef((props, ref) => (
    <TableContainer component={Paper} {...props} ref={ref} />
  )),
  Table: (props) => <Table {...props} sx={{ borderCollapse: 'separate' }} />,
  TableHead,
  TableRow: ({ item: _item, ...props }) => <TableRow {...props} />,
  TableBody: React.forwardRef((props, ref) => (
    <TableBody {...props} ref={ref} />
  )),
};

const exportLogs = (logs) => {
  const filename = 'logs.json';
  const jsonStr = JSON.stringify(logs, null, 2);

  const element = document.createElement('a');
  element.setAttribute(
    'href',
    `data:text/plain;charset=utf-8,${encodeURIComponent(jsonStr)}`,
  );
  element.setAttribute('download', filename);

  element.style.display = 'none';
  document.body.appendChild(element);
  element.click();
  document.body.removeChild(element);
};

const DEFAULT_MIN_WIDTH_CELL = 70;
const DEFAULT_MAX_WIDTH_CELL = 2048;

function LoggingPanel() {
  const [headers] = useState([
    {
      key: 'datum',
      header: 'Datum',
      width: 200,
      ref: createRef(),
    },
    {
      key: 'level',
      header: 'Level',
      width: 100,
      ref: createRef(),
    },
    {
      key: 'description',
      header: 'Description',
      width: 'auto',
      ref: createRef(),
    },
  ]);
  const logCtx = useContext(LoggingContext);
  const settingsCtx = useContext(SettingsContext);
  const [logLevel, setLogLevel] = useLocalStorage(
    'LoggingPanel:level',
    LoggingLevel.INFO,
  );
  const [showDetails, setShowDetails] = useState(true);
  const [searchTerm, setSearchTerm] = useState('');
  const [loggerColumnWidths, setLoggerColumnWidths] = useLocalStorage(
    'LoggingPanel:columnWidths',
    [
      { index: 0, width: 200 },
      { index: 1, width: 100 },
      { index: 2, width: 'auto' },
    ],
  );

  const isResizing = useRef(-1);

  const showLogLevel = (level) => {
    if (level === LoggingLevel.DEBUG) {
      return logLevel === LoggingLevel.DEBUG;
    }
    if (level === LoggingLevel.INFO) {
      return [LoggingLevel.DEBUG, LoggingLevel.INFO].includes(logLevel);
    }
    if (level === LoggingLevel.WARN) {
      return [
        LoggingLevel.DEBUG,
        LoggingLevel.INFO,
        LoggingLevel.WARN,
      ].includes(logLevel);
    }
    return true;
  };

  const saveColumnInfoLocalStorage = () => {
    const columnsInfo = [];
    headers.forEach((col, index) => {
      columnsInfo.push({
        index,
        width: headers[index].ref.current?.parentElement.style.width,
      });
    });
    setLoggerColumnWidths(columnsInfo);
  };

  const adjustWidthColumn = (index, width) => {
    const minWidth = headers[index].minWidth ?? DEFAULT_MIN_WIDTH_CELL;
    const maxWidth = headers[index].maxWidth ?? DEFAULT_MAX_WIDTH_CELL;
    let newWidth = width;
    if (width > maxWidth) {
      newWidth = maxWidth;
    } else if (width < minWidth) {
      newWidth = minWidth;
    }
    if (headers[index].ref.current)
      headers[index].ref.current.parentElement.style.width = `${newWidth}px`;
  };

  const setCursorDocument = (isColResizing) => {
    document.body.style.cursor = isColResizing ? 'col-resize' : 'auto';
  };

  const handleOnMouseMove = (e) => {
    if (isResizing.current >= 0) {
      const left =
        headers[
          isResizing.current
        ].ref.current?.parentElement?.getBoundingClientRect().left;
      const newWidth = e.clientX - left;
      adjustWidthColumn(isResizing.current, newWidth);
    }
  };

  const handleOnMouseUp = () => {
    if (isResizing.current >= 0) {
      isResizing.current = -1;
      saveColumnInfoLocalStorage();
      setCursorDocument(false);
    }
  };

  const onClickResizeColumn = (index) => {
    isResizing.current = index;
    setCursorDocument(true);
  };

  /** create header row with an empty column used width='auto' to avoid width change of resizable columns */
  function fixedHeaderContent() {
    return (
      <TableRow key="header">
        {headers.map((column, index) => {
          return (
            <TableCell
              className="tableCell resizable"
              key={column.key}
              variant="head"
              sx={{
                backgroundColor: 'background.paper',
              }}
              style={{ width: column.width }}
            >
              <Stack
                direction="row"
                spacing={1}
                sx={{
                  backgroundColor: 'background.paper',
                }}
              >
                <Typography>{column.header}</Typography>
                {column.key === 'description' && (
                  <Tooltip
                    title={showDetails ? 'Hide Details' : 'Show Details'}
                    placement="bottom"
                    sx={{ paddingRight: 1 }}
                  >
                    <ToggleButton
                      size="small"
                      value="showDetails"
                      sx={{ height: '1.8em', padding: 0, width: '1.8em' }}
                      selected={showDetails}
                      onChange={() => {
                        setShowDetails(!showDetails);
                      }}
                    >
                      <SubtitlesOutlinedIcon sx={{ fontSize: 'inherit' }} />
                    </ToggleButton>
                  </Tooltip>
                )}
                <Box
                  id={`${column.key}`}
                  onMouseDown={() => onClickResizeColumn(index)}
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
            backgroundColor: 'background.paper',
          }}
          style={{ width: 'auto' }}
        />
      </TableRow>
    );
  }

  function rowContent(_index, row) {
    const color = levelColors[row.level.toLowerCase()];
    return (
      <>
        <TableCell className="tableCell" key={row.key} sx={{ color }}>
          {row.datum}
        </TableCell>
        <TableCell className="tableCell" key={row.key} sx={{ color }}>
          {row.level}
        </TableCell>
        <TableCell className="tableCell" key={row.key} sx={{ color }}>
          <div>{row.description}</div>
          {showDetails && <div>{JSON.stringify(row.details)}</div>}
        </TableCell>
        <TableCell className="tableCell" key="empty" sx={{ color }} />
      </>
    );
  }

  useEffect(() => {
    document.onmousemove = handleOnMouseMove;
    document.onmouseup = handleOnMouseUp;
    return () => {
      document.onmousemove = null;
      document.onmouseup = null;
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  useEffect(() => {
    loggerColumnWidths.forEach((item) => {
      headers[item.index].width = item.width;
      if (headers[item.index].ref.current)
        headers[item.index].ref.current.parentElement.style.width = item.width;
    });
  }, [headers, loggerColumnWidths]);

  return (
    <Stack
      direction="column"
      spacing={1}
      height="100%"
      backgroundColor={settingsCtx.get('backgroundColor')}
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
            setLogLevel(event.target.value);
          }}
        >
          {[
            LoggingLevel.DEBUG,
            LoggingLevel.INFO,
            LoggingLevel.WARN,
            LoggingLevel.SUCCESS,
            LoggingLevel.ERROR,
          ].map((lvl) => {
            return (
              <MenuItem key={lvl} value={lvl}>
                {lvl}
              </MenuItem>
            );
          })}
        </Select>
        <Tooltip title="Export to JSON" placement="bottom">
          <IconButton
            edge="start"
            aria-label="Export to JSON"
            onClick={() =>
              exportLogs(
                logCtx.logs.filter(
                  (log) =>
                    showLogLevel(log.level) &&
                    log.description.includes(searchTerm),
                ),
              )
            }
          >
            <FileDownloadOutlinedIcon sx={{ fontSize: 'inherit' }} />
          </IconButton>
        </Tooltip>
        <Tooltip title="Delete All" placement="bottom">
          <IconButton
            edge="start"
            aria-label="Delete All"
            onClick={() => logCtx.clearLogs()}
          >
            <DeleteForeverIcon sx={{ fontSize: 'inherit' }} />
          </IconButton>
        </Tooltip>
      </Stack>
      <TableVirtuoso
        key="logging-panel"
        // useWindowScroll
        sx={{
          backgroundColor: settingsCtx.get('backgroundColor'),
        }}
        data={logCtx.logs.filter(
          (log) =>
            showLogLevel(log.level) && log.description.includes(searchTerm),
        )}
        components={VirtuosoTableComponents}
        fixedHeaderContent={() => fixedHeaderContent()}
        itemContent={(index, row) => rowContent(index, row)}
      />
    </Stack>
  );
}

LoggingPanel.defaultProps = {};

LoggingPanel.propTypes = {};

export default LoggingPanel;
