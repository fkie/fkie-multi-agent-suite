import EditIcon from "@mui/icons-material/Edit";
import PlayCircleOutlineIcon from "@mui/icons-material/PlayCircleOutline";
import { IconButton, Link, Stack, TableCell, TableRow, Tooltip, Typography } from "@mui/material";

import { useCallback, useMemo } from "react";

import { colorFromHostname } from "@/renderer/components/UI/Colors";
import { useRosContext } from "@/renderer/hooks/useRosContext";
import { useSettingsContext } from "@/renderer/hooks/useSettingsContext";
import { ProviderLaunchConfiguration } from "@/renderer/models";

interface ProviderPanelRowCfgProps {
  startConfig: ProviderLaunchConfiguration;
  editConfiguration: (config: ProviderLaunchConfiguration) => void;
}

export default function ProviderPanelRowCfg(props: ProviderPanelRowCfgProps): JSX.Element {
  const { startConfig, editConfiguration } = props;
  const rosCtx = useRosContext();
  const settingsCtx = useSettingsContext();

  const handleStartProvider = useCallback(() => {
    rosCtx.startConfig(startConfig, null);
  }, [startConfig, rosCtx.startConfig]);

  const getHostStyle = useCallback(
    (name: string) => {
      if (settingsCtx.get("colorizeHosts")) {
        // borderLeft: `3px dashed`,
        // borderColor: colorFromHostname(provider.name()),
        return {
          borderLeftStyle: "solid",
          borderLeftColor: colorFromHostname(name),
          borderLeftWidth: "0.6em",
        };
      }
      return {};
    },
    [settingsCtx.changed]
  );

  const createTableRow = useMemo(() => {
    return (
      <TableRow
        key={startConfig.params.id}
        style={{
          display: "block",
          padding: 0,
        }}
      >
        <TableCell
          style={{
            padding: 2,
            flexGrow: 1,
            width: "100%",
          }}
          sx={getHostStyle(startConfig.params.host)}
        >
          <Stack direction="row" spacing="0.5em" flexGrow={1}>
            <Link noWrap href="#" underline="none" color="inherit" onClick={() => {}}>
              <Typography variant="body2">{startConfig.params.name || startConfig.params.host}</Typography>
            </Link>
            <Tooltip title={startConfig.params.rosVersion === "2" ? "ROS_DOMAIN_ID" : "Network ID"} placement="right">
              <Typography color="grey" variant="body2">
                [{startConfig.params.networkId}]
              </Typography>
            </Tooltip>
            {startConfig.params.rmw.forceUse && (
              <Typography color="grey" variant="body2">
                {startConfig.params.rmw.current}
              </Typography>
            )}
          </Stack>
        </TableCell>
        <TableCell
          style={{
            padding: 0,
          }}
        >
          <Stack direction="row" spacing="0.2em">
            <Tooltip title={"Click to start provider"} placement="bottom" disableInteractive>
              <span>
                {window.commandExecutor && (
                  <IconButton
                    onClick={() => {
                      handleStartProvider();
                    }}
                    color="info"
                    size="small"
                  >
                    <PlayCircleOutlineIcon fontSize="inherit" />
                  </IconButton>
                )}
              </span>
            </Tooltip>
            <Tooltip title={"Click to start provider"} placement="bottom" disableInteractive>
              <span>
                {window.commandExecutor && (
                  <IconButton
                    onClick={() => {
                      editConfiguration(startConfig);
                    }}
                    // color="info"
                    size="small"
                  >
                    <EditIcon fontSize="inherit" />
                  </IconButton>
                )}
              </span>
            </Tooltip>
          </Stack>
        </TableCell>
      </TableRow>
    );
  }, [startConfig]);

  return createTableRow;
}
