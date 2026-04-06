import HistoryIcon from "@mui/icons-material/History";
import { Box, Stack, Tooltip, Typography } from "@mui/material";
import { useCallback, useEffect, useState } from "react";

import { useRosContext } from "@/renderer/hooks/useRosContext";
import { useSettingsContext } from "@/renderer/hooks/useSettingsContext";
import StyledRootTreeItem from "./StyledRootTreeItem";

interface HistoryGroupItemProps {
  itemId: string;
  providerId: string;
  providerName: string;
  children: React.ReactNode;
  onClick?: (labelText: string, itemId: string) => void;
  onDoubleClick?: (labelText: string, itemId: string, ctrlKey: boolean, shiftKey: boolean, altKey: boolean) => void;
}

export default function HistoryGroupItem(props: HistoryGroupItemProps): JSX.Element {
  const {
    itemId,
    providerId,
    providerName,
    onClick = (): void => {},
    onDoubleClick = (): void => {},
    ...children
  } = props;

  const rosCtx = useRosContext();
  const settingsCtx = useSettingsContext();
  const [colorizeHosts, setColorizeHosts] = useState<boolean>(settingsCtx.get("colorizeHosts") as boolean);

  useEffect(() => {
    setColorizeHosts(settingsCtx.get("colorizeHosts") as boolean);
  }, [settingsCtx.changed]);

  const getHostStyle = useCallback(
    (providerId: string | undefined): object => {
      if (colorizeHosts && providerId) {
        const hColor = rosCtx.providerColor(providerId);
        return {
          borderLeftStyle: "solid",
          borderLeftColor: hColor,
          borderLeftWidth: "0.6em",
          overflowWrap: "normal",
        };
      }
      return {};
    },
    [colorizeHosts, rosCtx.providerColor]
  );

  return (
    <StyledRootTreeItem
      itemId={itemId}
      label={
        <Box
          sx={{
            display: "flex",
            alignItems: "center",
            paddingLeft: 0.5,
          }}
          onClick={() => {
            onClick(providerName, itemId);
          }}
          onDoubleClick={(event) => {
            onDoubleClick(providerName, itemId, event.ctrlKey, event.shiftKey, event.altKey);
          }}
          style={getHostStyle(providerId)}
        >
          <HistoryIcon
            sx={{
              mr: 0.2,
              width: 20,
            }}
            style={{ fontSize: "inherit" }}
          />

          <Tooltip title={providerName} enterDelay={1000} enterNextDelay={1000}>
            <Stack direction="row">
              <Typography
                // noWrap
                variant="body2"
                sx={{ fontWeight: "bold", flexGrow: 1, ml: 0.5 }}
              >
                {providerName}
              </Typography>
            </Stack>
          </Tooltip>
        </Box>
      }
      {...children}
    />
  );
}
