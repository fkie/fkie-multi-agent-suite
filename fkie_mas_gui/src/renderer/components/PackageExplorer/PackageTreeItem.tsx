import Inventory2OutlinedIcon from "@mui/icons-material/Inventory2Outlined";
import { Box, Stack, Tooltip, Typography } from "@mui/material";
import { blue, red } from "@mui/material/colors";
import { useCallback, useEffect, useState } from "react";

import { useRosContext } from "@/renderer/hooks/useRosContext";
import { useSettingsContext } from "@/renderer/hooks/useSettingsContext";
import CopyButton from "../UI/CopyButton";
import StyledRootTreeItem from "./StyledRootTreeItem";

interface PackageTreeItemProps {
  itemId: string;
  packageName: string;
  providerId: string | undefined;
  providerName: string | undefined;
  path: string;
  exists: boolean;
  children: React.ReactNode;
  onClick?: (labelText: string, itemId: string) => void;
  onDoubleClick?: (labelText: string, itemId: string, ctrlKey: boolean, shiftKey: boolean, altKey: boolean) => void;
}

export default function PackageTreeItem(props: PackageTreeItemProps): JSX.Element {
  const {
    itemId,
    packageName,
    providerId = undefined,
    providerName = undefined,
    path,
    exists = true,
    onClick = (): void => {},
    onDoubleClick = (): void => {},
    ...children
  } = props;

  const iconColor: string = exists ? blue[700] : red[700];
  const enableCopy: boolean = false;

  const rosCtx = useRosContext();
  const settingsCtx = useSettingsContext();
  const [colorizeHosts, setColorizeHosts] = useState<boolean>(settingsCtx.get("colorizeHosts") as boolean);

  useEffect(() => {
    setColorizeHosts(settingsCtx.get("colorizeHosts") as boolean);
  }, [settingsCtx.changed]);

  const getHostStyle = useCallback(
    function getHostStyle(providerId: string | undefined): object {
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
            onClick(packageName, itemId);
          }}
          onDoubleClick={(event) => {
            onDoubleClick(packageName, itemId, event.ctrlKey, event.shiftKey, event.altKey);
          }}
          style={getHostStyle(providerId)}
        >
          <Inventory2OutlinedIcon
            sx={{
              mr: 0.2,
              width: 20,
              color: iconColor,
            }}
            style={{ fontSize: "inherit" }}
          />

          <Tooltip title={path} enterDelay={1000} enterNextDelay={1000}>
            <Stack direction="row">
              <Typography
                // noWrap
                variant="body2"
                sx={{ fontWeight: "bold", flexGrow: 1, ml: 0.5 }}
              >
                {packageName}
              </Typography>
              {providerName && (
                <Typography
                  // noWrap
                  variant="body2"
                  sx={{ fontWeight: "normal", flexGrow: 1, ml: 0.5 }}
                  color="grey"
                  noWrap
                >
                  | {providerName}
                </Typography>
              )}
            </Stack>
          </Tooltip>
          {path && enableCopy && <CopyButton value={path} />}
        </Box>
      }
      {...children}
    />
  );
}
