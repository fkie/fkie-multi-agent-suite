import Inventory2OutlinedIcon from "@mui/icons-material/Inventory2Outlined";
import { Box, Stack, Tooltip, Typography } from "@mui/material";
import { blue, red } from "@mui/material/colors";
import { useCallback, useContext, useEffect, useState } from "react";

import SettingsContext from "@/renderer/context/SettingsContext";
import { colorFromHostname } from "../UI";
import CopyButton from "../UI/CopyButton";
import StyledRootTreeItem from "./StyledRootTreeItem";

interface PackageTreeItemProps {
  itemId: string;
  packageName: string;
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
    providerName = undefined,
    path,
    exists = true,
    onClick = (): void => {},
    onDoubleClick = (): void => {},
    ...children
  } = props;

  const iconColor: string = exists ? blue[700] : red[700];
  const enableCopy: boolean = false;

  const settingsCtx = useContext(SettingsContext);
  const [colorizeHosts, setColorizeHosts] = useState<boolean>(settingsCtx.get("colorizeHosts") as boolean);

  useEffect(() => {
    setColorizeHosts(settingsCtx.get("colorizeHosts") as boolean);
  }, [settingsCtx.changed]);

  const getHostStyle = useCallback(
    function getHostStyle(providerName: string | undefined): object {
      if (colorizeHosts && providerName) {
        const hColor = colorFromHostname(providerName);
        return {
          borderLeftStyle: "solid",
          borderLeftColor: hColor,
          borderLeftWidth: "0.6em",
          overflowWrap: "normal",
          // borderBottomStyle: "solid",
          // borderBottomColor: hColor,
          // borderBottomWidth: "0.6em",
        };
      }
      return {};
    },
    [colorizeHosts]
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
          style={getHostStyle(providerName)}
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
