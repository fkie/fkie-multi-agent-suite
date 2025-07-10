import HistoryIcon from "@mui/icons-material/History";
import { Box, Stack, Tooltip, Typography } from "@mui/material";
import { forwardRef, LegacyRef, useCallback, useContext, useEffect, useState } from "react";

import SettingsContext from "@/renderer/context/SettingsContext";
import { colorFromHostname } from "../UI";
import StyledRootTreeItem from "./StyledRootTreeItem";

interface HistoryGroupItemProps {
  itemId: string;
  providerName: string;
  children: React.ReactNode;
  onClick?: (labelText: string, itemId: string) => void;
  onDoubleClick?: (labelText: string, itemId: string, ctrlKey: boolean, shiftKey: boolean, altKey: boolean) => void;
}

const HistoryGroupItem = forwardRef<HTMLDivElement, HistoryGroupItemProps>(function HistoryGroupItem(props, ref) {
  const { itemId, providerName, onClick = (): void => {}, onDoubleClick = (): void => {}, ...children } = props;

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
      ref={ref as LegacyRef<HTMLLIElement>}
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
          style={getHostStyle(providerName)}
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
});

export default HistoryGroupItem;
