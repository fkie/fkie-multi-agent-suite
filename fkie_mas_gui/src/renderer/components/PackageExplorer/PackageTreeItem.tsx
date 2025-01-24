import Inventory2OutlinedIcon from "@mui/icons-material/Inventory2Outlined";
import { Box, Tooltip, Typography } from "@mui/material";
import { blue, red } from "@mui/material/colors";
import { forwardRef, LegacyRef } from "react";

import CopyButton from "../UI/CopyButton";
import StyledRootTreeItem from "./StyledRootTreeItem";

interface PackageTreeItemProps {
  itemId: string;
  packageName: string;
  path: string;
  exists: boolean;
  children: React.ReactNode;
  onClick?: (labelText: string, itemId: string) => void;
  onDoubleClick?: (labelText: string, itemId: string, ctrlKey: boolean, shiftKey: boolean, altKey: boolean) => void;
}

const PackageTreeItem = forwardRef<HTMLDivElement, PackageTreeItemProps>(function PackageTreeItem(props, ref) {
  const {
    itemId,
    packageName,
    path,
    exists = true,
    onClick = (): void => {},
    onDoubleClick = (): void => {},
    ...children
  } = props;

  const iconColor: string = exists ? blue[700] : red[700];
  const enableCopy: boolean = false;

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
            onClick(packageName, itemId);
          }}
          onDoubleClick={(event) => {
            onDoubleClick(packageName, itemId, event.ctrlKey, event.shiftKey, event.altKey);
          }}
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
            <Typography
              // noWrap
              variant="body2"
              sx={{ fontWeight: "inherit", flexGrow: 1, ml: 0.5 }}
            >
              {packageName}
            </Typography>
          </Tooltip>
          {path && enableCopy && <CopyButton value={path} />}
        </Box>
      }
      {...children}
    />
  );
});

export default PackageTreeItem;
