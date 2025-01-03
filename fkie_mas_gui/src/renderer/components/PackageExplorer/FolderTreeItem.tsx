import FolderOutlinedIcon from "@mui/icons-material/FolderOutlined";
import { Box, Tooltip, Typography } from "@mui/material";
import { blue } from "@mui/material/colors";
import { forwardRef, LegacyRef } from "react";
import StyledTreeItem from "./StyledTreeItem";

interface FolderTreeItemProps {
  itemId: string;
  directoryName: string;
  path: string;
  children: React.ReactNode;
  onClick?: (directoryName: string, itemId: string) => void;
  onDoubleClick?: (directoryName: string, itemId: string, ctrlKey: boolean, shiftKey: boolean, altKey: boolean) => void;
}

const FolderTreeItem = forwardRef<HTMLDivElement, FolderTreeItemProps>(function FolderTreeItem(props, ref) {
  const { itemId, directoryName, path, onClick = () => {}, onDoubleClick = () => {}, ...children } = props;

  return (
    <StyledTreeItem
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
            if (onClick) onClick(directoryName, itemId);
          }}
          onDoubleClick={(event) => {
            if (onDoubleClick) onDoubleClick(directoryName, itemId, event.ctrlKey, event.shiftKey, event.altKey);
          }}
        >
          <FolderOutlinedIcon
            sx={{
              mr: 0.2,
              width: 20,
              color: blue[700],
            }}
            style={{ fontSize: "inherit" }}
          />

          <Tooltip title={path} enterDelay={1000} enterNextDelay={1000}>
            <Typography
              // noWrap
              variant="body2"
              sx={{ fontWeight: "inherit", flexGrow: 1, ml: 0.5 }}
            >
              {directoryName}
            </Typography>
          </Tooltip>
        </Box>
      }
      {...children}
    />
  );
});

export default FolderTreeItem;
