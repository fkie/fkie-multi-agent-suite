import FolderOutlinedIcon from "@mui/icons-material/FolderOutlined";
import { Box, Tooltip, Typography } from "@mui/material";
import { blue } from "@mui/material/colors";

import { treeItemClasses } from "@mui/x-tree-view";
import StyledTreeItem from "./StyledTreeItem";

interface FolderTreeItemProps {
  itemId: string;
  directoryName: string;
  path: string;
  children: React.ReactNode;
  onClick?: (directoryName: string, itemId: string) => void;
  onDoubleClick?: (directoryName: string, itemId: string, ctrlKey: boolean, shiftKey: boolean, altKey: boolean) => void;
}

export default function FolderTreeItem(props: FolderTreeItemProps): JSX.Element {
  const { itemId, directoryName, path, onClick = (): void => {}, onDoubleClick = (): void => {}, ...children } = props;

  return (
    <StyledTreeItem
      itemId={itemId}
      sx={{
        [`& .${treeItemClasses.content}`]: {
          paddingLeft: "8px",
        },
      }}
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
}
