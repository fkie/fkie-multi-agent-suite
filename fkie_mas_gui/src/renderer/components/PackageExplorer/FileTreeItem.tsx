import { Box, LinearProgress, Tooltip, Typography } from "@mui/material";
import { blue } from "@mui/material/colors";
import { useRef, useState } from "react";
import { FileIcon } from "react-file-icon";

import { LAUNCH_FILE_EXTENSIONS } from "@/renderer/context/SettingsContext";
import { getFileExtension, PathItem } from "@/renderer/models";
import { treeItemClasses } from "@mui/x-tree-view";
import StyledTreeItem from "../PackageExplorer/StyledTreeItem";
import CopyButton from "../UI/CopyButton";
import defaultFileIconStyles from "./FileIconDefaultStyles";

interface PackageItemTreeProps {
  itemId: string;
  file: PathItem;
  showPackage?: boolean;
  onClick?: (labelText: string, itemId: string) => void;
  onDoubleClick?: (labelText: string, itemId: string, ctrlKey: boolean, shiftKey: boolean, altKey: boolean) => void;
}

export default function FileTreeItem(props: PackageItemTreeProps): JSX.Element {
  const { itemId, file, showPackage = false, onClick = () => {}, onDoubleClick = () => {} } = props;

  const fileExtension = getFileExtension(file.name as string);
  const enableCopy: boolean = false;
  const isLaunchFile = LAUNCH_FILE_EXTENSIONS.find((fe) => file.path.endsWith(fe));
  const [label] = useState<string>(`${file.name?.replace("/", "")} ${showPackage ? `[${file.package}]` : ""}`);

  const [progress, setProgress] = useState(0);
  const intervalRef = useRef<NodeJS.Timeout | null>(null);
  const pressDuration = 800; // ms bis LongPress ausgelöst wird

  const startPress = () => {
    const startTime = Date.now();

    intervalRef.current = setInterval(() => {
      const elapsed = Date.now() - startTime;
      const percent = Math.min((elapsed / pressDuration) * 100, 100);
      setProgress(percent);

      if (percent >= 100) {
        if (intervalRef.current) clearInterval(intervalRef.current);
        intervalRef.current = null;
        onDoubleClick(label, itemId, false, false, false);
      }
    }, 16); // ~60fps
  };

  const stopPress = () => {
    if (intervalRef.current) {
      clearInterval(intervalRef.current);
      intervalRef.current = null;
    }
    setProgress(0);
  };

  return (
    <StyledTreeItem
      itemId={itemId}
      sx={{
        [`& .${treeItemClasses.content}`]: {
          paddingLeft: 0,
        },
      }}
      label={
        <Box
          sx={{
            display: "flex",
            alignItems: "center",
            paddingLeft: 0.0,
            position: "relative",
          }}
          onClick={() => onClick(label, itemId)}
          onDoubleClick={(event) => {
            onDoubleClick(label, itemId, event.ctrlKey, event.shiftKey, event.altKey);
          }}
          onMouseDown={startPress}
          onMouseUp={stopPress}
          onMouseLeave={stopPress}
          onTouchStart={startPress}
          onTouchEnd={stopPress}
        >
          {/* Progress Bar Overlay */}
          {progress > 0 && (
            <LinearProgress
              variant="determinate"
              value={progress}
              sx={{
                position: "absolute",
                bottom: 0,
                left: 0,
                width: "100%",
                height: 2,
              }}
            />
          )}

          <Box sx={{ mr: 0.2, width: 15, color: blue[700], flex: "none" }}>
            <FileIcon extension={fileExtension} radius={10} {...defaultFileIconStyles[fileExtension]} />
          </Box>

          <Tooltip title={isLaunchFile ? file.path : ""} enterDelay={1000} enterNextDelay={1000}>
            <Typography variant="body2" sx={{ fontWeight: "inherit", flexGrow: 1, ml: 0.5 }}>
              {file.name?.replace("/", "")}
              {showPackage ? ` [${file.package}]` : ""}
            </Typography>
          </Tooltip>

          {file.path && enableCopy && <CopyButton value={file.path} />}
        </Box>
      }
    />
  );
}
