import { LAUNCH_FILE_EXTENSIONS } from "@/renderer/context/SettingsContext";
import { getFileExtension, PathItem } from "@/renderer/models";
import { Box, Tooltip, Typography } from "@mui/material";
import { blue } from "@mui/material/colors";
import { forwardRef, LegacyRef, useState } from "react";
import { FileIcon } from "react-file-icon";
import CopyButton from "../UI/CopyButton";
import defaultFileIconStyles from "./FileIconDefaultStyles";
import StyledTreeItem from "./StyledTreeItem";

interface PackageItemTreeProps {
  itemId: string;
  file: PathItem;
  showPackage?: boolean;
  onClick?: (labelText: string, itemId: string) => void;
  onDoubleClick?: (labelText: string, itemId: string, ctrlKey: boolean, shiftKey: boolean, altKey: boolean) => void;
}

const FileTreeItem = forwardRef<HTMLDivElement, PackageItemTreeProps>(function FileTreeItem(props, ref) {
  const { itemId, file, showPackage = false, onClick = (): void => {}, onDoubleClick = (): void => {} } = props;

  const fileExtension = getFileExtension(file.name as string);
  const enableCopy: boolean = false;
  const isLaunchFile = LAUNCH_FILE_EXTENSIONS.find((fe) => file.path.endsWith(fe));
  const [label] = useState<string>(`${file.name?.replace("/", "")} ${showPackage ? `[${file.package}]` : ""}`);

  return (
    <StyledTreeItem
      ref={ref as LegacyRef<HTMLLIElement>}
      itemId={itemId}
      label={
        <Box
          sx={{
            display: "flex",
            alignItems: "center",
            paddingLeft: 0.0,
          }}
          onClick={() => {
            onClick(label, itemId);
          }}
          onDoubleClick={(event) => {
            onDoubleClick(label, itemId, event.ctrlKey, event.shiftKey, event.altKey);
          }}
        >
          <Box sx={{ mr: 0.2, width: 15, color: blue[700], flex: "none" }}>
            <FileIcon extension={fileExtension} radius={10} {...defaultFileIconStyles[fileExtension]} />
          </Box>

          <Tooltip title={isLaunchFile ? file.path : ""} enterDelay={1000} enterNextDelay={1000}>
            <Typography
              // noWrap
              variant="body2"
              sx={{ fontWeight: "inherit", flexGrow: 1, ml: 0.5 }}
            >
              {file.name?.replace("/", "")}
              {showPackage ? ` [${file.package}]` : ""}
            </Typography>
          </Tooltip>
          {file.path && enableCopy && <CopyButton value={file.path} />}
        </Box>
      }
    />
  );
});

export default FileTreeItem;
