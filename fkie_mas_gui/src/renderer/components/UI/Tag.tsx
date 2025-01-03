import { Chip, ChipPropsColorOverrides } from "@mui/material";
import { OverridableStringUnion } from "@mui/types";
import { forwardRef } from "react";
import CopyButton from "./CopyButton";

const chipDefaultColors = ["default", "primary", "secondary", "error", "info", "success", "warning"];

type TagColor =
  | OverridableStringUnion<
      "default" | "primary" | "secondary" | "error" | "info" | "success" | "warning",
      ChipPropsColorOverrides
    >
  | undefined;

interface TagProps {
  className?: string | undefined;
  title?: string | undefined;
  text: string;
  color?: string;
  copyButton?: string;
  wrap?: boolean;
  onClick?: (event: React.MouseEvent) => void;
  onDoubleClick?: (event: React.MouseEvent) => void;
}

const Tag = forwardRef<HTMLDivElement, TagProps>(function Tag(props, ref) {
  const { className, title = "", text = "", color = "info", copyButton = "", wrap = true } = props;
  const isDefaultColor = chipDefaultColors.includes(color);

  const chipSX = {
    // fontSize: SettingsCtx.fontSize,
    height: "auto",
  };

  if (wrap) {
    chipSX["& .MuiChip-label"] = {
      display: "block",
      whiteSpace: "normal",
      wordWrap: "break-word",
    };
  }

  let newText = text;
  if (title) {
    newText = ` ${newText}`;
  }

  return (
    <Chip
      className={className}
      ref={ref}
      size="small"
      color={isDefaultColor ? (color as TagColor) : "default"}
      style={isDefaultColor ? {} : { backgroundColor: color }}
      label={
        <>
          <strong>{title}</strong>
          {newText}
          {copyButton && <CopyButton value={copyButton} fontSize="0.6em" />}
        </>
      }
      sx={chipSX}
    />
  );
});

export default Tag;
