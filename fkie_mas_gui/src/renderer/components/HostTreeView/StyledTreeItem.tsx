import { grey } from "@mui/material/colors";
import { alpha, styled } from "@mui/material/styles";
import { TreeItem, treeItemClasses } from "@mui/x-tree-view";

const StyledTreeItem = styled(TreeItem)(({ theme }) => ({
  color: theme.palette.text.secondary,
  [`& .${treeItemClasses.content}`]: {
    color: theme.palette.text.secondary,
    minHeight: 25,
    borderRadius: theme.spacing(0.9),
    paddingRight: theme.spacing(1),
    fontWeight: theme.typography.fontWeightMedium,
    // smooth, subtle transition
    transition: theme.transitions.create("background-color", { duration: 120 }),

    "&.Mui-expanded": { fontWeight: theme.typography.fontWeightRegular },

    // subtle hover for unselected items
    "&:hover": {
      backgroundColor: alpha(theme.palette.text.primary, 0.04),
    },

    // selection independent of focus
    "&.Mui-selected, &.Mui-selected.Mui-focused": {
      backgroundColor: alpha(theme.palette.primary.main, 0.18),
      color: theme.palette.text.primary,
    },

    // hover on selected: only a hint stronger
    "&.Mui-selected:hover, &.Mui-selected.Mui-focused:hover": {
      backgroundColor: alpha(theme.palette.primary.main, 0.22),
    },
    [`& .${treeItemClasses.label}`]: {
      fontWeight: "inherit",
      color: "inherit",
      padding: theme.spacing(0),
    },
    [`& .${treeItemClasses.iconContainer}`]: {
      marginLeft: 0,
      marginRight: 0,
      padding: theme.spacing(0),
      width: 15,
    },
  },
  [`& .${treeItemClasses.groupTransition}`]: {
    marginLeft: 15,
    paddingLeft: 5,
    borderLeft: `1px dashed ${alpha(grey[600], 0.4)}`,
  },
  ...theme.applyStyles("light", { color: theme.palette.grey[800] }),
}));

export default StyledTreeItem;
