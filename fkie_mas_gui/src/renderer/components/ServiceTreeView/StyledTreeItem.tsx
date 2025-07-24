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
    "&.Mui-expanded": {
      fontWeight: theme.typography.fontWeightRegular,
    },
    "&:hover": {
      backgroundColor: theme.palette.action.hover,
    },
    "&.Mui-selected": {
      // backgroundColor: `var(--tree-view-bg-color, ${theme.palette.action.selected})`,
      // color: 'var(--tree-view-color)',
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
    [`& .${treeItemClasses.content}`]: {
      paddingLeft: 0
    },
  },
  [`& .${treeItemClasses.groupTransition}`]: {
    marginLeft: 15,
    paddingLeft: 5,
    borderLeft: `1px dashed ${alpha(grey[600], 0.4)}`,
  },
  ...theme.applyStyles("light", {
    color: theme.palette.grey[800],
  }),
}));

export default StyledTreeItem;
