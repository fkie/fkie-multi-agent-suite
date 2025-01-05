import ChevronRightIcon from "@mui/icons-material/ChevronRight";
import ExpandMoreIcon from "@mui/icons-material/ExpandMore";
import { Alert, AlertTitle, Box, Chip, ListItem, ListItemButton, ListItemText, Stack, Typography } from "@mui/material";
import { SimpleTreeView, TreeItem } from "@mui/x-tree-view";
import { forwardRef, useContext } from "react";
import { VariableSizeList } from "react-window";
import { SettingsContext } from "../../../context/SettingsContext";
import { LaunchContent, LaunchNodeInfo } from "../../../models";

type TComposableNodes = {
  manager: string;
  nodes: string[];
};

interface LaunchFilePanelProps {
  launchContent: LaunchContent;
}

const LaunchFilePanel = forwardRef<HTMLDivElement, LaunchFilePanelProps>(function LaunchFilePanel(props, ref) {
  const { launchContent } = props;

  const settingsCtx = useContext(SettingsContext);
  const nodes: LaunchNodeInfo[] = launchContent.nodes.filter((e) => !e.composable_container);
  const composableNodes: TComposableNodes[] = launchContent.nodes
    .filter((e: LaunchNodeInfo) => e.composable_container)
    .reduce((tree: TComposableNodes[], composableNode: LaunchNodeInfo) => {
      const manager = tree.find((n) => n.manager && n.manager === composableNode.composable_container);
      if (manager) {
        manager.nodes.push(composableNode.node_name as string);
      } else {
        tree.push({
          manager: composableNode.composable_container,
          nodes: [composableNode.node_name],
        } as TComposableNodes);
      }
      return tree;
    }, []);

  const renderNodeRow = ({ index, style }) => {
    return (
      <ListItem style={style} key={index} component="div" disablePadding dense disableGutters>
        <ListItemButton dense disableGutters>
          <ListItemText primary={`${nodes[index].node_name}`} />
        </ListItemButton>
      </ListItem>
    );
  };

  // const renderParameterRow = ({ columnIndex, rowIndex, style }) => {
  //   let text = '';

  //   if (columnIndex === 0) {
  //     text = launchContent.parameters[rowIndex].name;
  //   } else {
  //     text = launchContent.parameters[rowIndex].value;
  //   }

  //   return (
  //     <ListItem
  //       style={style}
  //       key={`${columnIndex}_${rowIndex}`}
  //       component="div"
  //       disablePadding
  //       dense
  //       disableGutters
  //     >
  //       <ListItemButton dense disableGutters>
  //         <ListItemText primary={`${text}`} />
  //       </ListItemButton>
  //     </ListItem>
  //   );
  // };
  const chipSX = {
    // fontSize: SettingsCtx.fontSize,
    height: "auto",
    "& .MuiChip-label": {
      display: "block",
      whiteSpace: "normal",
    },
  };

  return (
    <Box
      ref={ref}
      // width="100%"
      height="100%"
      overflow="auto"
      style={{ backgroundColor: settingsCtx.get("backgroundColor") as string }}
    >
      <Stack spacing={1}>
        <Stack direction="row" spacing={1}>
          {launchContent.host && launchContent.host.length > 0 && (
            <Chip
              size="small"
              color="default"
              label={
                <>
                  <strong>Host</strong>
                  {`  ${launchContent.host}  `}
                </>
              }
              sx={chipSX}
            />
          )}

          {launchContent.host && launchContent.host.length > 0 && (
            <Chip
              size="small"
              color="default"
              label={
                <>
                  <strong>MasterURI</strong>
                  {`  ${launchContent.masteruri}  `}
                </>
              }
              sx={chipSX}
            />
          )}
        </Stack>

        <Typography variant="body1">
          <strong>Path</strong>
          {`  ${launchContent.path}  `}
        </Typography>

        {/* TODO: Fix and add arguments of the launch file */}

        <Typography variant="body1">Arguments {`[${launchContent.args.length}]`}</Typography>

        {launchContent && launchContent.args && launchContent.args.length > 0 && (
          <SimpleTreeView
            aria-label="file system navigator"
            slots={{ collapseIcon: ExpandMoreIcon, expandIcon: ChevronRightIcon }}
          >
            {launchContent.args.map((arg) => {
              return (
                <TreeItem key={`${arg.name}_${arg.name}`} itemId={arg.name} label={`${arg.name}: [${arg.value}]`} />
              );
            })}
          </SimpleTreeView>
        )}

        {launchContent && composableNodes.length > 0 && (
          <>
            <Typography variant="body1">Nodelets {`[${composableNodes.length}]`}</Typography>
            <SimpleTreeView
              aria-label="file system navigator"
              slots={{ collapseIcon: ExpandMoreIcon, expandIcon: ChevronRightIcon }}
            >
              {composableNodes.map((composableNode) => {
                return (
                  <TreeItem itemId={composableNode.manager} label={composableNode.manager} key={composableNode.manager}>
                    {composableNode.nodes.map((node) => {
                      return <TreeItem itemId={node} label={node} key={node} />;
                    })}
                  </TreeItem>
                );
              })}
            </SimpleTreeView>
          </>
        )}

        {launchContent && nodes.length > 0 && (
          <>
            <Typography variant="body1">Nodes {`[${nodes.length}]`}</Typography>
            <Box
              sx={{
                width: "100%",
                height: 150,
              }}
            >
              <VariableSizeList height={150} width="100%" itemSize={() => 30} itemCount={nodes.length}>
                {renderNodeRow}
              </VariableSizeList>
            </Box>
          </>
        )}

        <Typography variant="body1">Parameters {`[${launchContent.parameters.length}]`}</Typography>

        {/* TODO: Do we want to show parameters? */}
        {/* {launchContent &&
          launchContent.parameters &&
          launchContent.parameters.length > 0 && (
            <>
              <Box
                sx={{
                  width: '100%',
                  height: 150,
                }}
              >
                <VariableSizeGrid
                  columnCount={2}
                  columnWidth={(index) => {
                    if (index === 0) {
                      if (width) return width * 0.8;

                      return 280;
                    }

                    if (index === 1) {
                      if (width) return width * 0.2;

                      return 20;
                    }

                    return 20;
                  }}
                  height={150}
                  width={width ? width - 40 : 300}
                  rowCount={launchContent.parameters.length}
                  rowHeight={(index) => 30}
                >
                  {renderParameterRow}
                </VariableSizeGrid>
              </Box>
            </>
          )} */}
      </Stack>

      {!launchContent && (
        <Alert severity="error" style={{ minWidth: 0 }}>
          <AlertTitle>Invalid Launch file</AlertTitle>
        </Alert>
      )}
    </Box>
  );
});

export default LaunchFilePanel;
