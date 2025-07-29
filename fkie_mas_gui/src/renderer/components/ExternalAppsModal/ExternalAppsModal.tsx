import AppsIcon from "@mui/icons-material/Apps";
import PlayArrowIcon from "@mui/icons-material/PlayArrow";
import RemoveOutlinedIcon from "@mui/icons-material/RemoveOutlined";
import {
  Button,
  Dialog,
  DialogActions,
  DialogContent,
  DialogTitle,
  IconButton,
  Link,
  Stack,
  Table,
  TableBody,
  TableCell,
  TableContainer,
  TableHead,
  TableRow,
  Tooltip,
  Typography,
} from "@mui/material";
import { ForwardedRef, useCallback, useContext, useRef, useState } from "react";

import { RosContext } from "@/renderer/context/RosContext";
import { SettingsContext } from "@/renderer/context/SettingsContext";
import { generateUniqueId } from "@/renderer/utils";
import DraggablePaper from "../UI/DraggablePaper";
import SelectDomainIdModal from "./SelectDomainIdModal";

const headers = [
  {
    key: "application",
    header: "Application",
  },
  {
    key: "run",
    header: "Run",
  },
];

// TODO: Make commands editable and save into configuration config
const applicationRows = [
  {
    id: generateUniqueId(),
    application: "RVIZ",
    commandROS1: "rosrun rviz rviz",
    commandROS2: "ros2 run rviz2 rviz2",
  },
  {
    id: generateUniqueId(),
    application: "RQT GUI",
    commandROS1: "rosrun rqt_gui rqt_gui",
    commandROS2: "ros2 run rqt_gui rqt_gui",
  },
  {
    id: generateUniqueId(),
    application: "Terminal",
    commandROS1: "terminator",
    commandROS2: "terminator",
  },
  {
    id: generateUniqueId(),
    application: "TF Tree",
    commandROS1: "rosrun rqt_tf_tree rqt_tf_tree",
    commandROS2: "ros2 run rqt_tf_tree rqt_tf_tree --force-discover",
  },
  {
    id: generateUniqueId(),
    application: "Logger Level",
    commandROS1: "rosrun rqt_logger_level rqt_logger_level",
    commandROS2: null,
  },
  {
    id: generateUniqueId(),
    application: "Console",
    commandROS1: "rosrun rqt_console rqt_console",
    commandROS2: "ros2 run rqt_console rqt_console",
  },
  {
    id: generateUniqueId(),
    application: "ROS Graph",
    commandROS1: "rosrun rqt_graph rqt_graph",
    commandROS2: "ros2 run rqt_graph rqt_graph",
  },
];

function ExternalAppsModal(ref): JSX.Element {
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);

  const [open, setOpen] = useState(false);
  const [showSelectDialog, setShowSelectDialog] = useState<{ command: string; domainIds: string[] } | undefined>();
  function handleOpen(): void {
    setOpen(true);
  }
  function handleClose(reason: "backdropClick" | "escapeKeyDown" | "confirmed" | "cancel"): void {
    if (reason && reason === "backdropClick") return;
    setShowSelectDialog(undefined);
    setOpen(false);
  }

  const runApp = useCallback(
    async (command: string) => {
      const domainIds: string[] = [
        ...new Set(
          rosCtx.providers
            .filter((prov) => prov.rosVersion === "2" && prov.rosState.ros_domain_id)
            .map((prov) => prov.rosState.ros_domain_id)
        ),
      ] as string[];

      let rmwImplementation = "";
      if (rosCtx.rosInfo?.rmwImplementation) {
        // set RMW_IMPLEMENTATION only if the variable is valid for the gui
        rmwImplementation = ` RMW_IMPLEMENTATION=${rosCtx.rosInfo.rmwImplementation}`
      }
      if (domainIds?.length === 0) {
        window.commandExecutor?.exec(null, command);
        handleClose("confirmed");
      } else if (domainIds?.length === 1) {
        window.commandExecutor?.exec(null, `ROS_DOMAIN_ID=${domainIds[0]}${rmwImplementation} ${command}`);
        handleClose("confirmed");
      } else if (domainIds) {
        setShowSelectDialog({ command: command, domainIds: domainIds });
      }
    },
    [setShowSelectDialog, rosCtx.providers]
  );

  const runAppWid = useCallback(async (command: string, domain_id: string) => {
    let rmwImplementation = "";
    if (rosCtx.rosInfo?.rmwImplementation) {
      // set RMW_IMPLEMENTATION only if the variable is valid for the gui
      rmwImplementation = ` RMW_IMPLEMENTATION=${rosCtx.rosInfo.rmwImplementation}`
    }
    window.commandExecutor?.exec(null, `ROS_DOMAIN_ID=${domain_id}${rmwImplementation} ${command}`);
    setShowSelectDialog(undefined);
  }, []);

  const dialogRef = useRef(ref);

  return (
    <Stack padding={0}>
      <Dialog
        open={open}
        onClose={(reason: "backdropClick" | "escapeKeyDown") => handleClose(reason)}
        fullWidth
        scroll="paper"
        maxWidth="md"
        ref={dialogRef as ForwardedRef<HTMLDivElement>}
        PaperProps={{
          component: DraggablePaper,
          dialogRef: dialogRef,
        }}
        aria-labelledby="draggable-dialog-title"
      >
        <DialogTitle className="draggable-dialog-title" style={{ cursor: "move" }} id="draggable-dialog-title">
          External Applications
        </DialogTitle>

        <DialogContent dividers={true}>
          <TableContainer>
            <Table>
              <TableHead>
                <TableRow>
                  {headers.map((header) => (
                    <TableCell key={header.key} sx={{ fontWeight: "bold" }}>
                      {header.header}
                    </TableCell>
                  ))}
                </TableRow>
              </TableHead>
              <TableBody>
                {applicationRows.map((row) => {
                  let command: string | null = null;

                  if (rosCtx.rosInfo) {
                    if (rosCtx.rosInfo.version === "1" && row.commandROS1) command = row.commandROS1;

                    if (rosCtx.rosInfo.version === "2" && row.commandROS2) command = row.commandROS2;
                  }
                  return (
                    <TableRow key={row.id} sx={{ "&:last-child td, &:last-child th": { border: 0 } }}>
                      <TableCell component="th" scope="row">
                        {!command && <Typography variant="body2">{row.application}</Typography>}
                        {command && (
                          <Link
                            noWrap
                            href="#"
                            underline="none"
                            color="inherit"
                            onClick={() => {
                              runApp(command);
                            }}
                          >
                            <Typography variant="body2">{row.application}</Typography>
                          </Link>
                        )}
                      </TableCell>
                      <TableCell>
                        {command && (
                          <IconButton
                            size="small"
                            onClick={() => {
                              runApp(command);
                            }}
                          >
                            <PlayArrowIcon />
                          </IconButton>
                        )}

                        {!command && (
                          <IconButton size="small">
                            <RemoveOutlinedIcon />
                          </IconButton>
                        )}
                      </TableCell>
                    </TableRow>
                  );
                })}
              </TableBody>
            </Table>
          </TableContainer>
        </DialogContent>
        <DialogActions>
          <Button autoFocus onClick={() => handleClose("cancel")}>
            Close
          </Button>
        </DialogActions>
      </Dialog>
      <Tooltip
        title="External Apps"
        placement="right"
        enterDelay={settingsCtx.get("tooltipEnterDelay") as number}
        disableInteractive
      >
        <IconButton
          sx={{
            padding: "0em",
            color: settingsCtx.get("useDarkMode") ? "#fff" : "rgba(0, 0, 0, 0.54)",
          }}
          onClick={handleOpen}
        >
          <AppsIcon sx={{ fontSize: "inherit" }} />
        </IconButton>
      </Tooltip>
      {showSelectDialog && (
        <SelectDomainIdModal
          domainIds={showSelectDialog.domainIds}
          onClose={(domainId) => {
            if (domainId) {
              runAppWid(showSelectDialog.command, domainId);
              handleClose("confirmed");
            } else {
              handleClose("cancel");
            }
          }}
        />
      )}
    </Stack>
  );
}

export default ExternalAppsModal;
