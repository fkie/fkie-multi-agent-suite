import { generateUniqueId } from "@/renderer/utils";
import AppsIcon from "@mui/icons-material/Apps";
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
import { useCallback, useEffect, useState } from "react";

import { useRosContext } from "@/renderer/hooks/useRosContext";
import { useSettingsContext } from "@/renderer/hooks/useSettingsContext";
import DraggablePaper from "../UI/DraggablePaper";

const headers = [
  {
    key: "application",
    header: "Application",
  },
  {
    key: "run",
    header: "Run for domain id",
  },
];

type RowType = {
  id: string;
  application: string;
  commandROS1: string;
  package: string;
  binary: string;
  namespace?: string;
  name?: string;
  args: string[];
  ros_args?: string[];
};

// TODO: Make commands editable and save into configuration config
const applicationRows: RowType[] = [
  {
    id: generateUniqueId(),
    application: "RVIZ",
    commandROS1: "rosrun rviz rviz",
    package: "rviz2",
    binary: "rviz2",
    name: "mas_rviz",
    args: [],
  },
  {
    id: generateUniqueId(),
    application: "RQT GUI",
    commandROS1: "rosrun rqt_gui rqt_gui",
    package: "rqt_gui",
    binary: "rqt_gui",
    name: "mas_rqt_gui",
    args: [],
  },
  {
    id: generateUniqueId(),
    application: "TF Tree",
    commandROS1: "rosrun rqt_tf_tree rqt_tf_tree",
    package: "rqt_tf_tree",
    binary: "rqt_tf_tree",
    name: "mas_rqt_tf_tree",
    args: ["--force-discover"],
  },
  {
    id: generateUniqueId(),
    application: "Logger Level",
    commandROS1: "rosrun rqt_logger_level rqt_logger_level",
    package: "",
    binary: "",
    args: [],
  },
  {
    id: generateUniqueId(),
    application: "Console",
    commandROS1: "rosrun rqt_console rqt_console",
    package: "rqt_console",
    binary: "rqt_console",
    name: "mas_rqt_rqt_console",
    args: [],
  },
  {
    id: generateUniqueId(),
    application: "ROS Graph",
    commandROS1: "rosrun rqt_graph rqt_graph",
    package: "rqt_graph",
    binary: "rqt_graph",
    name: "mas_rqt_rqt_graph",
    args: [],
  },
];

export default function ExternalAppsModal(): JSX.Element {
  const rosCtx = useRosContext();
  const settingsCtx = useSettingsContext();

  const [open, setOpen] = useState(false);
  const [localProviderDomains, setLocalProviderDomains] = useState<number[]>([]);

  useEffect(() => {
    const localProvs = rosCtx.getLocalProvider();
    setLocalProviderDomains(
      localProvs
        .filter((prov) => prov.isAvailable() && prov.rosState !== undefined)
        .map((prov) => prov.connection.domainId)
        .sort()
    );
  }, [rosCtx.providers]);

  function handleOpen(): void {
    setOpen(true);
  }

  function handleClose(reason: "backdropClick" | "escapeKeyDown" | "confirmed" | "cancel"): void {
    if (reason && reason === "backdropClick") return;
    setOpen(false);
  }

  const runApp = useCallback(
    async (command: RowType, domainId: number) => {
      const localProvs = rosCtx.getLocalProvider();
      for (const localProv of localProvs) {
        if (localProv.rosVersion === "2" && localProv.connection.domainId === domainId) {
          localProv.rosRun({
            package: command.package,
            binary: command.binary,
            name: command.name,
            args: command.args,
            ros_args: command.ros_args,
          });
        } else {
          if (localProv.connection.domainId === domainId) {
            window.commandExecutor?.exec(null, command.commandROS1);
            handleClose("confirmed");
          }
          // else {
          //   let rmwImplementation = "";
          //   if (rosCtx.rosInfo?.rmwImplementation) {
          //     // set RMW_IMPLEMENTATION only if the variable is valid for the gui
          //     rmwImplementation = ` RMW_IMPLEMENTATION=${rosCtx.rosInfo.rmwImplementation}`;
          //   }

          //   window.commandExecutor?.exec(
          //     null,
          //     `ROS_DOMAIN_ID=${localProv.connection.domainId}${rmwImplementation} ${command.commandROS1}`
          //   );
          // }
        }
      }
      handleClose("confirmed");
    },
    [rosCtx.providers, rosCtx.rosInfo, window.commandExecutor]
  );

  // const runAppWid = useCallback(
  //   async (command: string, domain_id: string) => {
  //     let rmwImplementation = "";
  //     if (rosCtx.rosInfo?.rmwImplementation) {
  //       // set RMW_IMPLEMENTATION only if the variable is valid for the gui
  //       rmwImplementation = ` RMW_IMPLEMENTATION=${rosCtx.rosInfo.rmwImplementation}`;
  //     }
  //     window.commandExecutor?.exec(null, `ROS_DOMAIN_ID=${domain_id}${rmwImplementation} ${command}`);
  //   },
  //   [rosCtx]
  // );

  return (
    <Stack padding={0}>
      <Dialog
        open={open}
        onClose={(reason: "backdropClick" | "escapeKeyDown") => handleClose(reason)}
        fullWidth
        scroll="paper"
        maxWidth="sm"
        PaperComponent={DraggablePaper}
        aria-labelledby="draggable-dialog-title"
      >
        <DialogTitle className="draggable-dialog-title" style={{ cursor: "move" }} id="draggable-dialog-title">
          External Applications
        </DialogTitle>

        <DialogContent dividers={false}>
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
                  let command: RowType | null = null;

                  if (rosCtx.rosInfo) {
                    if (rosCtx.rosInfo.version === "1" && row.commandROS1) command = row;

                    if (rosCtx.rosInfo.version === "2" && row.package) command = row;
                  }
                  return (
                    <TableRow key={row.id} sx={{ "&:last-child td, &:last-child th": { border: 0 } }}>
                      <TableCell component="th" scope="row">
                        <Typography variant="body2">{row.application}</Typography>
                      </TableCell>
                      <TableCell>
                        <Stack direction="row" spacing={1}>
                          {command &&
                            localProviderDomains.map((domainId) => {
                              return (
                                <Link
                                  key={`${domainId}`}
                                  noWrap
                                  href="#"
                                  underline="none"
                                  color="inherit"
                                  onClick={() => {
                                    runApp(command, domainId);
                                  }}
                                >
                                  <Typography variant="body2">[{domainId}]</Typography>
                                </Link>
                              );
                            })}
                        </Stack>
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
    </Stack>
  );
}
