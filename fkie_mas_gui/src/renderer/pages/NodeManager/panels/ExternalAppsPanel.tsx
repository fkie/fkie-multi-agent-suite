import { generateUniqueId } from "@/renderer/utils";
import { Alert, AlertTitle, Button, Divider, Stack, Typography } from "@mui/material";
import { useCallback, useEffect, useState } from "react";

import { useRosContext } from "@/renderer/hooks/useRosContext";
import { Provider } from "@/renderer/providers";
import { contentToId, TContentId } from "../../../components/layout/LayoutTabConfig";

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

interface ExternalAppsProps {
  contentId?: TContentId;
}

export default function ExternalAppsPanel(props: ExternalAppsProps): JSX.Element {
  const { contentId } = props;
  const rosCtx = useRosContext();

  const [localProvider, setLocalProvider] = useState<Provider | undefined>();

  useEffect(() => {
    const localProviders = rosCtx.getLocalProvider();
    for (const prov of localProviders) {
      if (
        prov.isAvailable() &&
        prov.rosState !== undefined &&
        (prov.connection.domainId === contentId?.domainId || prov.id === contentId?.providerId)
      ) {
        setLocalProvider(prov);
        return;
      }
    }
  }, [contentId, rosCtx.providers, rosCtx.getLocalProvider]);

  const runApp = useCallback(
    async (command: RowType) => {
      if (!localProvider) return;
      if (localProvider.rosVersion === "2") {
        localProvider.rosRun({
          package: command.package,
          binary: command.binary,
          name: command.name,
          args: command.args,
          ros_args: command.ros_args,
        });
      } else {
        // TODO check ROS1 environment
        window.commandExecutor?.exec(null, command.commandROS1);
      }
    },
    [localProvider, window.commandExecutor]
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
      {!localProvider && (
        <Alert severity="warning">
          <AlertTitle>No local running daemons for this domain found</AlertTitle>
          Please start a local MAS daemon for this domain or join one to start ROS apps.
        </Alert>
      )}
      <Stack divider={<Divider flexItem />}>
        {applicationRows.map((row) => {
          let command: RowType | null = null;

          if (rosCtx.rosInfo) {
            if (rosCtx.rosInfo.version === "1" && row.commandROS1) command = row;
            if (rosCtx.rosInfo.version === "2" && row.package) command = row;
          }
          if (!command) return null;

          // keep a non-null reference for the click handler
          const cmd = command;
          return (
            <Button
              fullWidth
              disableRipple={false}
              disabled={!localProvider}
              key={`${row.application}-${contentToId(contentId)}`}
              sx={{ justifyContent: "flex-start", textTransform: "none", textAlign: "left", borderRadius: 0, py: 1 }}
              color="inherit"
              onClick={() => {
                runApp(cmd);
              }}
            >
              <Stack direction="column" spacing="3px" alignItems="flex-start" sx={{ width: "100%" }}>
                <Typography variant="subtitle1" sx={{ fontWeight: "bold" }}>
                  {row.application}
                </Typography>
                <Typography variant="body2" noWrap sx={{ width: "100%" }}>
                  {`${cmd.binary} ${cmd.args.join(" ")}`}
                </Typography>
              </Stack>
            </Button>
          );
        })}
      </Stack>
    </Stack>
  );
}
