import AppsIcon from "@mui/icons-material/Apps";
import { useCallback, useMemo } from "react";

import OverflowMenu, { OverflowMenuItem } from "@/renderer/components/UI/OverflowMenu";
import { Provider } from "@/renderer/providers";
import { generateUniqueId } from "@/renderer/utils";

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

interface OverflowMenuExternalAppsProps {
  provider: Provider;
}

export default function OverflowMenuExternalApps(props: OverflowMenuExternalAppsProps): JSX.Element | null {
  const { provider } = props;

  const runApp = useCallback(
    async (command: RowType) => {
      if (!provider) return;
      if (provider.rosVersion === "2") {
        provider.rosRun({
          package: command.package,
          binary: command.binary,
          name: command.name,
          args: command.args,
          ros_args: command.ros_args,
        });
      } else {
        window.commandExecutor?.exec(null, command.commandROS1);
      }
    },
    [window.commandExecutor]
  );

  const createOptions: () => OverflowMenuItem[] = useCallback(() => {
    if (!provider) {
      const errorResult: OverflowMenuItem = {
        name: "No Provider available",
        key: "not-available",
        onClick: (): void => {},
      };
      return [errorResult];
    }

    if (provider.rosVersion === "1" && !window.commandExecutor) {
      const errorResult: OverflowMenuItem = {
        name: "No executer to start local nodes available",
        key: "not-executor",
        onClick: (): void => {},
      };
      return [errorResult];
    }

    const rosVersion = provider.rosVersion;

    return applicationRows
      .filter((row) => {
        if (rosVersion === "1" && row.commandROS1) return true;
        if (rosVersion === "2" && row.package) return true;
        return false;
      })
      .map((row) => {
        let command: RowType | null = null;
        if (rosVersion === "1" && row.commandROS1) command = row;
        if (rosVersion === "2" && row.package) command = row;
        const result: OverflowMenuItem = {
          name: row.application || row.name,
          key: `${row.application.replaceAll(" ", "-")}-${row.name}`,
          onClick: (): void => {
            if (command) runApp(command);
          },
        };
        return result;
      });
  }, [window.commandExecutor]);

  const createMenu = useMemo(() => {
    return (
      <OverflowMenu
        icon={<AppsIcon sx={{ fontSize: "inherit" }} />}
        options={createOptions()}
        id="node-details-options"
        tooltip="Start external apps on this host"
      />
    );
  }, [provider, window.commandExecutor]);

  return createMenu;
}
