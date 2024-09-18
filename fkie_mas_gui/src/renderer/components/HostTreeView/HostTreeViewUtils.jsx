import CircleIcon from "@mui/icons-material/Circle";
import MuiContrastIcon from "@mui/icons-material/Contrast";
import ReportIcon from "@mui/icons-material/Report";
import WarningIcon from "@mui/icons-material/Warning";
import BottomNavigationAction from "@mui/material/BottomNavigationAction";
import { blue, green, grey, orange, red, yellow } from "@mui/material/colors";
import { styled } from "@mui/material/styles";
import { DiagnosticLevel, RosNodeStatus, getMaxDiagnosticLevel } from "../../models";

const StyledBottomNavigationAction = styled(BottomNavigationAction)(() => ({
  padding: 0,
  margin: 0,
  maxWidth: 100,
  width: 60,
  minWidth: 60,
}));

const ContrastIcon = styled((props) => <MuiContrastIcon {...props} />)(() => ({
  transform: "rotate(-90deg)",
}));

const GroupStatus = {
  ALL_INACTIVE: 0,
  SOME_RUNNING: 1,
  ALL_RUNNING: 2,
};

const getNodeIcon = (status) => {
  switch (status) {
    case RosNodeStatus.RUNNING:
      return CircleIcon;

    case RosNodeStatus.DEAD:
      return WarningIcon;

    case RosNodeStatus.NOT_MONITORED:
      return ReportIcon;

    case RosNodeStatus.INACTIVE:
      return CircleIcon;

    default:
      return CircleIcon;
  }
};

const getNodeIconColor = (node, isDarkMode = false) => {
  switch (node.status) {
    case RosNodeStatus.RUNNING:
      switch (node.diagnosticLevel) {
        case DiagnosticLevel.OK:
          return isDarkMode ? green[600] : green[500];
        case DiagnosticLevel.WARN:
          return isDarkMode ? orange[600] : orange[400];
        case DiagnosticLevel.ERROR:
          return isDarkMode ? red[600] : red[500];
        case DiagnosticLevel.STALE:
          return isDarkMode ? yellow[700] : yellow[600];
        default:
          return isDarkMode ? blue[600] : blue[500];
      }
    case RosNodeStatus.DEAD:
      return isDarkMode ? orange[600] : orange[400];

    case RosNodeStatus.NOT_MONITORED:
      return isDarkMode ? blue[700] : blue[500];

    case RosNodeStatus.INACTIVE:
      return isDarkMode ? grey[600] : grey[500];

    default:
      return isDarkMode ? red[600] : red[500];
  }
};

const getGroupStatus = (treeItems) => {
  let groupStatus = GroupStatus.ALL_INACTIVE;
  let allRunning = true;
  treeItems.forEach((treeItem) => {
    if (treeItem.children && treeItem.children.length > 0) {
      const childrenStatus = getGroupStatus(treeItem.children);
      if (childrenStatus !== GroupStatus.ALL_INACTIVE) {
        groupStatus = GroupStatus.SOME_RUNNING;
      }
      if (childrenStatus !== GroupStatus.ALL_RUNNING) {
        allRunning = false;
      }
    }
    if (treeItem.node && treeItem.node.status === RosNodeStatus.RUNNING) {
      groupStatus = GroupStatus.SOME_RUNNING;
    } else if (treeItem.node) {
      allRunning = false;
    }
  });
  if (groupStatus === GroupStatus.SOME_RUNNING && allRunning) {
    groupStatus = GroupStatus.ALL_RUNNING;
  }
  return groupStatus;
};

const getGroupDiagnosticLevel = (treeItems) => {
  let groupLevel = DiagnosticLevel.OK;
  treeItems.forEach((treeItem) => {
    if (treeItem.children && treeItem.children.length > 0) {
      const childrenLevel = getGroupDiagnosticLevel(treeItem.children);
      groupLevel = getMaxDiagnosticLevel(groupLevel, childrenLevel);
    }
    if (treeItem.node) {
      groupLevel = getMaxDiagnosticLevel(groupLevel, treeItem.node.diagnosticLevel);
    }
  });
  return groupLevel;
};

const getGroupIcon = (treeItems) => {
  const groupStatus = getGroupStatus(treeItems);
  switch (groupStatus) {
    case GroupStatus.ALL_RUNNING:
      return CircleIcon;

    case GroupStatus.SOME_RUNNING:
      return ContrastIcon;

    case GroupStatus.ALL_INACTIVE:
      return CircleIcon;

    default:
      return CircleIcon;
  }
};

const getGroupIconColor = (treeItems, isDarkMode = false) => {
  const groupStatus = getGroupStatus(treeItems);
  switch (groupStatus) {
    case GroupStatus.ALL_RUNNING:
      switch (getGroupDiagnosticLevel(treeItems)) {
        case DiagnosticLevel.OK:
          return isDarkMode ? green[600] : green[500];
        case DiagnosticLevel.WARN:
          return isDarkMode ? orange[600] : orange[400];
        case DiagnosticLevel.ERROR:
          return isDarkMode ? red[600] : red[500];
        case DiagnosticLevel.STALE:
          return isDarkMode ? yellow[700] : yellow[500];
        default:
          return isDarkMode ? blue[600] : blue[500];
      }

    case GroupStatus.SOME_RUNNING:
      return isDarkMode ? green[700] : green[500];

    case GroupStatus.ALL_INACTIVE:
      return isDarkMode ? grey[600] : grey[500];

    default:
      return isDarkMode ? red[600] : red[500];
  }
};

const deleteFiles = async (logCtx, SSHCtx, host, files) => {
  // search the SSH credentials based on host
  const credentialHost = SSHCtx.getCredentialHost(host);
  if (!credentialHost) return "Host does not have valid SSH credentials";

  let command = "";
  files.forEach((f, index) => {
    if (f.length === 0) return;

    if (index < files.length - 1) {
      command = `${command} rm ${f} && `;
    } else {
      command = `${command} rm ${f} \n`;
    }
  });

  // delete logs
  const strResult = await SSHCtx.exec(credentialHost, command);
  return strResult.message;
};

const rosCleanPurge = async (logCtx, SSHCtx, host) => {
  // search the SSH credentials based on host
  const credentialHost = SSHCtx.getCredentialHost(host);
  if (!credentialHost) return "Host does not have valid SSH credentials";

  // purge logs
  // TODO: Make command configurable (perhaps logs are saved in a different folder)
  const strResult = await SSHCtx.exec(credentialHost, "rosclean purge -y");
  return strResult.message;
};

export {
  GroupStatus,
  StyledBottomNavigationAction,
  getGroupIcon,
  getGroupIconColor,
  getGroupStatus,
  getNodeIcon,
  getNodeIconColor,
};
