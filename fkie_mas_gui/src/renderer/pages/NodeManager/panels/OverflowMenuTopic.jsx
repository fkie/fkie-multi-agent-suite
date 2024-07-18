import ChatBubbleOutlineIcon from "@mui/icons-material/ChatBubbleOutline";
import ContentCopyIcon from "@mui/icons-material/ContentCopy";
import InfoOutlinedIcon from "@mui/icons-material/InfoOutlined";
import MoreVertSharpIcon from "@mui/icons-material/MoreVert";
import PlayCircleOutlineIcon from "@mui/icons-material/PlayCircleOutline";
import TroubleshootOutlinedIcon from "@mui/icons-material/TroubleshootOutlined";
import { Stack, Typography } from "@mui/material";
import PropTypes from "prop-types";
import { useMemo } from "react";
import OverflowMenu from "../../../components/UI/OverflowMenu";

function OverflowMenuTopic({ onClick, topicName, providerId }) {
  const createMenu = useMemo(() => {
    return (
      <OverflowMenu
        icon={<MoreVertSharpIcon sx={{ fontSize: "inherit" }} />}
        options={[
          {
            name: (
              <Stack direction="row" spacing={1} alignItems={"center"}>
                <InfoOutlinedIcon sx={{ fontSize: "inherit" }} />
                <Typography>Info</Typography>
              </Stack>
            ),
            key: `info-${topicName}`,
            onClick: () => {
              onClick("INFO", topicName, providerId);
            },
          },
          {
            name: (
              <Stack direction="row" spacing={1} alignItems={"center"}>
                <TroubleshootOutlinedIcon sx={{ fontSize: "inherit" }} />
                <Typography>Statistics</Typography>
              </Stack>
            ),
            key: `stats-${topicName}`,
            onClick: () => {
              onClick("HZ", topicName, providerId);
            },
          },
          {
            name: (
              <Stack direction="row" spacing={1} alignItems={"center"}>
                <ChatBubbleOutlineIcon sx={{ fontSize: "inherit" }} />
                <Typography>Echo</Typography>
              </Stack>
            ),
            key: `echo-${topicName}`,
            onClick: () => {
              onClick("ECHO", topicName, providerId);
            },
          },
          {
            name: (
              <Stack direction="row" spacing={1} alignItems={"center"}>
                <PlayCircleOutlineIcon sx={{ fontSize: "inherit" }} />
                <Typography>Publish</Typography>
              </Stack>
            ),
            key: `publish-${topicName}`,
            onClick: () => {
              onClick("PUBLISH", topicName, providerId);
            },
          },
          {
            name: (
              <Stack direction="row" spacing={1} alignItems={"center"}>
                <ContentCopyIcon sx={{ fontSize: "inherit" }} />
                <Typography>Copy name to Clipboard</Typography>
              </Stack>
            ),
            key: `clipboard-${topicName}`,
            onClick: () => {
              onClick("clipboard", topicName, providerId);
            },
          },
        ]}
        id="Topic Options"
      />
    );
  }, [topicName, providerId]);

  return createMenu;
}

OverflowMenuTopic.propTypes = {
  onClick: PropTypes.func.isRequired,
  topicName: PropTypes.string.isRequired,
  providerId: PropTypes.string.isRequired,
};

export default OverflowMenuTopic;
