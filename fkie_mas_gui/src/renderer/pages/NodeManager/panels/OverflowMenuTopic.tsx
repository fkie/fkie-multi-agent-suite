import ChatBubbleOutlineIcon from "@mui/icons-material/ChatBubbleOutline";
import ContentCopyIcon from "@mui/icons-material/ContentCopy";
import InfoOutlinedIcon from "@mui/icons-material/InfoOutlined";
import MoreVertSharpIcon from "@mui/icons-material/MoreVert";
import PlayCircleOutlineIcon from "@mui/icons-material/PlayCircleOutline";
import TroubleshootOutlinedIcon from "@mui/icons-material/TroubleshootOutlined";
import { Stack, Typography } from "@mui/material";
import { useMemo } from "react";

import OverflowMenu from "@/renderer/components/UI/OverflowMenu";

export enum EMenuTopic {
  INFO = "INFO",
  HZ = "HZ",
  ECHO = "ECHO",
  PUBLISH = "PUBLISH",
  clipboard = "clipboard",
}

interface OverflowMenuTopicProps {
  topicName: string;
  messageType: string;
  providerId: string;
  onClick: (type: EMenuTopic, topic: string, messageType: string, providerId: string) => void;
}

export default function OverflowMenuTopic(props: OverflowMenuTopicProps): JSX.Element {
  const { topicName, messageType, providerId, onClick } = props;

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
            onClick: (): void => {
              onClick(EMenuTopic.INFO, topicName, messageType, providerId);
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
            onClick: (): void => {
              onClick(EMenuTopic.HZ, topicName, messageType, providerId);
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
            onClick: (): void => {
              onClick(EMenuTopic.ECHO, topicName, messageType, providerId);
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
            onClick: (): void => {
              onClick(EMenuTopic.PUBLISH, topicName, messageType, providerId);
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
            onClick: (): void => {
              onClick(EMenuTopic.clipboard, topicName, messageType, providerId);
            },
          },
        ]}
        id="Topic Options"
      />
    );
  }, [topicName, providerId]);

  return createMenu;
}
