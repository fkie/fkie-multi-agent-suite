import ChatBubbleOutlineIcon from "@mui/icons-material/ChatBubbleOutline";
import ContentCopyIcon from "@mui/icons-material/ContentCopy";
import InfoOutlinedIcon from "@mui/icons-material/InfoOutlined";
import MoreVertSharpIcon from "@mui/icons-material/MoreVert";
import PlayCircleOutlineIcon from "@mui/icons-material/PlayCircleOutline";
import TroubleshootOutlinedIcon from "@mui/icons-material/TroubleshootOutlined";
import { Stack, Typography } from "@mui/material";
import { forwardRef, useMemo } from "react";
import OverflowMenu from "../../../components/UI/OverflowMenu";

export enum EMenuTopic {
  INFO = "INFO",
  HZ = "HZ",
  ECHO = "ECHO",
  PUBLISH = "PUBLISH",
  clipboard = "clipboard",
}

interface OverflowMenuTopicProps {
  topicName: string;
  providerId: string;
  onClick: (type: EMenuTopic, topic: string, providerId: string) => void;
}

const OverflowMenuTopic = forwardRef<HTMLDivElement, OverflowMenuTopicProps>(function OverflowMenuTopic(props, ref) {
  const { topicName, providerId, onClick } = props;

  const createMenu = useMemo(() => {
    return (
      <OverflowMenu
        ref={ref}
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
              onClick(EMenuTopic.INFO, topicName, providerId);
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
              onClick(EMenuTopic.HZ, topicName, providerId);
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
              onClick(EMenuTopic.ECHO, topicName, providerId);
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
              onClick(EMenuTopic.PUBLISH, topicName, providerId);
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
              onClick(EMenuTopic.clipboard, topicName, providerId);
            },
          },
        ]}
        id="Topic Options"
      />
    );
  }, [topicName, providerId]);

  return createMenu;
});

export default OverflowMenuTopic;
