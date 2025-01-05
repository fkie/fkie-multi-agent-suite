import ContentCopyIcon from "@mui/icons-material/ContentCopy";
import InfoOutlinedIcon from "@mui/icons-material/InfoOutlined";
import MoreVertSharpIcon from "@mui/icons-material/MoreVert";
import SyncAltOutlinedIcon from "@mui/icons-material/SyncAltOutlined";
import { Stack, Typography } from "@mui/material";
import { forwardRef, useMemo } from "react";
import OverflowMenu from "../../../components/UI/OverflowMenu";

export enum EMenuService {
  INFO = "INFO",
  SERVICE_CALL = "SERVICE_CALL",
  clipboard = "clipboard",
}

interface OverflowMenuServiceProps {
  serviceName: string;
  providerId: string;
  onClick: (type: EMenuService, topic: string, providerId: string) => void;
}

const OverflowMenuService = forwardRef<HTMLDivElement, OverflowMenuServiceProps>(
  function OverflowMenuService(props, ref) {
    const { serviceName, providerId, onClick } = props;

    const createMenu = useMemo(() => {
      return (
        <OverflowMenu
          ref={ref}
          icon={<MoreVertSharpIcon sx={{ fontSize: "inherit" }} />}
          options={[
            {
              name: (
                <Stack direction="row" spacing={1} alignItems="center">
                  <InfoOutlinedIcon sx={{ fontSize: "inherit" }} />
                  <Typography>Info</Typography>
                </Stack>
              ),
              key: `info-${serviceName}`,
              onClick: () => {
                onClick(EMenuService.INFO, serviceName, providerId);
              },
            },
            {
              name: (
                <Stack direction="row" spacing={1} alignItems="center">
                  <SyncAltOutlinedIcon sx={{ fontSize: "inherit" }} />
                  <Typography>Call</Typography>
                </Stack>
              ),
              key: `service-call-${serviceName}`,
              onClick: () => {
                onClick(EMenuService.SERVICE_CALL, serviceName, providerId);
              },
            },
            {
              name: (
                <Stack direction="row" spacing={1} alignItems="center">
                  <ContentCopyIcon sx={{ fontSize: "inherit" }} />
                  <Typography>Copy name to Clipboard</Typography>
                </Stack>
              ),
              key: `clipboard-${serviceName}`,
              onClick: () => {
                onClick(EMenuService.clipboard, serviceName, providerId);
              },
            },
          ]}
          id="Service Options"
        />
      );
    }, [serviceName, providerId]);

    return createMenu;
  }
);

export default OverflowMenuService;
