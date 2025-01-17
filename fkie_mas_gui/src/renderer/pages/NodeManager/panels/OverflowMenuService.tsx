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
  serviceType: string;
  providerId: string;
  onClick: (type: EMenuService, service: string, serviceType: string, providerId: string) => void;
}

const OverflowMenuService = forwardRef<HTMLDivElement, OverflowMenuServiceProps>(
  function OverflowMenuService(props, ref) {
    const { serviceName, serviceType, providerId, onClick } = props;

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
              onClick: (): void => {
                onClick(EMenuService.INFO, serviceName, serviceType, providerId);
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
              onClick: (): void => {
                onClick(EMenuService.SERVICE_CALL, serviceName, serviceType, providerId);
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
              onClick: (): void => {
                onClick(EMenuService.clipboard, serviceName, serviceType, providerId);
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
