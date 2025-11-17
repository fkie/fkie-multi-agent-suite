import MoreVertSharpIcon from "@mui/icons-material/MoreVert";
import { useMemo } from "react";

import OverflowMenu from "@/renderer/components/UI/OverflowMenu";

export enum EMenuProvider {
  INFO = "INFO",
  DELETE = "DELETE",
  clipboard = "clipboard",
}

interface OverflowMenuProviderProps {
  providerName: string;
  providerId: string;
  onClick: (type: EMenuProvider, providerName: string, providerId: string) => void;
}

export default function OverflowMenuProvider(props: OverflowMenuProviderProps): JSX.Element {
  const { providerName, providerId, onClick } = props;
  const createMenu = useMemo(() => {
    return (
      <OverflowMenu
        icon={<MoreVertSharpIcon sx={{ fontSize: "inherit" }} />}
        options={[
          {
            name: "Info",
            key: `info-${providerId}`,
            onClick: (): void => {
              onClick(EMenuProvider.INFO, providerId, providerName);
            },
          },
          {
            name: "Delete",
            key: `delete-${providerId}`,
            onClick: (): void => {
              onClick(EMenuProvider.DELETE, providerId, providerName);
            },
          },
        ]}
        id="provider-options"
      />
    );
  }, [providerId, providerName]);

  return createMenu;
}
