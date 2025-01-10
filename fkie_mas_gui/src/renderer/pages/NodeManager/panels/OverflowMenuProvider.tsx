import MoreVertSharpIcon from "@mui/icons-material/MoreVert";
import { forwardRef, useMemo } from "react";
import OverflowMenu from "../../../components/UI/OverflowMenu";

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

const OverflowMenuProvider = forwardRef<HTMLDivElement, OverflowMenuProviderProps>(
  function OverflowMenuProvider(props, ref) {
    const { providerName, providerId, onClick } = props;
    const createMenu = useMemo(() => {
      return (
        <OverflowMenu
          ref={ref}
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
);

export default OverflowMenuProvider;
