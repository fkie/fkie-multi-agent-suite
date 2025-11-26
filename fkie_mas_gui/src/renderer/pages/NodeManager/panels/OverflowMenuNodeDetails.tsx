import MoreVertSharpIcon from "@mui/icons-material/MoreVert";
import { useMemo } from "react";

import OverflowMenu from "@/renderer/components/UI/OverflowMenu";
import useLocalStorage from "@/renderer/hooks/useLocalStorage";

export default function OverflowMenuNodeDetails(): JSX.Element {
  const [showNodeInfo, setShowNodeInfo] = useLocalStorage("DetailsPanel:showNodeInfo", false);
  const [showPublishers, setShowPublishers] = useLocalStorage("DetailsPanel:showPublishers", true);
  const [showSubscribers, setShowSubscribers] = useLocalStorage("DetailsPanel:showSubscribers", true);
  const [showServices, setShowServices] = useLocalStorage("DetailsPanel:showServices", false);
  const [showConnections, setShowConnections] = useLocalStorage("DetailsPanel:showConnections", true);
  const [showLaunchParameter, setShowLaunchParameter] = useLocalStorage("DetailsPanel:showLaunchParameter", true);

  const createMenu = useMemo(() => {
    return (
      <OverflowMenu
        icon={<MoreVertSharpIcon sx={{ fontSize: "inherit" }} />}
        options={[
          {
            name: `${showNodeInfo ? "Hide" : "Show"} Node Info`,
            key: "toggle-node-info",
            onClick: (): void => {
              setShowNodeInfo((prev) => !prev);
            },
          },
          {
            name: `${showPublishers ? "Hide" : "Show"} Publishers`,
            key: "toggle-publishers",
            onClick: (): void => {
              setShowPublishers((prev) => !prev);
            },
          },
          {
            name: `${showSubscribers ? "Hide" : "Show"} Subscribers`,
            key: "toggle-subscribers",
            onClick: (): void => {
              setShowSubscribers((prev) => !prev);
            },
          },
          {
            name: `${showServices ? "Hide" : "Show"} Services`,
            key: "toggle-services",
            onClick: (): void => {
              setShowServices((prev) => !prev);
            },
          },
          {
            name: `${showConnections ? "Hide" : "Show"} Connections`,
            key: "toggle-connections",
            onClick: (): void => {
              setShowConnections((prev) => !prev);
            },
          },
          {
            name: `${showLaunchParameter ? "Hide" : "Show"} Launch Parameter`,
            key: "toggle-launch-parameter",
            onClick: (): void => {
              setShowLaunchParameter((prev) => !prev);
            },
          },
        ]}
        id="node-details-options"
      />
    );
  }, [showNodeInfo, showPublishers, showSubscribers, showServices, showConnections, showLaunchParameter]);

  return createMenu;
}
