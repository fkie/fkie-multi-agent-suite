import ExpandMoreIcon from "@mui/icons-material/ExpandMore";
import { Box, Button, Stack, Typography } from "@mui/material";
import { useCallback, useContext, useEffect, useMemo, useState } from "react";
import JsonView from "react18-json-view";

import { colorFromHostname } from "@/renderer/components/UI";
import CopyButton from "@/renderer/components/UI/CopyButton";
import SearchBar from "@/renderer/components/UI/SearchBar";
import Tag from "@/renderer/components/UI/Tag";
import NavigationContext from "@/renderer/context/NavigationContext";
import { RosContext } from "@/renderer/context/RosContext";
import { SettingsContext } from "@/renderer/context/SettingsContext";
import useLocalStorage from "@/renderer/hooks/useLocalStorage";
import { SystemWarning } from "@/renderer/models";
import { Provider } from "@/renderer/providers";
import { EVENT_PROVIDER_WARNINGS } from "@/renderer/providers/eventTypes";
import { EventProviderWarnings } from "@/renderer/providers/events";
import { generateUniqueId, tsStr } from "@/renderer/utils";
import { JSONObject, TSystemInfo } from "@/types";
import { useCustomEventListener } from "react-custom-events";

interface SystemInformationPanelProps {
  providerId?: string;
}

export default function SystemInformationPanel(props: SystemInformationPanelProps): JSX.Element {
  const { providerId = "" } = props;

  const navCtx = useContext(NavigationContext);
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);

  const [showProviderDetails, setShowProviderDetails] = useLocalStorage("DetailsPanel:showProviderDetails", true);
  const [systemInfoContent, setSystemInfoContent] = useState<TSystemInfo | null>(null);
  const [provider, setProvider] = useState<Provider | undefined>();
  const [providerDetails, setProviderDetails] = useState<JSONObject | TSystemInfo | null>(null);
  const [providerWarnings, setProviderWarnings] = useState<SystemWarning[]>([]);
  const [filter, setFilter] = useState("");
  const [backgroundColor, setBackgroundColor] = useState<string>(settingsCtx.get("backgroundColor") as string);
  const [colorizeHosts, setColorizeHosts] = useState<boolean>(settingsCtx.get("colorizeHosts") as boolean);
  const [useDarkMode, setUseDarkMode] = useState<boolean>(settingsCtx.get("useDarkMode") as boolean);

  useEffect(() => {
    setBackgroundColor(settingsCtx.get("backgroundColor") as string);
    setColorizeHosts(settingsCtx.get("colorizeHosts") as boolean);
    setUseDarkMode(settingsCtx.get("useDarkMode") as boolean);
  }, [settingsCtx.changed]);

  /** filter dictionaries with system information by given filter */
  function filterNestObject(item: TSystemInfo | JSONObject | null): TSystemInfo | JSONObject | null {
    if (!item) return item;
    return Object.keys(item).reduce((object, key) => {
      let addKey = false;
      let nested = item[key];
      switch (typeof nested) {
        case "object":
          nested = filterNestObject(item[key]);
          if (!["{}", "null"].includes(JSON.stringify(nested))) {
            addKey = true;
          }
          break;
        case "string":
          if (nested?.toLocaleLowerCase().includes(filter)) {
            addKey = true;
          }
          break;
        case "number":
          if (nested.toString().toLocaleLowerCase().includes(filter)) {
            addKey = true;
          }
          break;
        case "function":
          nested = nested();
          if (nested?.toLocaleLowerCase().includes(filter)) {
            addKey = true;
          }
          break;
        default:
          break;
      }
      if (key.toLocaleLowerCase().includes(filter)) {
        addKey = true;
      }
      if (addKey) {
        Object.assign(object, { [key]: nested });
        return object;
      }
      return object;
    }, {});
  }

  useEffect(() => {
    if (provider?.isLocalHost) {
      setSystemInfoContent(filterNestObject(rosCtx.systemInfo));
    }
  }, [provider, filter]);

  const updateWarnings: () => void = useCallback(() => {
    if (!provider) return;
    // join warnings to one list
    let warnings: SystemWarning[] = [];
    for (const w of provider.warnings) {
      warnings = [...warnings, ...(w.warnings || [])];
    }
    setProviderWarnings(warnings);
  }, [provider]);

  useCustomEventListener(EVENT_PROVIDER_WARNINGS, (data: EventProviderWarnings) => {
    if (provider?.id === data.provider.id) {
      updateWarnings();
    }
  });

  useEffect(() => {
    if (provider) {
      const infoContent: JSONObject = {};
      infoContent.URI = `${provider.host()}:${provider.connection.port}`;
      infoContent.ROS_VERSION = provider.rosState.ros_version ? provider.rosState.ros_version : "unknown";
      infoContent.ROS_DISTRO = provider.rosState.ros_distro ? provider.rosState.ros_distro : "unknown";
      if (provider.rosState.ros_version === "1") {
        infoContent.ROS_MASTER_URI = provider.rosState.masteruri ? provider.rosState.masteruri : "unknown";
      }
      if (provider.rosState.ros_version === "2") {
        infoContent.ROS_DOMAIN_ID = provider.rosState.ros_domain_id ? provider.rosState.ros_domain_id : "default";
        infoContent.RMW_IMPLEMENTATION = provider.systemEnv.RMW_IMPLEMENTATION
          ? provider.systemEnv.RMW_IMPLEMENTATION
          : "default";
      } else {
        infoContent.NETWORK_ID = provider.rosState.ros_domain_id ? provider.rosState.ros_domain_id : "unknown";
      }
      infoContent["System Information"] = JSON.parse(JSON.stringify(provider.systemInfo));
      infoContent["System Environment"] = JSON.parse(JSON.stringify(provider.systemEnv));
      // infoContent['Platform details'] = provider.platform_details;
      infoContent.Hostnames = provider.hostnames;
      if (provider.errorDetails) {
        infoContent["Last error"] = provider.errorDetails;
      }
      const versionDate = provider.daemonVersion.date ? ` (${provider.daemonVersion.date})` : "";
      infoContent["Daemon version"] = `${provider.daemonVersion.version}${versionDate}`;
      setProviderDetails(filterNestObject(infoContent));
      updateWarnings();
    }
  }, [provider, filter]);

  useEffect(() => {
    setProvider(rosCtx.getProviderById(providerId));
  }, [providerId, rosCtx]);

  const getHostStyle = useCallback(
    (providerName: string | undefined) => {
      if (providerName && colorizeHosts) {
        return {
          borderTopStyle: "solid",
          borderTopColor: colorFromHostname(providerName),
          borderTopWidth: "0.4em",
        };
      }
      return {};
    },
    [settingsCtx.changed]
  );

  const createProviderDetailsView = useMemo(() => {
    if (!provider) return <></>;
    const rmwImplementation = provider.systemEnv.RMW_IMPLEMENTATION as string;
    return (
      <Stack
        key={providerId}
        // spacing={1}
        alignItems="left"
        marginBottom={1.5}
      >
        <Stack paddingTop={0} marginBottom={0.5} sx={getHostStyle(provider.name())}>
          <Typography
            variant="subtitle1"
            style={{
              // cursor: "pointer",
              color: "#fff",
              backgroundColor: "#16a085",
            }}
            align="center"
          >
            <Stack spacing={0} sx={{ fontWeight: "bold", m: 0, paddingTop: "0.2em" }}>
              <Typography variant="subtitle2" align="center">
                {provider.connection.uri}
              </Typography>
              <Box>
                {provider.name()} <CopyButton value={`${provider?.name()}`} />
              </Box>
            </Stack>
          </Typography>
        </Stack>
        <Stack spacing={0.5}>
          <Stack direction="row" spacing={0.5}>
            <Tag
              color={provider.daemon ? "default" : "error"}
              title="daemon:"
              // title={`${RosNodeStatusInfo[node.status]}`}
              text={provider.daemon ? "running" : "not running"}
              wrap
            />
          </Stack>
          <Stack direction="row" spacing={0.5}>
            <Tag
              color={provider.discovery ? "default" : "warning"}
              title="discovery:"
              // title={`${RosNodeStatusInfo[node.status]}`}
              text={provider.discovery ? "running" : "not running"}
              wrap
            />
          </Stack>
          <Stack direction="row" spacing={0.5}>
            <Tag
              color={provider.daemon ? "default" : "error"}
              title="nodes:"
              text={`${navCtx.selectedNodes.length}`}
              wrap
            />
          </Stack>
          {rmwImplementation && (
            <Stack direction="row" spacing={0.5}>
              <Tag
                color={"default"}
                title="RMW_IMPLEMENTATION:"
                // title={`${RosNodeStatusInfo[node.status]}`}
                text={rmwImplementation}
                wrap
              />
            </Stack>
          )}
          {provider.hostnames && (
            <Stack direction="row" spacing={0.5}>
              <Tag
                color={"default"}
                title="host:"
                // title={`${RosNodeStatusInfo[node.status]}`}
                text={JSON.stringify(provider.hostnames)}
                wrap
              />
            </Stack>
          )}
          <Box height="100%" overflow="auto" sx={{ backgroundColor: backgroundColor }}>
            <Stack direction="column" spacing="0.5em" alignItems="left">
              {providerWarnings.length > 0 && (
                <Stack sx={{ width: "100%" }}>
                  <Typography variant="caption" fontWeight="bold">
                    Warnings
                  </Typography>
                  {providerWarnings.map((item) => {
                    return (
                      <Stack key={generateUniqueId()} sx={{ width: "100%" }}>
                        <Stack direction="row" spacing={1}>
                          {item.timestamp && (
                            <Typography variant="body1" fontSize="0.9em" color="#ff9800">
                              {tsStr(new Date(item.timestamp * 1000 || 0))}:
                            </Typography>
                          )}
                          <Typography variant="body1" fontSize="0.9em" color="#ff9800">
                            {item.msg}
                          </Typography>
                        </Stack>
                        <Typography paddingLeft={1} variant="body1" fontSize="0.8em" color="#ff9800">
                          {item.details}
                        </Typography>
                        {item.hint && (
                          <Typography paddingLeft={1} variant="body1" fontSize="0.9em" fontStyle="italic">
                            Hint: {item.hint}
                          </Typography>
                        )}
                      </Stack>
                    );
                  })}
                </Stack>
              )}
              <Stack
                direction="row"
                alignItems="center"
                onClick={() => {
                  setShowProviderDetails((prev) => !prev);
                }}
              >
                <ExpandMoreIcon
                  style={{ transform: `${showProviderDetails ? "" : "rotate(-90deg)"}` }}
                  fontSize="small"
                />
                <Button
                  size="small"
                  style={{
                    textTransform: "none",
                    color: "inherit",
                  }}
                >
                  <Typography variant="caption" fontWeight="bold">
                    Details:
                  </Typography>
                </Button>
              </Stack>
              {showProviderDetails && (providerDetails || systemInfoContent) && (
                <SearchBar
                  onSearch={(value) => setFilter(value.toLocaleLowerCase())}
                  placeholder="filter"
                  defaultValue=""
                />
              )}
              {showProviderDetails && providerDetails && (
                <JsonView
                  src={providerDetails}
                  dark={useDarkMode}
                  theme="a11y"
                  enableClipboard={false}
                  ignoreLargeArray={false}
                  collapseObjectsAfterLength={3}
                  displaySize={"collapsed"}
                  collapsed={(params: {
                    node: Record<string, unknown> | Array<unknown>; // Object or array
                    indexOrName: number | string | undefined;
                    depth: number;
                    size: number; // Object's size or array's length
                  }) => {
                    if (params.indexOrName === undefined) return false;
                    return true;
                  }}
                />
              )}
              {showProviderDetails && systemInfoContent && (
                <Stack sx={{ width: "100%" }}>
                  <Typography variant="caption" fontWeight="bold">
                    This System Information
                  </Typography>
                  <JsonView
                    src={systemInfoContent}
                    dark={useDarkMode}
                    theme="a11y"
                    enableClipboard={false}
                    ignoreLargeArray={false}
                    // collapseObjectsAfterLength={3}
                    displaySize={"collapsed"}
                    collapsed={(params: {
                      node: Record<string, unknown> | Array<unknown>; // Object or array
                      indexOrName: number | string | undefined;
                      depth: number;
                      size: number; // Object's size or array's length
                    }) => {
                      if (params.indexOrName === undefined) return false;
                      return true;
                    }}
                  />
                </Stack>
              )}
            </Stack>
          </Box>
        </Stack>
      </Stack>
    );
  }, [
    navCtx.selectedProviders,
    navCtx.selectedNodes,
    provider,
    showProviderDetails,
    providerDetails,
    systemInfoContent,
    useDarkMode,
    providerWarnings,
    backgroundColor,
  ]);

  return createProviderDetailsView;
}
