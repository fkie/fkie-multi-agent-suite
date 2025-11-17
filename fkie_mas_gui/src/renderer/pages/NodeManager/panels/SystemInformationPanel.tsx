import { Box, Stack, Typography } from "@mui/material";
import { useContext, useEffect, useMemo, useState } from "react";
import JsonView from "react18-json-view";

import CopyButton from "@/renderer/components/UI/CopyButton";
import SearchBar from "@/renderer/components/UI/SearchBar";
import { RosContext } from "@/renderer/context/RosContext";
import { SettingsContext } from "@/renderer/context/SettingsContext";
import { SystemWarning } from "@/renderer/models";
import { Provider } from "@/renderer/providers";
import { generateUniqueId } from "@/renderer/utils";
import { JSONObject, TSystemInfo } from "@/types";

interface SystemInformationPanelProps {
  providerId?: string;
}

export default function SystemInformationPanel(props: SystemInformationPanelProps): JSX.Element {
  const { providerId = "" } = props;

  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);

  const [systemInfoContent, setSystemInfoContent] = useState<TSystemInfo | null>(null);
  const [provider, setProvider] = useState<Provider | undefined>();
  const [providerDetails, setProviderDetails] = useState<JSONObject | TSystemInfo | null>({});
  const [providerWarnings, setProviderWarnings] = useState<SystemWarning[]>([]);
  const [filter, setFilter] = useState("");
  const [backgroundColor, setBackgroundColor] = useState<string>(settingsCtx.get("backgroundColor") as string);

  useEffect(() => {
    setBackgroundColor(settingsCtx.get("backgroundColor") as string);
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
        return {
          ...object,
          [key]: nested,
        };
      }
      return { ...object };
    }, {});
  }

  useEffect(() => {
    if (provider?.isLocalHost) {
      setSystemInfoContent(filterNestObject(rosCtx.systemInfo));
    }
  }, [provider, filter, setSystemInfoContent]);

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
      // join warnings to one list
      let warnings: SystemWarning[] = [];
      for (const w of provider.warnings) {
        warnings = [...warnings, ...(w.warnings || [])];
      }
      setProviderWarnings(warnings);
    }
  }, [provider, filter]);

  useEffect(() => {
    setProvider(rosCtx.getProviderById(providerId));
  }, [providerId, rosCtx, rosCtx.getProviderById]);

  const createProviderDetails = useMemo(() => {
    return (
      <Stack sx={{ width: "100%" }}>
        <h5>Provider Details</h5>
        <JsonView
          src={providerDetails}
          dark={settingsCtx.get("useDarkMode") as boolean}
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
      </Stack>
    );
  }, [providerDetails, settingsCtx.changed]);

  const createSystemInfoContent = useMemo(() => {
    return (
      <Stack sx={{ width: "100%" }}>
        <h5>This System Information</h5>
        <JsonView
          src={systemInfoContent}
          dark={settingsCtx.get("useDarkMode") as boolean}
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
    );
  }, [systemInfoContent, settingsCtx.changed]);

  return (
    <Box height="100%" overflow="auto" sx={{ backgroundColor: backgroundColor }}>
      <Stack direction="column" margin="0.5em" spacing="0.5em">
        <SearchBar onSearch={(value) => setFilter(value.toLocaleLowerCase())} placeholder="filter" defaultValue="" />
        <Stack spacing={1} direction="row">
          <Typography variant="h5" sx={{ fontWeight: "bold" }}>
            Name:
          </Typography>
          <Typography variant="h5">{provider?.name()}</Typography>
          <CopyButton value={`${provider?.name()}`} />
        </Stack>
        {providerWarnings.length > 0 && (
          <Stack sx={{ width: "100%" }}>
            <h5>Warnings</h5>
            {providerWarnings.map((item) => {
              return (
                <Stack key={generateUniqueId()} sx={{ width: "100%" }}>
                  <Typography variant="subtitle1" color="red">
                    {item.msg}
                  </Typography>
                  <Typography paddingLeft={1} variant="body1">
                    {item.details}
                  </Typography>
                  {item.hint && (
                    <Typography paddingLeft={1} variant="body1" fontStyle="italic">
                      Hint: {item.hint}
                    </Typography>
                  )}
                </Stack>
              );
            })}
          </Stack>
        )}
        {providerDetails && createProviderDetails}
        {systemInfoContent && createSystemInfoContent}
      </Stack>
    </Box>
  );
}
