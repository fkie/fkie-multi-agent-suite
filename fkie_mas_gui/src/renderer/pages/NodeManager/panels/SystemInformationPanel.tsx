import { SystemWarning } from "@/renderer/models";
import { Provider } from "@/renderer/providers";
import { JSONObject } from "@/types";
import { Box, Stack, Typography } from "@mui/material";
import { forwardRef, useContext, useEffect, useMemo, useState } from "react";
import { JSONTree } from "react-json-tree";
import { CopyButton, SearchBar } from "../../../components";
import { RosContext } from "../../../context/RosContext";
import { SettingsContext } from "../../../context/SettingsContext";
import { darkThemeJson } from "../../../themes/darkTheme";
import { lightThemeJson } from "../../../themes/lightTheme";
import { generateUniqueId } from "../../../utils";

interface SystemInformationPanelProps {
  providerId?: string;
}

const SystemInformationPanel = forwardRef<HTMLDivElement, SystemInformationPanelProps>(
  function SystemInformationPanel(props, ref) {
    const { providerId = "" } = props;

    const rosCtx = useContext(RosContext);
    const settingsCtx = useContext(SettingsContext);

    const [systemInfoContent, setSystemInfoContent] = useState(null);
    const [provider, setProvider] = useState<Provider | undefined>();
    const [providerDetails, setProviderDetails] = useState({});
    const [providerWarnings, setProviderWarnings] = useState<SystemWarning[]>([]);
    const [filter, setFilter] = useState("");
    const [backgroundColor, setBackgroundColor] = useState<string>(settingsCtx.get("backgroundColor") as string);

    useEffect(() => {
      setBackgroundColor(settingsCtx.get("backgroundColor") as string);
    }, [settingsCtx.changed]);

    /** filter dictionaries with system information by given filter */
    function filterNestObject(item) {
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
      if (provider && provider.isLocalHost) {
        setSystemInfoContent(filterNestObject(rosCtx.systemInfo));
      }
      // eslint-disable-next-line react-hooks/exhaustive-deps
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
          infoContent.ROS_DOMAIN_ID = provider.rosState.ros_domain_id ? provider.rosState.ros_domain_id : "unknown";
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
        provider.warnings.forEach((w) => {
          warnings = [...warnings, ...w.warnings];
        });
        setProviderWarnings(warnings);
      }
      // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [provider, filter]);

    useEffect(() => {
      setProvider(rosCtx.getProviderById(providerId));
    }, [providerId, rosCtx, rosCtx.getProviderById]);

    const createProviderDetails = useMemo(() => {
      return (
        <Stack sx={{ width: "100%" }}>
          <h5>Provider Details</h5>
          <JSONTree
            data={providerDetails}
            sortObjectKeys={true}
            theme={settingsCtx.get("useDarkMode") ? darkThemeJson : lightThemeJson}
            invertTheme={false}
            hideRoot={true}
          />
        </Stack>
      );
    }, [providerDetails, settingsCtx.changed]);

    const createSystemInfoContent = useMemo(() => {
      return (
        <Stack sx={{ width: "100%" }}>
          <h5>This System Information</h5>
          <JSONTree
            data={systemInfoContent}
            sortObjectKeys={true}
            theme={settingsCtx.get("useDarkMode") ? darkThemeJson : lightThemeJson}
            invertTheme={false}
            hideRoot={true}
          />
        </Stack>
      );
    }, [systemInfoContent, settingsCtx.changed]);

    return (
      <Box ref={ref} height="100%" overflow="auto" sx={{ backgroundColor: backgroundColor }}>
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
);

export default SystemInformationPanel;
