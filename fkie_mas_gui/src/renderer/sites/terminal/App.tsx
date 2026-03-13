import { Stack } from "@mui/material";
import { useEffect, useState } from "react";

import { useAlwaysCurrentRef } from "@/renderer/hooks/useAlwaysCurrentRef";
import { useLoggingContext } from "@/renderer/hooks/useLoggingContext";
import { useRosContext } from "@/renderer/hooks/useRosContext";
import { useSettingsContext } from "@/renderer/hooks/useSettingsContext";
import SingleTerminalPanel from "../../pages/NodeManager/panels/SingleTerminalPanel";
import { CmdType, cmdTypeFromString } from "../../providers";
import TerminalProvider from "../../providers/TerminalProvider";

interface ITerminalInfo {
  id: string;
  provider: TerminalProvider;
  info: CmdType;
  node: string;
  screen: string;
  cmd: string;
}

export default function TerminalApp(): JSX.Element {
  const logCtx = useLoggingContext();
  const rosCtx = useRosContext();
  const settingsCtx = useSettingsContext();
  const logCtxRef = useAlwaysCurrentRef(logCtx);
  const settingsCtxRef = useAlwaysCurrentRef(settingsCtx);
  const [paramInfo, setParamInfo] = useState<ITerminalInfo | null>(null);

  async function initProvider(): Promise<void> {
    const queryString = window.location.search;
    console.log(`queryString: ${queryString}`);
    const urlParams = new URLSearchParams(queryString);
    const id = urlParams.get("id");
    const host = urlParams.get("host");
    const port = urlParams.get("port");
    const info = urlParams.get("info");
    const node = urlParams.get("node");
    const screen = urlParams.get("screen");
    const cmd = urlParams.get("cmd");
    if (!host || !port) {
      logCtx.error(`invalid address ${host}:${port}`, "");
      return;
    }
    if (!id) {
      logCtx.error(`no id found ${id}`, "");
      return;
    }
    const nodeName = node ? node : "bash";
    document.title = `${info} - ${nodeName}`;
    const prov = new TerminalProvider(logCtxRef, settingsCtxRef, host, "", Number.parseInt(port), false);
    if (await prov.init()) {
      rosCtx.addProvider(prov);
      setParamInfo({
        id: id,
        provider: prov,
        info: cmdTypeFromString(info),
        node: node ? node : "",
        screen: screen ? screen : "",
        cmd: cmd ? cmd : "",
      });
    } else {
      logCtx.error(`connection to ${host}:${port} failed`, "");
    }
  }

  useEffect(() => {
    // Anything in here is fired on component mount.
    window.terminalManager?.onClose((id: string) => {
      // close window on stop request
      window.terminalManager?.close(id);
    });
    initProvider();
    return (): void => {
      // Anything in here is fired on component unmount.
    };
  }, []);

  return (
    <Stack width="100%" height="100vh">
      {paramInfo && rosCtx.mapProviderRosNodes.size > 0 && (
        <SingleTerminalPanel
          id={paramInfo.id}
          type={paramInfo.info}
          providerId={paramInfo.provider.id}
          nodeName={paramInfo.node}
          screen={paramInfo.screen}
          cmd={paramInfo.cmd}
        />
      )}
    </Stack>
  );
}
