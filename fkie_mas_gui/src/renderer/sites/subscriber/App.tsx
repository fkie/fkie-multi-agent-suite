import { Stack } from "@mui/material";
import { useCallback, useContext, useEffect, useState } from "react";
import { LoggingContext } from "../../context/LoggingContext";
import { RosContext } from "../../context/RosContext";
import { SettingsContext } from "../../context/SettingsContext";
import { getFileName } from "../../models";
import TopicEchoPanel from "../../pages/NodeManager/panels/TopicEchoPanel";
import SubscriberProvider from "../../providers/SubscriberProvider";

interface ISubscriberInfo {
  id: string;
  provider: SubscriberProvider;
  topic: string;
  showOptions: boolean;
  noData: boolean;
}

export default function SubscriberApp() {
  const logCtx = useContext(LoggingContext);
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const [subInfo, setSubInfo] = useState<ISubscriberInfo | null>(null);
  const [stopRequested, setStopRequested] = useState<string>("");

  const initProvider = useCallback(async () => {
    const queryString = window.location.search;
    console.log(`queryString: ${queryString}`);
    const urlParams = new URLSearchParams(queryString);
    const id = urlParams.get("id");
    const host = urlParams.get("host");
    const port = urlParams.get("port");
    const topic = urlParams.get("topic");
    const showOptionsParam = urlParams.get("showOptions");
    const showOptions = showOptionsParam ? JSON.parse(showOptionsParam) : false;
    const noDataParam = urlParams.get("noData");
    const noData = noDataParam ? JSON.parse(noDataParam) : false;
    if (!host || !port) {
      logCtx.error(`invalid address ${host}:${port}`, "", false);
      return;
    }
    if (!topic) {
      logCtx.error(`invalid topic ${topic}`, "", false);
      return;
    }
    if (!id) {
      logCtx.error(`no id found ${id}`, "", false);
      return;
    }
    document.title = `Echo - ${getFileName(topic)}`;
    const prov = new SubscriberProvider(settingsCtx, host, "", parseInt(port), false, logCtx);
    if (await prov.init()) {
      rosCtx.addProvider(prov);
      setSubInfo({
        id: id,
        provider: prov,
        topic: topic,
        showOptions: showOptions,
        noData: noData,
      });
    } else {
      logCtx.error(`connection to ${host}:${port} failed`, "", false);
    }
  }, [setSubInfo]);

  const stopSubscriber = async (topic: string, provider: SubscriberProvider) => {
    // stop ros node for given topic
    logCtx.info(`Stopping subscriber node for '${topic} on '${provider.name()}'`, "");
    const result = await provider.stopSubscriber(topic);
    if (result) {
      logCtx.info(`Stopped subscriber node for '${topic} on '${provider.name()}'`, "");
    } else {
      logCtx.error(`Can not stop subscriber node for: ${topic} on '${provider.name()}`, `${result}`);
    }
    // close window on stop request
    window.subscriberManager?.close(stopRequested);
  };

  useEffect(() => {
    if (stopRequested) {
      if (subInfo) {
        stopSubscriber(subInfo.topic, subInfo.provider);
      } else {
        // close window on stop request if no valid info is available
        window.subscriberManager?.close(stopRequested);
      }
    }
  }, [subInfo, stopRequested]);

  useEffect(() => {
    // Anything in here is fired on component mount.
    window.subscriberManager?.onClose((id: string) => {
      setStopRequested(id);
    });
    initProvider();
    return () => {
      // Anything in here is fired on component unmount.
    };
  }, []);

  return (
    <Stack width="100%" height="100vh">
      {subInfo && rosCtx.mapProviderRosNodes.size > 0 && (
        <TopicEchoPanel
          showOptions
          defaultProvider={subInfo.provider.id}
          defaultTopic={subInfo.topic}
          defaultNoData={subInfo.noData}
        />
      )}
    </Stack>
  );
}
