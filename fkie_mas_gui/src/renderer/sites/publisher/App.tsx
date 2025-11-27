import { Stack } from "@mui/material";
import { useContext, useEffect, useState } from "react";
import { LoggingContext } from "../../context/LoggingContext";
import { RosContext } from "../../context/RosContext";
import { SettingsContext } from "../../context/SettingsContext";
import { getFileName } from "../../models";
import TopicPublishPanel from "../../pages/NodeManager/panels/TopicPublishPanel";
import PublisherProvider from "../../providers/PublisherProvider";

interface IPublisherInfo {
  id: string;
  provider: PublisherProvider;
  topicName: string;
  topicType: string;
}

export default function PublisherApp(): JSX.Element {
  const logCtx = useContext(LoggingContext);
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const [pubInfo, setPubInfo] = useState<IPublisherInfo | null>(null);
  const [stopRequested, setStopRequested] = useState<string>("");

  async function initProvider(): Promise<void> {
    const queryString = window.location.search;
    console.log(`queryString: ${queryString}`);
    const urlParams = new URLSearchParams(queryString);
    const id = urlParams.get("id");
    const host = urlParams.get("host");
    const port = urlParams.get("port");
    const topic = urlParams.get("topicName");
    const topicType = urlParams.get("topicType");
    if (!host || !port) {
      logCtx.error(`invalid address ${host}:${port}`, "");
      return;
    }
    if (!topic) {
      logCtx.error(`invalid topic ${topic}`, "");
      return;
    }
    if (!topicType) {
      logCtx.error(`invalid topicType ${topicType}`, "");
      return;
    }
    if (!id) {
      logCtx.error(`no id found ${id}`, "");
      return;
    }
    document.title = `Publish - ${getFileName(topic)}`;
    const prov = new PublisherProvider(settingsCtx, host, "", Number.parseInt(port), false, logCtx);
    if (await prov.init()) {
      rosCtx.addProvider(prov);
      setPubInfo({
        id: id,
        provider: prov,
        topicName: topic,
        topicType: topicType,
      });
    } else {
      logCtx.error(`connection to ${host}:${port} failed`, "");
    }
  }

  async function stopPublisher(topic: string, provider: PublisherProvider): Promise<void> {
    // stop ros node for given topic
    logCtx.info(`Stopping publisher node for '${topic} on '${provider.name()}'`, "");
    // const result = await provider.stopPublisher(topic);
    // if (result) {
    //   logCtx.info(`Stopped publisher node for '${topic} on '${provider.name()}'`, "");
    // } else {
    //   logCtx.error(`Can not stop publisher node for: ${topic} on '${provider.name()}`, `${result}`);
    // }
    // close window on stop request
    window.publishManager?.close(stopRequested);
  }

  useEffect(() => {
    window.publishManager?.close(stopRequested);
    if (stopRequested) {
      if (pubInfo) {
        stopPublisher(pubInfo.topicName, pubInfo.provider);
      } else {
        // close window on stop request if no valid info is available
        window.publishManager?.close(stopRequested);
      }
    }
  }, [pubInfo, stopRequested]);

  useEffect(() => {
    // Anything in here is fired on component mount.
    window.publishManager?.onClose((id: string) => {
      setStopRequested(id);
    });
    initProvider();
    return (): void => {
      // Anything in here is fired on component unmount.
    };
  }, []);

  return (
    <Stack width="100%" height="100vh">
      {pubInfo && rosCtx.mapProviderRosNodes.size > 0 && (
        <TopicPublishPanel
          topicName={pubInfo.topicName}
          topicType={pubInfo.topicType}
          providerId={pubInfo.provider.id}
        />
      )}
    </Stack>
  );
}
