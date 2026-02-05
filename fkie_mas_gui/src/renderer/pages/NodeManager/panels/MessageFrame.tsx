import { Divider, Stack, Tooltip, Typography } from "@mui/material";
import { useContext, useEffect, useRef, useState } from "react";
import JsonView from "react18-json-view";

import { CopyButton } from "@/renderer/components/UI";
import { SettingsContext } from "@/renderer/context/SettingsContext";
import useLocalStorage from "@/renderer/hooks/useLocalStorage";
import { RosQos, TSubscriberEventExt } from "@/renderer/models";
import { qosFromJson } from "@/renderer/models/RosQos";
import { findIn } from "@/renderer/utils";
import { emitCustomEvent, useCustomEventListener } from "react-custom-events";
import { TEventCollapsedState } from "../layout/events";

interface MessageFrameProps {
  event: TSubscriberEventExt;
  domainId?: string;
  qos?: RosQos;
  filter?: string;
  initRootCollapsed?: boolean;
  initCollapsed?: (string | number)[];
}

export default function MessageFrame(props: MessageFrameProps): JSX.Element {
  const {
    event,
    domainId = 0,
    qos = undefined,
    filter = "",
    initRootCollapsed = false,
    initCollapsed = ["stamp", "covariance"],
  } = props;
  const settingsCtx = useContext(SettingsContext);

  const isCtrlPressed = useRef(false);
  const [rootIsCollapsed, setRootIsCollapsed] = useState(initRootCollapsed);
  const [collapsedKeys, setCollapsedKeys] = useState<(string | number)[]>(initCollapsed);
  const [showWholeFilteredMessage] = useLocalStorage<boolean>("TopicEcho:showWholeFilteredMessage", false);
  const [tooltipDelay, setTooltipDelay] = useState<number>(settingsCtx.get("tooltipEnterDelay") as number);

  useEffect(() => {
    setTooltipDelay(settingsCtx.get("tooltipEnterDelay") as number);
  }, [settingsCtx.changed]);

  useEffect(() => {
    const onKeyDown = (e: KeyboardEvent) => {
      if (e.key === "Control") {
        isCtrlPressed.current = true;
      }
    };

    const onKeyUp = (e: KeyboardEvent) => {
      if (e.key === "Control") {
        isCtrlPressed.current = false;
      }
    };

    window.addEventListener("keydown", onKeyDown);
    window.addEventListener("keyup", onKeyUp);

    return () => {
      window.removeEventListener("keydown", onKeyDown);
      window.removeEventListener("keyup", onKeyUp);
    };
  }, []);

  useCustomEventListener(
    `collapse_state_${event.topic}`,
    (data: TEventCollapsedState) => {
      if (data.key === "") {
        setRootIsCollapsed(data.isCollapsed);
      } else if (!rootIsCollapsed) {
        setCollapsedKeys((prev) => {
          if (data.isCollapsed) {
            if (prev.includes(data.key)) {
              return prev;
            }
            return [...prev, data.key];
          }
          return prev.filter((item) => item !== data.key);
        });
      }
    },
    [event, rootIsCollapsed]
  );

  function isObject(item: object | Array<object> | null): boolean {
    return (item && typeof item === "object") || Array.isArray(item);
  }

  function filterJson(data: object | Array<object> | null, filter: string): object | Array<object> | null {
    if (filter.length < 2) {
      return data;
    }
    const result = {};
    if (isObject(data)) {
      for (const key in data) {
        if (findIn(filter, [key])) {
          result[key] = data[key];
          if (showWholeFilteredMessage) return data;
        } else if (isObject(data[key])) {
          const res = filterJson(data[key], filter);
          if (res && Object.keys(res).length > 0) {
            result[key] = res;
            if (showWholeFilteredMessage) return data;
          }
        } else {
          if (findIn(filter, [JSON.stringify(data[key])])) {
            result[key] = data[key];
            if (showWholeFilteredMessage) return data;
          }
        }
      }
    }
    return result;
  }
  return (
    <Stack key={`box-${event.key}`} direction="column">
      <Divider sx={{ borderStyle: "dashed" }} textAlign="left" flexItem>
        <Stack direction="row" spacing={1} alignItems="center">
          {event.seq === undefined && (
            <Typography fontStyle="italic" fontSize="0.8em" color="gray">
              {event.receivedIndex} --- {new Date(event.timestamp).toLocaleTimeString()}
            </Typography>
          )}
          {event.seq !== undefined && (
            <Typography fontStyle="italic" fontSize="0.8em" color="gray">
              {event.seq} --- {new Date(event.timestamp).toLocaleTimeString()}
            </Typography>
          )}
          <Tooltip
            title="Copy message with 'ros2 topic pub' command"
            placement="bottom"
            enterDelay={tooltipDelay}
            enterNextDelay={tooltipDelay}
            disableInteractive
          >
            <div>
              <CopyButton
                value={`ROS_DOMAIN_ID=${domainId} ros2 topic pub -1 --keep-alive 3 ${qos ? qosFromJson(qos).toString() : ""} ${event.topic} ${event.message_type} '${JSON.stringify(event)}'`}
                logText="ros2 pub string copied"
                fontSize="0.8em"
              />
            </div>
          </Tooltip>
        </Stack>
      </Divider>
      <Stack direction="row">
        <JsonView
          key={`${event.key}`}
          src={filterJson(event?.data, filter)}
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
            if (
              params.indexOrName === undefined &&
              params.depth === 1 &&
              JSON.stringify(Object.keys(params.node)) === JSON.stringify(Object.keys(event?.data || {}))
            ) {
              return rootIsCollapsed;
            }
            if (params.indexOrName === undefined) return false;
            const idx = Number.isInteger(params.indexOrName) ? JSON.stringify(params.node) : params.indexOrName;
            return collapsedKeys.includes(idx);
          }}
          onCollapse={(params: {
            isCollapsing: boolean;
            node: Record<string, unknown> | Array<unknown>;
            indexOrName: string | number | undefined;
            depth: number;
          }) => {
            if (
              params.indexOrName === undefined &&
              params.depth === 1 &&
              JSON.stringify(Object.keys(params.node)) === JSON.stringify(Object.keys(event?.data || {}))
            ) {
              if (!isCtrlPressed.current && rootIsCollapsed !== !params.isCollapsing) {
                emitCustomEvent(`collapse_state_${event.topic}`, {
                  isCollapsed: !params.isCollapsing,
                  key: "",
                } as TEventCollapsedState);
              }
              setRootIsCollapsed(!params.isCollapsing);
            }
            if (params.indexOrName !== undefined) {
              const idx = Number.isInteger(params.indexOrName) ? JSON.stringify(params.node) : params.indexOrName;
              if (!params.isCollapsing) {
                if (!collapsedKeys.includes(idx)) {
                  setCollapsedKeys((prev) => [...prev, idx as string | number]);
                  if (!isCtrlPressed.current) {
                    emitCustomEvent(`collapse_state_${event.topic}`, {
                      isCollapsed: true,
                      key: idx,
                    } as TEventCollapsedState);
                  }
                }
              } else {
                if (collapsedKeys.includes(idx)) {
                  setCollapsedKeys((prev) => prev.filter((item) => item !== idx));
                  if (!isCtrlPressed.current) {
                    emitCustomEvent(`collapse_state_${event.topic}`, {
                      isCollapsed: false,
                      key: idx,
                    } as TEventCollapsedState);
                  }
                }
              }
            }
          }}
        />
      </Stack>
    </Stack>
  );
}
