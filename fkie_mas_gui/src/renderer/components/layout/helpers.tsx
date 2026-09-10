import { IconButton, Tooltip } from "@mui/material";
import { Model } from "flexlayout-react";
import { emitToggleComponent } from "./events";
import { TLayoutTabConfig } from "./LayoutTabConfig";

export function pAddTabStickyButton(props: {
  model: Model;
  container: React.ReactNode[];
  id: string;
  title: string;
  component: string;
  setId: string;
  icon: React.ReactNode;
  tooltipLoc?:
    | "right"
    | "top"
    | "bottom"
    | "bottom-end"
    | "bottom-start"
    | "left-end"
    | "left-start"
    | "left"
    | "right-end"
    | "right-start"
    | "top-end"
    | "top-start"
    | undefined;
  force?: boolean;
  config?: TLayoutTabConfig;
}): void {
  if (props.force || !props.model.getNodeById(props.id)) {
    props.container.push(
      <Tooltip
        key={`tooltip-${props.id}`}
        title={props.title}
        placement={props.tooltipLoc || "right"}
        disableInteractive
      >
        <span>
          <IconButton
            onClick={() => {
              emitToggleComponent({
                id: props.id,
                title: props.title,
                closable: true,
                component: props.component,
                toNodeId: props.setId,
                config: props.config,
              });
            }}
          >
            {props.icon}
          </IconButton>
        </span>
      </Tooltip>
    );
  }
}
