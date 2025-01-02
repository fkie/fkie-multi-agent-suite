import React from "react";

export type TTag = {
  id: string;
  data: string | React.ElementType;
  color: string;
  tooltip?: string;
  onClick?: (event: React.MouseEvent) => void;
};
