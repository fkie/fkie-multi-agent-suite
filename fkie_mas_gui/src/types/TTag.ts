import React from "react";

export type TTag = {
  id: string;
  data: string | object;
  color: string;
  tooltip?: string;
  onClick?: (event: React.MouseEvent) => void;
};
