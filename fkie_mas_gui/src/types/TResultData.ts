import JSONObject from "./JsonObject";

export type TResultData = {
  result: boolean;
  message: string;
  data: JSONObject | unknown;
};
