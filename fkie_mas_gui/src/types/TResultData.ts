import JSONObject from "./JsonObject";

export type TResultData = {
  result: boolean;
  message: string;
  data: JSONObject | unknown;
  error?: "not connected" | "running" | "max attempts" | "malformed response" | "error" | "runtime_error" | "connection closed";
};
