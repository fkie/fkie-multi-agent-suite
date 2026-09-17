import { JSONObject } from "@/types";

export interface ActionEvent {
  action_name: string;
  action_type: string;
  type: "feedback" | "result"; // feedback or result
  goal_id: string;
  status: string; // e.g. "accepted", "executing", "succeeded", "canceled", "aborted"
  data: JSONObject | null;
  timestamp: number;
  message: string;
}
