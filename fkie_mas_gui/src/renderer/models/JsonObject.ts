export type JSONValue =
  | string
  | number
  | boolean
  | { [x: string]: JSONValue }
  | Array<JSONValue>;

export default interface JSONObject {
  [x: string]: JSONValue;
}
