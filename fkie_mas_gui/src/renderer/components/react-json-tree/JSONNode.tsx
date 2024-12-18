import JSONArrayNode from "./JSONArrayNode.js";
import JSONIterableNode from "./JSONIterableNode.js";
import JSONObjectNode from "./JSONObjectNode.js";
import JSONValueNode from "./JSONValueNode.js";
import objType from "./objType.js";
import type { CommonInternalProps } from "./types.js";

interface Props extends CommonInternalProps {
  value: unknown;
}

export default function JSONNode({
  getItemString,
  keyPath,
  labelRenderer,
  styling,
  value,
  valueRenderer,
  isCustomNode,
  ...rest
}: Props) {
  const nodeType = isCustomNode(value) ? "Custom" : objType(value);

  const simpleNodeProps = {
    getItemString,
    keyPath,
    labelRenderer,
    nodeType,
    styling,
    value,
    valueRenderer,
  };

  const nestedNodeProps = {
    ...rest,
    ...simpleNodeProps,
    data: value,
    isCustomNode,
  };

  switch (nodeType) {
    case "Object":
    case "Error":
    case "WeakMap":
    case "WeakSet":
      return <JSONObjectNode {...nestedNodeProps} />;
    case "Array":
      return <JSONArrayNode {...nestedNodeProps} />;
    case "Iterable":
    case "Map":
    case "Set":
      return <JSONIterableNode {...nestedNodeProps} />;
    case "String":
      return <JSONValueNode key={keyPath[0]} {...simpleNodeProps} valueGetter={(raw: string) => `"${raw}"`} />;
    case "Number":
      return <JSONValueNode key={keyPath[0]} {...simpleNodeProps} />;
    case "Boolean":
      return <JSONValueNode key={keyPath[0]} {...simpleNodeProps} valueGetter={(raw) => (raw ? "true" : "false")} />;
    case "Date":
      return <JSONValueNode key={keyPath[0]} {...simpleNodeProps} valueGetter={(raw) => raw.toISOString()} />;
    case "Null":
      return <JSONValueNode key={keyPath[0]} {...simpleNodeProps} valueGetter={() => "null"} />;
    case "Undefined":
      return <JSONValueNode key={keyPath[0]} {...simpleNodeProps} valueGetter={() => "undefined"} />;
    case "Function":
    case "Symbol":
      return <JSONValueNode key={keyPath[0]} {...simpleNodeProps} valueGetter={(raw) => raw.toString()} />;
    case "Custom":
      return <JSONValueNode key={keyPath[0]} {...simpleNodeProps} />;
    default:
      return <JSONValueNode key={keyPath[0]} {...simpleNodeProps} valueGetter={() => `<${nodeType}>`} />;
  }
}
