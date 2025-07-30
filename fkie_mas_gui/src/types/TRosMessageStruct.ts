import JSONObject, { JSONValue } from "./JsonObject";

export type TRosMessageStruct = {
  type: string;
  name: string;
  useNow: boolean | undefined;
  def: TRosMessageStruct[] | undefined;
  default_value?: JSONValue;
  value?: JSONValue | TRosMessageStruct[];
  is_array: boolean;
};

export function rosMessageStructToString(
  msgStruct: TRosMessageStruct | TRosMessageStruct[] | undefined,
  asDict: boolean,
  withEmptyFields: boolean
): string | JSONValue {
  if (!msgStruct) return "{}";
  const result: { [fieldName: string]: JSONValue | JSONValue[] } = {};
  const struct: TRosMessageStruct[] = Array.isArray(msgStruct) ? msgStruct : msgStruct.def ? msgStruct.def : [];
  for (const field of struct) {
    if (field.def && field.def.length === 0) {
      // simple types
      if (field.value || withEmptyFields) {
        if (field.value || typeof field.value === "boolean" || field.type.startsWith("bool")) {
          if (field.is_array) {
            // split by ",", but not if comma is inside '"'
            const value_matches = (field.value as string)?.match(/(?:[^\s,"']+|"[^"]*")+/g);
            const values = value_matches
              ? value_matches.map((item) => {
                  return item.trim().replaceAll(/^"|"$/g, "");
                })
              : [];
            // TODO: add check for arrays with constant length
            result[field.name] = values.map((element) => {
              return str2typedValue(element, field.type);
            });
          } else {
            result[field.name] = str2typedValue(field.value, field.type);
          }
        } else if (field.default_value) {
          result[field.name] = field.default_value;
        } else {
          result[field.name] = "";
        }
      }
    } else if (field.is_array) {
      const resultArray: JSONValue | JSONValue[] = [];
      // it is a complex field type
      const val: TRosMessageStruct[] = (field?.value ? field?.value : field.def) as TRosMessageStruct[];
      for (const arrayElement of val) {
        resultArray.push(rosMessageStructToString(arrayElement, true, withEmptyFields));
      }
      // append created array
      if (resultArray.length > 0) {
        result[field.name] = resultArray;
      } else if (withEmptyFields) {
        // value array is empty: create a new array from definition
        result[field.name] = [rosMessageStructToString(field.def, true, withEmptyFields)];
      }
    } else {
      // it is a complex type, call subroutine
      if (field.useNow) {
        result[field.name] = "now";
      } else {
        const subResult = rosMessageStructToString(field.def, true, withEmptyFields);
        if (Object.keys(subResult).length > 0) {
          result[field.name] = subResult;
        }
      }
    }
  }
  return asDict ? result : dictToString(result);
}

function str2typedValue(
  value: TRosMessageStruct[] | JSONValue | undefined,
  valueType: string
): number | boolean | string {
  let result: number | boolean | string = `${value}`;
  if (valueType.search("int") !== -1) {
    result = Number(value);
  } else if (valueType.search("float") !== -1 || valueType.search("double") !== -1) {
    result = Number(value);
  } else if (valueType.startsWith("bool")) {
    try {
      result = ["yes", "true", "t", "y", "1"].includes(result.toLowerCase());
    } catch {
      // Do nothing
    }
  }
  return result;
}

function dictToString(dict: JSONObject): string {
  const json = JSON.stringify(dict);
  const result = json.replace(
    /("(\\u[a-zA-Z0-9]{4}|\\[^u]|[^\\"])*"(\s*:)?|\b(true|false|null)\b|-?\d+(?:\.\d*)?(?:[eE][+-]?\d+)?)/g,
    (match) => {
      let cls = "number";
      if (/^"/.test(match)) {
        if (/:$/.test(match)) {
          cls = "key";
        } else {
          cls = "string";
        }
      } else if (/true|false/.test(match)) {
        cls = "boolean";
      } else if (/null/.test(match)) {
        cls = "null";
      }
      return cls === "key" || cls === "number" || cls === "boolean" ? match.replaceAll('"', "") : match;
    }
  );
  return result.replaceAll(":", ": ");
}
