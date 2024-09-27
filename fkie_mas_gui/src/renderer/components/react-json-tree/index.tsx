// ES6 + inline style port of JSONViewer https://bitbucket.org/davevedder/react-json-viewer/
// all credits and original code to the author
// Dave Vedder <veddermatic@gmail.com> http://www.eskimospy.com/
// port by Daniele Zannotti http://www.github.com/dzannotti <dzannotti@me.com>

import { useMemo } from "react";
import type { StylingValue, Theme } from "react-base16-styling";
import { invertTheme } from "react-base16-styling";
import JSONNode from "./JSONNode.js";
import createStylingFromTheme from "./createStylingFromTheme.js";
import type {
  CommonExternalProps,
  GetItemString,
  IsCustomNode,
  LabelRenderer,
  ShouldExpandNodeInitially,
} from "./types.js";

interface Props extends Partial<CommonExternalProps> {
  data: unknown;
  theme?: Theme;
  invertTheme?: boolean;
}

const identity = (value: any) => value;
const expandRootNode: ShouldExpandNodeInitially = (_keyPath, _data, level) => level === 0;
const defaultItemString: GetItemString = (_type, _data, itemType, itemString) => (
  <span>
    {itemType} {itemString}
  </span>
);
const defaultLabelRenderer: LabelRenderer = ([label]) => <span>{label}:</span>;
const noCustomNode: IsCustomNode = () => false;

export function JSONTree({
  data: value,
  theme,
  invertTheme: shouldInvertTheme,
  keyPath = ["root"],
  labelRenderer = defaultLabelRenderer,
  valueRenderer = identity,
  shouldExpandNodeInitially = expandRootNode,
  hideRoot = false,
  getItemString = defaultItemString,
  postprocessValue = identity,
  isCustomNode = noCustomNode,
  collectionLimit = 50,
  sortObjectKeys = false,
  onCollapse = () => {},
  onExpand = () => {},
}: Props) {
  const styling = useMemo(
    () => createStylingFromTheme(shouldInvertTheme ? invertTheme(theme) : theme),
    [theme, shouldInvertTheme]
  );

  return (
    <ul {...styling("tree")}>
      <JSONNode
        keyPath={hideRoot ? [] : keyPath}
        value={postprocessValue(value)}
        isCustomNode={isCustomNode}
        styling={styling}
        labelRenderer={labelRenderer}
        valueRenderer={valueRenderer}
        shouldExpandNodeInitially={shouldExpandNodeInitially}
        hideRoot={hideRoot}
        getItemString={getItemString}
        postprocessValue={postprocessValue}
        collectionLimit={collectionLimit}
        sortObjectKeys={sortObjectKeys}
        onCollapse={onCollapse}
        onExpand={onExpand}
      />
    </ul>
  );
}

export type {
  CommonExternalProps,
  GetItemString,
  IsCustomNode,
  Key,
  KeyPath,
  LabelRenderer,
  PostprocessValue,
  ShouldExpandNodeInitially,
  SortObjectKeys,
  Styling,
  ValueRenderer
} from "./types.js";
export type { StylingValue };

