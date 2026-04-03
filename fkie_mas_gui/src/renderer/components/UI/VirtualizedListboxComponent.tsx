import {
  Children,
  cloneElement,
  createContext,
  forwardRef,
  isValidElement,
  useContext,
  useEffect,
  useRef,
} from "react";
import { ListChildComponentProps, VariableSizeList } from "react-window";

const LISTBOX_PADDING = 8;

type AutocompleteOptionProps = React.HTMLAttributes<HTMLLIElement> & {
  style?: React.CSSProperties;
};

type AutocompleteOptionElement = React.ReactElement<AutocompleteOptionProps>;
type AutocompleteListItemData = AutocompleteOptionElement[];

type OuterElementContextValue = React.HTMLAttributes<HTMLElement> | undefined;

const OuterElementContext = createContext<OuterElementContextValue>(undefined);

const OuterElementType = forwardRef<HTMLDivElement, React.HTMLAttributes<HTMLDivElement>>((props, ref) => {
  const outerProps = useContext(OuterElementContext);
  return <div ref={ref} {...outerProps} {...props} />;
});
OuterElementType.displayName = "AutocompleteOuterElementType";

function useResetCache(deps: unknown): React.RefObject<VariableSizeList<AutocompleteListItemData>> {
  const ref = useRef<VariableSizeList<AutocompleteListItemData>>(null);
  useEffect(() => {
    const list = ref.current;
    if (list !== null) {
      list.resetAfterIndex(0, true);
    }
  }, [deps]);
  return ref;
}

function renderRow(props: ListChildComponentProps<AutocompleteListItemData>): React.ReactElement | null {
  const { data, index, style } = props;
  const optionElement = data[index];

  const prevStyle: React.CSSProperties = optionElement.props.style ?? {};

  const combinedStyle: React.CSSProperties = {
    ...prevStyle,
    ...style,
    top: (style.top as number) + LISTBOX_PADDING,
  };

  return cloneElement<AutocompleteOptionProps>(optionElement, {
    style: combinedStyle,
  });
}

export const VirtualizedListboxComponent = forwardRef<HTMLDivElement, React.HTMLAttributes<HTMLElement>>(
  function VirtualizedListboxComponent(props, ref) {
    const { children, ...other } = props;

    const itemData: AutocompleteListItemData = [];
    const childrenArray = Children.toArray(children);

    for (const child of childrenArray) {
      if (isValidElement(child)) {
        itemData.push(child as AutocompleteOptionElement);
      }
    }

    const itemCount = itemData.length;
    const itemSize = 36;

    const listRef = useResetCache(itemCount);

    return (
      <div ref={ref}>
        <OuterElementContext.Provider value={other}>
          <VariableSizeList
            ref={listRef}
            itemData={itemData}
            height={Math.min(8, itemCount) * itemSize + 2 * LISTBOX_PADDING}
            width="100%"
            outerElementType={OuterElementType}
            innerElementType="ul"
            itemSize={() => itemSize}
            overscanCount={5}
            itemCount={itemCount}
          >
            {renderRow}
          </VariableSizeList>
        </OuterElementContext.Provider>
      </div>
    );
  }
);
VirtualizedListboxComponent.displayName = "VirtualizedListboxComponent";

export default VirtualizedListboxComponent;
