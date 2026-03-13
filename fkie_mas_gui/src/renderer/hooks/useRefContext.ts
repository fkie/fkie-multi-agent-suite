import { useEffect, useRef } from "react";

export function useRefContext<T>(obj: T): React.MutableRefObject<T> {
  const ref = useRef<T>(obj);

  useEffect(() => {
    ref.current = obj;
  }, [obj]);

  return ref;
}
