import { Dispatch, SetStateAction, useCallback, useEffect, useRef, useState } from "react";

// See: https://usehooks-ts.com/react-hook/use-event-listener
import useEventListener from "./useEventListener";

declare global {
  interface WindowEventMap {
    "local-storage": CustomEvent;
  }
}

type SetValue<T> = Dispatch<SetStateAction<T>>;

// A wrapper for "JSON.parse()"" to support "undefined" value
function parseJSON<T>(value: string | null): T | undefined {
  try {
    return value === "undefined" ? undefined : JSON.parse(value ?? "");
  } catch {
    console.error("parsing error on", { value });
    return undefined;
  }
}

export default function useLocalStorage<T>(key: string, initialValue: T): [T, SetValue<T>] {
  // Get from local storage then
  // parse stored json or return initialValue
  const readValue = useCallback((): T => {
    // Prevent build error "window is undefined" but keep keep working
    if (typeof window === "undefined") {
      return initialValue;
    }

    try {
      const item = window.localStorage.getItem(key);
      return item ? (parseJSON(item) as T) : initialValue;
    } catch (error) {
      const msg = `Error reading localStorage key "${key}":`
      console.warn(msg, error);
      return initialValue;
    }
  }, [initialValue, key]);

  // State to store our value
  // Pass initial state function to useState so logic is only executed once
  const [storedValue, setStoredValue] = useState<T>(readValue);

  const setValueRef = useRef<SetValue<T>>();

  setValueRef.current = (value): void => {
    // Prevent build error "window is undefined" but keeps working
    if (typeof window === "undefined") {
      const msg = `Tried setting localStorage key "${key}" even though environment is not a client`;
      console.warn(msg);
    }

    try {
      // Allow value to be a function so we have the same API as useState
      const newValue = value instanceof Function ? value(storedValue) : value;

      // Save to local storage
      window.localStorage.setItem(key, JSON.stringify(newValue));

      // Save state
      setStoredValue(newValue);

      const keyEvent = new CustomEvent("local-storage", { detail: key });

      // We dispatch a custom event so every useLocalStorage hook are notified
      window.dispatchEvent(keyEvent);
    } catch (error) {
      const msg = `Error setting localStorage key "${key}":`;
      console.warn(msg, error);
    }
  };

  // Return a wrapped version of useState's setter function that ...
  // ... persists the new value to localStorage.
  const setValue: SetValue<T> = useCallback((value) => setValueRef.current?.(value), []);

  useEffect(() => {
    setStoredValue(readValue());
  }, []);

  const handleStorageChange = useCallback(
    (data: CustomEvent) => {
      if (data.detail === key) {
        setStoredValue(readValue());
        data.stopPropagation();
      }
    },
    [key, readValue]
  );

  // this only works for other documents, not the current one
  // useEventListener('storage', handleStorageChange);

  // this is a custom event, triggered in writeValueToLocalStorage
  // See: useLocalStorage()
  useEventListener("local-storage", handleStorageChange);

  return [storedValue, setValue];
}
