import { MutableRefObject, useCallback, useMemo, useRef, useState } from "react";

interface StatusItem {
  action: string;
  itemName: string;
  success: boolean;
  message: string;
}

export type TQueueProps<T> = {
  /** Reactive snapshot – for rendering only. */
  queue: T[];
  /** Reactive snapshot – for rendering only. */
  currentIndex: number;
  /** Synchronous mirror of the queue – use this for de-duplication checks. */
  queueRef: MutableRefObject<T[]>;
  /** Synchronous mirror of the current index. */
  currentIndexRef: MutableRefObject<number>;
  update: (list: T[]) => void;
  clear: () => void;
  get: () => T | null;
  /** Read an item without relying on state captured in a closure. */
  getAt: (index: number) => T | null;
  /** Synchronous predicate check against the pending queue. */
  has: (predicate: (item: T) => boolean) => boolean;
  success: (action: string) => StatusItem[];
  failed: (action: string) => StatusItem[];
  addStatus: (action: string, itemName: string, success: boolean, message: string) => void;
};

export default function useQueue<T>(onProgress: (progress: number) => void): TQueueProps<T> {
  const [queue, setQueue] = useState<T[]>([]);
  const [currentIndex, setCurrentIndex] = useState<number>(-1);
  const [resultStatus, setResultStatus] = useState<StatusItem[]>([]);

  // synchronous mirrors: state updates are async and would allow duplicate enqueues
  const queueRef = useRef<T[]>([]);
  const currentIndexRef = useRef<number>(-1);
  // keep queue length stable for progress calculation
  const totalRef = useRef<number>(0);
  // guard: only one status per queue index may advance the queue
  const statusForIndexRef = useRef<number>(-1);

  /**
   * Append items to the queue.
   * Starts processing automatically if the queue was idle.
   */
  const update = useCallback(
    (list: T[]): void => {
      if (!list.length) return;

      const next = [...queueRef.current, ...list];
      queueRef.current = next;
      totalRef.current = next.length;
      setQueue(next);

      if (currentIndexRef.current === -1) {
        currentIndexRef.current = 0;
        setCurrentIndex(0);
      }

      onProgress(totalRef.current > 0 ? (currentIndexRef.current / totalRef.current) * 100 : 0);
    },
    [onProgress]
  );

  /**
   * Clear queue and reset all state.
   */
  const clear = useCallback((): void => {
    queueRef.current = [];
    currentIndexRef.current = -1;
    totalRef.current = 0;
    statusForIndexRef.current = -1;
    setQueue([]);
    setCurrentIndex(-1);
    setResultStatus([]);
    onProgress(0);
  }, [onProgress]);

  /**
   * Read an item by index from the synchronous mirror.
   */
  const getAt = useCallback((index: number): T | null => {
    return index >= 0 && index < queueRef.current.length ? queueRef.current[index] : null;
  }, []);

  /**
   * Get current queue item.
   */
  const get = useCallback((): T | null => getAt(currentIndexRef.current), [getAt]);

  /**
   * Synchronous check whether a matching item is still pending.
   */
  const has = useCallback((predicate: (item: T) => boolean): boolean => {
    return queueRef.current.slice(Math.max(currentIndexRef.current, 0)).some(predicate);
  }, []);

  /**
   * Add execution result for current item and advance queue.
   */
  const addStatus = useCallback(
    (action: string, itemName: string, success: boolean, message: string): void => {
      const index = currentIndexRef.current;
      if (index < 0) {
        console.warn(`[useQueue] addStatus ignored, queue is idle: ${action} ${itemName}`);
        return;
      }
      if (statusForIndexRef.current === index) {
        // a handler reported twice for the same item -> would skip/duplicate the next item
        console.warn(`[useQueue] duplicate addStatus for index ${index}: ${action} ${itemName} (${message})`);
        return;
      }
      statusForIndexRef.current = index;

      setResultStatus((prev) => [...prev, { action, itemName, success, message }]);

      const nextIndex = index + 1;
      currentIndexRef.current = nextIndex;
      setCurrentIndex(nextIndex);

      if (totalRef.current > 0) {
        onProgress((nextIndex / totalRef.current) * 100);
      }
    },
    [onProgress]
  );

  /**
   * Get successful results by action.
   */
  const success = useCallback(
    (action: string): StatusItem[] => resultStatus.filter((item) => item.success && item.action === action),
    [resultStatus]
  );

  /**
   * Get failed results by action.
   */
  const failed = useCallback(
    (action: string): StatusItem[] => resultStatus.filter((item) => !item.success && item.action === action),
    [resultStatus]
  );

  // stable object identity: an inline object literal re-triggers every effect depending on it
  return useMemo(
    () => ({
      queue,
      currentIndex,
      queueRef,
      currentIndexRef,
      update,
      clear,
      get,
      getAt,
      has,
      success,
      failed,
      addStatus,
    }),
    [queue, currentIndex, update, clear, get, getAt, has, success, failed, addStatus]
  );
}
