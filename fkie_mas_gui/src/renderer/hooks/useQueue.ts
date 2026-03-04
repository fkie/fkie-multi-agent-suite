import { useCallback, useRef, useState } from "react";

interface StatusItem {
  action: string;
  itemName: string;
  success: boolean;
  message: string;
}

export type TQueueProps<T> = {
  queue: T[];
  currentIndex: number;
  update: (list: T[]) => void;
  clear: () => void;
  get: () => T | null;
  success: (action: string) => StatusItem[];
  failed: (action: string) => StatusItem[];
  addStatus: (action: string, itemName: string, success: boolean, message: string) => void;
};

export default function useQueue<T>(onProgress: (progress: number) => void): TQueueProps<T> {
  const [queue, setQueue] = useState<T[]>([]);
  const [currentIndex, setCurrentIndex] = useState<number>(-1);
  const [resultStatus, setResultStatus] = useState<StatusItem[]>([]);

  // keep queue length stable for progress calculation
  const totalRef = useRef<number>(0);

  /**
   * Append items to the queue.
   * Starts processing automatically if queue was idle.
   */
  const update = useCallback(
    (list: T[]): void => {
      if (!list.length) return;

      setQueue((prev) => {
        const next = [...prev, ...list];
        totalRef.current = next.length;
        return next;
      });

      setCurrentIndex((prev) => (prev === -1 ? 0 : prev));
      onProgress(0);
    },
    [onProgress]
  );

  /**
   * Clear queue and reset all state.
   */
  const clear = useCallback((): void => {
    setQueue([]);
    setCurrentIndex(-1);
    setResultStatus([]);
    totalRef.current = 0;
    onProgress(0);
  }, [onProgress]);

  /**
   * Get current queue item.
   */
  const get = useCallback((): T | null => {
    return currentIndex >= 0 && currentIndex < queue.length ? queue[currentIndex] : null;
  }, [currentIndex, queue]);

  /**
   * Add execution result for current item and advance queue.
   */
  const addStatus = useCallback(
    (action: string, itemName: string, success: boolean, message: string) => {
      setResultStatus((prev) => [...prev, { action, itemName, success, message }]);

      setCurrentIndex((prev) => {
        const nextIndex = prev + 1;

        if (totalRef.current > 0) {
          onProgress((nextIndex / totalRef.current) * 100);
        }

        return nextIndex;
      });
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

  return {
    queue,
    currentIndex,
    update,
    clear,
    get,
    success,
    failed,
    addStatus,
  };
}
