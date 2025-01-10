import { useCallback, useEffect, useState } from "react";

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
  // offers variable to store success and failed results
  const [resultStatus, setResultStatus] = useState<StatusItem[]>([]);
  const [queue, setQueue] = useState<T[]>([]);
  const [addToQueue, setAddToQueue] = useState<T[]>([]);
  const [currentIndex, setCurrentIndex] = useState<number>(-1);

  useEffect(() => {
    const doReset = queue.length === 0;
    setQueue([...queue, ...addToQueue]);
    if (doReset) {
      onProgress(0);
      if (addToQueue.length > 0) setCurrentIndex(0);
    }
  }, [addToQueue]);

  /**
   * Append items to the queue
   * @param list List with items.
   */
  const update = useCallback(
    function (list: T[]): void {
      setAddToQueue(list);
    },
    [setAddToQueue]
  );

  /** Clear queue and all result states */
  const clear = useCallback(
    function (): void {
      setQueue([]);
      setCurrentIndex(-1);
      setResultStatus([]);
    },
    [setQueue, setCurrentIndex, setResultStatus]
  );

  /**
   * Returns item on the current index or null if index is invalid.
   * Change the current index by call next().
   */
  const get = useCallback(
    function (): T | null {
      if (currentIndex >= 0 && currentIndex < queue.length) {
        return queue[currentIndex];
      }
      return null;
    },
    [currentIndex, queue]
  );

  /**
   * Adds a status to the item of the current index.
   * Increase the current index and update the progress state.
   */
  const addStatus = useCallback(
    function (action: string, itemName: string, success: boolean, message: string): void {
      setResultStatus([...resultStatus, { action, itemName, success, message }]);
      // increase index and progress
      const nextIndex = currentIndex + 1;
      if (nextIndex <= queue.length) {
        onProgress((nextIndex / queue.length) * 100);
      }
      setCurrentIndex(nextIndex);
    },
    [resultStatus, setResultStatus, currentIndex, queue, onProgress, setCurrentIndex]
  );

  const success = useCallback(
    function (action: string): StatusItem[] {
      return resultStatus.filter((item) => item.success && action === item.action);
    },
    [resultStatus]
  );

  const failed = useCallback(
    function (action: string): StatusItem[] {
      return resultStatus.filter((item) => !item.success && action === item.action);
    },
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
