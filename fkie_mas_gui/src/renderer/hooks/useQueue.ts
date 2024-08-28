import { useCallback, useState } from "react";

interface StatusItem {
  action: string;
  itemName: string;
  success: boolean;
  message: string;
}

const useQueue = (onProgress: (progress: number) => void) => {
  // offers variable to store success and failed results
  const [resultStatus, setResultStatus] = useState<StatusItem[]>([]);
  const [queue, setQueue] = useState<[]>([]);
  const [currentIndex, setCurrentIndex] = useState<number>(-1);

  /**
   * Append items to the queue
   * @param list List with items.
   */
  const update = useCallback(
    (list: []) => {
      const doReset = queue.length === 0;
      setQueue((prev) => [...prev, ...list]);
      if (doReset) {
        onProgress(0);
        if (list.length > 0) setCurrentIndex(0);
      }
    },
    [setQueue, onProgress, queue, setCurrentIndex]
  );

  /** Clear queue and all result states */
  const clear = useCallback(() => {
    setQueue([]);
    setCurrentIndex(-1);
    setResultStatus([]);
  }, [setQueue, setCurrentIndex, setResultStatus]);

  /**
   * Returns item on the current index or null if index is invalid.
   * Change the current index by call next().
   */
  const get = useCallback(() => {
    if (currentIndex >= 0 && currentIndex < queue.length) {
      return queue[currentIndex];
    }
    return null;
  }, [currentIndex, queue]);

  /**
   * Increase the current index and update the progress state.
   */
  const next = useCallback(() => {
    const nextIndex = currentIndex + 1;
    if (nextIndex <= queue.length) {
      onProgress((nextIndex / queue.length) * 100);
    }
    setCurrentIndex(nextIndex);
  }, [currentIndex, onProgress, setCurrentIndex]);

  const addStatus = useCallback(
    (action: string, itemName: string, success: boolean, message: string) => {
      setResultStatus([...resultStatus, { action, itemName, success, message }]);
      next();
    },
    [resultStatus, setResultStatus, next]
  );

  const success = useCallback(
    (action: string) => {
      return resultStatus.filter((item) => item.success && action === item.action);
    },
    [resultStatus]
  );

  const failed = useCallback(
    (action: string) => {
      return resultStatus.filter((item) => !item.success && action === item.action);
    },
    [resultStatus]
  );

  return {
    update,
    clear,
    next,
    get,
    get queueItems() {
      return queue;
    },
    get size() {
      return queue.length;
    },
    get index() {
      return currentIndex;
    },
    success,
    failed,
    addStatus,
  };
};

export default useQueue;
