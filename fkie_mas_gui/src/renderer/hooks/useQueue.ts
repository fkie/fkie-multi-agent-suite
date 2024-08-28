import { useCallback, useEffect, useState } from "react";

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
  const [addToQueue, setAddToQueue] = useState<[]>([]);
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
    (list: []) => {
      setAddToQueue(list);
    },
    [setAddToQueue]
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
   * Adds a status to the item of the current index.
   * Increase the current index and update the progress state.
   */
  const addStatus = useCallback(
    (action: string, itemName: string, success: boolean, message: string) => {
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
    queue,
    currentIndex,
    update,
    clear,
    get,
    success,
    failed,
    addStatus,
  };
};

export default useQueue;
