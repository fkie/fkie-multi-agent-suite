import { useState } from 'react';

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
  const update = (list: []) => {
    if (queue.length === 0) {
      onProgress(0);
    }
    setQueue([...queue, ...list]);
  };

  /** Clear queue and all result states */
  const clear = () => {
    setQueue([]);
    setCurrentIndex(-1);
    setResultStatus([]);
  };

  /**
   * Returns item on the current index or null if index is invalid.
   * Change the current index by call next().
   */
  const get = () => {
    if (currentIndex >= 0 && currentIndex < queue.length) {
      return queue[currentIndex];
    }
    return null;
  };

  /**
   * Increase the current index and update the progress state.
   */
  const next = () => {
    const nextIndex = currentIndex + 1;
    if (nextIndex <= queue.length) {
      onProgress((nextIndex / queue.length) * 100);
    }
    setCurrentIndex(nextIndex);
  };

  const addStatus = (
    action: string,
    itemName: string,
    success: boolean,
    message: string,
  ) => {
    setResultStatus([...resultStatus, { action, itemName, success, message }]);
  };

  const success = (action: string) => {
    return resultStatus.filter(
      (item) => item.success && action === item.action,
    );
  };

  const failed = (action: string) => {
    return resultStatus.filter(
      (item) => !item.success && action === item.action,
    );
  };

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
