import { useEffect, useState } from "react";

export function useEditorKeyboard(onClose: () => void) {
  const [escapePressCount, setEscapePressCount] = useState(0);

  useEffect(() => {
    function handler(event: KeyboardEvent) {
      if (event.key === "Escape") {
        setEscapePressCount((prev) => prev + 1);

        if (escapePressCount === 1) {
          onClose();
        }

        setTimeout(() => {
          setEscapePressCount(0);
        }, 500);
      }
    }

    document.addEventListener("keydown", handler);

    return () => document.removeEventListener("keydown", handler);
  }, [escapePressCount, onClose]);
}
