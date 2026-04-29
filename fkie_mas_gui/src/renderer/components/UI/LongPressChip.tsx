import Box from "@mui/material/Box";
import Chip, { ChipProps } from "@mui/material/Chip";
import CircularProgress from "@mui/material/CircularProgress";
import React, { useRef, useState } from "react";

type Props = ChipProps & {
  onLongPress: () => void;
  delay?: number;
};

const LongPressChip: React.FC<Props> = ({ onLongPress, delay = 800, onClick, size, ...rest }) => {
  const timerRef = useRef<number | null>(null);
  const intervalRef = useRef<number | null>(null);
  const longPressTriggered = useRef(false);

  const [progress, setProgress] = useState(0);

  const start = () => {
    // Reset state at pointer down
    longPressTriggered.current = false;
    setProgress(0);

    const startTime = Date.now();

    // Update progress indicator while pressing
    intervalRef.current = window.setInterval(() => {
      const elapsed = Date.now() - startTime;
      setProgress(Math.min((elapsed / delay) * 100, 100));
    }, 100);

    // Fire long press after the given delay
    timerRef.current = window.setTimeout(() => {
      onLongPress();
      longPressTriggered.current = true;
    }, delay);
  };

  const clear = () => {
    // Clear timers when pointer is released or cancelled
    if (timerRef.current) clearTimeout(timerRef.current);
    if (intervalRef.current) clearInterval(intervalRef.current);

    timerRef.current = null;
    intervalRef.current = null;
    setProgress(0);
  };

  return (
    <Box position="relative" display="inline-flex">
      {progress > 0 && (
        <CircularProgress
          variant="determinate"
          value={progress}
          size={size === "small" ? 20 : 26}
          sx={{
            position: "absolute",
            top: 2,
            left: 4,
            transform: "translate(-50%, -50%)",
            pointerEvents: "none",
          }}
        />
      )}

      <Chip
        {...rest}
        size={size}
        onContextMenu={(e) => e.preventDefault()} // prevent default context menu on long press (important for mobile)
        onPointerDown={(e) => {
          // Only react to primary button for mouse; always for touch/pen
          if (e.button !== 0 && e.pointerType === "mouse") return;
          start();
          e.currentTarget.setPointerCapture(e.pointerId);
        }}
        onPointerUp={(e) => {
          clear();
          e.currentTarget.releasePointerCapture(e.pointerId);
        }}
        onPointerLeave={clear}
        onPointerCancel={clear}
        onClick={(e) => {
          // Suppress normal click if a long press was triggered
          if (!longPressTriggered.current && onClick) {
            onClick(e);
          }
        }}
      />
    </Box>
  );
};

export default LongPressChip;
