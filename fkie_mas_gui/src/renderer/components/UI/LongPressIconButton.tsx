import Box from "@mui/material/Box";
import CircularProgress from "@mui/material/CircularProgress";
import IconButton, { IconButtonProps } from "@mui/material/IconButton";
import React, { useRef, useState } from "react";

type Props = IconButtonProps & {
  onLongPress: () => void;
  delay?: number;
};

const LongPressIconButton: React.FC<Props> = ({ onLongPress, delay = 800, onClick, children, size, ...rest }) => {
  const timerRef = useRef<number | null>(null);
  const intervalRef = useRef<number | null>(null);

  const [progress, setProgress] = useState(0);
  const longPressTriggered = useRef(false);

  const start = () => {
    longPressTriggered.current = false;
    setProgress(0);

    const startTime = Date.now();

    intervalRef.current = window.setInterval(() => {
      const elapsed = Date.now() - startTime;
      setProgress(Math.min((elapsed / delay) * 100, 100));
    }, 100);

    timerRef.current = window.setTimeout(() => {
      onLongPress();
      longPressTriggered.current = true;
    }, delay);
  };

  const clear = () => {
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
          size={size === "medium" ? 32 : 24}
          sx={{
            position: "absolute",
            top: size === "medium" ? 4 : 2,
            left: size === "medium" ? 4 : 2,
            pointerEvents: "none",
          }}
        />
      )}

      <IconButton
        {...rest}
        size={size}
        onContextMenu={(e) => e.preventDefault()} // important for mobile
        onPointerDown={(e) => {
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
          if (!longPressTriggered.current && onClick) {
            onClick(e);
          }
        }}
      >
        {children}
      </IconButton>
    </Box>
  );
};

export default LongPressIconButton;
