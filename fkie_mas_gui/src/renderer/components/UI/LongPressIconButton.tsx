import Box from "@mui/material/Box";
import CircularProgress from "@mui/material/CircularProgress";
import IconButton, { IconButtonProps } from "@mui/material/IconButton";
import React, { useCallback, useEffect, useRef, useState } from "react";

type Props = IconButtonProps & {
  /** Fired on pointer release when held for at least `delay` ms and released inside the button */
  onLongPress: () => void;
  /** Required hold duration in ms */
  delay?: number;
};

const TICK_MS = 50;

/** Hit test against the element bounds (needed because pointer capture disables boundary events) */
const isPointInside = (el: HTMLElement, x: number, y: number) => {
  const r = el.getBoundingClientRect();
  return x >= r.left && x <= r.right && y >= r.top && y <= r.bottom;
};

const LongPressIconButton: React.FC<Props> = ({ onLongPress, delay = 800, onClick, children, sx, ...rest }) => {
  const intervalRef = useRef<number | null>(null);
  const startTimeRef = useRef<number | null>(null);
  const activePointerRef = useRef<number | null>(null);
  // marks that the press was already handled on pointer up -> skip the synthetic click
  const handledByPointerRef = useRef(false);

  const [progress, setProgress] = useState(0);
  const [active, setActive] = useState(false);
  const [inside, setInside] = useState(true);

  const stopTracking = useCallback(() => {
    if (intervalRef.current !== null) {
      clearInterval(intervalRef.current);
      intervalRef.current = null;
    }
    startTimeRef.current = null;
    activePointerRef.current = null;
    setProgress(0);
    setActive(false);
    setInside(true);
  }, []);

  // clean up timers on unmount
  useEffect(() => stopTracking, [stopTracking]);

  const handlePointerDown = (e: React.PointerEvent<HTMLButtonElement>) => {
    if (e.pointerType === "mouse" && e.button !== 0) return;
    if (startTimeRef.current !== null) return; // ignore additional pointers

    handledByPointerRef.current = false;
    startTimeRef.current = Date.now();
    activePointerRef.current = e.pointerId;
    setActive(true);
    setInside(true);
    setProgress(0);

    // progress ring is visual feedback only - it never triggers the action
    const startTime = startTimeRef.current;
    intervalRef.current = window.setInterval(() => {
      setProgress(Math.min(((Date.now() - startTime) / delay) * 100, 100));
    }, TICK_MS);

    // keep receiving move events outside the button (mouse); touch captures implicitly
    e.currentTarget.setPointerCapture(e.pointerId);
  };

  const handlePointerMove = (e: React.PointerEvent<HTMLButtonElement>) => {
    if (activePointerRef.current !== e.pointerId) return;
    setInside(isPointInside(e.currentTarget, e.clientX, e.clientY));
  };

  const handlePointerUp = (e: React.PointerEvent<HTMLButtonElement>) => {
    if (activePointerRef.current !== e.pointerId) return;

    if (e.currentTarget.hasPointerCapture(e.pointerId)) {
      e.currentTarget.releasePointerCapture(e.pointerId);
    }

    const startTime = startTimeRef.current;
    const releasedInside = isPointInside(e.currentTarget, e.clientX, e.clientY);
    stopTracking();

    handledByPointerRef.current = true; // suppress the following synthetic click in any case
    if (startTime === null) return;
    if (!releasedInside) return; // released outside -> no action, like a native click

    if (Date.now() - startTime >= delay) {
      onLongPress(); // long press action
    } else {
      onClick?.(e); // short press action
    }
  };

  const handleCancel = (e: React.PointerEvent<HTMLButtonElement>) => {
    if (activePointerRef.current !== e.pointerId) return;
    if (e.currentTarget.hasPointerCapture(e.pointerId)) {
      e.currentTarget.releasePointerCapture(e.pointerId);
    }
    handledByPointerRef.current = true;
    stopTracking(); // aborted -> no action at all
  };

  return (
    <Box position="relative" display="inline-flex">
      {active && (
        <CircularProgress
          variant="determinate"
          value={progress}
          size="100%"
          thickness={2.5}
          color={progress >= 100 ? "success" : "primary"}
          sx={{
            position: "absolute",
            inset: 0,
            pointerEvents: "none",
            opacity: inside ? 1 : 0.3, // feedback: releasing here would do nothing
          }}
        />
      )}

      <IconButton
        {...rest}
        onContextMenu={(e) => e.preventDefault()} // important for mobile
        onPointerDown={handlePointerDown}
        onPointerMove={handlePointerMove}
        onPointerUp={handlePointerUp}
        onPointerCancel={handleCancel}
        onClick={(e) => {
          if (handledByPointerRef.current) {
            handledByPointerRef.current = false; // already handled on pointer up
            return;
          }
          onClick?.(e); // keyboard activation (Enter / Space)
        }}
        sx={{ touchAction: "none", userSelect: "none", WebkitTouchCallout: "none", ...sx }}
      >
        {children}
      </IconButton>
    </Box>
  );
};

export default LongPressIconButton;
