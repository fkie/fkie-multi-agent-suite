export type TCmdTerminal = {
  success: boolean;

  error?: string;

  cmd: string;

  screen: string;

  log: string;

  external: boolean;

  /** Bare command without any wrapper - for UI display and copy&paste. */
  displayCmd: string;
};
