import { useEffect, useRef, useState } from "react";

interface GithubCredentialsDialogProps {
  open: boolean;
  onSubmit: (username: string, token: string, remember: boolean) => void;
  onCancel: () => void;
}

export default function GithubCredentialsDialog(props: GithubCredentialsDialogProps): JSX.Element | null {
  const { open, onSubmit, onCancel } = props;
  const [username, setUsername] = useState("");
  const [token, setToken] = useState("");
  const [remember, setRemember] = useState(false);
  const inputRef = useRef<HTMLInputElement>(null);

  useEffect(() => {
    if (open) {
      setUsername("");
      setToken("");
      setTimeout(() => inputRef.current?.focus(), 100);
    }
  }, [open]);

  if (!open) return null;

  const handleSubmit = (e?: React.FormEvent) => {
    e?.preventDefault();
    if (username.trim() && token.trim()) {
      onSubmit(username.trim(), token.trim(), remember);
    }
  };

  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === "Escape") onCancel();
  };

  return (
    <div
      onKeyDown={handleKeyDown}
      style={{
        position: "fixed",
        top: 0,
        left: 0,
        width: "100%",
        height: "100%",
        background: "rgba(0,0,0,0.5)",
        display: "flex",
        alignItems: "center",
        justifyContent: "center",
        zIndex: 10000,
      }}
    >
      <form
        onSubmit={handleSubmit}
        style={{
          background: "#1e1e1e",
          color: "#eee",
          padding: "24px",
          borderRadius: "8px",
          minWidth: "360px",
          boxShadow: "0 4px 12px rgba(0,0,0,0.5)",
        }}
      >
        <h3 style={{ margin: "0 0 8px 0" }}>GitHub API Rate Limit Reached</h3>
        <p style={{ margin: "0 0 16px 0", color: "#aaa", fontSize: "14px" }}>
          Please enter your GitHub credentials.
          <br />
          Recommended: <strong>Personal Access Token</strong> (Settings → Developer Settings → Tokens)
        </p>

        <label style={{ display: "block", marginBottom: "12px", fontSize: "14px" }}>
          Username
          <input
            ref={inputRef}
            type="text"
            value={username}
            onChange={(e) => setUsername(e.target.value)}
            style={{
              width: "100%",
              padding: "8px",
              marginTop: "4px",
              boxSizing: "border-box",
              background: "#2d2d2d",
              color: "#eee",
              border: "1px solid #555",
              borderRadius: "4px",
            }}
          />
        </label>

        <label style={{ display: "block", marginBottom: "16px", fontSize: "14px" }}>
          Token / Password
          <input
            type="password"
            value={token}
            onChange={(e) => setToken(e.target.value)}
            style={{
              width: "100%",
              padding: "8px",
              marginTop: "4px",
              boxSizing: "border-box",
              background: "#2d2d2d",
              color: "#eee",
              border: "1px solid #555",
              borderRadius: "4px",
            }}
          />
        </label>

        <label
          style={{
            display: "flex",
            alignItems: "center",
            gap: "8px",
            marginBottom: "20px",
            fontSize: "14px",
            cursor: "pointer",
          }}
        >
          <input
            type="checkbox"
            checked={remember}
            onChange={(e) => setRemember(e.target.checked)}
            style={{ width: "16px", height: "16px", cursor: "pointer" }}
          />
          Remember credentials
        </label>

        <div style={{ display: "flex", gap: "8px", justifyContent: "flex-end" }}>
          <button
            type="button"
            onClick={onCancel}
            style={{
              padding: "8px 16px",
              cursor: "pointer",
              background: "#444",
              color: "#eee",
              border: "none",
              borderRadius: "4px",
            }}
          >
            Cancel
          </button>
          <button
            type="submit"
            disabled={!username.trim() || !token.trim()}
            style={{
              padding: "8px 16px",
              cursor: "pointer",
              background: "#2ea44f",
              color: "white",
              border: "none",
              borderRadius: "4px",
              opacity: !username.trim() || !token.trim() ? 0.5 : 1,
            }}
          >
            OK
          </button>
        </div>
      </form>
    </div>
  );
}
