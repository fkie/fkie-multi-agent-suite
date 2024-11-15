import RosContext from "@/renderer/context/RosContext";
import { ProviderLaunchConfiguration } from "@/renderer/models";
import Provider from "@/renderer/providers/Provider";
import { Button, Dialog, DialogActions, DialogContent, DialogContentText, DialogTitle, TextField } from "@mui/material";
import { useContext, useState } from "react";
import { ConnectConfig } from "ssh2";
import { ConnectionState } from "../../providers";

const PasswordDialog = ({
  provider,
  connectConfig,
  launchConfig,
  onClose,
}: {
  provider: Provider;
  connectConfig: ConnectConfig;
  launchConfig: ProviderLaunchConfiguration;
  onClose: (provider: Provider) => void;
}) => {
  const rosCtx = useContext(RosContext);
  const [username, setUsername] = useState(connectConfig.username);
  const [password, setPassword] = useState("");
  const [open, setOpen] = useState(true);
  const [showSetup, setShowSetup] = useState(false);

  const handleSubmit = () => {
    console.log("Password:", password);
    connectConfig.username = username;
    connectConfig.password = password;
    rosCtx.startConfig(launchConfig, connectConfig);
    handleClose(null, "confirm");
  };

  const handleCancel = () => {
    handleClose(null, "cancel");
  };

  const handleClose = (_event, reason) => {
    console.log(`close: ${reason}`);
    if (reason && reason === "backdropClick") return;
    if (reason === "cancel" || reason === "escapeKeyDown") {
      provider.setConnectionState(ConnectionState.STATES.AUTHZ, "");
    }
    setOpen(false);
    if (onClose) {
      onClose(provider);
    }
  };

  const codeSnippet = `
  ssh-keygen -f ~/.ssh/id_${connectConfig.host}
  ssh-copy-id -i ~/.ssh/id_${connectConfig.host} ${connectConfig.username}@${connectConfig.host}

  Add to ~/.ssh/config:
    Host ${connectConfig.host}
      User ${connectConfig.username}
      IdentityFile ~/.ssh/id_${connectConfig.host}
      HostName ${connectConfig.host} (optional)
  `;

  return (
    <Dialog open={open} onClose={handleClose}>
      <DialogTitle>SSH login @{connectConfig.host}</DialogTitle>
      <DialogContent>
        <Button
          size="small"
          style={{
            marginLeft: 1,
            textTransform: "none",
            justifyContent: "left",
          }}
          onClick={() => setShowSetup((prev) => !prev)}
        >
          How to setup ssh to avoid this dialog
        </Button>
        {showSetup && (
          <DialogContentText>
            <pre>
              <code>{codeSnippet}</code>
            </pre>
          </DialogContentText>
        )}
        <TextField
          autoFocus
          margin="dense"
          label="Username"
          type="string"
          fullWidth
          variant="outlined"
          value={username}
          onChange={(e) => setUsername(e.target.value)}
          onKeyDown={(event) => {
            if (event.key === "Enter") {
              document.getElementById(`password-${provider.id}`)?.focus();
            }
          }}
        />
        <TextField
          id={`password-${provider.id}`}
          margin="dense"
          label="Password"
          type="password"
          fullWidth
          variant="outlined"
          value={password}
          onChange={(e) => setPassword(e.target.value)}
          onKeyDown={(e) => {
            if (e.key === "Enter") {
              handleSubmit();
            }
          }}
        />
      </DialogContent>
      <DialogActions>
        <Button onClick={() => handleCancel()} color="primary">
          Cancel
        </Button>
        <Button onClick={() => handleSubmit()} color="success">
          Ok
        </Button>
      </DialogActions>
    </Dialog>
  );
};

export default PasswordDialog;
