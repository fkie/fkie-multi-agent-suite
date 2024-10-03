import RosContext from "@/renderer/context/RosContext";
import { ProviderLaunchConfiguration } from "@/renderer/models";
import Provider from "@/renderer/providers/Provider";
import { Button, Dialog, DialogActions, DialogContent, DialogTitle, Link, TextField } from "@mui/material";
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

  return (
    <Dialog open={open} onClose={handleClose}>
      <DialogTitle>SSH login @{connectConfig.host}</DialogTitle>
      <DialogContent>
        <Link href="https://linuxize.com/post/using-the-ssh-config-file/" target="_blank" rel="noopener">
          Setup ssh to avoid this dialog
        </Link>
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
