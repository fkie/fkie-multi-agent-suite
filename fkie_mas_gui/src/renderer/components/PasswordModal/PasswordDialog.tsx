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
    handleClose();
  };

  const handleCancel = () => {
    provider.setConnectionState(ConnectionState.STATES.UNREACHABLE, "");
    handleClose();
  };

  const handleClose = () => {
    setOpen(false);
    if (onClose) {
      onClose(provider);
    }
  };

  return (
    <Dialog open={open} onClose={() => handleClose()}>
      <DialogTitle>SSH login @{connectConfig.host}</DialogTitle>
      <Link href="https://linuxize.com/post/using-the-ssh-config-file/" target="_blank" rel="noopener">
        Setup ssh to avoid this dialog
      </Link>
      <DialogContent>
        <TextField
          autoFocus
          margin="dense"
          label="Username"
          type="string"
          fullWidth
          variant="outlined"
          value={username}
          onChange={(e) => setUsername(e.target.value)}
          onKeyDown={(e) => {
            console.log(`key ${e.key}`);
            if (e.key === "Enter") {
              // handleSubmit()
              e.key = "Tab";
            }
          }}
        />
        <TextField
          autoFocus
          margin="dense"
          label="Passwort"
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
        <Button onClick={() => handleSubmit()} color="primary">
          Ok
        </Button>
        <Button onClick={() => handleCancel()} color="primary">
          Cancel
        </Button>
      </DialogActions>
    </Dialog>
  );
};

export default PasswordDialog;
