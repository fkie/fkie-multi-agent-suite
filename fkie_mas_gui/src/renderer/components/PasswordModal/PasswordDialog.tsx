import { useContext, useState } from "react";
import { ConnectConfig } from "ssh2";
import { Dialog, DialogActions, DialogContent, DialogTitle, TextField, Button } from "@mui/material";
import { ProviderLaunchConfiguration } from "@/renderer/models";
import { ConnectionState } from "../../providers";
import RosContext from "@/renderer/context/RosContext";
import Provider from "@/renderer/providers/Provider";

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
