import DeleteIcon from "@mui/icons-material/Delete";
import {
  Alert,
  AlertTitle,
  Button,
  FormControl,
  FormGroup,
  FormLabel,
  IconButton,
  Paper,
  Stack,
  Table,
  TableBody,
  TableCell,
  TableContainer,
  TableHead,
  TableRow,
  TextField,
} from "@mui/material";
import React, { useCallback, useContext, useState } from "react";
import { LoggingContext } from "../../context/LoggingContext";
import { SSHContext } from "../../context/SSHContext";
import useLocalStorage from "../../hooks/useLocalStorage";
import SearchBar from "../UI/SearchBar";

const headers = [
  {
    key: "username",
    header: "User Name",
  },
  {
    key: "host",
    header: "Host",
  },
  {
    key: "port",
    header: "Port",
  },
  {
    key: "delete",
    header: "Delete",
  },
];

function SSHCredentialsPanel() {
  const SSHCtx = useContext(SSHContext);
  const logCtx = useContext(LoggingContext);

  const [addingCredential, setAddingCredential] = useState(false);
  const [username, setUsername] = useState("");
  const [host, setHost] = useState("");
  const [port, setPort] = useState(22);
  const [password, setPassword] = useState("");

  const [openInfoPwd, setOpenInfoPwd] = useLocalStorage("SettingsModal:openInfoPwd", true);
  const [openInfoCred, setOpenInfoCred] = useLocalStorage("SettingsModal:openInfoCred", []);

  const [errorMessage, setErrorMessage] = useState("");
  const [filter, setFilter] = useState("");

  const onDeleteCredential = (credential) => {
    if (!credential) {
      logCtx.error("Invalid credential ID", `credential: ${credential}`);
      return;
    }

    if (SSHCtx.deleteCredential(credential)) {
      logCtx.success("Credential removed successfully", "Both credential and password on keychain removed");
    }
  };

  const onAddCredential = useCallback(async () => {
    if (addingCredential) {
      return;
    }
    if (username.length === 0 || host.length === 0 || port.length === 0 || password.length === 0) {
      setErrorMessage(() => "Could not add new credential, Invalid input data");
      return;
    }
    setAddingCredential(true);
    logCtx.debug(`Try to add credential for host ${host}`, "");
    const res = await SSHCtx.addCredential({
      username,
      host,
      port,
      password,
    });
    if (!res.result) {
      logCtx.error("Could not add new credential", res.message);

      setErrorMessage(() => `Could not add new credential: ${res.message}`);
    } else {
      setUsername("");
      setHost("");
      setPort(22);
      setPassword("");
      setErrorMessage(() => "");
    }
    setAddingCredential(false);
  }, [addingCredential, username, host, port, password, logCtx, SSHCtx]);

  return (
    <Stack spacing={2}>
      <SearchBar onSearch={setFilter} placeholder="Filter Credentials" defaultValue="" />
      <TableContainer component={Paper}>
        <Table stickyHeader size="small" aria-label="ssh credential table">
          <TableHead>
            <TableRow>
              {headers.map((header) => (
                <TableCell key={header.key} sx={{ fontWeight: "bold" }}>
                  {header.header}
                </TableCell>
              ))}
            </TableRow>
          </TableHead>
          <TableBody>
            {SSHCtx.credentials
              .filter((row) => {
                return (
                  row.username.toLowerCase().includes(filter) ||
                  row.host.toLowerCase().includes(filter) ||
                  row.toString().includes(filter)
                );
              })
              .map((row) => (
                <TableRow key={row.id} sx={{ "&:last-child td, &:last-child th": { border: 0 } }}>
                  {headers.map((header) => {
                    if (header.key.includes("delete")) {
                      return (
                        <TableCell key={`${row.id}_${header.key}`}>
                          <IconButton
                            onClick={() => {
                              onDeleteCredential(row);
                            }}
                          >
                            <DeleteIcon sx={{ fontSize: "inherit" }} />
                          </IconButton>
                        </TableCell>
                      );
                    }

                    return <TableCell key={`${row.id}_${header.key}`}>{row[header.key]}</TableCell>;
                  })}
                </TableRow>
              ))}
          </TableBody>
        </Table>
      </TableContainer>

      {openInfoCred && (
        <Alert
          severity="info"
          onClose={() => {
            setOpenInfoCred(false);
          }}
        >
          <AlertTitle>We might configure SSH credentials to run MAS nodes on remote host.</AlertTitle>
          {`The host field must coincide with the provider's host to be able to use these credentials.`}
        </Alert>
      )}

      <FormControl component="fieldset" variant="standard">
        <FormLabel component="legend">Add a new SSH credential:</FormLabel>
        <FormGroup>
          <Stack direction="row" spacing={1} alignContent="center" alignItems="center">
            <TextField
              id="input-username"
              type="text"
              size="small"
              label="User Name"
              placeholder="User Name"
              value={username}
              onChange={(event) => {
                setUsername(event.target.value);
              }}
            />
            <TextField
              id="input-host"
              type="text"
              size="small"
              label="Host"
              placeholder="Host"
              value={host}
              onChange={(event) => {
                setHost(event.target.value);
              }}
            />

            <TextField
              type="number"
              id="input-port"
              label="Port"
              size="small"
              variant="outlined"
              // InputProps={{ inputProps: { min: 0, max: 15 } }}
              // fullWidth={true}
              value={port}
              onChange={(e) => setPort(Number(`${e.target.value}`))}
            />
            <TextField
              id="input-password"
              type="password"
              size="small"
              label="password"
              placeholder="password"
              value={password}
              onChange={(event) => {
                setPassword(event.target.value);
              }}
              onKeyDown={(ev) => {
                if (ev.key === "Enter") {
                  ev.preventDefault();
                  onAddCredential();
                }
              }}
            />
            <Button
              type="submit"
              variant="contained"
              color="success"
              disabled={addingCredential}
              onClick={(event) => {
                event.preventDefault();
                onAddCredential();
              }}
            >
              Add Credential
            </Button>
          </Stack>
        </FormGroup>
      </FormControl>

      {errorMessage.length > 0 && (
        <Alert severity="error">
          <AlertTitle>{errorMessage}</AlertTitle>
          Please check the given input data.
        </Alert>
      )}

      {openInfoPwd && (
        <Alert
          severity="info"
          onClose={() => {
            setOpenInfoPwd(false);
          }}
        >
          <AlertTitle>
            {`We do not store passwords. We use instead the native system's
            keychain.`}
          </AlertTitle>
          Please check the project [node-keytar] for more information.
        </Alert>
      )}
    </Stack>
  );
}
export default SSHCredentialsPanel;
