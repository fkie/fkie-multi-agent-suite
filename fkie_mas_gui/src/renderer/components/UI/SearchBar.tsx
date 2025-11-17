import CloseIcon from "@mui/icons-material/Close";
import SearchIcon from "@mui/icons-material/Search";
import { IconButton, Menu, MenuItem, TextField } from "@mui/material";
import { useEffect, useState } from "react";

interface SearchBarProps {
  onSearch: (term: string) => void;
  onCloseRequest?: () => void;
  placeholder?: string;
  defaultValue?: string;
  fullWidth?: boolean;
  searchIcon?: React.ReactNode;
  autoFocus?: boolean;
}

export default function SearchBar(props: SearchBarProps): JSX.Element {
  const {
    onSearch,
    onCloseRequest = (): void => {},
    placeholder = "Filter",
    defaultValue = "",
    fullWidth = true,
    searchIcon = (
      <SearchIcon
        sx={{
          marginRight: 1,
          color: "gray",
          fontSize: "inherit",
        }}
      />
    ),
    autoFocus = true,
  } = props;
  const [searched, setSearched] = useState(defaultValue);
  const [contextMenu, setContextMenu] = useState<{ mouseX: number; mouseY: number } | null>(null);

  useEffect(() => {
    onSearch(searched);
  }, [searched]);

  useEffect(() => {
    setSearched(defaultValue);
  }, [defaultValue]);

  return (
    <>
      <TextField
        autoFocus={autoFocus}
        onChange={(newValue) => {
          setSearched(newValue.target.value);
        }}
        onKeyUp={(e) => {
          // resend search value on Enter
          if (e.key === "Enter") {
            onSearch(searched);
          }
          if (e.key === "Escape") {
            setSearched("");
            if (onCloseRequest) onCloseRequest();
          }
        }}
        onContextMenu={(event) => {
          event.preventDefault();
          setContextMenu(
            contextMenu === null
              ? {
                  mouseX: event.clientX + 2,
                  mouseY: event.clientY - 6,
                }
              : null
          );
        }}
        size="small"
        variant="standard"
        placeholder={placeholder}
        value={searched}
        fullWidth={fullWidth}
        slotProps={{
          input: {
            startAdornment: searchIcon ? searchIcon : <></>,
            endAdornment: (
              <IconButton
                sx={{
                  visibility: searched ? "visible" : "hidden",
                  fontSize: "inherit",
                }}
                onClick={() => setSearched("")}
              >
                <CloseIcon sx={{ fontSize: "inherit" }} />
              </IconButton>
            ),
            style: { fontSize: "0.9em" },
          },
        }}
        sx={{
          m: 0,
          ml: 0.5,
          "& .Mui-focused .MuiIconButton-root": { color: "primary.main" },
        }}
      />
      <Menu
        open={contextMenu != null}
        onClose={() => setContextMenu(null)}
        anchorReference="anchorPosition"
        anchorPosition={contextMenu != null ? { top: contextMenu.mouseY, left: contextMenu.mouseX } : undefined}
      >
        <MenuItem
          sx={{ fontSize: "0.8em" }}
          onClick={async () => {
            const text = await navigator.clipboard.readText();
            if (text) {
              setSearched(text);
            }
            setContextMenu(null);
          }}
        >
          Paste
        </MenuItem>
      </Menu>
    </>
  );
}
