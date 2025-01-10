import CloseIcon from "@mui/icons-material/Close";
import SearchIcon from "@mui/icons-material/Search";
import { IconButton, TextField } from "@mui/material";
import { forwardRef, useEffect, useState } from "react";

interface SearchBarProps {
  onSearch: (term: string) => void;
  onCloseRequest?: () => void;
  placeholder?: string;
  defaultValue?: string;
  fullWidth?: boolean;
  searchIcon?: boolean;
  autoFocus?: boolean;
}

const SearchBar = forwardRef<HTMLDivElement, SearchBarProps>(function SearchBar(props, ref) {
  const {
    onSearch,
    onCloseRequest = (): void => {},
    placeholder = "Filter",
    defaultValue = "",
    fullWidth = true,
    searchIcon = true,
    autoFocus = true,
  } = props;
  const [searched, setSearched] = useState(defaultValue);

  useEffect(() => {
    onSearch(searched);
  }, [searched]);

  useEffect(() => {
    setSearched(defaultValue);
  }, [defaultValue]);

  return (
    <TextField
      ref={ref}
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
      size="small"
      variant="standard"
      placeholder={placeholder}
      value={searched}
      fullWidth={fullWidth}
      slotProps={{
        input: {
          startAdornment: searchIcon ? (
            <SearchIcon
              sx={{
                marginRight: 1,
                color: "gray",
                fontSize: "inherit",
              }}
            />
          ) : (
            <></>
          ),
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
  );
});

export default SearchBar;
