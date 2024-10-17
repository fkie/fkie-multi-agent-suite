import CloseIcon from "@mui/icons-material/Close";
import SearchIcon from "@mui/icons-material/Search";
import { IconButton, TextField } from "@mui/material";
import PropTypes from "prop-types";
import { useEffect, useState } from "react";

function SearchBar({
  onSearch,
  onCloseRequest = () => {},
  placeholder = "Filter",
  defaultValue = "",
  fullWidth = true,
  searchIcon = true,
  autoFocus = true,
}) {
  const [searched, setSearched] = useState(defaultValue);

  useEffect(() => {
    onSearch(searched);
  }, [onSearch, searched]);

  return (
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
      size="small"
      variant="standard"
      placeholder={placeholder}
      value={searched}
      fullWidth={fullWidth}
      InputProps={{
        startAdornment: searchIcon ? (
          <SearchIcon
            disabled
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
      }}
      sx={{
        m: 0,
        ml: 0.5,
        "& .Mui-focused .MuiIconButton-root": { color: "primary.main" },
      }}
    />
  );
}

SearchBar.propTypes = {
  onSearch: PropTypes.func.isRequired,
  onCloseRequest: PropTypes.func,
  placeholder: PropTypes.string,
  defaultValue: PropTypes.string,
  fullWidth: PropTypes.bool,
  searchIcon: PropTypes.bool,
  autoFocus: PropTypes.bool,
};

export default SearchBar;
