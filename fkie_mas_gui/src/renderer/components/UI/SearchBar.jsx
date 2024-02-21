import CloseIcon from '@mui/icons-material/Close';
import SearchIcon from '@mui/icons-material/Search';
import { IconButton, TextField } from '@mui/material';
import PropTypes from 'prop-types';
import { useEffect, useState } from 'react';

function SearchBar({ onSearch, placeholder, defaultValue, fullWidth }) {
  const [searched, setSearched] = useState(defaultValue);

  useEffect(() => {
    onSearch(searched);
  }, [searched, onSearch]);

  return (
    <TextField
      onChange={(newValue) => {
        setSearched(newValue.target.value);
      }}
      onKeyUp={(e) => {
        // resend search value on Enter
        if (e.key === 'Enter') {
          onSearch(searched);
        }
      }}
      size="small"
      variant="standard"
      placeholder={placeholder}
      value={searched}
      fullWidth={fullWidth}
      InputProps={{
        startAdornment: (
          <SearchIcon
            disabled
            sx={{ marginRight: 1, color: 'gray', fontSize: 'inherit' }}
          />
        ),
        endAdornment: (
          <IconButton
            sx={{ visibility: searched ? 'visible' : 'hidden' }}
            onClick={() => setSearched('')}
          >
            <CloseIcon />
          </IconButton>
        ),
      }}
      sx={{
        m: 0,
        ml: 0.5,
        '& .Mui-focused .MuiIconButton-root': { color: 'primary.main' },
      }}
    />
  );
}

SearchBar.defaultProps = {
  placeholder: 'Filter',
  defaultValue: '',
  fullWidth: true,
};

SearchBar.propTypes = {
  onSearch: PropTypes.func.isRequired,
  placeholder: PropTypes.string,
  defaultValue: PropTypes.string,
  fullWidth: PropTypes.bool,
};

export default SearchBar;
