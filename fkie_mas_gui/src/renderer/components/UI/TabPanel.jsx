import { Stack } from "@mui/material";
import PropTypes from "prop-types";

function TabPanel({ children, value, index, ...other }) {
  return (
    <div
      role="tabpanel"
      hidden={value !== index}
      id={`connect-to-ros-tabpanel-${index}`}
      aria-labelledby={`connect-to-ros-tab-${index}`}
      {...other}
    >
      {value === index && <Stack spacing={1}>{children}</Stack>}
    </div>
  );
}

TabPanel.propTypes = {
  children: PropTypes.node,
  index: PropTypes.number.isRequired,
  value: PropTypes.number.isRequired,
};

export default TabPanel;
