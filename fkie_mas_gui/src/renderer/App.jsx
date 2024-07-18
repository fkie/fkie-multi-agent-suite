// import { useWindowHeight } from '@react-hook/window-size/throttled';
import { useContext, useEffect, useState } from "react";
import { Outlet, Route, Routes } from "react-router-dom";
// https://github.com/azouaoui-med/react-pro-sidebar/blob/master/storybook/Playground.tsx
import { createTheme, CssBaseline, Stack } from "@mui/material";
import { ThemeProvider } from "@mui/material/styles";
import { SettingsContext } from "./context/SettingsContext";
import AboutPage from "./pages/About/About";
import NodeManager from "./pages/NodeManager/NodeManager";
// load default style for flexlayout-react. Dark/Light theme changes are in ./themes
import "flexlayout-react/style/gray.css";
import "./App.scss";
import { darkThemeDef, lightThemeDef } from "./themes";

function NavBar() {
  // const windowHeight = useWindowHeight();

  return (
    <Stack
    // direction="row"
    // style={{
    //   height: windowHeight,
    //   width: '100%',
    // }}
    >
      <CssBaseline />
      <Outlet />
    </Stack>
  );
}

export default function App() {
  const settingsCtx = useContext(SettingsContext);
  const [lightTheme, setLightTheme] = useState(createTheme(lightThemeDef));
  const [darkTheme, setDarkTheme] = useState(createTheme(darkThemeDef));

  const handleWindowError = (e) => {
    // fix "ResizeObserver loop limit exceeded" while change size of the editor
    if (
      ["ResizeObserver loop limit exceeded", "ResizeObserver loop completed with undelivered notifications."].includes(
        e.message
      )
    ) {
      const resizeObserverErrDiv = document.getElementById("webpack-dev-server-client-overlay-div");
      const resizeObserverErr = document.getElementById("webpack-dev-server-client-overlay");
      if (resizeObserverErr) {
        resizeObserverErr.setAttribute("style", "display: none");
      }
      if (resizeObserverErrDiv) {
        resizeObserverErrDiv.setAttribute("style", "display: none");
      }
    }
  };

  useEffect(() => {
    // update font size globally
    lightThemeDef.typography.fontSize = settingsCtx.get("fontSize");
    lightThemeDef.components.MuiCssBaseline.styleOverrides.body["& .flexlayout__layout"]["--font-size"] =
      settingsCtx.get("fontSize");
    setDarkTheme(createTheme(darkThemeDef));
    setLightTheme(createTheme(lightThemeDef));
  }, [settingsCtx, settingsCtx.changed]);

  useEffect(() => {
    // Anything in here is fired on component mount.
    window.addEventListener("error", handleWindowError);
    return () => {
      // Anything in here is fired on component unmount.
      window.removeEventListener("error", handleWindowError);
    };
  }, []);

  return (
    <ThemeProvider theme={settingsCtx.get("useDarkMode") ? darkTheme : lightTheme}>
      <CssBaseline />
      <Routes>
        <Route path="/" element={<NavBar />}>
          <Route path="/" element={<NodeManager />} />
          <Route path="/about" element={<AboutPage />} />
          <Route path="*" element={<NodeManager />} />
        </Route>
      </Routes>
    </ThemeProvider>
  );
}
