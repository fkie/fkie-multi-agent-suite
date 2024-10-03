import { Stack } from "@mui/material";
import { Outlet, Route, Routes } from "react-router-dom";
import "./App.scss";
import { MonacoProvider } from "./context/MonacoContext";
import AboutPage from "./pages/About/About";
import NodeManager from "./pages/NodeManager/NodeManager";

function NavBar() {
  return (
    <Stack>
      <Outlet />
    </Stack>
  );
}

export default function App() {
  return (
    <MonacoProvider>
      <Routes>
        <Route path="/" element={<NavBar />}>
          <Route path="/" element={<NodeManager />} />
          <Route path="/about" element={<AboutPage />} />
          <Route path="*" element={<NodeManager />} />
        </Route>
      </Routes>
    </MonacoProvider>
  );
}
