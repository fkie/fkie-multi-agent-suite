import "./App.scss";
import { AutoUpdateProvider } from "./context/AutoUpdateContext";
import { MonacoProvider } from "./context/MonacoContext";
import NodeManager from "./pages/NodeManager/NodeManager";

export default function App() {
  return (
    <AutoUpdateProvider>
      <MonacoProvider>
        <NodeManager />
      </MonacoProvider>
    </AutoUpdateProvider>
  );
}
