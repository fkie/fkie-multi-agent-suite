import { AutoUpdateProvider } from "./context/AutoUpdateContext";
import { MonacoProvider } from "./context/MonacoContext";
import { MsgHistoryProvider } from "./context/MsgHistoryContext";
import NodeManager from "./pages/NodeManager/NodeManager";

export default function App(): JSX.Element {
  return (
    <MsgHistoryProvider>
      <AutoUpdateProvider>
        <MonacoProvider>
          <NodeManager />
        </MonacoProvider>
      </AutoUpdateProvider>
    </MsgHistoryProvider>
  );
}
