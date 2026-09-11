import { useContext } from "react";
import { AutoUpdateProvider } from "./context/AutoUpdateContext";
import { DirtyEditorGuardProvider } from "./context/DirtyEditorGuard";
import { ElectronContext } from "./context/ElectronContext";
import { MonacoProvider } from "./context/MonacoContext";
import { MonacoInitProvider } from "./context/MonacoInitContext";
import NodeManager from "./pages/NodeManager/NodeManager";

export default function App(): JSX.Element {
  const electronCtx = useContext(ElectronContext);
  return (
    <AutoUpdateProvider>
      <MonacoProvider>
        <MonacoInitProvider>
          <DirtyEditorGuardProvider
            onCancel={() => electronCtx.cancelCloseApp()}
            onFocus={() => electronCtx.cancelCloseTimer()}
          >
            <NodeManager />
          </DirtyEditorGuardProvider>
        </MonacoInitProvider>
      </MonacoProvider>
    </AutoUpdateProvider>
  );
}
