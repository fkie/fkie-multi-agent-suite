import { MenuItem, Select } from "@mui/material";
import { forwardRef, useContext, useEffect, useState } from "react";

import { RosContext } from "@/renderer/context/RosContext";

interface ProviderSelectorProps {
  defaultProvider: string;
  setSelectedProvider: (providerId: string) => void;
}

const ProviderSelector = forwardRef<HTMLDivElement, ProviderSelectorProps>(function ProviderSelector(props, ref) {
  const { defaultProvider = "", setSelectedProvider = (): void => {} } = props;
  const rosCtx = useContext(RosContext);
  const [currentProvider, setCurrentProvider] = useState<string>(defaultProvider);
  const [providerNames, setProviderNames] = useState<{ name: string; id: string }[]>(
    defaultProvider ? [{ name: rosCtx.getProviderName(defaultProvider), id: defaultProvider }] : []
  );

  useEffect(() => {
    const provNames: { name: string; id: string }[] = [];
    let hasLocal = false;
    let newSelectedProvider = "";
    if (rosCtx.providers.length === 1) {
      // select the first host if only one is available
      const currProv = rosCtx.providers[0];
      provNames.push({
        name: currProv.name(),
        id: currProv.id,
      });
      newSelectedProvider = currProv.id;
    } else {
      // get all connected hosts and select the first local host
      for (const prov of rosCtx.providers) {
        if (currentProvider === "") {
          if (prov?.isLocalHost && !hasLocal) {
            hasLocal = true;
            newSelectedProvider = prov.id;
          }
        } else if (currentProvider === prov.id) {
          newSelectedProvider = currentProvider;
        }
        provNames.push({ name: prov.name(), id: prov.id });
      }
    }
    provNames.sort((a, b) => -b.name.localeCompare(a.name));
    if (newSelectedProvider !== currentProvider) {
      // current provider is not in the name list, add to avoid warnings
      provNames.push({ name: currentProvider, id: currentProvider });
    }
    setProviderNames(provNames);
    setCurrentProvider(newSelectedProvider);
  }, [currentProvider, rosCtx.providers]);

  // inform about new provider
  useEffect(() => {
    setSelectedProvider(currentProvider);
  }, [currentProvider]);

  return (
    <Select
      ref={ref}
      autoWidth={false}
      value={currentProvider}
      onChange={(event) => {
        // wait before changing provider
        // it prevents React to render too early child components
        setTimeout(() => {
          setCurrentProvider(event.target.value);
        }, 200);
      }}
      size="small"
      displayEmpty
    >
      {providerNames.map((item) => {
        return (
          <MenuItem key={item.id} value={item.id}>
            {item.name}
          </MenuItem>
        );
      })}
    </Select>
  );
});

export default ProviderSelector;
