import hostile from "hostile";
import si from "systeminformation";

export type TSystemInfo = {
  time?: si.Systeminformation.TimeData;
  cpu?: si.Systeminformation.CpuData;
  cpuCurrentSpeed?: si.Systeminformation.CpuCurrentSpeedData;
  cpuTemperature?: si.Systeminformation.CpuTemperatureData;
  mem?: si.Systeminformation.MemData;
  battery?: si.Systeminformation.BatteryData;
  graphics?: si.Systeminformation.GraphicsData;
  osInfo?: si.Systeminformation.OsData;
  networkInterfaces?: si.Systeminformation.NetworkInterfacesData[];
  // networkConnections?: si.Systeminformation.NetworkConnectionsData[];
  hosts?: hostile.Lines;

  getInfo?: () => Promise<TSystemInfo>;
};
