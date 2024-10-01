import { TSystemInfo } from "@/types";
import log from "electron-log";
import hostile from "hostile";
import si from "systeminformation";

/**
 * Read general local system information
 */
class SystemInfo implements TSystemInfo {
  time?: si.Systeminformation.TimeData;

  cpu?: si.Systeminformation.CpuData;

  cpuCurrentSpeed?: si.Systeminformation.CpuCurrentSpeedData;

  cpuTemperature?: si.Systeminformation.CpuTemperatureData;

  mem?: si.Systeminformation.MemData;

  battery?: si.Systeminformation.BatteryData;

  graphics?: si.Systeminformation.GraphicsData;

  osInfo?: si.Systeminformation.OsData;

  networkInterfaces?: si.Systeminformation.NetworkInterfacesData[];

  hosts?: hostile.Lines;

  // networkConnections?: si.Systeminformation.NetworkConnectionsData[];

  public getInfo: () => Promise<TSystemInfo> = () => {
    return new Promise((resolve, reject) => {
      const fetchInfo = async (): Promise<void> => {
        try {
          this.time = await si.time();
          this.cpu = await si.cpu();
          this.cpuCurrentSpeed = await si.cpuCurrentSpeed();
          this.cpuTemperature = await si.cpuTemperature();
          this.mem = await si.mem();
          this.battery = await si.battery();
          this.graphics = await si.graphics();
          this.osInfo = await si.osInfo();
          const networkInterfaces:
            | si.Systeminformation.NetworkInterfacesData
            | si.Systeminformation.NetworkInterfacesData[] = await si.networkInterfaces();
          if (!Array.isArray(networkInterfaces)) {
            this.networkInterfaces = [networkInterfaces];
          } else {
            this.networkInterfaces = networkInterfaces;
          }
          // this.networkConnections = await si.networkConnections();

          // get available hosts
          // If `preserveFormatting` is true, then include comments, blank lines and other
          // non-host entries in the result
          const preserveFormatting = false;
          this.hosts = hostile.get(preserveFormatting);

          resolve({
            time: this.time,
            cpu: this.cpu,
            cpuCurrentSpeed: this.cpuCurrentSpeed,
            cpuTemperature: this.cpuTemperature,
            mem: this.mem,
            battery: this.battery,
            graphics: this.graphics,
            osInfo: this.osInfo,
            networkInterfaces: this.networkInterfaces,
            // networkConnections: this.networkConnections,
            hosts: this.hosts,
          } as TSystemInfo);
        } catch (error) {
          log.error(`SystemInfo: getInfo error: ${error}`);
          reject(error);
        }
      };

      fetchInfo();
    });
  };

  /**
   * Get a string representation of this object
   *
   */
  public toString: () => string = () => {
    return JSON.stringify(this.getInfo());
  };
}

export { SystemInfo };
