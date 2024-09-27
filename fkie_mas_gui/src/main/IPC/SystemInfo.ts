import log from "electron-log";
import hostile from "hostile";
import si from "systeminformation";

interface ISystemInfo {
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
}

/**
 * Read general local system information
 */
class SystemInfo {
  time?: si.Systeminformation.TimeData;

  cpu?: si.Systeminformation.CpuData;

  cpuCurrentSpeed?: si.Systeminformation.CpuCurrentSpeedData;

  cpuTemperature?: si.Systeminformation.CpuTemperatureData;

  mem?: si.Systeminformation.MemData;

  battery?: si.Systeminformation.BatteryData;

  graphics?: si.Systeminformation.GraphicsData;

  osInfo?: si.Systeminformation.OsData;

  networkInterfaces?: si.Systeminformation.NetworkInterfacesData | si.Systeminformation.NetworkInterfacesData[];

  hosts?: hostile.Lines;

  // networkConnections?: si.Systeminformation.NetworkConnectionsData[];

  public getInfo: () => Promise<ISystemInfo> = () => {
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
          this.networkInterfaces = await si.networkInterfaces();
          if (!Array.isArray(this.networkInterfaces)) {
            this.networkInterfaces = [this.networkInterfaces];
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
          });
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
export type { ISystemInfo };
