import log from "electron-log";
import hostile from "hostile";
import si from "systeminformation";
import { TSystemInfo } from "@/types";

/** Environment variables that are relevant for ROS / RMW configuration. */
const ENV_ALLOW_LIST: string[] = [
  "ROS_VERSION",
  "ROS_DISTRO",
  "ROS_DOMAIN_ID",
  "ROS_MASTER_URI",
  "ROS_HOSTNAME",
  "ROS_IP",
  "ROS_LOCALHOST_ONLY",
  "ROS_AUTOMATIC_DISCOVERY_RANGE",
  "ROS_STATIC_PEERS",
  "RMW_IMPLEMENTATION",
  "ZENOH_CONFIG_OVERRIDE",
  "ZENOH_ROUTER_CONFIG_URI",
  "ZENOH_SESSION_CONFIG_URI",
  "CYCLONEDDS_URI",
  "FASTRTPS_DEFAULT_PROFILES_FILE",
  "FASTDDS_DEFAULT_PROFILES_FILE",
  "NDDS_DISCOVERY_PEERS",
  "NDDS_QOS_PROFILES",
];

/**
 * Read general local system information
 */
export class SystemInfo implements TSystemInfo {
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

  environment?: Record<string, string>;

  // networkConnections?: si.Systeminformation.NetworkConnectionsData[];

  /**
   * Collect the relevant environment variables of the main process.
   * Only variables from the allow-list are exposed to avoid leaking secrets
   * (tokens, passwords, ...) into the renderer process.
   */
  private readEnvironment: () => Record<string, string> = () => {
    const env: Record<string, string> = {};
    for (const name of ENV_ALLOW_LIST) {
      const value = process.env[name];
      // skip undefined and empty values
      if (value !== undefined && value !== "") {
        env[name] = value;
      }
    }
    return env;
  };

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

          this.environment = this.readEnvironment();

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
            environment: this.environment,
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
