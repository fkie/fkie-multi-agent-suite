export class RosProviderState {
  /** Parameter from ROS state */
  masteruri: string | undefined = undefined;
  ros_version: string | undefined = undefined;
  ros_distro: string | undefined = undefined;
  ros_domain_id: string | undefined = undefined;
  origin: boolean | undefined = undefined;
  name: string | undefined = undefined;
  port: number = 0;
  host: string = "";
  hostnames: string[] = [];
}

export default RosProviderState;
