interface ImportMetaEnv {
  readonly VITE_ROS_VERSION: string;
  readonly VITE_ROS_DOMAIN_ID: string;
  readonly VITE_JOIN_ID: string;
}

interface ImportMeta {
  readonly env: ImportMetaEnv;
}
