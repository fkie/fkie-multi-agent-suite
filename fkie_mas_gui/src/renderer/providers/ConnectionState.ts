export class ConnectionState extends String {
  static STATES = {
    UNKNOWN: 'unknown',
    STARTING: 'starting',
    CONNECTING: 'connecting',
    CROSSBAR_CONNECTED: 'crossbar connected',
    CROSSBAR_REGISTERED: 'crossbar registered',
    CONNECTED: 'connected',
    CLOSED: 'closed',
    LOST: 'lost',
    UNREACHABLE: 'unreachable',
    UNSUPPORTED: 'unsupported',
    NO_SSH_CREDENTIALS: 'no SSH credentials',
    ERRORED: 'errored',
  };
}

export default { ConnectionState };
