import { app } from 'electron'
import log from 'electron-log'

const ARGUMENTS = {
  SHOW_OUTPUT_FROM_BACKGROUND_PROCESSES: 'show-output-from-background-processes',

  TTYD_PATH: 'ttyd-path',
  TTYD_PORT: 'ttyd-port',

  DISCOVERY_MCAST_PORT: 'discovery-mcast-port',
  DISCOVERY_MCAST_GROUP: 'discovery-mcast-group',
  DISCOVERY_HEARTBEAT_HZ: 'discovery-heartbeat-hz'
}

/**
 * Register one command line argument and log its value
 */
const registerArgument = (name: string, value: string): void => {
  app.commandLine.appendSwitch(name, value)
  log.info(` --${name}: ${value}`)
}

/**
 * Register command line arguments
 */
const registerArguments = (): void => {
  log.info('Program arguments: ')

  // Program arguments
  registerArgument(ARGUMENTS.SHOW_OUTPUT_FROM_BACKGROUND_PROCESSES, 'true')

  // Terminal manager
  registerArgument(ARGUMENTS.TTYD_PATH, '')
  registerArgument(ARGUMENTS.TTYD_PORT, '7681')

  registerArgument(ARGUMENTS.DISCOVERY_MCAST_PORT, '11511')
  registerArgument(ARGUMENTS.DISCOVERY_MCAST_GROUP, '226.0.0.0')
  registerArgument(ARGUMENTS.DISCOVERY_HEARTBEAT_HZ, '0.5')
}

/**
 * Check if an arguments was registered
 */
const hasArgument = (name: string): boolean => {
  return app.commandLine.hasSwitch(name)
}

/**
 * Get the value of a registered argument
 */
const getArgument: (name: string) => string = (name: string) => {
  return app.commandLine.getSwitchValue(name)
}

export { ARGUMENTS, getArgument, hasArgument, registerArguments }
