import { LayoutTabConfig } from '../pages/NodeManager/layout';
import { generateUniqueId } from './index';

export const EVENT_OPEN_COMPONENT = 'EVENT_OPEN_COMPONENT' as const;
export const EVENT_OPEN_SETTINGS = 'EVENT_OPEN_SETTINGS' as const;
export const EVENT_OPEN_CONNECT = 'EVENT_OPEN_CONNECT' as const;

export function eventOpenComponent(
  id: string,
  title: string,
  component: any,
  multiple: boolean = true,
  closable: boolean = true,
  panelGroup: string = '', // panel or tab id where to place the new tab
  config: LayoutTabConfig = new LayoutTabConfig(false, panelGroup), // a place to hold json config for the hosted component
) {
  return {
    id: multiple ? `${id}-${generateUniqueId()}` : id,
    title,
    closable,
    component,
    panelGroup: !panelGroup && multiple ? `${id}` : panelGroup,
    config,
  };
}

export class SETTING extends String {
  static IDS = {
    INTERFACE: 'interface',
    SSH: 'ssh',
  };
}

export function eventOpenSettings(id: SETTING) {
  return { id };
}
