import { generateUniqueId } from './index';

export const EVENT_OPEN_COMPONENT = 'EVENT_OPEN_COMPONENT' as const;
export const EVENT_OPEN_SETTINGS = 'EVENT_OPEN_SETTINGS' as const;

export function eventOpenComponent(
  id: string,
  title: string,
  component: any,
  multiple: boolean = true,
  closable: boolean = true,
  panelGroup: string = '',
) {
  return {
    id: multiple ? `${id}-${generateUniqueId()}` : id,
    title,
    closable,
    component,
    panelGroup: !panelGroup && multiple ? `${id}` : panelGroup,
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
