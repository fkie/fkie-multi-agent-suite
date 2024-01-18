export interface ICredential {
  id: string;
  host: string;
  port: number;
  username: string;
  service: string;
  account: string;
  password: string;
}

export function instanceOfICredential(object: any): object is ICredential {
  const props = [
    'id',
    'host',
    'port',
    'username',
    'service',
    'account',
    'password',
  ];
  return props.every((x) => x in object);
}
