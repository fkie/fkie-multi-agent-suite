export type TLaunchArg = {
  name: string;
  value: string;
  default_value: string | null | undefined;
  description: string | null | undefined;
  choices: string[] | null | undefined;
};
