// useful link:
// https://blog.atomist.com/declaration-file-fix/

// TODO: Replace this module definitions when available
declare module "zmodem.js/src/zmodem_browser" {
  type Offer = any;
  // type Sentry = any;
  type Session = any;
  type Detection = any;
  type Browser = any;

  export default class SentryClass {
    constructor();

    consume(data: any): any;
  }
  type Sentry = SentryClass;
  const Sentry: any;
}
