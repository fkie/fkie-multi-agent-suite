export default class RosDuration {
  sec: number;

  nanosec: number;

  constructor(sec = 0, nanosec = 0) {
    this.sec = sec;
    this.nanosec = nanosec;
  }
}
