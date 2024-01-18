class RosDuration {
  sec: number;

  nanosec: number;

  /**
   * Class Constructor
   *
   * @param {number} sec - Success or not
   * @param {number} nanosec - Explaning optional message.
   */
  constructor(sec = 0, nanosec = 0) {
    this.sec = sec;
    this.nanosec = nanosec;
  }
}

export default RosDuration;
