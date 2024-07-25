/*
Based on: https://github.com/tsl0922/ttyd

MIT License
Copyright (c) 2016 Shuanglei Tao <tsl0922@gmail.com>
*/

import React from 'react';

import { saveAs } from 'file-saver';
import { IDisposable, ITerminalAddon, Terminal } from '@xterm/xterm';
import * as Zmodem from 'zmodem.js/src/zmodem_browser';

export interface FlowControl {
  limit: number;
  highWater: number;
  lowWater: number;

  pause: () => void;
  resume: () => void;
}

interface Props {
  id: string;
  sender: (data: ArrayLike<number>) => void;
  control: FlowControl;
}

export class ZmodemAddon extends React.Component<Props> implements ITerminalAddon {
  private terminal: Terminal | undefined;

  private keyDispose: IDisposable | undefined;

  private sentry: Zmodem.Sentry | undefined;

  private session: Zmodem.Session;

  private written = 0;

  dispose(): void {}

  private pending = 0;

  private gotFocus: boolean = false;

  constructor(props: Props) {
    super(props);

    this.handleError = this.handleError.bind(this);
    this.zmodemInit = this.zmodemInit.bind(this);
    this.zmodemReset = this.zmodemReset.bind(this);
    this.zmodemWrite = this.zmodemWrite.bind(this);
    this.zmodemSend = this.zmodemSend.bind(this);
    this.zmodemDetect = this.zmodemDetect.bind(this);
    this.receiveFile = this.receiveFile.bind(this);
    this.writeProgress = this.writeProgress.bind(this);

    this.zmodemInit();
  }

  private handleError(e: Error, reason: string) {
    console.error(`[ttyd] zmodem ${reason}: `, e);
    this.zmodemReset();
  }

  activate(terminal: Terminal): void {
    this.terminal = terminal;
  }

  consume(data: ArrayBuffer) {
    const { sentry, handleError } = this;
    try {
      if (sentry) sentry.consume(data);
    } catch (e: unknown) {
      handleError(e as Error, 'consume');
    }
  }

  private zmodemInit() {
    this.session = null;
    this.sentry = new Zmodem.Sentry({
      to_terminal: (octets: ArrayBuffer) => this.zmodemWrite(octets),
      sender: (octets: ArrayLike<number>) => this.zmodemSend(octets),
      on_retract: () => this.zmodemReset(),
      on_detect: (detection: Zmodem.Detection) => this.zmodemDetect(detection),
    });
  }

  private zmodemReset() {
    if (!this.terminal) return;

    this.terminal.options.disableStdin = false;

    if (this.keyDispose) {
      this.keyDispose.dispose();
      this.keyDispose = undefined;
    }
    this.zmodemInit();

    this.terminal.focus();
  }

  private zmodemWrite(data: ArrayBuffer): void {
    const { control } = this.props;
    const { limit, highWater, lowWater, pause, resume } = control;
    const { terminal } = this;
    const rawData = new Uint8Array(data);
    if (terminal) {
      if (!this.gotFocus) {
        // workaround to focus textarea
        terminal.focus();
        this.gotFocus = true;
      }
    }

    this.written += rawData.length;
    if (this.written > limit) {
      if (terminal)
        terminal.write(rawData, () => {
          this.pending = Math.max(this.pending - 1, 0);
          if (this.pending < lowWater) {
            resume();
          }
        });

      this.pending += 1;
      this.written = 0;
      if (this.pending > highWater) {
        pause();
      }
    }

    if (terminal) {
      terminal.write(rawData);
    }
  }

  private zmodemSend(data: ArrayLike<number>): void {
    const { sender } = this.props;
    sender(data);
  }

  private zmodemDetect(detection: Zmodem.Detection): void {
    const { terminal, receiveFile, zmodemReset } = this;
    if (!terminal) return;

    terminal.options.disableStdin = true;

    this.keyDispose = terminal.onKey((e) => {
      const event = e.domEvent;
      if (event.ctrlKey && event.key === 'c') {
        detection.deny();
      }
    });

    this.session = detection.confirm();
    this.session.on('session_end', zmodemReset);

    if (this.session.type !== 'send') {
      receiveFile();
    }
  }

  // @bind
  // private sendFile(event: Event) {
  //   const { session, writeProgress, handleError } = this;
  //   const files: FileList = (event.target as HTMLInputElement).files!;

  //   Zmodem.Browser.send_files(session, files, {
  //     on_progress: (_: unknown, offer: Zmodem.Offer) => writeProgress(offer),
  //   })
  //     .then(() => session.close())
  //     .catch((e: Error) => handleError(e, 'send'));
  // }

  private receiveFile() {
    const { session, writeProgress, handleError } = this;

    session.on('offer', (offer: Zmodem.Offer) => {
      const fileBuffer: Uint8Array[] = [];
      offer.on('input', (payload: any) => {
        writeProgress(offer);
        fileBuffer.push(new Uint8Array(payload));
      });
      offer
        .accept()
        .then(() => {
          const blob = new Blob(fileBuffer, {
            type: 'application/octet-stream',
          });
          saveAs(blob, offer.get_details().name);
          return true;
        })
        .catch((e: Error) => handleError(e, 'receive'));
    });

    session.start();
  }

  private writeProgress(offer: Zmodem.Offer) {
    const { terminal, bytesHuman } = this;

    const file = offer.get_details();
    const [name, size] = file;
    const offset = offer.get_offset();
    const percent = ((100 * offset) / size).toFixed(2);

    if (terminal) terminal.write(`${name} ${percent}% ${bytesHuman(offset, 2)}/${bytesHuman(size, 2)}\r`);
  }

  private bytesHuman(bytes: number, precision: number): string {
    let mPrecision = precision;
    if (!/^([-+])?|(\.\d+)(\d+(\.\d+)?|(\d+\.)|Infinity)$/.test(bytes.toString())) {
      return '-';
    }
    if (bytes === 0) return '0';
    if (typeof mPrecision === 'undefined') mPrecision = 1;
    const units = ['bytes', 'KB', 'MB', 'GB', 'TB', 'PB'];
    const num = Math.floor(Math.log(bytes) / Math.log(1024));
    const value = (bytes / 1024 ** Math.floor(num)).toFixed(precision);
    return `${value} ${units[num]}`;
  }

  render() {
    return <></>;
  }
}
