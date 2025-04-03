/**
 * vkBeautify - javascript plugin to pretty-print or minify text in XML, JSON, CSS and SQL formats.
 *
 * Version - 0.99.00.beta
 * Copyright (c) 2012 Vadim Kiryukhin
 * vkiryukhin @ gmail.com
 * http://www.eslinstructor.net/vkbeautify/
 *
 * MIT license:
 *   http://www.opensource.org/licenses/mit-license.php
 */

export default class XmlBeautify {
  step: string;
  shift: string[];

  constructor() {
    this.step = "  ";
    this.shift = this._createShiftArr(this.step);
  }

  _createShiftArr(step: number | string): string[] {
    let space = "    ";

    if (typeof step === "string") {
      // argument is string
      space = step;
    } else {
      // argument is integer
      switch (step) {
        case 1:
          space = " ";
          break;
        case 2:
          space = "  ";
          break;
        case 3:
          space = "   ";
          break;
        case 4:
          space = "    ";
          break;
        case 5:
          space = "     ";
          break;
        case 6:
          space = "      ";
          break;
        case 7:
          space = "       ";
          break;
        case 8:
          space = "        ";
          break;
        case 9:
          space = "         ";
          break;
        case 10:
          space = "          ";
          break;
        case 11:
          space = "           ";
          break;
        case 12:
          space = "            ";
          break;
      }
    }

    const shift = ["\n"]; // array of shifts
    for (let ix = 0; ix < 100; ix++) {
      shift.push(shift[ix] + space);
    }
    return shift;
  }

  beautify(xml: string, step = 2): string {
    const ar = xml
      .replace(/>\s{0,}</g, "><")
      .replace(/</g, "~::~<")
      .replace(/\s*xmlns:/g, "~::~xmlns:")
      .replace(/\s*xmlns=/g, "~::~xmlns=")
      .split("~::~");
    const len = ar.length;
    let inComment = false;
    let deep = 0;
    let str = "";
    let ix = 0;
    const shift = step ? this._createShiftArr(step) : this.shift;

    for (ix = 0; ix < len; ix++) {
      // start comment or <![CDATA[...]]> or <!DOCTYPE //
      if (ar[ix].search(/<!/) > -1) {
        str += shift[deep] + ar[ix];
        inComment = true;
        // end comment  or <![CDATA[...]]> //
        if (ar[ix].search(/-->|--!>/) > -1 || ar[ix].search(/\]>/) > -1 || ar[ix].search(/!DOCTYPE/) > -1) {
          inComment = false;
        }
      }
      // end comment  or <![CDATA[...]]> //
      else if (ar[ix].search(/-->|--!>/) > -1 || ar[ix].search(/\]>/) > -1) {
        str += ar[ix];
        inComment = false;
      }
      // <elm></elm> //
      else if (
        /^<\w/.exec(ar[ix - 1]) &&
        /^<\/\w/.exec(ar[ix]) &&
        /^<[\w:\-.,]+/.exec(ar[ix - 1])?.join() === /^<\/[\w:\-.,]+/.exec(ar[ix])?.join().replace("/", "")
      ) {
        str += ar[ix];
        if (!inComment) deep--;
      }
      // <elm> //
      else if (ar[ix].search(/<\w/) > -1 && ar[ix].search(/<\//) == -1 && ar[ix].search(/\/>/) == -1) {
        str = !inComment ? (str += shift[deep++] + ar[ix]) : (str += ar[ix]);
      }
      // <elm>...</elm> //
      else if (ar[ix].search(/<\w/) > -1 && ar[ix].search(/<\//) > -1) {
        str = !inComment ? (str += shift[deep] + ar[ix]) : (str += ar[ix]);
      }
      // </elm> //
      else if (ar[ix].search(/<\//) > -1) {
        str = !inComment ? (str += shift[--deep] + ar[ix]) : (str += ar[ix]);
      }
      // <elm/> //
      else if (ar[ix].search(/\/>/) > -1) {
        str = !inComment ? (str += shift[deep] + ar[ix]) : (str += ar[ix]);
      }
      // <? xml ... ?> //
      else if (ar[ix].search(/<\?/) > -1) {
        str += shift[deep] + ar[ix];
      }
      // xmlns //
      else if (ar[ix].search(/xmlns:/) > -1 || ar[ix].search(/xmlns=/) > -1) {
        str += shift[deep] + ar[ix];
      } else {
        str += ar[ix];
      }
    }

    return str[0] == "\n" ? str.slice(1) : str;
  }
}
