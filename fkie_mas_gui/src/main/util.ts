import path from "path";
import { URL } from "url";

export let resolveHtmlPath: (htmlFileName: string) => string;

if (process.env.NODE_ENV === "development") {
  const port = process.env.PORT || 1212;
  resolveHtmlPath = (htmlFileName: string): string => {
    const url = new URL(`http://localhost:${port}`);
    url.pathname = htmlFileName;
    return url.href;
  };
} else {
  resolveHtmlPath = (htmlFileName: string): string => {
    return `file://${path.resolve(__dirname, "../renderer/", htmlFileName)}`;
  };
}

/**
 * Promise-based delay
 *
 * @param {number} ms - Delay time in milliseconds
 */
const delay: (ms: number) => void = (ms: number) => {
  return new Promise((resolve) => {
    setTimeout(() => {
      resolve("");
    }, ms);
  });
};

export { delay };
