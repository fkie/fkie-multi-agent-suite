import { RosNode } from "../models";

export function generateUniqueId(): string {
  return Date.now().toString(36) + Math.random().toString(36).substr(2);
}

export function extractSubstring(data: string, s1: string, s2: string): string {
  if (!data || data.length === 0) return "";

  let indexS1 = 0;
  let indexS2 = data.length;

  if (s1 && s1.length > 0) indexS1 = data.lastIndexOf(s1);
  if (s2 && s2.length > 0) indexS2 = data.lastIndexOf(s2);

  return data.substring(indexS1 + s1.length, indexS2);
}

export function delay(ms: number): Promise<string> {
  return new Promise((resolve) => {
    setTimeout(() => {
      resolve("");
    }, ms);
  });
}

export function pathJoin(pathArr: string[]): string {
  return pathArr
    .map((path) => {
      let result = path;
      if (result[0] === "/") {
        result = result.slice(1);
      }
      if (result[result.length - 1] === "/") {
        result = result.slice(0, result.length - 1);
      }
      return result;
    })
    .join("/");
}

export function splitOrSearchTerm(searchTerm: string): string[] {
  return searchTerm.split(" ").filter((item) => item.length > 0);
}

export function splitAndSearchTerm(searchTerm: string): string[] {
  return searchTerm.split("+").filter((item) => item.length > 0);
}

/** Search for a given search term in given word without split the term.
 * Returns true if one of the words contains the search term.
 */
export function findTerm(searchTerm: string, words: string[]): boolean {
  // eslint-disable-next-line no-restricted-syntax
  for (const w of words) {
    if (Array.isArray(w)) {
      // eslint-disable-next-line no-restricted-syntax
      for (const sw of w) {
        if (sw.toLowerCase().includes(searchTerm.toLowerCase())) {
          return true;
        }
      }
    } else if (w) {
      if (w.toLowerCase().includes(searchTerm.toLowerCase())) {
        return true;
      }
    }
  }
  return false;
}

/** Splits the search term first by <space> for OR and then by "+" for AND.
 */
export function findIn(searchTerms: string, words: string[]): boolean {
  let invert = false;
  const searchOrTerms = splitOrSearchTerm(searchTerms);
  // eslint-disable-next-line no-restricted-syntax
  for (const sO of searchOrTerms) {
    const searchAndTerms = splitAndSearchTerm(sO);
    if (searchAndTerms.length === 1) {
      invert = searchAndTerms[0].startsWith("!");
      const searchFor = invert ? searchAndTerms[0].slice(1) : searchAndTerms[0];
      const found = findTerm(searchFor, words);
      if (invert && found) {
        return false;
      }
      if (!invert && found) {
        return true;
      }
    } else {
      // returns only true if all search terms are found
      let foundAnd = true;
      // eslint-disable-next-line no-restricted-syntax
      for (const sA of searchAndTerms) {
        const sInvert = sA.startsWith("!");
        const searchFor = sInvert ? sA.slice(1) : sA;
        const found = findTerm(searchFor, words);
        if (sInvert && found) {
          // if inverted and found whole entry will not be shown
          return false;
        }
        if (!sInvert && !found) {
          foundAnd = false;
        }
      }
      if (foundAnd) {
        return true;
      }
    }
  }
  return invert;
}

export function removeDDSuid(item: string): string {
  if (item) {
    const lastIndex = item.lastIndexOf("-");
    return lastIndex === -1 ? item : item.substring(0, lastIndex);
  }
  return item;
}

/**
 * Return the first and last letter of the base name
 */
export function getRosNameAbb(name: string): string {
  if (!name) return name;
  const base = name.replace(/^.*[\\/]/, "").replace(/@.*/, "");
  if (base) {
    let name = base
      .split("_")
      .map((item) => item[0])
      .join("");
    if (name.length < 2) {
      const midMatch = base.match(/^.*(\d+).*?/);
      if (midMatch) {
        name += midMatch[1];
        const index = base.indexOf(midMatch[1]);
        if (base.length > index + 1) {
          name += base[index + 1];
        }
      }
    }
    if (name.length < 3) {
      name += base[base.length - 1];
    }
    // check if ends with number
    const lastMatch = base.match(/^.*(\d+)$/);
    if (lastMatch) {
      name += lastMatch[1];
    }
    return name;
  }
  return name;
}

export function xor(a1: boolean, a2: boolean): boolean {
  return (a1 && !a2) || (!a1 && a2);
}

export function nodeNameWithoutNamespace(node: RosNode): string {
  const name = node.namespace && node.namespace !== "/" ? node.name.replace(node.namespace, "") : node.name;
  return name[0] === "/" ? name.slice(1) : name;
}

export function basename(name: string): string {
  const result: string = name.split("/").slice(-1)[0];
  return result;
}

export function normalizeNameWithPrefix(name: string, prefix: string): string {
  if (!prefix) return name;
  if (name.startsWith(prefix)) {
    return name.slice(prefix.length + 1);
  }
  return name.startsWith("/") ? name.slice(1) : name;
}
