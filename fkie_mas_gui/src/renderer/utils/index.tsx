import { RosNode } from "../models";

const generateUniqueId = () => {
  return Date.now().toString(36) + Math.random().toString(36).substr(2);
};

const extractSubstring: (data: string, s1: string, s2: string) => string = (data, s1, s2) => {
  if (!data || data.length === 0) return "";

  let indexS1 = 0;
  let indexS2 = data.length;

  if (s1 && s1.length > 0) indexS1 = data.lastIndexOf(s1);
  if (s2 && s2.length > 0) indexS2 = data.lastIndexOf(s2);

  return data.substring(indexS1 + s1.length, indexS2);
};

const delay: (ms: number) => Promise<string> = (ms) => {
  return new Promise((resolve) => {
    setTimeout(() => {
      resolve("");
    }, ms);
  });
};

const pathJoin = (pathArr: string[]) => {
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
};

const splitOrSearchTerm: (searchTerm: string) => string[] = (searchTerm) => {
  return searchTerm.split(" ").filter((item) => item.length > 0);
};

const splitAndSearchTerm: (searchTerm: string) => string[] = (searchTerm) => {
  return searchTerm.split("+").filter((item) => item.length > 0);
};

/** Search for a given search term in given word without split the term.
 * Returns true if one of the words contains the search term.
 */
const findTerm: (searchTerm: string, words: string[]) => boolean = (searchTerm, words) => {
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
};

/** Splits the search term first by <space> for OR and then by "+" for AND.
 */
const findIn: (searchTerms: string, words: string[]) => boolean = (searchTerms, words) => {
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
};

const removeDDSuid: (item: string) => string = (item) => {
  const lastIndex = item.lastIndexOf("-");
  return lastIndex === -1 ? item : item.substring(0, lastIndex);
};

/**
 * Return the first and last letter of the base name
 *
 * @param {string} path - Ros topic/service/node name
 */
const getRosNameAbb: (name: string) => string = (name) => {
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
};

const xor: (a1: boolean, a2: boolean) => boolean = (a1, a2) => {
  return (a1 && !a2) || (!a1 && a2);
};

const nameWithoutNamespace = (node: RosNode) => {
  const name = node.namespace && node.namespace !== "/" ? node.name.replace(node.namespace, "") : node.name;
  return name[0] === "/" ? name.slice(1) : name;
};

export {
  delay,
  extractSubstring,
  findIn,
  generateUniqueId,
  getRosNameAbb,
  nameWithoutNamespace,
  pathJoin,
  removeDDSuid,
  xor,
};
