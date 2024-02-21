const generateUniqueId = () => {
  return Date.now().toString(36) + Math.random().toString(36).substr(2);
};

const extractSubstring = (data, s1, s2) => {
  if (!data || data.length === 0) return '';

  let indexS1 = 0;
  let indexS2 = data.length;

  if (s1 && s1.length > 0) indexS1 = data.lastIndexOf(s1);
  if (s2 && s2.length > 0) indexS2 = data.lastIndexOf(s2);

  return data.substring(indexS1 + s1.length, indexS2);
};

const delay = (ms) => {
  return new Promise((resolve) => {
    setTimeout(() => {
      resolve('');
    }, ms);
  });
};

const pathJoin = (pathArr) => {
  return pathArr
    .map(function (path) {
      if (path[0] === '/') {
        path = path.slice(1);
      }
      if (path[path.length - 1] === '/') {
        path = path.slice(0, path.length - 1);
      }
      return path;
    })
    .join('/');
};

const splitOrSearchTerm = (searchTerm) => {
  return searchTerm.split(' ').filter((item) => item.length > 0);
};

const splitAndSearchTerm = (searchTerm) => {
  return searchTerm.split('+').filter((item) => item.length > 0);
};

/** Search for a given search term in given word without split the term.
 * Returns true if one of the words contains the search term.
 */
const findTerm = (searchTerm, words) => {
  for (const w of words) {
    if (Array.isArray(w)) {
      for (const sw of w) {
        if (sw.indexOf(searchTerm) !== -1) {
          return true;
        }
      }
    } else {
      if (w.indexOf(searchTerm) !== -1) {
        return true;
      }
    }
  }
  return false;
};

/** Splits the search term first by <space> for OR and then by "+" for AND.
 */
const findIn = (searchTerms, words) => {
  const searchOrTerms = splitOrSearchTerm(searchTerms);
  for (const sO of searchOrTerms) {
    const searchAndTerms = splitAndSearchTerm(sO);
    if (searchAndTerms.length === 1) {
      if (findTerm(searchAndTerms[0], words)) {
        return true;
      }
    } else {
      // returns only true if all search terms are found
      let foundAnd = true;
      for (const sA of searchAndTerms) {
        if (!findTerm(sA, words)) {
          foundAnd = false;
        }
      }
      if (foundAnd) {
        return true;
      }
    }
  }
  return false;
};

export { delay, extractSubstring, findIn, generateUniqueId, pathJoin };
