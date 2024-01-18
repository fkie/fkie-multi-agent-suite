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

export { delay, extractSubstring, generateUniqueId, pathJoin };
