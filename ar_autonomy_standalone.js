(function(f){if(typeof exports==="object"&&typeof module!=="undefined"){module.exports=f()}else if(typeof define==="function"&&define.amd){define([],f)}else{var g;if(typeof window!=="undefined"){g=window}else if(typeof global!=="undefined"){g=global}else if(typeof self!=="undefined"){g=self}else{g=this}g.Foo = f()}})(function(){var define,module,exports;return (function(){function r(e,n,t){function o(i,f){if(!n[i]){if(!e[i]){var c="function"==typeof require&&require;if(!f&&c)return c(i,!0);if(u)return u(i,!0);var a=new Error("Cannot find module '"+i+"'");throw a.code="MODULE_NOT_FOUND",a}var p=n[i]={exports:{}};e[i][0].call(p.exports,function(r){var n=e[i][1][r];return o(n||r)},p,p.exports,r,e,n,t)}return n[i].exports}for(var u="function"==typeof require&&require,i=0;i<t.length;i++)o(t[i]);return o}return r})()({1:[function(_dereq_,module,exports){

},{}],2:[function(_dereq_,module,exports){
(function (global){
'use strict';

// compare and isBuffer taken from https://github.com/feross/buffer/blob/680e9e5e488f22aac27599a57dc844a6315928dd/index.js
// original notice:

/*!
 * The buffer module from node.js, for the browser.
 *
 * @author   Feross Aboukhadijeh <feross@feross.org> <http://feross.org>
 * @license  MIT
 */
function compare(a, b) {
  if (a === b) {
    return 0;
  }

  var x = a.length;
  var y = b.length;

  for (var i = 0, len = Math.min(x, y); i < len; ++i) {
    if (a[i] !== b[i]) {
      x = a[i];
      y = b[i];
      break;
    }
  }

  if (x < y) {
    return -1;
  }
  if (y < x) {
    return 1;
  }
  return 0;
}
function isBuffer(b) {
  if (global.Buffer && typeof global.Buffer.isBuffer === 'function') {
    return global.Buffer.isBuffer(b);
  }
  return !!(b != null && b._isBuffer);
}

// based on node assert, original notice:

// http://wiki.commonjs.org/wiki/Unit_Testing/1.0
//
// THIS IS NOT TESTED NOR LIKELY TO WORK OUTSIDE V8!
//
// Originally from narwhal.js (http://narwhaljs.org)
// Copyright (c) 2009 Thomas Robinson <280north.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the 'Software'), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
// ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

var util = _dereq_('util/');
var hasOwn = Object.prototype.hasOwnProperty;
var pSlice = Array.prototype.slice;
var functionsHaveNames = (function () {
  return function foo() {}.name === 'foo';
}());
function pToString (obj) {
  return Object.prototype.toString.call(obj);
}
function isView(arrbuf) {
  if (isBuffer(arrbuf)) {
    return false;
  }
  if (typeof global.ArrayBuffer !== 'function') {
    return false;
  }
  if (typeof ArrayBuffer.isView === 'function') {
    return ArrayBuffer.isView(arrbuf);
  }
  if (!arrbuf) {
    return false;
  }
  if (arrbuf instanceof DataView) {
    return true;
  }
  if (arrbuf.buffer && arrbuf.buffer instanceof ArrayBuffer) {
    return true;
  }
  return false;
}
// 1. The assert module provides functions that throw
// AssertionError's when particular conditions are not met. The
// assert module must conform to the following interface.

var assert = module.exports = ok;

// 2. The AssertionError is defined in assert.
// new assert.AssertionError({ message: message,
//                             actual: actual,
//                             expected: expected })

var regex = /\s*function\s+([^\(\s]*)\s*/;
// based on https://github.com/ljharb/function.prototype.name/blob/adeeeec8bfcc6068b187d7d9fb3d5bb1d3a30899/implementation.js
function getName(func) {
  if (!util.isFunction(func)) {
    return;
  }
  if (functionsHaveNames) {
    return func.name;
  }
  var str = func.toString();
  var match = str.match(regex);
  return match && match[1];
}
assert.AssertionError = function AssertionError(options) {
  this.name = 'AssertionError';
  this.actual = options.actual;
  this.expected = options.expected;
  this.operator = options.operator;
  if (options.message) {
    this.message = options.message;
    this.generatedMessage = false;
  } else {
    this.message = getMessage(this);
    this.generatedMessage = true;
  }
  var stackStartFunction = options.stackStartFunction || fail;
  if (Error.captureStackTrace) {
    Error.captureStackTrace(this, stackStartFunction);
  } else {
    // non v8 browsers so we can have a stacktrace
    var err = new Error();
    if (err.stack) {
      var out = err.stack;

      // try to strip useless frames
      var fn_name = getName(stackStartFunction);
      var idx = out.indexOf('\n' + fn_name);
      if (idx >= 0) {
        // once we have located the function frame
        // we need to strip out everything before it (and its line)
        var next_line = out.indexOf('\n', idx + 1);
        out = out.substring(next_line + 1);
      }

      this.stack = out;
    }
  }
};

// assert.AssertionError instanceof Error
util.inherits(assert.AssertionError, Error);

function truncate(s, n) {
  if (typeof s === 'string') {
    return s.length < n ? s : s.slice(0, n);
  } else {
    return s;
  }
}
function inspect(something) {
  if (functionsHaveNames || !util.isFunction(something)) {
    return util.inspect(something);
  }
  var rawname = getName(something);
  var name = rawname ? ': ' + rawname : '';
  return '[Function' +  name + ']';
}
function getMessage(self) {
  return truncate(inspect(self.actual), 128) + ' ' +
         self.operator + ' ' +
         truncate(inspect(self.expected), 128);
}

// At present only the three keys mentioned above are used and
// understood by the spec. Implementations or sub modules can pass
// other keys to the AssertionError's constructor - they will be
// ignored.

// 3. All of the following functions must throw an AssertionError
// when a corresponding condition is not met, with a message that
// may be undefined if not provided.  All assertion methods provide
// both the actual and expected values to the assertion error for
// display purposes.

function fail(actual, expected, message, operator, stackStartFunction) {
  throw new assert.AssertionError({
    message: message,
    actual: actual,
    expected: expected,
    operator: operator,
    stackStartFunction: stackStartFunction
  });
}

// EXTENSION! allows for well behaved errors defined elsewhere.
assert.fail = fail;

// 4. Pure assertion tests whether a value is truthy, as determined
// by !!guard.
// assert.ok(guard, message_opt);
// This statement is equivalent to assert.equal(true, !!guard,
// message_opt);. To test strictly for the value true, use
// assert.strictEqual(true, guard, message_opt);.

function ok(value, message) {
  if (!value) fail(value, true, message, '==', assert.ok);
}
assert.ok = ok;

// 5. The equality assertion tests shallow, coercive equality with
// ==.
// assert.equal(actual, expected, message_opt);

assert.equal = function equal(actual, expected, message) {
  if (actual != expected) fail(actual, expected, message, '==', assert.equal);
};

// 6. The non-equality assertion tests for whether two objects are not equal
// with != assert.notEqual(actual, expected, message_opt);

assert.notEqual = function notEqual(actual, expected, message) {
  if (actual == expected) {
    fail(actual, expected, message, '!=', assert.notEqual);
  }
};

// 7. The equivalence assertion tests a deep equality relation.
// assert.deepEqual(actual, expected, message_opt);

assert.deepEqual = function deepEqual(actual, expected, message) {
  if (!_deepEqual(actual, expected, false)) {
    fail(actual, expected, message, 'deepEqual', assert.deepEqual);
  }
};

assert.deepStrictEqual = function deepStrictEqual(actual, expected, message) {
  if (!_deepEqual(actual, expected, true)) {
    fail(actual, expected, message, 'deepStrictEqual', assert.deepStrictEqual);
  }
};

function _deepEqual(actual, expected, strict, memos) {
  // 7.1. All identical values are equivalent, as determined by ===.
  if (actual === expected) {
    return true;
  } else if (isBuffer(actual) && isBuffer(expected)) {
    return compare(actual, expected) === 0;

  // 7.2. If the expected value is a Date object, the actual value is
  // equivalent if it is also a Date object that refers to the same time.
  } else if (util.isDate(actual) && util.isDate(expected)) {
    return actual.getTime() === expected.getTime();

  // 7.3 If the expected value is a RegExp object, the actual value is
  // equivalent if it is also a RegExp object with the same source and
  // properties (`global`, `multiline`, `lastIndex`, `ignoreCase`).
  } else if (util.isRegExp(actual) && util.isRegExp(expected)) {
    return actual.source === expected.source &&
           actual.global === expected.global &&
           actual.multiline === expected.multiline &&
           actual.lastIndex === expected.lastIndex &&
           actual.ignoreCase === expected.ignoreCase;

  // 7.4. Other pairs that do not both pass typeof value == 'object',
  // equivalence is determined by ==.
  } else if ((actual === null || typeof actual !== 'object') &&
             (expected === null || typeof expected !== 'object')) {
    return strict ? actual === expected : actual == expected;

  // If both values are instances of typed arrays, wrap their underlying
  // ArrayBuffers in a Buffer each to increase performance
  // This optimization requires the arrays to have the same type as checked by
  // Object.prototype.toString (aka pToString). Never perform binary
  // comparisons for Float*Arrays, though, since e.g. +0 === -0 but their
  // bit patterns are not identical.
  } else if (isView(actual) && isView(expected) &&
             pToString(actual) === pToString(expected) &&
             !(actual instanceof Float32Array ||
               actual instanceof Float64Array)) {
    return compare(new Uint8Array(actual.buffer),
                   new Uint8Array(expected.buffer)) === 0;

  // 7.5 For all other Object pairs, including Array objects, equivalence is
  // determined by having the same number of owned properties (as verified
  // with Object.prototype.hasOwnProperty.call), the same set of keys
  // (although not necessarily the same order), equivalent values for every
  // corresponding key, and an identical 'prototype' property. Note: this
  // accounts for both named and indexed properties on Arrays.
  } else if (isBuffer(actual) !== isBuffer(expected)) {
    return false;
  } else {
    memos = memos || {actual: [], expected: []};

    var actualIndex = memos.actual.indexOf(actual);
    if (actualIndex !== -1) {
      if (actualIndex === memos.expected.indexOf(expected)) {
        return true;
      }
    }

    memos.actual.push(actual);
    memos.expected.push(expected);

    return objEquiv(actual, expected, strict, memos);
  }
}

function isArguments(object) {
  return Object.prototype.toString.call(object) == '[object Arguments]';
}

function objEquiv(a, b, strict, actualVisitedObjects) {
  if (a === null || a === undefined || b === null || b === undefined)
    return false;
  // if one is a primitive, the other must be same
  if (util.isPrimitive(a) || util.isPrimitive(b))
    return a === b;
  if (strict && Object.getPrototypeOf(a) !== Object.getPrototypeOf(b))
    return false;
  var aIsArgs = isArguments(a);
  var bIsArgs = isArguments(b);
  if ((aIsArgs && !bIsArgs) || (!aIsArgs && bIsArgs))
    return false;
  if (aIsArgs) {
    a = pSlice.call(a);
    b = pSlice.call(b);
    return _deepEqual(a, b, strict);
  }
  var ka = objectKeys(a);
  var kb = objectKeys(b);
  var key, i;
  // having the same number of owned properties (keys incorporates
  // hasOwnProperty)
  if (ka.length !== kb.length)
    return false;
  //the same set of keys (although not necessarily the same order),
  ka.sort();
  kb.sort();
  //~~~cheap key test
  for (i = ka.length - 1; i >= 0; i--) {
    if (ka[i] !== kb[i])
      return false;
  }
  //equivalent values for every corresponding key, and
  //~~~possibly expensive deep test
  for (i = ka.length - 1; i >= 0; i--) {
    key = ka[i];
    if (!_deepEqual(a[key], b[key], strict, actualVisitedObjects))
      return false;
  }
  return true;
}

// 8. The non-equivalence assertion tests for any deep inequality.
// assert.notDeepEqual(actual, expected, message_opt);

assert.notDeepEqual = function notDeepEqual(actual, expected, message) {
  if (_deepEqual(actual, expected, false)) {
    fail(actual, expected, message, 'notDeepEqual', assert.notDeepEqual);
  }
};

assert.notDeepStrictEqual = notDeepStrictEqual;
function notDeepStrictEqual(actual, expected, message) {
  if (_deepEqual(actual, expected, true)) {
    fail(actual, expected, message, 'notDeepStrictEqual', notDeepStrictEqual);
  }
}


// 9. The strict equality assertion tests strict equality, as determined by ===.
// assert.strictEqual(actual, expected, message_opt);

assert.strictEqual = function strictEqual(actual, expected, message) {
  if (actual !== expected) {
    fail(actual, expected, message, '===', assert.strictEqual);
  }
};

// 10. The strict non-equality assertion tests for strict inequality, as
// determined by !==.  assert.notStrictEqual(actual, expected, message_opt);

assert.notStrictEqual = function notStrictEqual(actual, expected, message) {
  if (actual === expected) {
    fail(actual, expected, message, '!==', assert.notStrictEqual);
  }
};

function expectedException(actual, expected) {
  if (!actual || !expected) {
    return false;
  }

  if (Object.prototype.toString.call(expected) == '[object RegExp]') {
    return expected.test(actual);
  }

  try {
    if (actual instanceof expected) {
      return true;
    }
  } catch (e) {
    // Ignore.  The instanceof check doesn't work for arrow functions.
  }

  if (Error.isPrototypeOf(expected)) {
    return false;
  }

  return expected.call({}, actual) === true;
}

function _tryBlock(block) {
  var error;
  try {
    block();
  } catch (e) {
    error = e;
  }
  return error;
}

function _throws(shouldThrow, block, expected, message) {
  var actual;

  if (typeof block !== 'function') {
    throw new TypeError('"block" argument must be a function');
  }

  if (typeof expected === 'string') {
    message = expected;
    expected = null;
  }

  actual = _tryBlock(block);

  message = (expected && expected.name ? ' (' + expected.name + ').' : '.') +
            (message ? ' ' + message : '.');

  if (shouldThrow && !actual) {
    fail(actual, expected, 'Missing expected exception' + message);
  }

  var userProvidedMessage = typeof message === 'string';
  var isUnwantedException = !shouldThrow && util.isError(actual);
  var isUnexpectedException = !shouldThrow && actual && !expected;

  if ((isUnwantedException &&
      userProvidedMessage &&
      expectedException(actual, expected)) ||
      isUnexpectedException) {
    fail(actual, expected, 'Got unwanted exception' + message);
  }

  if ((shouldThrow && actual && expected &&
      !expectedException(actual, expected)) || (!shouldThrow && actual)) {
    throw actual;
  }
}

// 11. Expected to throw an error:
// assert.throws(block, Error_opt, message_opt);

assert.throws = function(block, /*optional*/error, /*optional*/message) {
  _throws(true, block, error, message);
};

// EXTENSION! This is annoying to write outside this module.
assert.doesNotThrow = function(block, /*optional*/error, /*optional*/message) {
  _throws(false, block, error, message);
};

assert.ifError = function(err) { if (err) throw err; };

var objectKeys = Object.keys || function (obj) {
  var keys = [];
  for (var key in obj) {
    if (hasOwn.call(obj, key)) keys.push(key);
  }
  return keys;
};

}).call(this,typeof global !== "undefined" ? global : typeof self !== "undefined" ? self : typeof window !== "undefined" ? window : {})
},{"util/":5}],3:[function(_dereq_,module,exports){
if (typeof Object.create === 'function') {
  // implementation from standard node.js 'util' module
  module.exports = function inherits(ctor, superCtor) {
    ctor.super_ = superCtor
    ctor.prototype = Object.create(superCtor.prototype, {
      constructor: {
        value: ctor,
        enumerable: false,
        writable: true,
        configurable: true
      }
    });
  };
} else {
  // old school shim for old browsers
  module.exports = function inherits(ctor, superCtor) {
    ctor.super_ = superCtor
    var TempCtor = function () {}
    TempCtor.prototype = superCtor.prototype
    ctor.prototype = new TempCtor()
    ctor.prototype.constructor = ctor
  }
}

},{}],4:[function(_dereq_,module,exports){
module.exports = function isBuffer(arg) {
  return arg && typeof arg === 'object'
    && typeof arg.copy === 'function'
    && typeof arg.fill === 'function'
    && typeof arg.readUInt8 === 'function';
}
},{}],5:[function(_dereq_,module,exports){
(function (process,global){
// Copyright Joyent, Inc. and other Node contributors.
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to permit
// persons to whom the Software is furnished to do so, subject to the
// following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
// NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
// USE OR OTHER DEALINGS IN THE SOFTWARE.

var formatRegExp = /%[sdj%]/g;
exports.format = function(f) {
  if (!isString(f)) {
    var objects = [];
    for (var i = 0; i < arguments.length; i++) {
      objects.push(inspect(arguments[i]));
    }
    return objects.join(' ');
  }

  var i = 1;
  var args = arguments;
  var len = args.length;
  var str = String(f).replace(formatRegExp, function(x) {
    if (x === '%%') return '%';
    if (i >= len) return x;
    switch (x) {
      case '%s': return String(args[i++]);
      case '%d': return Number(args[i++]);
      case '%j':
        try {
          return JSON.stringify(args[i++]);
        } catch (_) {
          return '[Circular]';
        }
      default:
        return x;
    }
  });
  for (var x = args[i]; i < len; x = args[++i]) {
    if (isNull(x) || !isObject(x)) {
      str += ' ' + x;
    } else {
      str += ' ' + inspect(x);
    }
  }
  return str;
};


// Mark that a method should not be used.
// Returns a modified function which warns once by default.
// If --no-deprecation is set, then it is a no-op.
exports.deprecate = function(fn, msg) {
  // Allow for deprecating things in the process of starting up.
  if (isUndefined(global.process)) {
    return function() {
      return exports.deprecate(fn, msg).apply(this, arguments);
    };
  }

  if (process.noDeprecation === true) {
    return fn;
  }

  var warned = false;
  function deprecated() {
    if (!warned) {
      if (process.throwDeprecation) {
        throw new Error(msg);
      } else if (process.traceDeprecation) {
        console.trace(msg);
      } else {
        console.error(msg);
      }
      warned = true;
    }
    return fn.apply(this, arguments);
  }

  return deprecated;
};


var debugs = {};
var debugEnviron;
exports.debuglog = function(set) {
  if (isUndefined(debugEnviron))
    debugEnviron = process.env.NODE_DEBUG || '';
  set = set.toUpperCase();
  if (!debugs[set]) {
    if (new RegExp('\\b' + set + '\\b', 'i').test(debugEnviron)) {
      var pid = process.pid;
      debugs[set] = function() {
        var msg = exports.format.apply(exports, arguments);
        console.error('%s %d: %s', set, pid, msg);
      };
    } else {
      debugs[set] = function() {};
    }
  }
  return debugs[set];
};


/**
 * Echos the value of a value. Trys to print the value out
 * in the best way possible given the different types.
 *
 * @param {Object} obj The object to print out.
 * @param {Object} opts Optional options object that alters the output.
 */
/* legacy: obj, showHidden, depth, colors*/
function inspect(obj, opts) {
  // default options
  var ctx = {
    seen: [],
    stylize: stylizeNoColor
  };
  // legacy...
  if (arguments.length >= 3) ctx.depth = arguments[2];
  if (arguments.length >= 4) ctx.colors = arguments[3];
  if (isBoolean(opts)) {
    // legacy...
    ctx.showHidden = opts;
  } else if (opts) {
    // got an "options" object
    exports._extend(ctx, opts);
  }
  // set default options
  if (isUndefined(ctx.showHidden)) ctx.showHidden = false;
  if (isUndefined(ctx.depth)) ctx.depth = 2;
  if (isUndefined(ctx.colors)) ctx.colors = false;
  if (isUndefined(ctx.customInspect)) ctx.customInspect = true;
  if (ctx.colors) ctx.stylize = stylizeWithColor;
  return formatValue(ctx, obj, ctx.depth);
}
exports.inspect = inspect;


// http://en.wikipedia.org/wiki/ANSI_escape_code#graphics
inspect.colors = {
  'bold' : [1, 22],
  'italic' : [3, 23],
  'underline' : [4, 24],
  'inverse' : [7, 27],
  'white' : [37, 39],
  'grey' : [90, 39],
  'black' : [30, 39],
  'blue' : [34, 39],
  'cyan' : [36, 39],
  'green' : [32, 39],
  'magenta' : [35, 39],
  'red' : [31, 39],
  'yellow' : [33, 39]
};

// Don't use 'blue' not visible on cmd.exe
inspect.styles = {
  'special': 'cyan',
  'number': 'yellow',
  'boolean': 'yellow',
  'undefined': 'grey',
  'null': 'bold',
  'string': 'green',
  'date': 'magenta',
  // "name": intentionally not styling
  'regexp': 'red'
};


function stylizeWithColor(str, styleType) {
  var style = inspect.styles[styleType];

  if (style) {
    return '\u001b[' + inspect.colors[style][0] + 'm' + str +
           '\u001b[' + inspect.colors[style][1] + 'm';
  } else {
    return str;
  }
}


function stylizeNoColor(str, styleType) {
  return str;
}


function arrayToHash(array) {
  var hash = {};

  array.forEach(function(val, idx) {
    hash[val] = true;
  });

  return hash;
}


function formatValue(ctx, value, recurseTimes) {
  // Provide a hook for user-specified inspect functions.
  // Check that value is an object with an inspect function on it
  if (ctx.customInspect &&
      value &&
      isFunction(value.inspect) &&
      // Filter out the util module, it's inspect function is special
      value.inspect !== exports.inspect &&
      // Also filter out any prototype objects using the circular check.
      !(value.constructor && value.constructor.prototype === value)) {
    var ret = value.inspect(recurseTimes, ctx);
    if (!isString(ret)) {
      ret = formatValue(ctx, ret, recurseTimes);
    }
    return ret;
  }

  // Primitive types cannot have properties
  var primitive = formatPrimitive(ctx, value);
  if (primitive) {
    return primitive;
  }

  // Look up the keys of the object.
  var keys = Object.keys(value);
  var visibleKeys = arrayToHash(keys);

  if (ctx.showHidden) {
    keys = Object.getOwnPropertyNames(value);
  }

  // IE doesn't make error fields non-enumerable
  // http://msdn.microsoft.com/en-us/library/ie/dww52sbt(v=vs.94).aspx
  if (isError(value)
      && (keys.indexOf('message') >= 0 || keys.indexOf('description') >= 0)) {
    return formatError(value);
  }

  // Some type of object without properties can be shortcutted.
  if (keys.length === 0) {
    if (isFunction(value)) {
      var name = value.name ? ': ' + value.name : '';
      return ctx.stylize('[Function' + name + ']', 'special');
    }
    if (isRegExp(value)) {
      return ctx.stylize(RegExp.prototype.toString.call(value), 'regexp');
    }
    if (isDate(value)) {
      return ctx.stylize(Date.prototype.toString.call(value), 'date');
    }
    if (isError(value)) {
      return formatError(value);
    }
  }

  var base = '', array = false, braces = ['{', '}'];

  // Make Array say that they are Array
  if (isArray(value)) {
    array = true;
    braces = ['[', ']'];
  }

  // Make functions say that they are functions
  if (isFunction(value)) {
    var n = value.name ? ': ' + value.name : '';
    base = ' [Function' + n + ']';
  }

  // Make RegExps say that they are RegExps
  if (isRegExp(value)) {
    base = ' ' + RegExp.prototype.toString.call(value);
  }

  // Make dates with properties first say the date
  if (isDate(value)) {
    base = ' ' + Date.prototype.toUTCString.call(value);
  }

  // Make error with message first say the error
  if (isError(value)) {
    base = ' ' + formatError(value);
  }

  if (keys.length === 0 && (!array || value.length == 0)) {
    return braces[0] + base + braces[1];
  }

  if (recurseTimes < 0) {
    if (isRegExp(value)) {
      return ctx.stylize(RegExp.prototype.toString.call(value), 'regexp');
    } else {
      return ctx.stylize('[Object]', 'special');
    }
  }

  ctx.seen.push(value);

  var output;
  if (array) {
    output = formatArray(ctx, value, recurseTimes, visibleKeys, keys);
  } else {
    output = keys.map(function(key) {
      return formatProperty(ctx, value, recurseTimes, visibleKeys, key, array);
    });
  }

  ctx.seen.pop();

  return reduceToSingleString(output, base, braces);
}


function formatPrimitive(ctx, value) {
  if (isUndefined(value))
    return ctx.stylize('undefined', 'undefined');
  if (isString(value)) {
    var simple = '\'' + JSON.stringify(value).replace(/^"|"$/g, '')
                                             .replace(/'/g, "\\'")
                                             .replace(/\\"/g, '"') + '\'';
    return ctx.stylize(simple, 'string');
  }
  if (isNumber(value))
    return ctx.stylize('' + value, 'number');
  if (isBoolean(value))
    return ctx.stylize('' + value, 'boolean');
  // For some reason typeof null is "object", so special case here.
  if (isNull(value))
    return ctx.stylize('null', 'null');
}


function formatError(value) {
  return '[' + Error.prototype.toString.call(value) + ']';
}


function formatArray(ctx, value, recurseTimes, visibleKeys, keys) {
  var output = [];
  for (var i = 0, l = value.length; i < l; ++i) {
    if (hasOwnProperty(value, String(i))) {
      output.push(formatProperty(ctx, value, recurseTimes, visibleKeys,
          String(i), true));
    } else {
      output.push('');
    }
  }
  keys.forEach(function(key) {
    if (!key.match(/^\d+$/)) {
      output.push(formatProperty(ctx, value, recurseTimes, visibleKeys,
          key, true));
    }
  });
  return output;
}


function formatProperty(ctx, value, recurseTimes, visibleKeys, key, array) {
  var name, str, desc;
  desc = Object.getOwnPropertyDescriptor(value, key) || { value: value[key] };
  if (desc.get) {
    if (desc.set) {
      str = ctx.stylize('[Getter/Setter]', 'special');
    } else {
      str = ctx.stylize('[Getter]', 'special');
    }
  } else {
    if (desc.set) {
      str = ctx.stylize('[Setter]', 'special');
    }
  }
  if (!hasOwnProperty(visibleKeys, key)) {
    name = '[' + key + ']';
  }
  if (!str) {
    if (ctx.seen.indexOf(desc.value) < 0) {
      if (isNull(recurseTimes)) {
        str = formatValue(ctx, desc.value, null);
      } else {
        str = formatValue(ctx, desc.value, recurseTimes - 1);
      }
      if (str.indexOf('\n') > -1) {
        if (array) {
          str = str.split('\n').map(function(line) {
            return '  ' + line;
          }).join('\n').substr(2);
        } else {
          str = '\n' + str.split('\n').map(function(line) {
            return '   ' + line;
          }).join('\n');
        }
      }
    } else {
      str = ctx.stylize('[Circular]', 'special');
    }
  }
  if (isUndefined(name)) {
    if (array && key.match(/^\d+$/)) {
      return str;
    }
    name = JSON.stringify('' + key);
    if (name.match(/^"([a-zA-Z_][a-zA-Z_0-9]*)"$/)) {
      name = name.substr(1, name.length - 2);
      name = ctx.stylize(name, 'name');
    } else {
      name = name.replace(/'/g, "\\'")
                 .replace(/\\"/g, '"')
                 .replace(/(^"|"$)/g, "'");
      name = ctx.stylize(name, 'string');
    }
  }

  return name + ': ' + str;
}


function reduceToSingleString(output, base, braces) {
  var numLinesEst = 0;
  var length = output.reduce(function(prev, cur) {
    numLinesEst++;
    if (cur.indexOf('\n') >= 0) numLinesEst++;
    return prev + cur.replace(/\u001b\[\d\d?m/g, '').length + 1;
  }, 0);

  if (length > 60) {
    return braces[0] +
           (base === '' ? '' : base + '\n ') +
           ' ' +
           output.join(',\n  ') +
           ' ' +
           braces[1];
  }

  return braces[0] + base + ' ' + output.join(', ') + ' ' + braces[1];
}


// NOTE: These type checking functions intentionally don't use `instanceof`
// because it is fragile and can be easily faked with `Object.create()`.
function isArray(ar) {
  return Array.isArray(ar);
}
exports.isArray = isArray;

function isBoolean(arg) {
  return typeof arg === 'boolean';
}
exports.isBoolean = isBoolean;

function isNull(arg) {
  return arg === null;
}
exports.isNull = isNull;

function isNullOrUndefined(arg) {
  return arg == null;
}
exports.isNullOrUndefined = isNullOrUndefined;

function isNumber(arg) {
  return typeof arg === 'number';
}
exports.isNumber = isNumber;

function isString(arg) {
  return typeof arg === 'string';
}
exports.isString = isString;

function isSymbol(arg) {
  return typeof arg === 'symbol';
}
exports.isSymbol = isSymbol;

function isUndefined(arg) {
  return arg === void 0;
}
exports.isUndefined = isUndefined;

function isRegExp(re) {
  return isObject(re) && objectToString(re) === '[object RegExp]';
}
exports.isRegExp = isRegExp;

function isObject(arg) {
  return typeof arg === 'object' && arg !== null;
}
exports.isObject = isObject;

function isDate(d) {
  return isObject(d) && objectToString(d) === '[object Date]';
}
exports.isDate = isDate;

function isError(e) {
  return isObject(e) &&
      (objectToString(e) === '[object Error]' || e instanceof Error);
}
exports.isError = isError;

function isFunction(arg) {
  return typeof arg === 'function';
}
exports.isFunction = isFunction;

function isPrimitive(arg) {
  return arg === null ||
         typeof arg === 'boolean' ||
         typeof arg === 'number' ||
         typeof arg === 'string' ||
         typeof arg === 'symbol' ||  // ES6 symbol
         typeof arg === 'undefined';
}
exports.isPrimitive = isPrimitive;

exports.isBuffer = _dereq_('./support/isBuffer');

function objectToString(o) {
  return Object.prototype.toString.call(o);
}


function pad(n) {
  return n < 10 ? '0' + n.toString(10) : n.toString(10);
}


var months = ['Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun', 'Jul', 'Aug', 'Sep',
              'Oct', 'Nov', 'Dec'];

// 26 Feb 16:19:34
function timestamp() {
  var d = new Date();
  var time = [pad(d.getHours()),
              pad(d.getMinutes()),
              pad(d.getSeconds())].join(':');
  return [d.getDate(), months[d.getMonth()], time].join(' ');
}


// log is just a thin wrapper to console.log that prepends a timestamp
exports.log = function() {
  console.log('%s - %s', timestamp(), exports.format.apply(exports, arguments));
};


/**
 * Inherit the prototype methods from one constructor into another.
 *
 * The Function.prototype.inherits from lang.js rewritten as a standalone
 * function (not on Function.prototype). NOTE: If this file is to be loaded
 * during bootstrapping this function needs to be rewritten using some native
 * functions as prototype setup using normal JavaScript does not work as
 * expected during bootstrapping (see mirror.js in r114903).
 *
 * @param {function} ctor Constructor function which needs to inherit the
 *     prototype.
 * @param {function} superCtor Constructor function to inherit prototype from.
 */
exports.inherits = _dereq_('inherits');

exports._extend = function(origin, add) {
  // Don't do anything if add isn't an object
  if (!add || !isObject(add)) return origin;

  var keys = Object.keys(add);
  var i = keys.length;
  while (i--) {
    origin[keys[i]] = add[keys[i]];
  }
  return origin;
};

function hasOwnProperty(obj, prop) {
  return Object.prototype.hasOwnProperty.call(obj, prop);
}

}).call(this,_dereq_('_process'),typeof global !== "undefined" ? global : typeof self !== "undefined" ? self : typeof window !== "undefined" ? window : {})
},{"./support/isBuffer":4,"_process":19,"inherits":3}],6:[function(_dereq_,module,exports){
'use strict'

exports.byteLength = byteLength
exports.toByteArray = toByteArray
exports.fromByteArray = fromByteArray

var lookup = []
var revLookup = []
var Arr = typeof Uint8Array !== 'undefined' ? Uint8Array : Array

var code = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/'
for (var i = 0, len = code.length; i < len; ++i) {
  lookup[i] = code[i]
  revLookup[code.charCodeAt(i)] = i
}

// Support decoding URL-safe base64 strings, as Node.js does.
// See: https://en.wikipedia.org/wiki/Base64#URL_applications
revLookup['-'.charCodeAt(0)] = 62
revLookup['_'.charCodeAt(0)] = 63

function getLens (b64) {
  var len = b64.length

  if (len % 4 > 0) {
    throw new Error('Invalid string. Length must be a multiple of 4')
  }

  // Trim off extra bytes after placeholder bytes are found
  // See: https://github.com/beatgammit/base64-js/issues/42
  var validLen = b64.indexOf('=')
  if (validLen === -1) validLen = len

  var placeHoldersLen = validLen === len
    ? 0
    : 4 - (validLen % 4)

  return [validLen, placeHoldersLen]
}

// base64 is 4/3 + up to two characters of the original data
function byteLength (b64) {
  var lens = getLens(b64)
  var validLen = lens[0]
  var placeHoldersLen = lens[1]
  return ((validLen + placeHoldersLen) * 3 / 4) - placeHoldersLen
}

function _byteLength (b64, validLen, placeHoldersLen) {
  return ((validLen + placeHoldersLen) * 3 / 4) - placeHoldersLen
}

function toByteArray (b64) {
  var tmp
  var lens = getLens(b64)
  var validLen = lens[0]
  var placeHoldersLen = lens[1]

  var arr = new Arr(_byteLength(b64, validLen, placeHoldersLen))

  var curByte = 0

  // if there are placeholders, only get up to the last complete 4 chars
  var len = placeHoldersLen > 0
    ? validLen - 4
    : validLen

  for (var i = 0; i < len; i += 4) {
    tmp =
      (revLookup[b64.charCodeAt(i)] << 18) |
      (revLookup[b64.charCodeAt(i + 1)] << 12) |
      (revLookup[b64.charCodeAt(i + 2)] << 6) |
      revLookup[b64.charCodeAt(i + 3)]
    arr[curByte++] = (tmp >> 16) & 0xFF
    arr[curByte++] = (tmp >> 8) & 0xFF
    arr[curByte++] = tmp & 0xFF
  }

  if (placeHoldersLen === 2) {
    tmp =
      (revLookup[b64.charCodeAt(i)] << 2) |
      (revLookup[b64.charCodeAt(i + 1)] >> 4)
    arr[curByte++] = tmp & 0xFF
  }

  if (placeHoldersLen === 1) {
    tmp =
      (revLookup[b64.charCodeAt(i)] << 10) |
      (revLookup[b64.charCodeAt(i + 1)] << 4) |
      (revLookup[b64.charCodeAt(i + 2)] >> 2)
    arr[curByte++] = (tmp >> 8) & 0xFF
    arr[curByte++] = tmp & 0xFF
  }

  return arr
}

function tripletToBase64 (num) {
  return lookup[num >> 18 & 0x3F] +
    lookup[num >> 12 & 0x3F] +
    lookup[num >> 6 & 0x3F] +
    lookup[num & 0x3F]
}

function encodeChunk (uint8, start, end) {
  var tmp
  var output = []
  for (var i = start; i < end; i += 3) {
    tmp =
      ((uint8[i] << 16) & 0xFF0000) +
      ((uint8[i + 1] << 8) & 0xFF00) +
      (uint8[i + 2] & 0xFF)
    output.push(tripletToBase64(tmp))
  }
  return output.join('')
}

function fromByteArray (uint8) {
  var tmp
  var len = uint8.length
  var extraBytes = len % 3 // if we have 1 byte left, pad 2 bytes
  var parts = []
  var maxChunkLength = 16383 // must be multiple of 3

  // go through the array every three bytes, we'll deal with trailing stuff later
  for (var i = 0, len2 = len - extraBytes; i < len2; i += maxChunkLength) {
    parts.push(encodeChunk(
      uint8, i, (i + maxChunkLength) > len2 ? len2 : (i + maxChunkLength)
    ))
  }

  // pad the end with zeros, but make sure to not forget the extra bytes
  if (extraBytes === 1) {
    tmp = uint8[len - 1]
    parts.push(
      lookup[tmp >> 2] +
      lookup[(tmp << 4) & 0x3F] +
      '=='
    )
  } else if (extraBytes === 2) {
    tmp = (uint8[len - 2] << 8) + uint8[len - 1]
    parts.push(
      lookup[tmp >> 10] +
      lookup[(tmp >> 4) & 0x3F] +
      lookup[(tmp << 2) & 0x3F] +
      '='
    )
  }

  return parts.join('')
}

},{}],7:[function(_dereq_,module,exports){
arguments[4][1][0].apply(exports,arguments)
},{"dup":1}],8:[function(_dereq_,module,exports){
/*!
 * The buffer module from node.js, for the browser.
 *
 * @author   Feross Aboukhadijeh <https://feross.org>
 * @license  MIT
 */
/* eslint-disable no-proto */

'use strict'

var base64 = _dereq_('base64-js')
var ieee754 = _dereq_('ieee754')

exports.Buffer = Buffer
exports.SlowBuffer = SlowBuffer
exports.INSPECT_MAX_BYTES = 50

var K_MAX_LENGTH = 0x7fffffff
exports.kMaxLength = K_MAX_LENGTH

/**
 * If `Buffer.TYPED_ARRAY_SUPPORT`:
 *   === true    Use Uint8Array implementation (fastest)
 *   === false   Print warning and recommend using `buffer` v4.x which has an Object
 *               implementation (most compatible, even IE6)
 *
 * Browsers that support typed arrays are IE 10+, Firefox 4+, Chrome 7+, Safari 5.1+,
 * Opera 11.6+, iOS 4.2+.
 *
 * We report that the browser does not support typed arrays if the are not subclassable
 * using __proto__. Firefox 4-29 lacks support for adding new properties to `Uint8Array`
 * (See: https://bugzilla.mozilla.org/show_bug.cgi?id=695438). IE 10 lacks support
 * for __proto__ and has a buggy typed array implementation.
 */
Buffer.TYPED_ARRAY_SUPPORT = typedArraySupport()

if (!Buffer.TYPED_ARRAY_SUPPORT && typeof console !== 'undefined' &&
    typeof console.error === 'function') {
  console.error(
    'This browser lacks typed array (Uint8Array) support which is required by ' +
    '`buffer` v5.x. Use `buffer` v4.x if you require old browser support.'
  )
}

function typedArraySupport () {
  // Can typed array instances can be augmented?
  try {
    var arr = new Uint8Array(1)
    arr.__proto__ = {__proto__: Uint8Array.prototype, foo: function () { return 42 }}
    return arr.foo() === 42
  } catch (e) {
    return false
  }
}

Object.defineProperty(Buffer.prototype, 'parent', {
  get: function () {
    if (!(this instanceof Buffer)) {
      return undefined
    }
    return this.buffer
  }
})

Object.defineProperty(Buffer.prototype, 'offset', {
  get: function () {
    if (!(this instanceof Buffer)) {
      return undefined
    }
    return this.byteOffset
  }
})

function createBuffer (length) {
  if (length > K_MAX_LENGTH) {
    throw new RangeError('Invalid typed array length')
  }
  // Return an augmented `Uint8Array` instance
  var buf = new Uint8Array(length)
  buf.__proto__ = Buffer.prototype
  return buf
}

/**
 * The Buffer constructor returns instances of `Uint8Array` that have their
 * prototype changed to `Buffer.prototype`. Furthermore, `Buffer` is a subclass of
 * `Uint8Array`, so the returned instances will have all the node `Buffer` methods
 * and the `Uint8Array` methods. Square bracket notation works as expected -- it
 * returns a single octet.
 *
 * The `Uint8Array` prototype remains unmodified.
 */

function Buffer (arg, encodingOrOffset, length) {
  // Common case.
  if (typeof arg === 'number') {
    if (typeof encodingOrOffset === 'string') {
      throw new Error(
        'If encoding is specified then the first argument must be a string'
      )
    }
    return allocUnsafe(arg)
  }
  return from(arg, encodingOrOffset, length)
}

// Fix subarray() in ES2016. See: https://github.com/feross/buffer/pull/97
if (typeof Symbol !== 'undefined' && Symbol.species &&
    Buffer[Symbol.species] === Buffer) {
  Object.defineProperty(Buffer, Symbol.species, {
    value: null,
    configurable: true,
    enumerable: false,
    writable: false
  })
}

Buffer.poolSize = 8192 // not used by this implementation

function from (value, encodingOrOffset, length) {
  if (typeof value === 'number') {
    throw new TypeError('"value" argument must not be a number')
  }

  if (isArrayBuffer(value) || (value && isArrayBuffer(value.buffer))) {
    return fromArrayBuffer(value, encodingOrOffset, length)
  }

  if (typeof value === 'string') {
    return fromString(value, encodingOrOffset)
  }

  return fromObject(value)
}

/**
 * Functionally equivalent to Buffer(arg, encoding) but throws a TypeError
 * if value is a number.
 * Buffer.from(str[, encoding])
 * Buffer.from(array)
 * Buffer.from(buffer)
 * Buffer.from(arrayBuffer[, byteOffset[, length]])
 **/
Buffer.from = function (value, encodingOrOffset, length) {
  return from(value, encodingOrOffset, length)
}

// Note: Change prototype *after* Buffer.from is defined to workaround Chrome bug:
// https://github.com/feross/buffer/pull/148
Buffer.prototype.__proto__ = Uint8Array.prototype
Buffer.__proto__ = Uint8Array

function assertSize (size) {
  if (typeof size !== 'number') {
    throw new TypeError('"size" argument must be of type number')
  } else if (size < 0) {
    throw new RangeError('"size" argument must not be negative')
  }
}

function alloc (size, fill, encoding) {
  assertSize(size)
  if (size <= 0) {
    return createBuffer(size)
  }
  if (fill !== undefined) {
    // Only pay attention to encoding if it's a string. This
    // prevents accidentally sending in a number that would
    // be interpretted as a start offset.
    return typeof encoding === 'string'
      ? createBuffer(size).fill(fill, encoding)
      : createBuffer(size).fill(fill)
  }
  return createBuffer(size)
}

/**
 * Creates a new filled Buffer instance.
 * alloc(size[, fill[, encoding]])
 **/
Buffer.alloc = function (size, fill, encoding) {
  return alloc(size, fill, encoding)
}

function allocUnsafe (size) {
  assertSize(size)
  return createBuffer(size < 0 ? 0 : checked(size) | 0)
}

/**
 * Equivalent to Buffer(num), by default creates a non-zero-filled Buffer instance.
 * */
Buffer.allocUnsafe = function (size) {
  return allocUnsafe(size)
}
/**
 * Equivalent to SlowBuffer(num), by default creates a non-zero-filled Buffer instance.
 */
Buffer.allocUnsafeSlow = function (size) {
  return allocUnsafe(size)
}

function fromString (string, encoding) {
  if (typeof encoding !== 'string' || encoding === '') {
    encoding = 'utf8'
  }

  if (!Buffer.isEncoding(encoding)) {
    throw new TypeError('Unknown encoding: ' + encoding)
  }

  var length = byteLength(string, encoding) | 0
  var buf = createBuffer(length)

  var actual = buf.write(string, encoding)

  if (actual !== length) {
    // Writing a hex string, for example, that contains invalid characters will
    // cause everything after the first invalid character to be ignored. (e.g.
    // 'abxxcd' will be treated as 'ab')
    buf = buf.slice(0, actual)
  }

  return buf
}

function fromArrayLike (array) {
  var length = array.length < 0 ? 0 : checked(array.length) | 0
  var buf = createBuffer(length)
  for (var i = 0; i < length; i += 1) {
    buf[i] = array[i] & 255
  }
  return buf
}

function fromArrayBuffer (array, byteOffset, length) {
  if (byteOffset < 0 || array.byteLength < byteOffset) {
    throw new RangeError('"offset" is outside of buffer bounds')
  }

  if (array.byteLength < byteOffset + (length || 0)) {
    throw new RangeError('"length" is outside of buffer bounds')
  }

  var buf
  if (byteOffset === undefined && length === undefined) {
    buf = new Uint8Array(array)
  } else if (length === undefined) {
    buf = new Uint8Array(array, byteOffset)
  } else {
    buf = new Uint8Array(array, byteOffset, length)
  }

  // Return an augmented `Uint8Array` instance
  buf.__proto__ = Buffer.prototype
  return buf
}

function fromObject (obj) {
  if (Buffer.isBuffer(obj)) {
    var len = checked(obj.length) | 0
    var buf = createBuffer(len)

    if (buf.length === 0) {
      return buf
    }

    obj.copy(buf, 0, 0, len)
    return buf
  }

  if (obj) {
    if (ArrayBuffer.isView(obj) || 'length' in obj) {
      if (typeof obj.length !== 'number' || numberIsNaN(obj.length)) {
        return createBuffer(0)
      }
      return fromArrayLike(obj)
    }

    if (obj.type === 'Buffer' && Array.isArray(obj.data)) {
      return fromArrayLike(obj.data)
    }
  }

  throw new TypeError('The first argument must be one of type string, Buffer, ArrayBuffer, Array, or Array-like Object.')
}

function checked (length) {
  // Note: cannot use `length < K_MAX_LENGTH` here because that fails when
  // length is NaN (which is otherwise coerced to zero.)
  if (length >= K_MAX_LENGTH) {
    throw new RangeError('Attempt to allocate Buffer larger than maximum ' +
                         'size: 0x' + K_MAX_LENGTH.toString(16) + ' bytes')
  }
  return length | 0
}

function SlowBuffer (length) {
  if (+length != length) { // eslint-disable-line eqeqeq
    length = 0
  }
  return Buffer.alloc(+length)
}

Buffer.isBuffer = function isBuffer (b) {
  return b != null && b._isBuffer === true
}

Buffer.compare = function compare (a, b) {
  if (!Buffer.isBuffer(a) || !Buffer.isBuffer(b)) {
    throw new TypeError('Arguments must be Buffers')
  }

  if (a === b) return 0

  var x = a.length
  var y = b.length

  for (var i = 0, len = Math.min(x, y); i < len; ++i) {
    if (a[i] !== b[i]) {
      x = a[i]
      y = b[i]
      break
    }
  }

  if (x < y) return -1
  if (y < x) return 1
  return 0
}

Buffer.isEncoding = function isEncoding (encoding) {
  switch (String(encoding).toLowerCase()) {
    case 'hex':
    case 'utf8':
    case 'utf-8':
    case 'ascii':
    case 'latin1':
    case 'binary':
    case 'base64':
    case 'ucs2':
    case 'ucs-2':
    case 'utf16le':
    case 'utf-16le':
      return true
    default:
      return false
  }
}

Buffer.concat = function concat (list, length) {
  if (!Array.isArray(list)) {
    throw new TypeError('"list" argument must be an Array of Buffers')
  }

  if (list.length === 0) {
    return Buffer.alloc(0)
  }

  var i
  if (length === undefined) {
    length = 0
    for (i = 0; i < list.length; ++i) {
      length += list[i].length
    }
  }

  var buffer = Buffer.allocUnsafe(length)
  var pos = 0
  for (i = 0; i < list.length; ++i) {
    var buf = list[i]
    if (ArrayBuffer.isView(buf)) {
      buf = Buffer.from(buf)
    }
    if (!Buffer.isBuffer(buf)) {
      throw new TypeError('"list" argument must be an Array of Buffers')
    }
    buf.copy(buffer, pos)
    pos += buf.length
  }
  return buffer
}

function byteLength (string, encoding) {
  if (Buffer.isBuffer(string)) {
    return string.length
  }
  if (ArrayBuffer.isView(string) || isArrayBuffer(string)) {
    return string.byteLength
  }
  if (typeof string !== 'string') {
    string = '' + string
  }

  var len = string.length
  if (len === 0) return 0

  // Use a for loop to avoid recursion
  var loweredCase = false
  for (;;) {
    switch (encoding) {
      case 'ascii':
      case 'latin1':
      case 'binary':
        return len
      case 'utf8':
      case 'utf-8':
      case undefined:
        return utf8ToBytes(string).length
      case 'ucs2':
      case 'ucs-2':
      case 'utf16le':
      case 'utf-16le':
        return len * 2
      case 'hex':
        return len >>> 1
      case 'base64':
        return base64ToBytes(string).length
      default:
        if (loweredCase) return utf8ToBytes(string).length // assume utf8
        encoding = ('' + encoding).toLowerCase()
        loweredCase = true
    }
  }
}
Buffer.byteLength = byteLength

function slowToString (encoding, start, end) {
  var loweredCase = false

  // No need to verify that "this.length <= MAX_UINT32" since it's a read-only
  // property of a typed array.

  // This behaves neither like String nor Uint8Array in that we set start/end
  // to their upper/lower bounds if the value passed is out of range.
  // undefined is handled specially as per ECMA-262 6th Edition,
  // Section 13.3.3.7 Runtime Semantics: KeyedBindingInitialization.
  if (start === undefined || start < 0) {
    start = 0
  }
  // Return early if start > this.length. Done here to prevent potential uint32
  // coercion fail below.
  if (start > this.length) {
    return ''
  }

  if (end === undefined || end > this.length) {
    end = this.length
  }

  if (end <= 0) {
    return ''
  }

  // Force coersion to uint32. This will also coerce falsey/NaN values to 0.
  end >>>= 0
  start >>>= 0

  if (end <= start) {
    return ''
  }

  if (!encoding) encoding = 'utf8'

  while (true) {
    switch (encoding) {
      case 'hex':
        return hexSlice(this, start, end)

      case 'utf8':
      case 'utf-8':
        return utf8Slice(this, start, end)

      case 'ascii':
        return asciiSlice(this, start, end)

      case 'latin1':
      case 'binary':
        return latin1Slice(this, start, end)

      case 'base64':
        return base64Slice(this, start, end)

      case 'ucs2':
      case 'ucs-2':
      case 'utf16le':
      case 'utf-16le':
        return utf16leSlice(this, start, end)

      default:
        if (loweredCase) throw new TypeError('Unknown encoding: ' + encoding)
        encoding = (encoding + '').toLowerCase()
        loweredCase = true
    }
  }
}

// This property is used by `Buffer.isBuffer` (and the `is-buffer` npm package)
// to detect a Buffer instance. It's not possible to use `instanceof Buffer`
// reliably in a browserify context because there could be multiple different
// copies of the 'buffer' package in use. This method works even for Buffer
// instances that were created from another copy of the `buffer` package.
// See: https://github.com/feross/buffer/issues/154
Buffer.prototype._isBuffer = true

function swap (b, n, m) {
  var i = b[n]
  b[n] = b[m]
  b[m] = i
}

Buffer.prototype.swap16 = function swap16 () {
  var len = this.length
  if (len % 2 !== 0) {
    throw new RangeError('Buffer size must be a multiple of 16-bits')
  }
  for (var i = 0; i < len; i += 2) {
    swap(this, i, i + 1)
  }
  return this
}

Buffer.prototype.swap32 = function swap32 () {
  var len = this.length
  if (len % 4 !== 0) {
    throw new RangeError('Buffer size must be a multiple of 32-bits')
  }
  for (var i = 0; i < len; i += 4) {
    swap(this, i, i + 3)
    swap(this, i + 1, i + 2)
  }
  return this
}

Buffer.prototype.swap64 = function swap64 () {
  var len = this.length
  if (len % 8 !== 0) {
    throw new RangeError('Buffer size must be a multiple of 64-bits')
  }
  for (var i = 0; i < len; i += 8) {
    swap(this, i, i + 7)
    swap(this, i + 1, i + 6)
    swap(this, i + 2, i + 5)
    swap(this, i + 3, i + 4)
  }
  return this
}

Buffer.prototype.toString = function toString () {
  var length = this.length
  if (length === 0) return ''
  if (arguments.length === 0) return utf8Slice(this, 0, length)
  return slowToString.apply(this, arguments)
}

Buffer.prototype.toLocaleString = Buffer.prototype.toString

Buffer.prototype.equals = function equals (b) {
  if (!Buffer.isBuffer(b)) throw new TypeError('Argument must be a Buffer')
  if (this === b) return true
  return Buffer.compare(this, b) === 0
}

Buffer.prototype.inspect = function inspect () {
  var str = ''
  var max = exports.INSPECT_MAX_BYTES
  if (this.length > 0) {
    str = this.toString('hex', 0, max).match(/.{2}/g).join(' ')
    if (this.length > max) str += ' ... '
  }
  return '<Buffer ' + str + '>'
}

Buffer.prototype.compare = function compare (target, start, end, thisStart, thisEnd) {
  if (!Buffer.isBuffer(target)) {
    throw new TypeError('Argument must be a Buffer')
  }

  if (start === undefined) {
    start = 0
  }
  if (end === undefined) {
    end = target ? target.length : 0
  }
  if (thisStart === undefined) {
    thisStart = 0
  }
  if (thisEnd === undefined) {
    thisEnd = this.length
  }

  if (start < 0 || end > target.length || thisStart < 0 || thisEnd > this.length) {
    throw new RangeError('out of range index')
  }

  if (thisStart >= thisEnd && start >= end) {
    return 0
  }
  if (thisStart >= thisEnd) {
    return -1
  }
  if (start >= end) {
    return 1
  }

  start >>>= 0
  end >>>= 0
  thisStart >>>= 0
  thisEnd >>>= 0

  if (this === target) return 0

  var x = thisEnd - thisStart
  var y = end - start
  var len = Math.min(x, y)

  var thisCopy = this.slice(thisStart, thisEnd)
  var targetCopy = target.slice(start, end)

  for (var i = 0; i < len; ++i) {
    if (thisCopy[i] !== targetCopy[i]) {
      x = thisCopy[i]
      y = targetCopy[i]
      break
    }
  }

  if (x < y) return -1
  if (y < x) return 1
  return 0
}

// Finds either the first index of `val` in `buffer` at offset >= `byteOffset`,
// OR the last index of `val` in `buffer` at offset <= `byteOffset`.
//
// Arguments:
// - buffer - a Buffer to search
// - val - a string, Buffer, or number
// - byteOffset - an index into `buffer`; will be clamped to an int32
// - encoding - an optional encoding, relevant is val is a string
// - dir - true for indexOf, false for lastIndexOf
function bidirectionalIndexOf (buffer, val, byteOffset, encoding, dir) {
  // Empty buffer means no match
  if (buffer.length === 0) return -1

  // Normalize byteOffset
  if (typeof byteOffset === 'string') {
    encoding = byteOffset
    byteOffset = 0
  } else if (byteOffset > 0x7fffffff) {
    byteOffset = 0x7fffffff
  } else if (byteOffset < -0x80000000) {
    byteOffset = -0x80000000
  }
  byteOffset = +byteOffset  // Coerce to Number.
  if (numberIsNaN(byteOffset)) {
    // byteOffset: it it's undefined, null, NaN, "foo", etc, search whole buffer
    byteOffset = dir ? 0 : (buffer.length - 1)
  }

  // Normalize byteOffset: negative offsets start from the end of the buffer
  if (byteOffset < 0) byteOffset = buffer.length + byteOffset
  if (byteOffset >= buffer.length) {
    if (dir) return -1
    else byteOffset = buffer.length - 1
  } else if (byteOffset < 0) {
    if (dir) byteOffset = 0
    else return -1
  }

  // Normalize val
  if (typeof val === 'string') {
    val = Buffer.from(val, encoding)
  }

  // Finally, search either indexOf (if dir is true) or lastIndexOf
  if (Buffer.isBuffer(val)) {
    // Special case: looking for empty string/buffer always fails
    if (val.length === 0) {
      return -1
    }
    return arrayIndexOf(buffer, val, byteOffset, encoding, dir)
  } else if (typeof val === 'number') {
    val = val & 0xFF // Search for a byte value [0-255]
    if (typeof Uint8Array.prototype.indexOf === 'function') {
      if (dir) {
        return Uint8Array.prototype.indexOf.call(buffer, val, byteOffset)
      } else {
        return Uint8Array.prototype.lastIndexOf.call(buffer, val, byteOffset)
      }
    }
    return arrayIndexOf(buffer, [ val ], byteOffset, encoding, dir)
  }

  throw new TypeError('val must be string, number or Buffer')
}

function arrayIndexOf (arr, val, byteOffset, encoding, dir) {
  var indexSize = 1
  var arrLength = arr.length
  var valLength = val.length

  if (encoding !== undefined) {
    encoding = String(encoding).toLowerCase()
    if (encoding === 'ucs2' || encoding === 'ucs-2' ||
        encoding === 'utf16le' || encoding === 'utf-16le') {
      if (arr.length < 2 || val.length < 2) {
        return -1
      }
      indexSize = 2
      arrLength /= 2
      valLength /= 2
      byteOffset /= 2
    }
  }

  function read (buf, i) {
    if (indexSize === 1) {
      return buf[i]
    } else {
      return buf.readUInt16BE(i * indexSize)
    }
  }

  var i
  if (dir) {
    var foundIndex = -1
    for (i = byteOffset; i < arrLength; i++) {
      if (read(arr, i) === read(val, foundIndex === -1 ? 0 : i - foundIndex)) {
        if (foundIndex === -1) foundIndex = i
        if (i - foundIndex + 1 === valLength) return foundIndex * indexSize
      } else {
        if (foundIndex !== -1) i -= i - foundIndex
        foundIndex = -1
      }
    }
  } else {
    if (byteOffset + valLength > arrLength) byteOffset = arrLength - valLength
    for (i = byteOffset; i >= 0; i--) {
      var found = true
      for (var j = 0; j < valLength; j++) {
        if (read(arr, i + j) !== read(val, j)) {
          found = false
          break
        }
      }
      if (found) return i
    }
  }

  return -1
}

Buffer.prototype.includes = function includes (val, byteOffset, encoding) {
  return this.indexOf(val, byteOffset, encoding) !== -1
}

Buffer.prototype.indexOf = function indexOf (val, byteOffset, encoding) {
  return bidirectionalIndexOf(this, val, byteOffset, encoding, true)
}

Buffer.prototype.lastIndexOf = function lastIndexOf (val, byteOffset, encoding) {
  return bidirectionalIndexOf(this, val, byteOffset, encoding, false)
}

function hexWrite (buf, string, offset, length) {
  offset = Number(offset) || 0
  var remaining = buf.length - offset
  if (!length) {
    length = remaining
  } else {
    length = Number(length)
    if (length > remaining) {
      length = remaining
    }
  }

  var strLen = string.length

  if (length > strLen / 2) {
    length = strLen / 2
  }
  for (var i = 0; i < length; ++i) {
    var parsed = parseInt(string.substr(i * 2, 2), 16)
    if (numberIsNaN(parsed)) return i
    buf[offset + i] = parsed
  }
  return i
}

function utf8Write (buf, string, offset, length) {
  return blitBuffer(utf8ToBytes(string, buf.length - offset), buf, offset, length)
}

function asciiWrite (buf, string, offset, length) {
  return blitBuffer(asciiToBytes(string), buf, offset, length)
}

function latin1Write (buf, string, offset, length) {
  return asciiWrite(buf, string, offset, length)
}

function base64Write (buf, string, offset, length) {
  return blitBuffer(base64ToBytes(string), buf, offset, length)
}

function ucs2Write (buf, string, offset, length) {
  return blitBuffer(utf16leToBytes(string, buf.length - offset), buf, offset, length)
}

Buffer.prototype.write = function write (string, offset, length, encoding) {
  // Buffer#write(string)
  if (offset === undefined) {
    encoding = 'utf8'
    length = this.length
    offset = 0
  // Buffer#write(string, encoding)
  } else if (length === undefined && typeof offset === 'string') {
    encoding = offset
    length = this.length
    offset = 0
  // Buffer#write(string, offset[, length][, encoding])
  } else if (isFinite(offset)) {
    offset = offset >>> 0
    if (isFinite(length)) {
      length = length >>> 0
      if (encoding === undefined) encoding = 'utf8'
    } else {
      encoding = length
      length = undefined
    }
  } else {
    throw new Error(
      'Buffer.write(string, encoding, offset[, length]) is no longer supported'
    )
  }

  var remaining = this.length - offset
  if (length === undefined || length > remaining) length = remaining

  if ((string.length > 0 && (length < 0 || offset < 0)) || offset > this.length) {
    throw new RangeError('Attempt to write outside buffer bounds')
  }

  if (!encoding) encoding = 'utf8'

  var loweredCase = false
  for (;;) {
    switch (encoding) {
      case 'hex':
        return hexWrite(this, string, offset, length)

      case 'utf8':
      case 'utf-8':
        return utf8Write(this, string, offset, length)

      case 'ascii':
        return asciiWrite(this, string, offset, length)

      case 'latin1':
      case 'binary':
        return latin1Write(this, string, offset, length)

      case 'base64':
        // Warning: maxLength not taken into account in base64Write
        return base64Write(this, string, offset, length)

      case 'ucs2':
      case 'ucs-2':
      case 'utf16le':
      case 'utf-16le':
        return ucs2Write(this, string, offset, length)

      default:
        if (loweredCase) throw new TypeError('Unknown encoding: ' + encoding)
        encoding = ('' + encoding).toLowerCase()
        loweredCase = true
    }
  }
}

Buffer.prototype.toJSON = function toJSON () {
  return {
    type: 'Buffer',
    data: Array.prototype.slice.call(this._arr || this, 0)
  }
}

function base64Slice (buf, start, end) {
  if (start === 0 && end === buf.length) {
    return base64.fromByteArray(buf)
  } else {
    return base64.fromByteArray(buf.slice(start, end))
  }
}

function utf8Slice (buf, start, end) {
  end = Math.min(buf.length, end)
  var res = []

  var i = start
  while (i < end) {
    var firstByte = buf[i]
    var codePoint = null
    var bytesPerSequence = (firstByte > 0xEF) ? 4
      : (firstByte > 0xDF) ? 3
      : (firstByte > 0xBF) ? 2
      : 1

    if (i + bytesPerSequence <= end) {
      var secondByte, thirdByte, fourthByte, tempCodePoint

      switch (bytesPerSequence) {
        case 1:
          if (firstByte < 0x80) {
            codePoint = firstByte
          }
          break
        case 2:
          secondByte = buf[i + 1]
          if ((secondByte & 0xC0) === 0x80) {
            tempCodePoint = (firstByte & 0x1F) << 0x6 | (secondByte & 0x3F)
            if (tempCodePoint > 0x7F) {
              codePoint = tempCodePoint
            }
          }
          break
        case 3:
          secondByte = buf[i + 1]
          thirdByte = buf[i + 2]
          if ((secondByte & 0xC0) === 0x80 && (thirdByte & 0xC0) === 0x80) {
            tempCodePoint = (firstByte & 0xF) << 0xC | (secondByte & 0x3F) << 0x6 | (thirdByte & 0x3F)
            if (tempCodePoint > 0x7FF && (tempCodePoint < 0xD800 || tempCodePoint > 0xDFFF)) {
              codePoint = tempCodePoint
            }
          }
          break
        case 4:
          secondByte = buf[i + 1]
          thirdByte = buf[i + 2]
          fourthByte = buf[i + 3]
          if ((secondByte & 0xC0) === 0x80 && (thirdByte & 0xC0) === 0x80 && (fourthByte & 0xC0) === 0x80) {
            tempCodePoint = (firstByte & 0xF) << 0x12 | (secondByte & 0x3F) << 0xC | (thirdByte & 0x3F) << 0x6 | (fourthByte & 0x3F)
            if (tempCodePoint > 0xFFFF && tempCodePoint < 0x110000) {
              codePoint = tempCodePoint
            }
          }
      }
    }

    if (codePoint === null) {
      // we did not generate a valid codePoint so insert a
      // replacement char (U+FFFD) and advance only 1 byte
      codePoint = 0xFFFD
      bytesPerSequence = 1
    } else if (codePoint > 0xFFFF) {
      // encode to utf16 (surrogate pair dance)
      codePoint -= 0x10000
      res.push(codePoint >>> 10 & 0x3FF | 0xD800)
      codePoint = 0xDC00 | codePoint & 0x3FF
    }

    res.push(codePoint)
    i += bytesPerSequence
  }

  return decodeCodePointsArray(res)
}

// Based on http://stackoverflow.com/a/22747272/680742, the browser with
// the lowest limit is Chrome, with 0x10000 args.
// We go 1 magnitude less, for safety
var MAX_ARGUMENTS_LENGTH = 0x1000

function decodeCodePointsArray (codePoints) {
  var len = codePoints.length
  if (len <= MAX_ARGUMENTS_LENGTH) {
    return String.fromCharCode.apply(String, codePoints) // avoid extra slice()
  }

  // Decode in chunks to avoid "call stack size exceeded".
  var res = ''
  var i = 0
  while (i < len) {
    res += String.fromCharCode.apply(
      String,
      codePoints.slice(i, i += MAX_ARGUMENTS_LENGTH)
    )
  }
  return res
}

function asciiSlice (buf, start, end) {
  var ret = ''
  end = Math.min(buf.length, end)

  for (var i = start; i < end; ++i) {
    ret += String.fromCharCode(buf[i] & 0x7F)
  }
  return ret
}

function latin1Slice (buf, start, end) {
  var ret = ''
  end = Math.min(buf.length, end)

  for (var i = start; i < end; ++i) {
    ret += String.fromCharCode(buf[i])
  }
  return ret
}

function hexSlice (buf, start, end) {
  var len = buf.length

  if (!start || start < 0) start = 0
  if (!end || end < 0 || end > len) end = len

  var out = ''
  for (var i = start; i < end; ++i) {
    out += toHex(buf[i])
  }
  return out
}

function utf16leSlice (buf, start, end) {
  var bytes = buf.slice(start, end)
  var res = ''
  for (var i = 0; i < bytes.length; i += 2) {
    res += String.fromCharCode(bytes[i] + (bytes[i + 1] * 256))
  }
  return res
}

Buffer.prototype.slice = function slice (start, end) {
  var len = this.length
  start = ~~start
  end = end === undefined ? len : ~~end

  if (start < 0) {
    start += len
    if (start < 0) start = 0
  } else if (start > len) {
    start = len
  }

  if (end < 0) {
    end += len
    if (end < 0) end = 0
  } else if (end > len) {
    end = len
  }

  if (end < start) end = start

  var newBuf = this.subarray(start, end)
  // Return an augmented `Uint8Array` instance
  newBuf.__proto__ = Buffer.prototype
  return newBuf
}

/*
 * Need to make sure that buffer isn't trying to write out of bounds.
 */
function checkOffset (offset, ext, length) {
  if ((offset % 1) !== 0 || offset < 0) throw new RangeError('offset is not uint')
  if (offset + ext > length) throw new RangeError('Trying to access beyond buffer length')
}

Buffer.prototype.readUIntLE = function readUIntLE (offset, byteLength, noAssert) {
  offset = offset >>> 0
  byteLength = byteLength >>> 0
  if (!noAssert) checkOffset(offset, byteLength, this.length)

  var val = this[offset]
  var mul = 1
  var i = 0
  while (++i < byteLength && (mul *= 0x100)) {
    val += this[offset + i] * mul
  }

  return val
}

Buffer.prototype.readUIntBE = function readUIntBE (offset, byteLength, noAssert) {
  offset = offset >>> 0
  byteLength = byteLength >>> 0
  if (!noAssert) {
    checkOffset(offset, byteLength, this.length)
  }

  var val = this[offset + --byteLength]
  var mul = 1
  while (byteLength > 0 && (mul *= 0x100)) {
    val += this[offset + --byteLength] * mul
  }

  return val
}

Buffer.prototype.readUInt8 = function readUInt8 (offset, noAssert) {
  offset = offset >>> 0
  if (!noAssert) checkOffset(offset, 1, this.length)
  return this[offset]
}

Buffer.prototype.readUInt16LE = function readUInt16LE (offset, noAssert) {
  offset = offset >>> 0
  if (!noAssert) checkOffset(offset, 2, this.length)
  return this[offset] | (this[offset + 1] << 8)
}

Buffer.prototype.readUInt16BE = function readUInt16BE (offset, noAssert) {
  offset = offset >>> 0
  if (!noAssert) checkOffset(offset, 2, this.length)
  return (this[offset] << 8) | this[offset + 1]
}

Buffer.prototype.readUInt32LE = function readUInt32LE (offset, noAssert) {
  offset = offset >>> 0
  if (!noAssert) checkOffset(offset, 4, this.length)

  return ((this[offset]) |
      (this[offset + 1] << 8) |
      (this[offset + 2] << 16)) +
      (this[offset + 3] * 0x1000000)
}

Buffer.prototype.readUInt32BE = function readUInt32BE (offset, noAssert) {
  offset = offset >>> 0
  if (!noAssert) checkOffset(offset, 4, this.length)

  return (this[offset] * 0x1000000) +
    ((this[offset + 1] << 16) |
    (this[offset + 2] << 8) |
    this[offset + 3])
}

Buffer.prototype.readIntLE = function readIntLE (offset, byteLength, noAssert) {
  offset = offset >>> 0
  byteLength = byteLength >>> 0
  if (!noAssert) checkOffset(offset, byteLength, this.length)

  var val = this[offset]
  var mul = 1
  var i = 0
  while (++i < byteLength && (mul *= 0x100)) {
    val += this[offset + i] * mul
  }
  mul *= 0x80

  if (val >= mul) val -= Math.pow(2, 8 * byteLength)

  return val
}

Buffer.prototype.readIntBE = function readIntBE (offset, byteLength, noAssert) {
  offset = offset >>> 0
  byteLength = byteLength >>> 0
  if (!noAssert) checkOffset(offset, byteLength, this.length)

  var i = byteLength
  var mul = 1
  var val = this[offset + --i]
  while (i > 0 && (mul *= 0x100)) {
    val += this[offset + --i] * mul
  }
  mul *= 0x80

  if (val >= mul) val -= Math.pow(2, 8 * byteLength)

  return val
}

Buffer.prototype.readInt8 = function readInt8 (offset, noAssert) {
  offset = offset >>> 0
  if (!noAssert) checkOffset(offset, 1, this.length)
  if (!(this[offset] & 0x80)) return (this[offset])
  return ((0xff - this[offset] + 1) * -1)
}

Buffer.prototype.readInt16LE = function readInt16LE (offset, noAssert) {
  offset = offset >>> 0
  if (!noAssert) checkOffset(offset, 2, this.length)
  var val = this[offset] | (this[offset + 1] << 8)
  return (val & 0x8000) ? val | 0xFFFF0000 : val
}

Buffer.prototype.readInt16BE = function readInt16BE (offset, noAssert) {
  offset = offset >>> 0
  if (!noAssert) checkOffset(offset, 2, this.length)
  var val = this[offset + 1] | (this[offset] << 8)
  return (val & 0x8000) ? val | 0xFFFF0000 : val
}

Buffer.prototype.readInt32LE = function readInt32LE (offset, noAssert) {
  offset = offset >>> 0
  if (!noAssert) checkOffset(offset, 4, this.length)

  return (this[offset]) |
    (this[offset + 1] << 8) |
    (this[offset + 2] << 16) |
    (this[offset + 3] << 24)
}

Buffer.prototype.readInt32BE = function readInt32BE (offset, noAssert) {
  offset = offset >>> 0
  if (!noAssert) checkOffset(offset, 4, this.length)

  return (this[offset] << 24) |
    (this[offset + 1] << 16) |
    (this[offset + 2] << 8) |
    (this[offset + 3])
}

Buffer.prototype.readFloatLE = function readFloatLE (offset, noAssert) {
  offset = offset >>> 0
  if (!noAssert) checkOffset(offset, 4, this.length)
  return ieee754.read(this, offset, true, 23, 4)
}

Buffer.prototype.readFloatBE = function readFloatBE (offset, noAssert) {
  offset = offset >>> 0
  if (!noAssert) checkOffset(offset, 4, this.length)
  return ieee754.read(this, offset, false, 23, 4)
}

Buffer.prototype.readDoubleLE = function readDoubleLE (offset, noAssert) {
  offset = offset >>> 0
  if (!noAssert) checkOffset(offset, 8, this.length)
  return ieee754.read(this, offset, true, 52, 8)
}

Buffer.prototype.readDoubleBE = function readDoubleBE (offset, noAssert) {
  offset = offset >>> 0
  if (!noAssert) checkOffset(offset, 8, this.length)
  return ieee754.read(this, offset, false, 52, 8)
}

function checkInt (buf, value, offset, ext, max, min) {
  if (!Buffer.isBuffer(buf)) throw new TypeError('"buffer" argument must be a Buffer instance')
  if (value > max || value < min) throw new RangeError('"value" argument is out of bounds')
  if (offset + ext > buf.length) throw new RangeError('Index out of range')
}

Buffer.prototype.writeUIntLE = function writeUIntLE (value, offset, byteLength, noAssert) {
  value = +value
  offset = offset >>> 0
  byteLength = byteLength >>> 0
  if (!noAssert) {
    var maxBytes = Math.pow(2, 8 * byteLength) - 1
    checkInt(this, value, offset, byteLength, maxBytes, 0)
  }

  var mul = 1
  var i = 0
  this[offset] = value & 0xFF
  while (++i < byteLength && (mul *= 0x100)) {
    this[offset + i] = (value / mul) & 0xFF
  }

  return offset + byteLength
}

Buffer.prototype.writeUIntBE = function writeUIntBE (value, offset, byteLength, noAssert) {
  value = +value
  offset = offset >>> 0
  byteLength = byteLength >>> 0
  if (!noAssert) {
    var maxBytes = Math.pow(2, 8 * byteLength) - 1
    checkInt(this, value, offset, byteLength, maxBytes, 0)
  }

  var i = byteLength - 1
  var mul = 1
  this[offset + i] = value & 0xFF
  while (--i >= 0 && (mul *= 0x100)) {
    this[offset + i] = (value / mul) & 0xFF
  }

  return offset + byteLength
}

Buffer.prototype.writeUInt8 = function writeUInt8 (value, offset, noAssert) {
  value = +value
  offset = offset >>> 0
  if (!noAssert) checkInt(this, value, offset, 1, 0xff, 0)
  this[offset] = (value & 0xff)
  return offset + 1
}

Buffer.prototype.writeUInt16LE = function writeUInt16LE (value, offset, noAssert) {
  value = +value
  offset = offset >>> 0
  if (!noAssert) checkInt(this, value, offset, 2, 0xffff, 0)
  this[offset] = (value & 0xff)
  this[offset + 1] = (value >>> 8)
  return offset + 2
}

Buffer.prototype.writeUInt16BE = function writeUInt16BE (value, offset, noAssert) {
  value = +value
  offset = offset >>> 0
  if (!noAssert) checkInt(this, value, offset, 2, 0xffff, 0)
  this[offset] = (value >>> 8)
  this[offset + 1] = (value & 0xff)
  return offset + 2
}

Buffer.prototype.writeUInt32LE = function writeUInt32LE (value, offset, noAssert) {
  value = +value
  offset = offset >>> 0
  if (!noAssert) checkInt(this, value, offset, 4, 0xffffffff, 0)
  this[offset + 3] = (value >>> 24)
  this[offset + 2] = (value >>> 16)
  this[offset + 1] = (value >>> 8)
  this[offset] = (value & 0xff)
  return offset + 4
}

Buffer.prototype.writeUInt32BE = function writeUInt32BE (value, offset, noAssert) {
  value = +value
  offset = offset >>> 0
  if (!noAssert) checkInt(this, value, offset, 4, 0xffffffff, 0)
  this[offset] = (value >>> 24)
  this[offset + 1] = (value >>> 16)
  this[offset + 2] = (value >>> 8)
  this[offset + 3] = (value & 0xff)
  return offset + 4
}

Buffer.prototype.writeIntLE = function writeIntLE (value, offset, byteLength, noAssert) {
  value = +value
  offset = offset >>> 0
  if (!noAssert) {
    var limit = Math.pow(2, (8 * byteLength) - 1)

    checkInt(this, value, offset, byteLength, limit - 1, -limit)
  }

  var i = 0
  var mul = 1
  var sub = 0
  this[offset] = value & 0xFF
  while (++i < byteLength && (mul *= 0x100)) {
    if (value < 0 && sub === 0 && this[offset + i - 1] !== 0) {
      sub = 1
    }
    this[offset + i] = ((value / mul) >> 0) - sub & 0xFF
  }

  return offset + byteLength
}

Buffer.prototype.writeIntBE = function writeIntBE (value, offset, byteLength, noAssert) {
  value = +value
  offset = offset >>> 0
  if (!noAssert) {
    var limit = Math.pow(2, (8 * byteLength) - 1)

    checkInt(this, value, offset, byteLength, limit - 1, -limit)
  }

  var i = byteLength - 1
  var mul = 1
  var sub = 0
  this[offset + i] = value & 0xFF
  while (--i >= 0 && (mul *= 0x100)) {
    if (value < 0 && sub === 0 && this[offset + i + 1] !== 0) {
      sub = 1
    }
    this[offset + i] = ((value / mul) >> 0) - sub & 0xFF
  }

  return offset + byteLength
}

Buffer.prototype.writeInt8 = function writeInt8 (value, offset, noAssert) {
  value = +value
  offset = offset >>> 0
  if (!noAssert) checkInt(this, value, offset, 1, 0x7f, -0x80)
  if (value < 0) value = 0xff + value + 1
  this[offset] = (value & 0xff)
  return offset + 1
}

Buffer.prototype.writeInt16LE = function writeInt16LE (value, offset, noAssert) {
  value = +value
  offset = offset >>> 0
  if (!noAssert) checkInt(this, value, offset, 2, 0x7fff, -0x8000)
  this[offset] = (value & 0xff)
  this[offset + 1] = (value >>> 8)
  return offset + 2
}

Buffer.prototype.writeInt16BE = function writeInt16BE (value, offset, noAssert) {
  value = +value
  offset = offset >>> 0
  if (!noAssert) checkInt(this, value, offset, 2, 0x7fff, -0x8000)
  this[offset] = (value >>> 8)
  this[offset + 1] = (value & 0xff)
  return offset + 2
}

Buffer.prototype.writeInt32LE = function writeInt32LE (value, offset, noAssert) {
  value = +value
  offset = offset >>> 0
  if (!noAssert) checkInt(this, value, offset, 4, 0x7fffffff, -0x80000000)
  this[offset] = (value & 0xff)
  this[offset + 1] = (value >>> 8)
  this[offset + 2] = (value >>> 16)
  this[offset + 3] = (value >>> 24)
  return offset + 4
}

Buffer.prototype.writeInt32BE = function writeInt32BE (value, offset, noAssert) {
  value = +value
  offset = offset >>> 0
  if (!noAssert) checkInt(this, value, offset, 4, 0x7fffffff, -0x80000000)
  if (value < 0) value = 0xffffffff + value + 1
  this[offset] = (value >>> 24)
  this[offset + 1] = (value >>> 16)
  this[offset + 2] = (value >>> 8)
  this[offset + 3] = (value & 0xff)
  return offset + 4
}

function checkIEEE754 (buf, value, offset, ext, max, min) {
  if (offset + ext > buf.length) throw new RangeError('Index out of range')
  if (offset < 0) throw new RangeError('Index out of range')
}

function writeFloat (buf, value, offset, littleEndian, noAssert) {
  value = +value
  offset = offset >>> 0
  if (!noAssert) {
    checkIEEE754(buf, value, offset, 4, 3.4028234663852886e+38, -3.4028234663852886e+38)
  }
  ieee754.write(buf, value, offset, littleEndian, 23, 4)
  return offset + 4
}

Buffer.prototype.writeFloatLE = function writeFloatLE (value, offset, noAssert) {
  return writeFloat(this, value, offset, true, noAssert)
}

Buffer.prototype.writeFloatBE = function writeFloatBE (value, offset, noAssert) {
  return writeFloat(this, value, offset, false, noAssert)
}

function writeDouble (buf, value, offset, littleEndian, noAssert) {
  value = +value
  offset = offset >>> 0
  if (!noAssert) {
    checkIEEE754(buf, value, offset, 8, 1.7976931348623157E+308, -1.7976931348623157E+308)
  }
  ieee754.write(buf, value, offset, littleEndian, 52, 8)
  return offset + 8
}

Buffer.prototype.writeDoubleLE = function writeDoubleLE (value, offset, noAssert) {
  return writeDouble(this, value, offset, true, noAssert)
}

Buffer.prototype.writeDoubleBE = function writeDoubleBE (value, offset, noAssert) {
  return writeDouble(this, value, offset, false, noAssert)
}

// copy(targetBuffer, targetStart=0, sourceStart=0, sourceEnd=buffer.length)
Buffer.prototype.copy = function copy (target, targetStart, start, end) {
  if (!Buffer.isBuffer(target)) throw new TypeError('argument should be a Buffer')
  if (!start) start = 0
  if (!end && end !== 0) end = this.length
  if (targetStart >= target.length) targetStart = target.length
  if (!targetStart) targetStart = 0
  if (end > 0 && end < start) end = start

  // Copy 0 bytes; we're done
  if (end === start) return 0
  if (target.length === 0 || this.length === 0) return 0

  // Fatal error conditions
  if (targetStart < 0) {
    throw new RangeError('targetStart out of bounds')
  }
  if (start < 0 || start >= this.length) throw new RangeError('Index out of range')
  if (end < 0) throw new RangeError('sourceEnd out of bounds')

  // Are we oob?
  if (end > this.length) end = this.length
  if (target.length - targetStart < end - start) {
    end = target.length - targetStart + start
  }

  var len = end - start

  if (this === target && typeof Uint8Array.prototype.copyWithin === 'function') {
    // Use built-in when available, missing from IE11
    this.copyWithin(targetStart, start, end)
  } else if (this === target && start < targetStart && targetStart < end) {
    // descending copy from end
    for (var i = len - 1; i >= 0; --i) {
      target[i + targetStart] = this[i + start]
    }
  } else {
    Uint8Array.prototype.set.call(
      target,
      this.subarray(start, end),
      targetStart
    )
  }

  return len
}

// Usage:
//    buffer.fill(number[, offset[, end]])
//    buffer.fill(buffer[, offset[, end]])
//    buffer.fill(string[, offset[, end]][, encoding])
Buffer.prototype.fill = function fill (val, start, end, encoding) {
  // Handle string cases:
  if (typeof val === 'string') {
    if (typeof start === 'string') {
      encoding = start
      start = 0
      end = this.length
    } else if (typeof end === 'string') {
      encoding = end
      end = this.length
    }
    if (encoding !== undefined && typeof encoding !== 'string') {
      throw new TypeError('encoding must be a string')
    }
    if (typeof encoding === 'string' && !Buffer.isEncoding(encoding)) {
      throw new TypeError('Unknown encoding: ' + encoding)
    }
    if (val.length === 1) {
      var code = val.charCodeAt(0)
      if ((encoding === 'utf8' && code < 128) ||
          encoding === 'latin1') {
        // Fast path: If `val` fits into a single byte, use that numeric value.
        val = code
      }
    }
  } else if (typeof val === 'number') {
    val = val & 255
  }

  // Invalid ranges are not set to a default, so can range check early.
  if (start < 0 || this.length < start || this.length < end) {
    throw new RangeError('Out of range index')
  }

  if (end <= start) {
    return this
  }

  start = start >>> 0
  end = end === undefined ? this.length : end >>> 0

  if (!val) val = 0

  var i
  if (typeof val === 'number') {
    for (i = start; i < end; ++i) {
      this[i] = val
    }
  } else {
    var bytes = Buffer.isBuffer(val)
      ? val
      : new Buffer(val, encoding)
    var len = bytes.length
    if (len === 0) {
      throw new TypeError('The value "' + val +
        '" is invalid for argument "value"')
    }
    for (i = 0; i < end - start; ++i) {
      this[i + start] = bytes[i % len]
    }
  }

  return this
}

// HELPER FUNCTIONS
// ================

var INVALID_BASE64_RE = /[^+/0-9A-Za-z-_]/g

function base64clean (str) {
  // Node takes equal signs as end of the Base64 encoding
  str = str.split('=')[0]
  // Node strips out invalid characters like \n and \t from the string, base64-js does not
  str = str.trim().replace(INVALID_BASE64_RE, '')
  // Node converts strings with length < 2 to ''
  if (str.length < 2) return ''
  // Node allows for non-padded base64 strings (missing trailing ===), base64-js does not
  while (str.length % 4 !== 0) {
    str = str + '='
  }
  return str
}

function toHex (n) {
  if (n < 16) return '0' + n.toString(16)
  return n.toString(16)
}

function utf8ToBytes (string, units) {
  units = units || Infinity
  var codePoint
  var length = string.length
  var leadSurrogate = null
  var bytes = []

  for (var i = 0; i < length; ++i) {
    codePoint = string.charCodeAt(i)

    // is surrogate component
    if (codePoint > 0xD7FF && codePoint < 0xE000) {
      // last char was a lead
      if (!leadSurrogate) {
        // no lead yet
        if (codePoint > 0xDBFF) {
          // unexpected trail
          if ((units -= 3) > -1) bytes.push(0xEF, 0xBF, 0xBD)
          continue
        } else if (i + 1 === length) {
          // unpaired lead
          if ((units -= 3) > -1) bytes.push(0xEF, 0xBF, 0xBD)
          continue
        }

        // valid lead
        leadSurrogate = codePoint

        continue
      }

      // 2 leads in a row
      if (codePoint < 0xDC00) {
        if ((units -= 3) > -1) bytes.push(0xEF, 0xBF, 0xBD)
        leadSurrogate = codePoint
        continue
      }

      // valid surrogate pair
      codePoint = (leadSurrogate - 0xD800 << 10 | codePoint - 0xDC00) + 0x10000
    } else if (leadSurrogate) {
      // valid bmp char, but last char was a lead
      if ((units -= 3) > -1) bytes.push(0xEF, 0xBF, 0xBD)
    }

    leadSurrogate = null

    // encode utf8
    if (codePoint < 0x80) {
      if ((units -= 1) < 0) break
      bytes.push(codePoint)
    } else if (codePoint < 0x800) {
      if ((units -= 2) < 0) break
      bytes.push(
        codePoint >> 0x6 | 0xC0,
        codePoint & 0x3F | 0x80
      )
    } else if (codePoint < 0x10000) {
      if ((units -= 3) < 0) break
      bytes.push(
        codePoint >> 0xC | 0xE0,
        codePoint >> 0x6 & 0x3F | 0x80,
        codePoint & 0x3F | 0x80
      )
    } else if (codePoint < 0x110000) {
      if ((units -= 4) < 0) break
      bytes.push(
        codePoint >> 0x12 | 0xF0,
        codePoint >> 0xC & 0x3F | 0x80,
        codePoint >> 0x6 & 0x3F | 0x80,
        codePoint & 0x3F | 0x80
      )
    } else {
      throw new Error('Invalid code point')
    }
  }

  return bytes
}

function asciiToBytes (str) {
  var byteArray = []
  for (var i = 0; i < str.length; ++i) {
    // Node's code seems to be doing this and not & 0x7F..
    byteArray.push(str.charCodeAt(i) & 0xFF)
  }
  return byteArray
}

function utf16leToBytes (str, units) {
  var c, hi, lo
  var byteArray = []
  for (var i = 0; i < str.length; ++i) {
    if ((units -= 2) < 0) break

    c = str.charCodeAt(i)
    hi = c >> 8
    lo = c % 256
    byteArray.push(lo)
    byteArray.push(hi)
  }

  return byteArray
}

function base64ToBytes (str) {
  return base64.toByteArray(base64clean(str))
}

function blitBuffer (src, dst, offset, length) {
  for (var i = 0; i < length; ++i) {
    if ((i + offset >= dst.length) || (i >= src.length)) break
    dst[i + offset] = src[i]
  }
  return i
}

// ArrayBuffers from another context (i.e. an iframe) do not pass the `instanceof` check
// but they should be treated as valid. See: https://github.com/feross/buffer/issues/166
function isArrayBuffer (obj) {
  return obj instanceof ArrayBuffer ||
    (obj != null && obj.constructor != null && obj.constructor.name === 'ArrayBuffer' &&
      typeof obj.byteLength === 'number')
}

function numberIsNaN (obj) {
  return obj !== obj // eslint-disable-line no-self-compare
}

},{"base64-js":6,"ieee754":13}],9:[function(_dereq_,module,exports){
(function (global){
/*global window, global*/
var util = _dereq_("util")
var assert = _dereq_("assert")
var now = _dereq_("date-now")

var slice = Array.prototype.slice
var console
var times = {}

if (typeof global !== "undefined" && global.console) {
    console = global.console
} else if (typeof window !== "undefined" && window.console) {
    console = window.console
} else {
    console = {}
}

var functions = [
    [log, "log"],
    [info, "info"],
    [warn, "warn"],
    [error, "error"],
    [time, "time"],
    [timeEnd, "timeEnd"],
    [trace, "trace"],
    [dir, "dir"],
    [consoleAssert, "assert"]
]

for (var i = 0; i < functions.length; i++) {
    var tuple = functions[i]
    var f = tuple[0]
    var name = tuple[1]

    if (!console[name]) {
        console[name] = f
    }
}

module.exports = console

function log() {}

function info() {
    console.log.apply(console, arguments)
}

function warn() {
    console.log.apply(console, arguments)
}

function error() {
    console.warn.apply(console, arguments)
}

function time(label) {
    times[label] = now()
}

function timeEnd(label) {
    var time = times[label]
    if (!time) {
        throw new Error("No such label: " + label)
    }

    var duration = now() - time
    console.log(label + ": " + duration + "ms")
}

function trace() {
    var err = new Error()
    err.name = "Trace"
    err.message = util.format.apply(null, arguments)
    console.error(err.stack)
}

function dir(object) {
    console.log(util.inspect(object) + "\n")
}

function consoleAssert(expression) {
    if (!expression) {
        var arr = slice.call(arguments, 1)
        assert.ok(false, util.format.apply(null, arr))
    }
}

}).call(this,typeof global !== "undefined" ? global : typeof self !== "undefined" ? self : typeof window !== "undefined" ? window : {})
},{"assert":2,"date-now":11,"util":39}],10:[function(_dereq_,module,exports){
(function (Buffer){
// Copyright Joyent, Inc. and other Node contributors.
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to permit
// persons to whom the Software is furnished to do so, subject to the
// following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
// NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
// USE OR OTHER DEALINGS IN THE SOFTWARE.

// NOTE: These type checking functions intentionally don't use `instanceof`
// because it is fragile and can be easily faked with `Object.create()`.

function isArray(arg) {
  if (Array.isArray) {
    return Array.isArray(arg);
  }
  return objectToString(arg) === '[object Array]';
}
exports.isArray = isArray;

function isBoolean(arg) {
  return typeof arg === 'boolean';
}
exports.isBoolean = isBoolean;

function isNull(arg) {
  return arg === null;
}
exports.isNull = isNull;

function isNullOrUndefined(arg) {
  return arg == null;
}
exports.isNullOrUndefined = isNullOrUndefined;

function isNumber(arg) {
  return typeof arg === 'number';
}
exports.isNumber = isNumber;

function isString(arg) {
  return typeof arg === 'string';
}
exports.isString = isString;

function isSymbol(arg) {
  return typeof arg === 'symbol';
}
exports.isSymbol = isSymbol;

function isUndefined(arg) {
  return arg === void 0;
}
exports.isUndefined = isUndefined;

function isRegExp(re) {
  return objectToString(re) === '[object RegExp]';
}
exports.isRegExp = isRegExp;

function isObject(arg) {
  return typeof arg === 'object' && arg !== null;
}
exports.isObject = isObject;

function isDate(d) {
  return objectToString(d) === '[object Date]';
}
exports.isDate = isDate;

function isError(e) {
  return (objectToString(e) === '[object Error]' || e instanceof Error);
}
exports.isError = isError;

function isFunction(arg) {
  return typeof arg === 'function';
}
exports.isFunction = isFunction;

function isPrimitive(arg) {
  return arg === null ||
         typeof arg === 'boolean' ||
         typeof arg === 'number' ||
         typeof arg === 'string' ||
         typeof arg === 'symbol' ||  // ES6 symbol
         typeof arg === 'undefined';
}
exports.isPrimitive = isPrimitive;

exports.isBuffer = Buffer.isBuffer;

function objectToString(o) {
  return Object.prototype.toString.call(o);
}

}).call(this,{"isBuffer":_dereq_("../../is-buffer/index.js")})
},{"../../is-buffer/index.js":15}],11:[function(_dereq_,module,exports){
module.exports = now

function now() {
    return new Date().getTime()
}

},{}],12:[function(_dereq_,module,exports){
// Copyright Joyent, Inc. and other Node contributors.
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to permit
// persons to whom the Software is furnished to do so, subject to the
// following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
// NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
// USE OR OTHER DEALINGS IN THE SOFTWARE.

var objectCreate = Object.create || objectCreatePolyfill
var objectKeys = Object.keys || objectKeysPolyfill
var bind = Function.prototype.bind || functionBindPolyfill

function EventEmitter() {
  if (!this._events || !Object.prototype.hasOwnProperty.call(this, '_events')) {
    this._events = objectCreate(null);
    this._eventsCount = 0;
  }

  this._maxListeners = this._maxListeners || undefined;
}
module.exports = EventEmitter;

// Backwards-compat with node 0.10.x
EventEmitter.EventEmitter = EventEmitter;

EventEmitter.prototype._events = undefined;
EventEmitter.prototype._maxListeners = undefined;

// By default EventEmitters will print a warning if more than 10 listeners are
// added to it. This is a useful default which helps finding memory leaks.
var defaultMaxListeners = 10;

var hasDefineProperty;
try {
  var o = {};
  if (Object.defineProperty) Object.defineProperty(o, 'x', { value: 0 });
  hasDefineProperty = o.x === 0;
} catch (err) { hasDefineProperty = false }
if (hasDefineProperty) {
  Object.defineProperty(EventEmitter, 'defaultMaxListeners', {
    enumerable: true,
    get: function() {
      return defaultMaxListeners;
    },
    set: function(arg) {
      // check whether the input is a positive number (whose value is zero or
      // greater and not a NaN).
      if (typeof arg !== 'number' || arg < 0 || arg !== arg)
        throw new TypeError('"defaultMaxListeners" must be a positive number');
      defaultMaxListeners = arg;
    }
  });
} else {
  EventEmitter.defaultMaxListeners = defaultMaxListeners;
}

// Obviously not all Emitters should be limited to 10. This function allows
// that to be increased. Set to zero for unlimited.
EventEmitter.prototype.setMaxListeners = function setMaxListeners(n) {
  if (typeof n !== 'number' || n < 0 || isNaN(n))
    throw new TypeError('"n" argument must be a positive number');
  this._maxListeners = n;
  return this;
};

function $getMaxListeners(that) {
  if (that._maxListeners === undefined)
    return EventEmitter.defaultMaxListeners;
  return that._maxListeners;
}

EventEmitter.prototype.getMaxListeners = function getMaxListeners() {
  return $getMaxListeners(this);
};

// These standalone emit* functions are used to optimize calling of event
// handlers for fast cases because emit() itself often has a variable number of
// arguments and can be deoptimized because of that. These functions always have
// the same number of arguments and thus do not get deoptimized, so the code
// inside them can execute faster.
function emitNone(handler, isFn, self) {
  if (isFn)
    handler.call(self);
  else {
    var len = handler.length;
    var listeners = arrayClone(handler, len);
    for (var i = 0; i < len; ++i)
      listeners[i].call(self);
  }
}
function emitOne(handler, isFn, self, arg1) {
  if (isFn)
    handler.call(self, arg1);
  else {
    var len = handler.length;
    var listeners = arrayClone(handler, len);
    for (var i = 0; i < len; ++i)
      listeners[i].call(self, arg1);
  }
}
function emitTwo(handler, isFn, self, arg1, arg2) {
  if (isFn)
    handler.call(self, arg1, arg2);
  else {
    var len = handler.length;
    var listeners = arrayClone(handler, len);
    for (var i = 0; i < len; ++i)
      listeners[i].call(self, arg1, arg2);
  }
}
function emitThree(handler, isFn, self, arg1, arg2, arg3) {
  if (isFn)
    handler.call(self, arg1, arg2, arg3);
  else {
    var len = handler.length;
    var listeners = arrayClone(handler, len);
    for (var i = 0; i < len; ++i)
      listeners[i].call(self, arg1, arg2, arg3);
  }
}

function emitMany(handler, isFn, self, args) {
  if (isFn)
    handler.apply(self, args);
  else {
    var len = handler.length;
    var listeners = arrayClone(handler, len);
    for (var i = 0; i < len; ++i)
      listeners[i].apply(self, args);
  }
}

EventEmitter.prototype.emit = function emit(type) {
  var er, handler, len, args, i, events;
  var doError = (type === 'error');

  events = this._events;
  if (events)
    doError = (doError && events.error == null);
  else if (!doError)
    return false;

  // If there is no 'error' event listener then throw.
  if (doError) {
    if (arguments.length > 1)
      er = arguments[1];
    if (er instanceof Error) {
      throw er; // Unhandled 'error' event
    } else {
      // At least give some kind of context to the user
      var err = new Error('Unhandled "error" event. (' + er + ')');
      err.context = er;
      throw err;
    }
    return false;
  }

  handler = events[type];

  if (!handler)
    return false;

  var isFn = typeof handler === 'function';
  len = arguments.length;
  switch (len) {
      // fast cases
    case 1:
      emitNone(handler, isFn, this);
      break;
    case 2:
      emitOne(handler, isFn, this, arguments[1]);
      break;
    case 3:
      emitTwo(handler, isFn, this, arguments[1], arguments[2]);
      break;
    case 4:
      emitThree(handler, isFn, this, arguments[1], arguments[2], arguments[3]);
      break;
      // slower
    default:
      args = new Array(len - 1);
      for (i = 1; i < len; i++)
        args[i - 1] = arguments[i];
      emitMany(handler, isFn, this, args);
  }

  return true;
};

function _addListener(target, type, listener, prepend) {
  var m;
  var events;
  var existing;

  if (typeof listener !== 'function')
    throw new TypeError('"listener" argument must be a function');

  events = target._events;
  if (!events) {
    events = target._events = objectCreate(null);
    target._eventsCount = 0;
  } else {
    // To avoid recursion in the case that type === "newListener"! Before
    // adding it to the listeners, first emit "newListener".
    if (events.newListener) {
      target.emit('newListener', type,
          listener.listener ? listener.listener : listener);

      // Re-assign `events` because a newListener handler could have caused the
      // this._events to be assigned to a new object
      events = target._events;
    }
    existing = events[type];
  }

  if (!existing) {
    // Optimize the case of one listener. Don't need the extra array object.
    existing = events[type] = listener;
    ++target._eventsCount;
  } else {
    if (typeof existing === 'function') {
      // Adding the second element, need to change to array.
      existing = events[type] =
          prepend ? [listener, existing] : [existing, listener];
    } else {
      // If we've already got an array, just append.
      if (prepend) {
        existing.unshift(listener);
      } else {
        existing.push(listener);
      }
    }

    // Check for listener leak
    if (!existing.warned) {
      m = $getMaxListeners(target);
      if (m && m > 0 && existing.length > m) {
        existing.warned = true;
        var w = new Error('Possible EventEmitter memory leak detected. ' +
            existing.length + ' "' + String(type) + '" listeners ' +
            'added. Use emitter.setMaxListeners() to ' +
            'increase limit.');
        w.name = 'MaxListenersExceededWarning';
        w.emitter = target;
        w.type = type;
        w.count = existing.length;
        if (typeof console === 'object' && console.warn) {
          console.warn('%s: %s', w.name, w.message);
        }
      }
    }
  }

  return target;
}

EventEmitter.prototype.addListener = function addListener(type, listener) {
  return _addListener(this, type, listener, false);
};

EventEmitter.prototype.on = EventEmitter.prototype.addListener;

EventEmitter.prototype.prependListener =
    function prependListener(type, listener) {
      return _addListener(this, type, listener, true);
    };

function onceWrapper() {
  if (!this.fired) {
    this.target.removeListener(this.type, this.wrapFn);
    this.fired = true;
    switch (arguments.length) {
      case 0:
        return this.listener.call(this.target);
      case 1:
        return this.listener.call(this.target, arguments[0]);
      case 2:
        return this.listener.call(this.target, arguments[0], arguments[1]);
      case 3:
        return this.listener.call(this.target, arguments[0], arguments[1],
            arguments[2]);
      default:
        var args = new Array(arguments.length);
        for (var i = 0; i < args.length; ++i)
          args[i] = arguments[i];
        this.listener.apply(this.target, args);
    }
  }
}

function _onceWrap(target, type, listener) {
  var state = { fired: false, wrapFn: undefined, target: target, type: type, listener: listener };
  var wrapped = bind.call(onceWrapper, state);
  wrapped.listener = listener;
  state.wrapFn = wrapped;
  return wrapped;
}

EventEmitter.prototype.once = function once(type, listener) {
  if (typeof listener !== 'function')
    throw new TypeError('"listener" argument must be a function');
  this.on(type, _onceWrap(this, type, listener));
  return this;
};

EventEmitter.prototype.prependOnceListener =
    function prependOnceListener(type, listener) {
      if (typeof listener !== 'function')
        throw new TypeError('"listener" argument must be a function');
      this.prependListener(type, _onceWrap(this, type, listener));
      return this;
    };

// Emits a 'removeListener' event if and only if the listener was removed.
EventEmitter.prototype.removeListener =
    function removeListener(type, listener) {
      var list, events, position, i, originalListener;

      if (typeof listener !== 'function')
        throw new TypeError('"listener" argument must be a function');

      events = this._events;
      if (!events)
        return this;

      list = events[type];
      if (!list)
        return this;

      if (list === listener || list.listener === listener) {
        if (--this._eventsCount === 0)
          this._events = objectCreate(null);
        else {
          delete events[type];
          if (events.removeListener)
            this.emit('removeListener', type, list.listener || listener);
        }
      } else if (typeof list !== 'function') {
        position = -1;

        for (i = list.length - 1; i >= 0; i--) {
          if (list[i] === listener || list[i].listener === listener) {
            originalListener = list[i].listener;
            position = i;
            break;
          }
        }

        if (position < 0)
          return this;

        if (position === 0)
          list.shift();
        else
          spliceOne(list, position);

        if (list.length === 1)
          events[type] = list[0];

        if (events.removeListener)
          this.emit('removeListener', type, originalListener || listener);
      }

      return this;
    };

EventEmitter.prototype.removeAllListeners =
    function removeAllListeners(type) {
      var listeners, events, i;

      events = this._events;
      if (!events)
        return this;

      // not listening for removeListener, no need to emit
      if (!events.removeListener) {
        if (arguments.length === 0) {
          this._events = objectCreate(null);
          this._eventsCount = 0;
        } else if (events[type]) {
          if (--this._eventsCount === 0)
            this._events = objectCreate(null);
          else
            delete events[type];
        }
        return this;
      }

      // emit removeListener for all listeners on all events
      if (arguments.length === 0) {
        var keys = objectKeys(events);
        var key;
        for (i = 0; i < keys.length; ++i) {
          key = keys[i];
          if (key === 'removeListener') continue;
          this.removeAllListeners(key);
        }
        this.removeAllListeners('removeListener');
        this._events = objectCreate(null);
        this._eventsCount = 0;
        return this;
      }

      listeners = events[type];

      if (typeof listeners === 'function') {
        this.removeListener(type, listeners);
      } else if (listeners) {
        // LIFO order
        for (i = listeners.length - 1; i >= 0; i--) {
          this.removeListener(type, listeners[i]);
        }
      }

      return this;
    };

function _listeners(target, type, unwrap) {
  var events = target._events;

  if (!events)
    return [];

  var evlistener = events[type];
  if (!evlistener)
    return [];

  if (typeof evlistener === 'function')
    return unwrap ? [evlistener.listener || evlistener] : [evlistener];

  return unwrap ? unwrapListeners(evlistener) : arrayClone(evlistener, evlistener.length);
}

EventEmitter.prototype.listeners = function listeners(type) {
  return _listeners(this, type, true);
};

EventEmitter.prototype.rawListeners = function rawListeners(type) {
  return _listeners(this, type, false);
};

EventEmitter.listenerCount = function(emitter, type) {
  if (typeof emitter.listenerCount === 'function') {
    return emitter.listenerCount(type);
  } else {
    return listenerCount.call(emitter, type);
  }
};

EventEmitter.prototype.listenerCount = listenerCount;
function listenerCount(type) {
  var events = this._events;

  if (events) {
    var evlistener = events[type];

    if (typeof evlistener === 'function') {
      return 1;
    } else if (evlistener) {
      return evlistener.length;
    }
  }

  return 0;
}

EventEmitter.prototype.eventNames = function eventNames() {
  return this._eventsCount > 0 ? Reflect.ownKeys(this._events) : [];
};

// About 1.5x faster than the two-arg version of Array#splice().
function spliceOne(list, index) {
  for (var i = index, k = i + 1, n = list.length; k < n; i += 1, k += 1)
    list[i] = list[k];
  list.pop();
}

function arrayClone(arr, n) {
  var copy = new Array(n);
  for (var i = 0; i < n; ++i)
    copy[i] = arr[i];
  return copy;
}

function unwrapListeners(arr) {
  var ret = new Array(arr.length);
  for (var i = 0; i < ret.length; ++i) {
    ret[i] = arr[i].listener || arr[i];
  }
  return ret;
}

function objectCreatePolyfill(proto) {
  var F = function() {};
  F.prototype = proto;
  return new F;
}
function objectKeysPolyfill(obj) {
  var keys = [];
  for (var k in obj) if (Object.prototype.hasOwnProperty.call(obj, k)) {
    keys.push(k);
  }
  return k;
}
function functionBindPolyfill(context) {
  var fn = this;
  return function () {
    return fn.apply(context, arguments);
  };
}

},{}],13:[function(_dereq_,module,exports){
exports.read = function (buffer, offset, isLE, mLen, nBytes) {
  var e, m
  var eLen = (nBytes * 8) - mLen - 1
  var eMax = (1 << eLen) - 1
  var eBias = eMax >> 1
  var nBits = -7
  var i = isLE ? (nBytes - 1) : 0
  var d = isLE ? -1 : 1
  var s = buffer[offset + i]

  i += d

  e = s & ((1 << (-nBits)) - 1)
  s >>= (-nBits)
  nBits += eLen
  for (; nBits > 0; e = (e * 256) + buffer[offset + i], i += d, nBits -= 8) {}

  m = e & ((1 << (-nBits)) - 1)
  e >>= (-nBits)
  nBits += mLen
  for (; nBits > 0; m = (m * 256) + buffer[offset + i], i += d, nBits -= 8) {}

  if (e === 0) {
    e = 1 - eBias
  } else if (e === eMax) {
    return m ? NaN : ((s ? -1 : 1) * Infinity)
  } else {
    m = m + Math.pow(2, mLen)
    e = e - eBias
  }
  return (s ? -1 : 1) * m * Math.pow(2, e - mLen)
}

exports.write = function (buffer, value, offset, isLE, mLen, nBytes) {
  var e, m, c
  var eLen = (nBytes * 8) - mLen - 1
  var eMax = (1 << eLen) - 1
  var eBias = eMax >> 1
  var rt = (mLen === 23 ? Math.pow(2, -24) - Math.pow(2, -77) : 0)
  var i = isLE ? 0 : (nBytes - 1)
  var d = isLE ? 1 : -1
  var s = value < 0 || (value === 0 && 1 / value < 0) ? 1 : 0

  value = Math.abs(value)

  if (isNaN(value) || value === Infinity) {
    m = isNaN(value) ? 1 : 0
    e = eMax
  } else {
    e = Math.floor(Math.log(value) / Math.LN2)
    if (value * (c = Math.pow(2, -e)) < 1) {
      e--
      c *= 2
    }
    if (e + eBias >= 1) {
      value += rt / c
    } else {
      value += rt * Math.pow(2, 1 - eBias)
    }
    if (value * c >= 2) {
      e++
      c /= 2
    }

    if (e + eBias >= eMax) {
      m = 0
      e = eMax
    } else if (e + eBias >= 1) {
      m = ((value * c) - 1) * Math.pow(2, mLen)
      e = e + eBias
    } else {
      m = value * Math.pow(2, eBias - 1) * Math.pow(2, mLen)
      e = 0
    }
  }

  for (; mLen >= 8; buffer[offset + i] = m & 0xff, i += d, m /= 256, mLen -= 8) {}

  e = (e << mLen) | m
  eLen += mLen
  for (; eLen > 0; buffer[offset + i] = e & 0xff, i += d, e /= 256, eLen -= 8) {}

  buffer[offset + i - d] |= s * 128
}

},{}],14:[function(_dereq_,module,exports){
arguments[4][3][0].apply(exports,arguments)
},{"dup":3}],15:[function(_dereq_,module,exports){
/*!
 * Determine if an object is a Buffer
 *
 * @author   Feross Aboukhadijeh <https://feross.org>
 * @license  MIT
 */

// The _isBuffer check is for Safari 5-7 support, because it's missing
// Object.prototype.constructor. Remove this eventually
module.exports = function (obj) {
  return obj != null && (isBuffer(obj) || isSlowBuffer(obj) || !!obj._isBuffer)
}

function isBuffer (obj) {
  return !!obj.constructor && typeof obj.constructor.isBuffer === 'function' && obj.constructor.isBuffer(obj)
}

// For Node v0.10 support. Remove this eventually.
function isSlowBuffer (obj) {
  return typeof obj.readFloatLE === 'function' && typeof obj.slice === 'function' && isBuffer(obj.slice(0, 0))
}

},{}],16:[function(_dereq_,module,exports){
var toString = {}.toString;

module.exports = Array.isArray || function (arr) {
  return toString.call(arr) == '[object Array]';
};

},{}],17:[function(_dereq_,module,exports){
(function (process){
// Copyright Joyent, Inc. and other Node contributors.
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to permit
// persons to whom the Software is furnished to do so, subject to the
// following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
// NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
// USE OR OTHER DEALINGS IN THE SOFTWARE.

// resolves . and .. elements in a path array with directory names there
// must be no slashes, empty elements, or device names (c:\) in the array
// (so also no leading and trailing slashes - it does not distinguish
// relative and absolute paths)
function normalizeArray(parts, allowAboveRoot) {
  // if the path tries to go above the root, `up` ends up > 0
  var up = 0;
  for (var i = parts.length - 1; i >= 0; i--) {
    var last = parts[i];
    if (last === '.') {
      parts.splice(i, 1);
    } else if (last === '..') {
      parts.splice(i, 1);
      up++;
    } else if (up) {
      parts.splice(i, 1);
      up--;
    }
  }

  // if the path is allowed to go above the root, restore leading ..s
  if (allowAboveRoot) {
    for (; up--; up) {
      parts.unshift('..');
    }
  }

  return parts;
}

// Split a filename into [root, dir, basename, ext], unix version
// 'root' is just a slash, or nothing.
var splitPathRe =
    /^(\/?|)([\s\S]*?)((?:\.{1,2}|[^\/]+?|)(\.[^.\/]*|))(?:[\/]*)$/;
var splitPath = function(filename) {
  return splitPathRe.exec(filename).slice(1);
};

// path.resolve([from ...], to)
// posix version
exports.resolve = function() {
  var resolvedPath = '',
      resolvedAbsolute = false;

  for (var i = arguments.length - 1; i >= -1 && !resolvedAbsolute; i--) {
    var path = (i >= 0) ? arguments[i] : process.cwd();

    // Skip empty and invalid entries
    if (typeof path !== 'string') {
      throw new TypeError('Arguments to path.resolve must be strings');
    } else if (!path) {
      continue;
    }

    resolvedPath = path + '/' + resolvedPath;
    resolvedAbsolute = path.charAt(0) === '/';
  }

  // At this point the path should be resolved to a full absolute path, but
  // handle relative paths to be safe (might happen when process.cwd() fails)

  // Normalize the path
  resolvedPath = normalizeArray(filter(resolvedPath.split('/'), function(p) {
    return !!p;
  }), !resolvedAbsolute).join('/');

  return ((resolvedAbsolute ? '/' : '') + resolvedPath) || '.';
};

// path.normalize(path)
// posix version
exports.normalize = function(path) {
  var isAbsolute = exports.isAbsolute(path),
      trailingSlash = substr(path, -1) === '/';

  // Normalize the path
  path = normalizeArray(filter(path.split('/'), function(p) {
    return !!p;
  }), !isAbsolute).join('/');

  if (!path && !isAbsolute) {
    path = '.';
  }
  if (path && trailingSlash) {
    path += '/';
  }

  return (isAbsolute ? '/' : '') + path;
};

// posix version
exports.isAbsolute = function(path) {
  return path.charAt(0) === '/';
};

// posix version
exports.join = function() {
  var paths = Array.prototype.slice.call(arguments, 0);
  return exports.normalize(filter(paths, function(p, index) {
    if (typeof p !== 'string') {
      throw new TypeError('Arguments to path.join must be strings');
    }
    return p;
  }).join('/'));
};


// path.relative(from, to)
// posix version
exports.relative = function(from, to) {
  from = exports.resolve(from).substr(1);
  to = exports.resolve(to).substr(1);

  function trim(arr) {
    var start = 0;
    for (; start < arr.length; start++) {
      if (arr[start] !== '') break;
    }

    var end = arr.length - 1;
    for (; end >= 0; end--) {
      if (arr[end] !== '') break;
    }

    if (start > end) return [];
    return arr.slice(start, end - start + 1);
  }

  var fromParts = trim(from.split('/'));
  var toParts = trim(to.split('/'));

  var length = Math.min(fromParts.length, toParts.length);
  var samePartsLength = length;
  for (var i = 0; i < length; i++) {
    if (fromParts[i] !== toParts[i]) {
      samePartsLength = i;
      break;
    }
  }

  var outputParts = [];
  for (var i = samePartsLength; i < fromParts.length; i++) {
    outputParts.push('..');
  }

  outputParts = outputParts.concat(toParts.slice(samePartsLength));

  return outputParts.join('/');
};

exports.sep = '/';
exports.delimiter = ':';

exports.dirname = function(path) {
  var result = splitPath(path),
      root = result[0],
      dir = result[1];

  if (!root && !dir) {
    // No dirname whatsoever
    return '.';
  }

  if (dir) {
    // It has a dirname, strip trailing slash
    dir = dir.substr(0, dir.length - 1);
  }

  return root + dir;
};


exports.basename = function(path, ext) {
  var f = splitPath(path)[2];
  // TODO: make this comparison case-insensitive on windows?
  if (ext && f.substr(-1 * ext.length) === ext) {
    f = f.substr(0, f.length - ext.length);
  }
  return f;
};


exports.extname = function(path) {
  return splitPath(path)[3];
};

function filter (xs, f) {
    if (xs.filter) return xs.filter(f);
    var res = [];
    for (var i = 0; i < xs.length; i++) {
        if (f(xs[i], i, xs)) res.push(xs[i]);
    }
    return res;
}

// String.prototype.substr - negative index don't work in IE8
var substr = 'ab'.substr(-1) === 'b'
    ? function (str, start, len) { return str.substr(start, len) }
    : function (str, start, len) {
        if (start < 0) start = str.length + start;
        return str.substr(start, len);
    }
;

}).call(this,_dereq_('_process'))
},{"_process":19}],18:[function(_dereq_,module,exports){
(function (process){
'use strict';

if (!process.version ||
    process.version.indexOf('v0.') === 0 ||
    process.version.indexOf('v1.') === 0 && process.version.indexOf('v1.8.') !== 0) {
  module.exports = { nextTick: nextTick };
} else {
  module.exports = process
}

function nextTick(fn, arg1, arg2, arg3) {
  if (typeof fn !== 'function') {
    throw new TypeError('"callback" argument must be a function');
  }
  var len = arguments.length;
  var args, i;
  switch (len) {
  case 0:
  case 1:
    return process.nextTick(fn);
  case 2:
    return process.nextTick(function afterTickOne() {
      fn.call(null, arg1);
    });
  case 3:
    return process.nextTick(function afterTickTwo() {
      fn.call(null, arg1, arg2);
    });
  case 4:
    return process.nextTick(function afterTickThree() {
      fn.call(null, arg1, arg2, arg3);
    });
  default:
    args = new Array(len - 1);
    i = 0;
    while (i < args.length) {
      args[i++] = arguments[i];
    }
    return process.nextTick(function afterTick() {
      fn.apply(null, args);
    });
  }
}


}).call(this,_dereq_('_process'))
},{"_process":19}],19:[function(_dereq_,module,exports){
// shim for using process in browser
var process = module.exports = {};

// cached from whatever global is present so that test runners that stub it
// don't break things.  But we need to wrap it in a try catch in case it is
// wrapped in strict mode code which doesn't define any globals.  It's inside a
// function because try/catches deoptimize in certain engines.

var cachedSetTimeout;
var cachedClearTimeout;

function defaultSetTimout() {
    throw new Error('setTimeout has not been defined');
}
function defaultClearTimeout () {
    throw new Error('clearTimeout has not been defined');
}
(function () {
    try {
        if (typeof setTimeout === 'function') {
            cachedSetTimeout = setTimeout;
        } else {
            cachedSetTimeout = defaultSetTimout;
        }
    } catch (e) {
        cachedSetTimeout = defaultSetTimout;
    }
    try {
        if (typeof clearTimeout === 'function') {
            cachedClearTimeout = clearTimeout;
        } else {
            cachedClearTimeout = defaultClearTimeout;
        }
    } catch (e) {
        cachedClearTimeout = defaultClearTimeout;
    }
} ())
function runTimeout(fun) {
    if (cachedSetTimeout === setTimeout) {
        //normal enviroments in sane situations
        return setTimeout(fun, 0);
    }
    // if setTimeout wasn't available but was latter defined
    if ((cachedSetTimeout === defaultSetTimout || !cachedSetTimeout) && setTimeout) {
        cachedSetTimeout = setTimeout;
        return setTimeout(fun, 0);
    }
    try {
        // when when somebody has screwed with setTimeout but no I.E. maddness
        return cachedSetTimeout(fun, 0);
    } catch(e){
        try {
            // When we are in I.E. but the script has been evaled so I.E. doesn't trust the global object when called normally
            return cachedSetTimeout.call(null, fun, 0);
        } catch(e){
            // same as above but when it's a version of I.E. that must have the global object for 'this', hopfully our context correct otherwise it will throw a global error
            return cachedSetTimeout.call(this, fun, 0);
        }
    }


}
function runClearTimeout(marker) {
    if (cachedClearTimeout === clearTimeout) {
        //normal enviroments in sane situations
        return clearTimeout(marker);
    }
    // if clearTimeout wasn't available but was latter defined
    if ((cachedClearTimeout === defaultClearTimeout || !cachedClearTimeout) && clearTimeout) {
        cachedClearTimeout = clearTimeout;
        return clearTimeout(marker);
    }
    try {
        // when when somebody has screwed with setTimeout but no I.E. maddness
        return cachedClearTimeout(marker);
    } catch (e){
        try {
            // When we are in I.E. but the script has been evaled so I.E. doesn't  trust the global object when called normally
            return cachedClearTimeout.call(null, marker);
        } catch (e){
            // same as above but when it's a version of I.E. that must have the global object for 'this', hopfully our context correct otherwise it will throw a global error.
            // Some versions of I.E. have different rules for clearTimeout vs setTimeout
            return cachedClearTimeout.call(this, marker);
        }
    }



}
var queue = [];
var draining = false;
var currentQueue;
var queueIndex = -1;

function cleanUpNextTick() {
    if (!draining || !currentQueue) {
        return;
    }
    draining = false;
    if (currentQueue.length) {
        queue = currentQueue.concat(queue);
    } else {
        queueIndex = -1;
    }
    if (queue.length) {
        drainQueue();
    }
}

function drainQueue() {
    if (draining) {
        return;
    }
    var timeout = runTimeout(cleanUpNextTick);
    draining = true;

    var len = queue.length;
    while(len) {
        currentQueue = queue;
        queue = [];
        while (++queueIndex < len) {
            if (currentQueue) {
                currentQueue[queueIndex].run();
            }
        }
        queueIndex = -1;
        len = queue.length;
    }
    currentQueue = null;
    draining = false;
    runClearTimeout(timeout);
}

process.nextTick = function (fun) {
    var args = new Array(arguments.length - 1);
    if (arguments.length > 1) {
        for (var i = 1; i < arguments.length; i++) {
            args[i - 1] = arguments[i];
        }
    }
    queue.push(new Item(fun, args));
    if (queue.length === 1 && !draining) {
        runTimeout(drainQueue);
    }
};

// v8 likes predictible objects
function Item(fun, array) {
    this.fun = fun;
    this.array = array;
}
Item.prototype.run = function () {
    this.fun.apply(null, this.array);
};
process.title = 'browser';
process.browser = true;
process.env = {};
process.argv = [];
process.version = ''; // empty string to avoid regexp issues
process.versions = {};

function noop() {}

process.on = noop;
process.addListener = noop;
process.once = noop;
process.off = noop;
process.removeListener = noop;
process.removeAllListeners = noop;
process.emit = noop;
process.prependListener = noop;
process.prependOnceListener = noop;

process.listeners = function (name) { return [] }

process.binding = function (name) {
    throw new Error('process.binding is not supported');
};

process.cwd = function () { return '/' };
process.chdir = function (dir) {
    throw new Error('process.chdir is not supported');
};
process.umask = function() { return 0; };

},{}],20:[function(_dereq_,module,exports){
module.exports = _dereq_('./lib/_stream_duplex.js');

},{"./lib/_stream_duplex.js":21}],21:[function(_dereq_,module,exports){
// Copyright Joyent, Inc. and other Node contributors.
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to permit
// persons to whom the Software is furnished to do so, subject to the
// following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
// NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
// USE OR OTHER DEALINGS IN THE SOFTWARE.

// a duplex stream is just a stream that is both readable and writable.
// Since JS doesn't have multiple prototypal inheritance, this class
// prototypally inherits from Readable, and then parasitically from
// Writable.

'use strict';

/*<replacement>*/

var pna = _dereq_('process-nextick-args');
/*</replacement>*/

/*<replacement>*/
var objectKeys = Object.keys || function (obj) {
  var keys = [];
  for (var key in obj) {
    keys.push(key);
  }return keys;
};
/*</replacement>*/

module.exports = Duplex;

/*<replacement>*/
var util = _dereq_('core-util-is');
util.inherits = _dereq_('inherits');
/*</replacement>*/

var Readable = _dereq_('./_stream_readable');
var Writable = _dereq_('./_stream_writable');

util.inherits(Duplex, Readable);

{
  // avoid scope creep, the keys array can then be collected
  var keys = objectKeys(Writable.prototype);
  for (var v = 0; v < keys.length; v++) {
    var method = keys[v];
    if (!Duplex.prototype[method]) Duplex.prototype[method] = Writable.prototype[method];
  }
}

function Duplex(options) {
  if (!(this instanceof Duplex)) return new Duplex(options);

  Readable.call(this, options);
  Writable.call(this, options);

  if (options && options.readable === false) this.readable = false;

  if (options && options.writable === false) this.writable = false;

  this.allowHalfOpen = true;
  if (options && options.allowHalfOpen === false) this.allowHalfOpen = false;

  this.once('end', onend);
}

Object.defineProperty(Duplex.prototype, 'writableHighWaterMark', {
  // making it explicit this property is not enumerable
  // because otherwise some prototype manipulation in
  // userland will fail
  enumerable: false,
  get: function () {
    return this._writableState.highWaterMark;
  }
});

// the no-half-open enforcer
function onend() {
  // if we allow half-open state, or if the writable side ended,
  // then we're ok.
  if (this.allowHalfOpen || this._writableState.ended) return;

  // no more data can be written.
  // But allow more writes to happen in this tick.
  pna.nextTick(onEndNT, this);
}

function onEndNT(self) {
  self.end();
}

Object.defineProperty(Duplex.prototype, 'destroyed', {
  get: function () {
    if (this._readableState === undefined || this._writableState === undefined) {
      return false;
    }
    return this._readableState.destroyed && this._writableState.destroyed;
  },
  set: function (value) {
    // we ignore the value if the stream
    // has not been initialized yet
    if (this._readableState === undefined || this._writableState === undefined) {
      return;
    }

    // backward compatibility, the user is explicitly
    // managing destroyed
    this._readableState.destroyed = value;
    this._writableState.destroyed = value;
  }
});

Duplex.prototype._destroy = function (err, cb) {
  this.push(null);
  this.end();

  pna.nextTick(cb, err);
};
},{"./_stream_readable":23,"./_stream_writable":25,"core-util-is":10,"inherits":14,"process-nextick-args":18}],22:[function(_dereq_,module,exports){
// Copyright Joyent, Inc. and other Node contributors.
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to permit
// persons to whom the Software is furnished to do so, subject to the
// following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
// NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
// USE OR OTHER DEALINGS IN THE SOFTWARE.

// a passthrough stream.
// basically just the most minimal sort of Transform stream.
// Every written chunk gets output as-is.

'use strict';

module.exports = PassThrough;

var Transform = _dereq_('./_stream_transform');

/*<replacement>*/
var util = _dereq_('core-util-is');
util.inherits = _dereq_('inherits');
/*</replacement>*/

util.inherits(PassThrough, Transform);

function PassThrough(options) {
  if (!(this instanceof PassThrough)) return new PassThrough(options);

  Transform.call(this, options);
}

PassThrough.prototype._transform = function (chunk, encoding, cb) {
  cb(null, chunk);
};
},{"./_stream_transform":24,"core-util-is":10,"inherits":14}],23:[function(_dereq_,module,exports){
(function (process,global){
// Copyright Joyent, Inc. and other Node contributors.
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to permit
// persons to whom the Software is furnished to do so, subject to the
// following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
// NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
// USE OR OTHER DEALINGS IN THE SOFTWARE.

'use strict';

/*<replacement>*/

var pna = _dereq_('process-nextick-args');
/*</replacement>*/

module.exports = Readable;

/*<replacement>*/
var isArray = _dereq_('isarray');
/*</replacement>*/

/*<replacement>*/
var Duplex;
/*</replacement>*/

Readable.ReadableState = ReadableState;

/*<replacement>*/
var EE = _dereq_('events').EventEmitter;

var EElistenerCount = function (emitter, type) {
  return emitter.listeners(type).length;
};
/*</replacement>*/

/*<replacement>*/
var Stream = _dereq_('./internal/streams/stream');
/*</replacement>*/

/*<replacement>*/

var Buffer = _dereq_('safe-buffer').Buffer;
var OurUint8Array = global.Uint8Array || function () {};
function _uint8ArrayToBuffer(chunk) {
  return Buffer.from(chunk);
}
function _isUint8Array(obj) {
  return Buffer.isBuffer(obj) || obj instanceof OurUint8Array;
}

/*</replacement>*/

/*<replacement>*/
var util = _dereq_('core-util-is');
util.inherits = _dereq_('inherits');
/*</replacement>*/

/*<replacement>*/
var debugUtil = _dereq_('util');
var debug = void 0;
if (debugUtil && debugUtil.debuglog) {
  debug = debugUtil.debuglog('stream');
} else {
  debug = function () {};
}
/*</replacement>*/

var BufferList = _dereq_('./internal/streams/BufferList');
var destroyImpl = _dereq_('./internal/streams/destroy');
var StringDecoder;

util.inherits(Readable, Stream);

var kProxyEvents = ['error', 'close', 'destroy', 'pause', 'resume'];

function prependListener(emitter, event, fn) {
  // Sadly this is not cacheable as some libraries bundle their own
  // event emitter implementation with them.
  if (typeof emitter.prependListener === 'function') return emitter.prependListener(event, fn);

  // This is a hack to make sure that our error handler is attached before any
  // userland ones.  NEVER DO THIS. This is here only because this code needs
  // to continue to work with older versions of Node.js that do not include
  // the prependListener() method. The goal is to eventually remove this hack.
  if (!emitter._events || !emitter._events[event]) emitter.on(event, fn);else if (isArray(emitter._events[event])) emitter._events[event].unshift(fn);else emitter._events[event] = [fn, emitter._events[event]];
}

function ReadableState(options, stream) {
  Duplex = Duplex || _dereq_('./_stream_duplex');

  options = options || {};

  // Duplex streams are both readable and writable, but share
  // the same options object.
  // However, some cases require setting options to different
  // values for the readable and the writable sides of the duplex stream.
  // These options can be provided separately as readableXXX and writableXXX.
  var isDuplex = stream instanceof Duplex;

  // object stream flag. Used to make read(n) ignore n and to
  // make all the buffer merging and length checks go away
  this.objectMode = !!options.objectMode;

  if (isDuplex) this.objectMode = this.objectMode || !!options.readableObjectMode;

  // the point at which it stops calling _read() to fill the buffer
  // Note: 0 is a valid value, means "don't call _read preemptively ever"
  var hwm = options.highWaterMark;
  var readableHwm = options.readableHighWaterMark;
  var defaultHwm = this.objectMode ? 16 : 16 * 1024;

  if (hwm || hwm === 0) this.highWaterMark = hwm;else if (isDuplex && (readableHwm || readableHwm === 0)) this.highWaterMark = readableHwm;else this.highWaterMark = defaultHwm;

  // cast to ints.
  this.highWaterMark = Math.floor(this.highWaterMark);

  // A linked list is used to store data chunks instead of an array because the
  // linked list can remove elements from the beginning faster than
  // array.shift()
  this.buffer = new BufferList();
  this.length = 0;
  this.pipes = null;
  this.pipesCount = 0;
  this.flowing = null;
  this.ended = false;
  this.endEmitted = false;
  this.reading = false;

  // a flag to be able to tell if the event 'readable'/'data' is emitted
  // immediately, or on a later tick.  We set this to true at first, because
  // any actions that shouldn't happen until "later" should generally also
  // not happen before the first read call.
  this.sync = true;

  // whenever we return null, then we set a flag to say
  // that we're awaiting a 'readable' event emission.
  this.needReadable = false;
  this.emittedReadable = false;
  this.readableListening = false;
  this.resumeScheduled = false;

  // has it been destroyed
  this.destroyed = false;

  // Crypto is kind of old and crusty.  Historically, its default string
  // encoding is 'binary' so we have to make this configurable.
  // Everything else in the universe uses 'utf8', though.
  this.defaultEncoding = options.defaultEncoding || 'utf8';

  // the number of writers that are awaiting a drain event in .pipe()s
  this.awaitDrain = 0;

  // if true, a maybeReadMore has been scheduled
  this.readingMore = false;

  this.decoder = null;
  this.encoding = null;
  if (options.encoding) {
    if (!StringDecoder) StringDecoder = _dereq_('string_decoder/').StringDecoder;
    this.decoder = new StringDecoder(options.encoding);
    this.encoding = options.encoding;
  }
}

function Readable(options) {
  Duplex = Duplex || _dereq_('./_stream_duplex');

  if (!(this instanceof Readable)) return new Readable(options);

  this._readableState = new ReadableState(options, this);

  // legacy
  this.readable = true;

  if (options) {
    if (typeof options.read === 'function') this._read = options.read;

    if (typeof options.destroy === 'function') this._destroy = options.destroy;
  }

  Stream.call(this);
}

Object.defineProperty(Readable.prototype, 'destroyed', {
  get: function () {
    if (this._readableState === undefined) {
      return false;
    }
    return this._readableState.destroyed;
  },
  set: function (value) {
    // we ignore the value if the stream
    // has not been initialized yet
    if (!this._readableState) {
      return;
    }

    // backward compatibility, the user is explicitly
    // managing destroyed
    this._readableState.destroyed = value;
  }
});

Readable.prototype.destroy = destroyImpl.destroy;
Readable.prototype._undestroy = destroyImpl.undestroy;
Readable.prototype._destroy = function (err, cb) {
  this.push(null);
  cb(err);
};

// Manually shove something into the read() buffer.
// This returns true if the highWaterMark has not been hit yet,
// similar to how Writable.write() returns true if you should
// write() some more.
Readable.prototype.push = function (chunk, encoding) {
  var state = this._readableState;
  var skipChunkCheck;

  if (!state.objectMode) {
    if (typeof chunk === 'string') {
      encoding = encoding || state.defaultEncoding;
      if (encoding !== state.encoding) {
        chunk = Buffer.from(chunk, encoding);
        encoding = '';
      }
      skipChunkCheck = true;
    }
  } else {
    skipChunkCheck = true;
  }

  return readableAddChunk(this, chunk, encoding, false, skipChunkCheck);
};

// Unshift should *always* be something directly out of read()
Readable.prototype.unshift = function (chunk) {
  return readableAddChunk(this, chunk, null, true, false);
};

function readableAddChunk(stream, chunk, encoding, addToFront, skipChunkCheck) {
  var state = stream._readableState;
  if (chunk === null) {
    state.reading = false;
    onEofChunk(stream, state);
  } else {
    var er;
    if (!skipChunkCheck) er = chunkInvalid(state, chunk);
    if (er) {
      stream.emit('error', er);
    } else if (state.objectMode || chunk && chunk.length > 0) {
      if (typeof chunk !== 'string' && !state.objectMode && Object.getPrototypeOf(chunk) !== Buffer.prototype) {
        chunk = _uint8ArrayToBuffer(chunk);
      }

      if (addToFront) {
        if (state.endEmitted) stream.emit('error', new Error('stream.unshift() after end event'));else addChunk(stream, state, chunk, true);
      } else if (state.ended) {
        stream.emit('error', new Error('stream.push() after EOF'));
      } else {
        state.reading = false;
        if (state.decoder && !encoding) {
          chunk = state.decoder.write(chunk);
          if (state.objectMode || chunk.length !== 0) addChunk(stream, state, chunk, false);else maybeReadMore(stream, state);
        } else {
          addChunk(stream, state, chunk, false);
        }
      }
    } else if (!addToFront) {
      state.reading = false;
    }
  }

  return needMoreData(state);
}

function addChunk(stream, state, chunk, addToFront) {
  if (state.flowing && state.length === 0 && !state.sync) {
    stream.emit('data', chunk);
    stream.read(0);
  } else {
    // update the buffer info.
    state.length += state.objectMode ? 1 : chunk.length;
    if (addToFront) state.buffer.unshift(chunk);else state.buffer.push(chunk);

    if (state.needReadable) emitReadable(stream);
  }
  maybeReadMore(stream, state);
}

function chunkInvalid(state, chunk) {
  var er;
  if (!_isUint8Array(chunk) && typeof chunk !== 'string' && chunk !== undefined && !state.objectMode) {
    er = new TypeError('Invalid non-string/buffer chunk');
  }
  return er;
}

// if it's past the high water mark, we can push in some more.
// Also, if we have no data yet, we can stand some
// more bytes.  This is to work around cases where hwm=0,
// such as the repl.  Also, if the push() triggered a
// readable event, and the user called read(largeNumber) such that
// needReadable was set, then we ought to push more, so that another
// 'readable' event will be triggered.
function needMoreData(state) {
  return !state.ended && (state.needReadable || state.length < state.highWaterMark || state.length === 0);
}

Readable.prototype.isPaused = function () {
  return this._readableState.flowing === false;
};

// backwards compatibility.
Readable.prototype.setEncoding = function (enc) {
  if (!StringDecoder) StringDecoder = _dereq_('string_decoder/').StringDecoder;
  this._readableState.decoder = new StringDecoder(enc);
  this._readableState.encoding = enc;
  return this;
};

// Don't raise the hwm > 8MB
var MAX_HWM = 0x800000;
function computeNewHighWaterMark(n) {
  if (n >= MAX_HWM) {
    n = MAX_HWM;
  } else {
    // Get the next highest power of 2 to prevent increasing hwm excessively in
    // tiny amounts
    n--;
    n |= n >>> 1;
    n |= n >>> 2;
    n |= n >>> 4;
    n |= n >>> 8;
    n |= n >>> 16;
    n++;
  }
  return n;
}

// This function is designed to be inlinable, so please take care when making
// changes to the function body.
function howMuchToRead(n, state) {
  if (n <= 0 || state.length === 0 && state.ended) return 0;
  if (state.objectMode) return 1;
  if (n !== n) {
    // Only flow one buffer at a time
    if (state.flowing && state.length) return state.buffer.head.data.length;else return state.length;
  }
  // If we're asking for more than the current hwm, then raise the hwm.
  if (n > state.highWaterMark) state.highWaterMark = computeNewHighWaterMark(n);
  if (n <= state.length) return n;
  // Don't have enough
  if (!state.ended) {
    state.needReadable = true;
    return 0;
  }
  return state.length;
}

// you can override either this method, or the async _read(n) below.
Readable.prototype.read = function (n) {
  debug('read', n);
  n = parseInt(n, 10);
  var state = this._readableState;
  var nOrig = n;

  if (n !== 0) state.emittedReadable = false;

  // if we're doing read(0) to trigger a readable event, but we
  // already have a bunch of data in the buffer, then just trigger
  // the 'readable' event and move on.
  if (n === 0 && state.needReadable && (state.length >= state.highWaterMark || state.ended)) {
    debug('read: emitReadable', state.length, state.ended);
    if (state.length === 0 && state.ended) endReadable(this);else emitReadable(this);
    return null;
  }

  n = howMuchToRead(n, state);

  // if we've ended, and we're now clear, then finish it up.
  if (n === 0 && state.ended) {
    if (state.length === 0) endReadable(this);
    return null;
  }

  // All the actual chunk generation logic needs to be
  // *below* the call to _read.  The reason is that in certain
  // synthetic stream cases, such as passthrough streams, _read
  // may be a completely synchronous operation which may change
  // the state of the read buffer, providing enough data when
  // before there was *not* enough.
  //
  // So, the steps are:
  // 1. Figure out what the state of things will be after we do
  // a read from the buffer.
  //
  // 2. If that resulting state will trigger a _read, then call _read.
  // Note that this may be asynchronous, or synchronous.  Yes, it is
  // deeply ugly to write APIs this way, but that still doesn't mean
  // that the Readable class should behave improperly, as streams are
  // designed to be sync/async agnostic.
  // Take note if the _read call is sync or async (ie, if the read call
  // has returned yet), so that we know whether or not it's safe to emit
  // 'readable' etc.
  //
  // 3. Actually pull the requested chunks out of the buffer and return.

  // if we need a readable event, then we need to do some reading.
  var doRead = state.needReadable;
  debug('need readable', doRead);

  // if we currently have less than the highWaterMark, then also read some
  if (state.length === 0 || state.length - n < state.highWaterMark) {
    doRead = true;
    debug('length less than watermark', doRead);
  }

  // however, if we've ended, then there's no point, and if we're already
  // reading, then it's unnecessary.
  if (state.ended || state.reading) {
    doRead = false;
    debug('reading or ended', doRead);
  } else if (doRead) {
    debug('do read');
    state.reading = true;
    state.sync = true;
    // if the length is currently zero, then we *need* a readable event.
    if (state.length === 0) state.needReadable = true;
    // call internal read method
    this._read(state.highWaterMark);
    state.sync = false;
    // If _read pushed data synchronously, then `reading` will be false,
    // and we need to re-evaluate how much data we can return to the user.
    if (!state.reading) n = howMuchToRead(nOrig, state);
  }

  var ret;
  if (n > 0) ret = fromList(n, state);else ret = null;

  if (ret === null) {
    state.needReadable = true;
    n = 0;
  } else {
    state.length -= n;
  }

  if (state.length === 0) {
    // If we have nothing in the buffer, then we want to know
    // as soon as we *do* get something into the buffer.
    if (!state.ended) state.needReadable = true;

    // If we tried to read() past the EOF, then emit end on the next tick.
    if (nOrig !== n && state.ended) endReadable(this);
  }

  if (ret !== null) this.emit('data', ret);

  return ret;
};

function onEofChunk(stream, state) {
  if (state.ended) return;
  if (state.decoder) {
    var chunk = state.decoder.end();
    if (chunk && chunk.length) {
      state.buffer.push(chunk);
      state.length += state.objectMode ? 1 : chunk.length;
    }
  }
  state.ended = true;

  // emit 'readable' now to make sure it gets picked up.
  emitReadable(stream);
}

// Don't emit readable right away in sync mode, because this can trigger
// another read() call => stack overflow.  This way, it might trigger
// a nextTick recursion warning, but that's not so bad.
function emitReadable(stream) {
  var state = stream._readableState;
  state.needReadable = false;
  if (!state.emittedReadable) {
    debug('emitReadable', state.flowing);
    state.emittedReadable = true;
    if (state.sync) pna.nextTick(emitReadable_, stream);else emitReadable_(stream);
  }
}

function emitReadable_(stream) {
  debug('emit readable');
  stream.emit('readable');
  flow(stream);
}

// at this point, the user has presumably seen the 'readable' event,
// and called read() to consume some data.  that may have triggered
// in turn another _read(n) call, in which case reading = true if
// it's in progress.
// However, if we're not ended, or reading, and the length < hwm,
// then go ahead and try to read some more preemptively.
function maybeReadMore(stream, state) {
  if (!state.readingMore) {
    state.readingMore = true;
    pna.nextTick(maybeReadMore_, stream, state);
  }
}

function maybeReadMore_(stream, state) {
  var len = state.length;
  while (!state.reading && !state.flowing && !state.ended && state.length < state.highWaterMark) {
    debug('maybeReadMore read 0');
    stream.read(0);
    if (len === state.length)
      // didn't get any data, stop spinning.
      break;else len = state.length;
  }
  state.readingMore = false;
}

// abstract method.  to be overridden in specific implementation classes.
// call cb(er, data) where data is <= n in length.
// for virtual (non-string, non-buffer) streams, "length" is somewhat
// arbitrary, and perhaps not very meaningful.
Readable.prototype._read = function (n) {
  this.emit('error', new Error('_read() is not implemented'));
};

Readable.prototype.pipe = function (dest, pipeOpts) {
  var src = this;
  var state = this._readableState;

  switch (state.pipesCount) {
    case 0:
      state.pipes = dest;
      break;
    case 1:
      state.pipes = [state.pipes, dest];
      break;
    default:
      state.pipes.push(dest);
      break;
  }
  state.pipesCount += 1;
  debug('pipe count=%d opts=%j', state.pipesCount, pipeOpts);

  var doEnd = (!pipeOpts || pipeOpts.end !== false) && dest !== process.stdout && dest !== process.stderr;

  var endFn = doEnd ? onend : unpipe;
  if (state.endEmitted) pna.nextTick(endFn);else src.once('end', endFn);

  dest.on('unpipe', onunpipe);
  function onunpipe(readable, unpipeInfo) {
    debug('onunpipe');
    if (readable === src) {
      if (unpipeInfo && unpipeInfo.hasUnpiped === false) {
        unpipeInfo.hasUnpiped = true;
        cleanup();
      }
    }
  }

  function onend() {
    debug('onend');
    dest.end();
  }

  // when the dest drains, it reduces the awaitDrain counter
  // on the source.  This would be more elegant with a .once()
  // handler in flow(), but adding and removing repeatedly is
  // too slow.
  var ondrain = pipeOnDrain(src);
  dest.on('drain', ondrain);

  var cleanedUp = false;
  function cleanup() {
    debug('cleanup');
    // cleanup event handlers once the pipe is broken
    dest.removeListener('close', onclose);
    dest.removeListener('finish', onfinish);
    dest.removeListener('drain', ondrain);
    dest.removeListener('error', onerror);
    dest.removeListener('unpipe', onunpipe);
    src.removeListener('end', onend);
    src.removeListener('end', unpipe);
    src.removeListener('data', ondata);

    cleanedUp = true;

    // if the reader is waiting for a drain event from this
    // specific writer, then it would cause it to never start
    // flowing again.
    // So, if this is awaiting a drain, then we just call it now.
    // If we don't know, then assume that we are waiting for one.
    if (state.awaitDrain && (!dest._writableState || dest._writableState.needDrain)) ondrain();
  }

  // If the user pushes more data while we're writing to dest then we'll end up
  // in ondata again. However, we only want to increase awaitDrain once because
  // dest will only emit one 'drain' event for the multiple writes.
  // => Introduce a guard on increasing awaitDrain.
  var increasedAwaitDrain = false;
  src.on('data', ondata);
  function ondata(chunk) {
    debug('ondata');
    increasedAwaitDrain = false;
    var ret = dest.write(chunk);
    if (false === ret && !increasedAwaitDrain) {
      // If the user unpiped during `dest.write()`, it is possible
      // to get stuck in a permanently paused state if that write
      // also returned false.
      // => Check whether `dest` is still a piping destination.
      if ((state.pipesCount === 1 && state.pipes === dest || state.pipesCount > 1 && indexOf(state.pipes, dest) !== -1) && !cleanedUp) {
        debug('false write response, pause', src._readableState.awaitDrain);
        src._readableState.awaitDrain++;
        increasedAwaitDrain = true;
      }
      src.pause();
    }
  }

  // if the dest has an error, then stop piping into it.
  // however, don't suppress the throwing behavior for this.
  function onerror(er) {
    debug('onerror', er);
    unpipe();
    dest.removeListener('error', onerror);
    if (EElistenerCount(dest, 'error') === 0) dest.emit('error', er);
  }

  // Make sure our error handler is attached before userland ones.
  prependListener(dest, 'error', onerror);

  // Both close and finish should trigger unpipe, but only once.
  function onclose() {
    dest.removeListener('finish', onfinish);
    unpipe();
  }
  dest.once('close', onclose);
  function onfinish() {
    debug('onfinish');
    dest.removeListener('close', onclose);
    unpipe();
  }
  dest.once('finish', onfinish);

  function unpipe() {
    debug('unpipe');
    src.unpipe(dest);
  }

  // tell the dest that it's being piped to
  dest.emit('pipe', src);

  // start the flow if it hasn't been started already.
  if (!state.flowing) {
    debug('pipe resume');
    src.resume();
  }

  return dest;
};

function pipeOnDrain(src) {
  return function () {
    var state = src._readableState;
    debug('pipeOnDrain', state.awaitDrain);
    if (state.awaitDrain) state.awaitDrain--;
    if (state.awaitDrain === 0 && EElistenerCount(src, 'data')) {
      state.flowing = true;
      flow(src);
    }
  };
}

Readable.prototype.unpipe = function (dest) {
  var state = this._readableState;
  var unpipeInfo = { hasUnpiped: false };

  // if we're not piping anywhere, then do nothing.
  if (state.pipesCount === 0) return this;

  // just one destination.  most common case.
  if (state.pipesCount === 1) {
    // passed in one, but it's not the right one.
    if (dest && dest !== state.pipes) return this;

    if (!dest) dest = state.pipes;

    // got a match.
    state.pipes = null;
    state.pipesCount = 0;
    state.flowing = false;
    if (dest) dest.emit('unpipe', this, unpipeInfo);
    return this;
  }

  // slow case. multiple pipe destinations.

  if (!dest) {
    // remove all.
    var dests = state.pipes;
    var len = state.pipesCount;
    state.pipes = null;
    state.pipesCount = 0;
    state.flowing = false;

    for (var i = 0; i < len; i++) {
      dests[i].emit('unpipe', this, unpipeInfo);
    }return this;
  }

  // try to find the right one.
  var index = indexOf(state.pipes, dest);
  if (index === -1) return this;

  state.pipes.splice(index, 1);
  state.pipesCount -= 1;
  if (state.pipesCount === 1) state.pipes = state.pipes[0];

  dest.emit('unpipe', this, unpipeInfo);

  return this;
};

// set up data events if they are asked for
// Ensure readable listeners eventually get something
Readable.prototype.on = function (ev, fn) {
  var res = Stream.prototype.on.call(this, ev, fn);

  if (ev === 'data') {
    // Start flowing on next tick if stream isn't explicitly paused
    if (this._readableState.flowing !== false) this.resume();
  } else if (ev === 'readable') {
    var state = this._readableState;
    if (!state.endEmitted && !state.readableListening) {
      state.readableListening = state.needReadable = true;
      state.emittedReadable = false;
      if (!state.reading) {
        pna.nextTick(nReadingNextTick, this);
      } else if (state.length) {
        emitReadable(this);
      }
    }
  }

  return res;
};
Readable.prototype.addListener = Readable.prototype.on;

function nReadingNextTick(self) {
  debug('readable nexttick read 0');
  self.read(0);
}

// pause() and resume() are remnants of the legacy readable stream API
// If the user uses them, then switch into old mode.
Readable.prototype.resume = function () {
  var state = this._readableState;
  if (!state.flowing) {
    debug('resume');
    state.flowing = true;
    resume(this, state);
  }
  return this;
};

function resume(stream, state) {
  if (!state.resumeScheduled) {
    state.resumeScheduled = true;
    pna.nextTick(resume_, stream, state);
  }
}

function resume_(stream, state) {
  if (!state.reading) {
    debug('resume read 0');
    stream.read(0);
  }

  state.resumeScheduled = false;
  state.awaitDrain = 0;
  stream.emit('resume');
  flow(stream);
  if (state.flowing && !state.reading) stream.read(0);
}

Readable.prototype.pause = function () {
  debug('call pause flowing=%j', this._readableState.flowing);
  if (false !== this._readableState.flowing) {
    debug('pause');
    this._readableState.flowing = false;
    this.emit('pause');
  }
  return this;
};

function flow(stream) {
  var state = stream._readableState;
  debug('flow', state.flowing);
  while (state.flowing && stream.read() !== null) {}
}

// wrap an old-style stream as the async data source.
// This is *not* part of the readable stream interface.
// It is an ugly unfortunate mess of history.
Readable.prototype.wrap = function (stream) {
  var _this = this;

  var state = this._readableState;
  var paused = false;

  stream.on('end', function () {
    debug('wrapped end');
    if (state.decoder && !state.ended) {
      var chunk = state.decoder.end();
      if (chunk && chunk.length) _this.push(chunk);
    }

    _this.push(null);
  });

  stream.on('data', function (chunk) {
    debug('wrapped data');
    if (state.decoder) chunk = state.decoder.write(chunk);

    // don't skip over falsy values in objectMode
    if (state.objectMode && (chunk === null || chunk === undefined)) return;else if (!state.objectMode && (!chunk || !chunk.length)) return;

    var ret = _this.push(chunk);
    if (!ret) {
      paused = true;
      stream.pause();
    }
  });

  // proxy all the other methods.
  // important when wrapping filters and duplexes.
  for (var i in stream) {
    if (this[i] === undefined && typeof stream[i] === 'function') {
      this[i] = function (method) {
        return function () {
          return stream[method].apply(stream, arguments);
        };
      }(i);
    }
  }

  // proxy certain important events.
  for (var n = 0; n < kProxyEvents.length; n++) {
    stream.on(kProxyEvents[n], this.emit.bind(this, kProxyEvents[n]));
  }

  // when we try to consume some more bytes, simply unpause the
  // underlying stream.
  this._read = function (n) {
    debug('wrapped _read', n);
    if (paused) {
      paused = false;
      stream.resume();
    }
  };

  return this;
};

Object.defineProperty(Readable.prototype, 'readableHighWaterMark', {
  // making it explicit this property is not enumerable
  // because otherwise some prototype manipulation in
  // userland will fail
  enumerable: false,
  get: function () {
    return this._readableState.highWaterMark;
  }
});

// exposed for testing purposes only.
Readable._fromList = fromList;

// Pluck off n bytes from an array of buffers.
// Length is the combined lengths of all the buffers in the list.
// This function is designed to be inlinable, so please take care when making
// changes to the function body.
function fromList(n, state) {
  // nothing buffered
  if (state.length === 0) return null;

  var ret;
  if (state.objectMode) ret = state.buffer.shift();else if (!n || n >= state.length) {
    // read it all, truncate the list
    if (state.decoder) ret = state.buffer.join('');else if (state.buffer.length === 1) ret = state.buffer.head.data;else ret = state.buffer.concat(state.length);
    state.buffer.clear();
  } else {
    // read part of list
    ret = fromListPartial(n, state.buffer, state.decoder);
  }

  return ret;
}

// Extracts only enough buffered data to satisfy the amount requested.
// This function is designed to be inlinable, so please take care when making
// changes to the function body.
function fromListPartial(n, list, hasStrings) {
  var ret;
  if (n < list.head.data.length) {
    // slice is the same for buffers and strings
    ret = list.head.data.slice(0, n);
    list.head.data = list.head.data.slice(n);
  } else if (n === list.head.data.length) {
    // first chunk is a perfect match
    ret = list.shift();
  } else {
    // result spans more than one buffer
    ret = hasStrings ? copyFromBufferString(n, list) : copyFromBuffer(n, list);
  }
  return ret;
}

// Copies a specified amount of characters from the list of buffered data
// chunks.
// This function is designed to be inlinable, so please take care when making
// changes to the function body.
function copyFromBufferString(n, list) {
  var p = list.head;
  var c = 1;
  var ret = p.data;
  n -= ret.length;
  while (p = p.next) {
    var str = p.data;
    var nb = n > str.length ? str.length : n;
    if (nb === str.length) ret += str;else ret += str.slice(0, n);
    n -= nb;
    if (n === 0) {
      if (nb === str.length) {
        ++c;
        if (p.next) list.head = p.next;else list.head = list.tail = null;
      } else {
        list.head = p;
        p.data = str.slice(nb);
      }
      break;
    }
    ++c;
  }
  list.length -= c;
  return ret;
}

// Copies a specified amount of bytes from the list of buffered data chunks.
// This function is designed to be inlinable, so please take care when making
// changes to the function body.
function copyFromBuffer(n, list) {
  var ret = Buffer.allocUnsafe(n);
  var p = list.head;
  var c = 1;
  p.data.copy(ret);
  n -= p.data.length;
  while (p = p.next) {
    var buf = p.data;
    var nb = n > buf.length ? buf.length : n;
    buf.copy(ret, ret.length - n, 0, nb);
    n -= nb;
    if (n === 0) {
      if (nb === buf.length) {
        ++c;
        if (p.next) list.head = p.next;else list.head = list.tail = null;
      } else {
        list.head = p;
        p.data = buf.slice(nb);
      }
      break;
    }
    ++c;
  }
  list.length -= c;
  return ret;
}

function endReadable(stream) {
  var state = stream._readableState;

  // If we get here before consuming all the bytes, then that is a
  // bug in node.  Should never happen.
  if (state.length > 0) throw new Error('"endReadable()" called on non-empty stream');

  if (!state.endEmitted) {
    state.ended = true;
    pna.nextTick(endReadableNT, state, stream);
  }
}

function endReadableNT(state, stream) {
  // Check that we didn't get one last unshift.
  if (!state.endEmitted && state.length === 0) {
    state.endEmitted = true;
    stream.readable = false;
    stream.emit('end');
  }
}

function indexOf(xs, x) {
  for (var i = 0, l = xs.length; i < l; i++) {
    if (xs[i] === x) return i;
  }
  return -1;
}
}).call(this,_dereq_('_process'),typeof global !== "undefined" ? global : typeof self !== "undefined" ? self : typeof window !== "undefined" ? window : {})
},{"./_stream_duplex":21,"./internal/streams/BufferList":26,"./internal/streams/destroy":27,"./internal/streams/stream":28,"_process":19,"core-util-is":10,"events":12,"inherits":14,"isarray":16,"process-nextick-args":18,"safe-buffer":33,"string_decoder/":35,"util":7}],24:[function(_dereq_,module,exports){
// Copyright Joyent, Inc. and other Node contributors.
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to permit
// persons to whom the Software is furnished to do so, subject to the
// following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
// NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
// USE OR OTHER DEALINGS IN THE SOFTWARE.

// a transform stream is a readable/writable stream where you do
// something with the data.  Sometimes it's called a "filter",
// but that's not a great name for it, since that implies a thing where
// some bits pass through, and others are simply ignored.  (That would
// be a valid example of a transform, of course.)
//
// While the output is causally related to the input, it's not a
// necessarily symmetric or synchronous transformation.  For example,
// a zlib stream might take multiple plain-text writes(), and then
// emit a single compressed chunk some time in the future.
//
// Here's how this works:
//
// The Transform stream has all the aspects of the readable and writable
// stream classes.  When you write(chunk), that calls _write(chunk,cb)
// internally, and returns false if there's a lot of pending writes
// buffered up.  When you call read(), that calls _read(n) until
// there's enough pending readable data buffered up.
//
// In a transform stream, the written data is placed in a buffer.  When
// _read(n) is called, it transforms the queued up data, calling the
// buffered _write cb's as it consumes chunks.  If consuming a single
// written chunk would result in multiple output chunks, then the first
// outputted bit calls the readcb, and subsequent chunks just go into
// the read buffer, and will cause it to emit 'readable' if necessary.
//
// This way, back-pressure is actually determined by the reading side,
// since _read has to be called to start processing a new chunk.  However,
// a pathological inflate type of transform can cause excessive buffering
// here.  For example, imagine a stream where every byte of input is
// interpreted as an integer from 0-255, and then results in that many
// bytes of output.  Writing the 4 bytes {ff,ff,ff,ff} would result in
// 1kb of data being output.  In this case, you could write a very small
// amount of input, and end up with a very large amount of output.  In
// such a pathological inflating mechanism, there'd be no way to tell
// the system to stop doing the transform.  A single 4MB write could
// cause the system to run out of memory.
//
// However, even in such a pathological case, only a single written chunk
// would be consumed, and then the rest would wait (un-transformed) until
// the results of the previous transformed chunk were consumed.

'use strict';

module.exports = Transform;

var Duplex = _dereq_('./_stream_duplex');

/*<replacement>*/
var util = _dereq_('core-util-is');
util.inherits = _dereq_('inherits');
/*</replacement>*/

util.inherits(Transform, Duplex);

function afterTransform(er, data) {
  var ts = this._transformState;
  ts.transforming = false;

  var cb = ts.writecb;

  if (!cb) {
    return this.emit('error', new Error('write callback called multiple times'));
  }

  ts.writechunk = null;
  ts.writecb = null;

  if (data != null) // single equals check for both `null` and `undefined`
    this.push(data);

  cb(er);

  var rs = this._readableState;
  rs.reading = false;
  if (rs.needReadable || rs.length < rs.highWaterMark) {
    this._read(rs.highWaterMark);
  }
}

function Transform(options) {
  if (!(this instanceof Transform)) return new Transform(options);

  Duplex.call(this, options);

  this._transformState = {
    afterTransform: afterTransform.bind(this),
    needTransform: false,
    transforming: false,
    writecb: null,
    writechunk: null,
    writeencoding: null
  };

  // start out asking for a readable event once data is transformed.
  this._readableState.needReadable = true;

  // we have implemented the _read method, and done the other things
  // that Readable wants before the first _read call, so unset the
  // sync guard flag.
  this._readableState.sync = false;

  if (options) {
    if (typeof options.transform === 'function') this._transform = options.transform;

    if (typeof options.flush === 'function') this._flush = options.flush;
  }

  // When the writable side finishes, then flush out anything remaining.
  this.on('prefinish', prefinish);
}

function prefinish() {
  var _this = this;

  if (typeof this._flush === 'function') {
    this._flush(function (er, data) {
      done(_this, er, data);
    });
  } else {
    done(this, null, null);
  }
}

Transform.prototype.push = function (chunk, encoding) {
  this._transformState.needTransform = false;
  return Duplex.prototype.push.call(this, chunk, encoding);
};

// This is the part where you do stuff!
// override this function in implementation classes.
// 'chunk' is an input chunk.
//
// Call `push(newChunk)` to pass along transformed output
// to the readable side.  You may call 'push' zero or more times.
//
// Call `cb(err)` when you are done with this chunk.  If you pass
// an error, then that'll put the hurt on the whole operation.  If you
// never call cb(), then you'll never get another chunk.
Transform.prototype._transform = function (chunk, encoding, cb) {
  throw new Error('_transform() is not implemented');
};

Transform.prototype._write = function (chunk, encoding, cb) {
  var ts = this._transformState;
  ts.writecb = cb;
  ts.writechunk = chunk;
  ts.writeencoding = encoding;
  if (!ts.transforming) {
    var rs = this._readableState;
    if (ts.needTransform || rs.needReadable || rs.length < rs.highWaterMark) this._read(rs.highWaterMark);
  }
};

// Doesn't matter what the args are here.
// _transform does all the work.
// That we got here means that the readable side wants more data.
Transform.prototype._read = function (n) {
  var ts = this._transformState;

  if (ts.writechunk !== null && ts.writecb && !ts.transforming) {
    ts.transforming = true;
    this._transform(ts.writechunk, ts.writeencoding, ts.afterTransform);
  } else {
    // mark that we need a transform, so that any data that comes in
    // will get processed, now that we've asked for it.
    ts.needTransform = true;
  }
};

Transform.prototype._destroy = function (err, cb) {
  var _this2 = this;

  Duplex.prototype._destroy.call(this, err, function (err2) {
    cb(err2);
    _this2.emit('close');
  });
};

function done(stream, er, data) {
  if (er) return stream.emit('error', er);

  if (data != null) // single equals check for both `null` and `undefined`
    stream.push(data);

  // if there's nothing in the write buffer, then that means
  // that nothing more will ever be provided
  if (stream._writableState.length) throw new Error('Calling transform done when ws.length != 0');

  if (stream._transformState.transforming) throw new Error('Calling transform done when still transforming');

  return stream.push(null);
}
},{"./_stream_duplex":21,"core-util-is":10,"inherits":14}],25:[function(_dereq_,module,exports){
(function (process,global,setImmediate){
// Copyright Joyent, Inc. and other Node contributors.
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to permit
// persons to whom the Software is furnished to do so, subject to the
// following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
// NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
// USE OR OTHER DEALINGS IN THE SOFTWARE.

// A bit simpler than readable streams.
// Implement an async ._write(chunk, encoding, cb), and it'll handle all
// the drain event emission and buffering.

'use strict';

/*<replacement>*/

var pna = _dereq_('process-nextick-args');
/*</replacement>*/

module.exports = Writable;

/* <replacement> */
function WriteReq(chunk, encoding, cb) {
  this.chunk = chunk;
  this.encoding = encoding;
  this.callback = cb;
  this.next = null;
}

// It seems a linked list but it is not
// there will be only 2 of these for each stream
function CorkedRequest(state) {
  var _this = this;

  this.next = null;
  this.entry = null;
  this.finish = function () {
    onCorkedFinish(_this, state);
  };
}
/* </replacement> */

/*<replacement>*/
var asyncWrite = !process.browser && ['v0.10', 'v0.9.'].indexOf(process.version.slice(0, 5)) > -1 ? setImmediate : pna.nextTick;
/*</replacement>*/

/*<replacement>*/
var Duplex;
/*</replacement>*/

Writable.WritableState = WritableState;

/*<replacement>*/
var util = _dereq_('core-util-is');
util.inherits = _dereq_('inherits');
/*</replacement>*/

/*<replacement>*/
var internalUtil = {
  deprecate: _dereq_('util-deprecate')
};
/*</replacement>*/

/*<replacement>*/
var Stream = _dereq_('./internal/streams/stream');
/*</replacement>*/

/*<replacement>*/

var Buffer = _dereq_('safe-buffer').Buffer;
var OurUint8Array = global.Uint8Array || function () {};
function _uint8ArrayToBuffer(chunk) {
  return Buffer.from(chunk);
}
function _isUint8Array(obj) {
  return Buffer.isBuffer(obj) || obj instanceof OurUint8Array;
}

/*</replacement>*/

var destroyImpl = _dereq_('./internal/streams/destroy');

util.inherits(Writable, Stream);

function nop() {}

function WritableState(options, stream) {
  Duplex = Duplex || _dereq_('./_stream_duplex');

  options = options || {};

  // Duplex streams are both readable and writable, but share
  // the same options object.
  // However, some cases require setting options to different
  // values for the readable and the writable sides of the duplex stream.
  // These options can be provided separately as readableXXX and writableXXX.
  var isDuplex = stream instanceof Duplex;

  // object stream flag to indicate whether or not this stream
  // contains buffers or objects.
  this.objectMode = !!options.objectMode;

  if (isDuplex) this.objectMode = this.objectMode || !!options.writableObjectMode;

  // the point at which write() starts returning false
  // Note: 0 is a valid value, means that we always return false if
  // the entire buffer is not flushed immediately on write()
  var hwm = options.highWaterMark;
  var writableHwm = options.writableHighWaterMark;
  var defaultHwm = this.objectMode ? 16 : 16 * 1024;

  if (hwm || hwm === 0) this.highWaterMark = hwm;else if (isDuplex && (writableHwm || writableHwm === 0)) this.highWaterMark = writableHwm;else this.highWaterMark = defaultHwm;

  // cast to ints.
  this.highWaterMark = Math.floor(this.highWaterMark);

  // if _final has been called
  this.finalCalled = false;

  // drain event flag.
  this.needDrain = false;
  // at the start of calling end()
  this.ending = false;
  // when end() has been called, and returned
  this.ended = false;
  // when 'finish' is emitted
  this.finished = false;

  // has it been destroyed
  this.destroyed = false;

  // should we decode strings into buffers before passing to _write?
  // this is here so that some node-core streams can optimize string
  // handling at a lower level.
  var noDecode = options.decodeStrings === false;
  this.decodeStrings = !noDecode;

  // Crypto is kind of old and crusty.  Historically, its default string
  // encoding is 'binary' so we have to make this configurable.
  // Everything else in the universe uses 'utf8', though.
  this.defaultEncoding = options.defaultEncoding || 'utf8';

  // not an actual buffer we keep track of, but a measurement
  // of how much we're waiting to get pushed to some underlying
  // socket or file.
  this.length = 0;

  // a flag to see when we're in the middle of a write.
  this.writing = false;

  // when true all writes will be buffered until .uncork() call
  this.corked = 0;

  // a flag to be able to tell if the onwrite cb is called immediately,
  // or on a later tick.  We set this to true at first, because any
  // actions that shouldn't happen until "later" should generally also
  // not happen before the first write call.
  this.sync = true;

  // a flag to know if we're processing previously buffered items, which
  // may call the _write() callback in the same tick, so that we don't
  // end up in an overlapped onwrite situation.
  this.bufferProcessing = false;

  // the callback that's passed to _write(chunk,cb)
  this.onwrite = function (er) {
    onwrite(stream, er);
  };

  // the callback that the user supplies to write(chunk,encoding,cb)
  this.writecb = null;

  // the amount that is being written when _write is called.
  this.writelen = 0;

  this.bufferedRequest = null;
  this.lastBufferedRequest = null;

  // number of pending user-supplied write callbacks
  // this must be 0 before 'finish' can be emitted
  this.pendingcb = 0;

  // emit prefinish if the only thing we're waiting for is _write cbs
  // This is relevant for synchronous Transform streams
  this.prefinished = false;

  // True if the error was already emitted and should not be thrown again
  this.errorEmitted = false;

  // count buffered requests
  this.bufferedRequestCount = 0;

  // allocate the first CorkedRequest, there is always
  // one allocated and free to use, and we maintain at most two
  this.corkedRequestsFree = new CorkedRequest(this);
}

WritableState.prototype.getBuffer = function getBuffer() {
  var current = this.bufferedRequest;
  var out = [];
  while (current) {
    out.push(current);
    current = current.next;
  }
  return out;
};

(function () {
  try {
    Object.defineProperty(WritableState.prototype, 'buffer', {
      get: internalUtil.deprecate(function () {
        return this.getBuffer();
      }, '_writableState.buffer is deprecated. Use _writableState.getBuffer ' + 'instead.', 'DEP0003')
    });
  } catch (_) {}
})();

// Test _writableState for inheritance to account for Duplex streams,
// whose prototype chain only points to Readable.
var realHasInstance;
if (typeof Symbol === 'function' && Symbol.hasInstance && typeof Function.prototype[Symbol.hasInstance] === 'function') {
  realHasInstance = Function.prototype[Symbol.hasInstance];
  Object.defineProperty(Writable, Symbol.hasInstance, {
    value: function (object) {
      if (realHasInstance.call(this, object)) return true;
      if (this !== Writable) return false;

      return object && object._writableState instanceof WritableState;
    }
  });
} else {
  realHasInstance = function (object) {
    return object instanceof this;
  };
}

function Writable(options) {
  Duplex = Duplex || _dereq_('./_stream_duplex');

  // Writable ctor is applied to Duplexes, too.
  // `realHasInstance` is necessary because using plain `instanceof`
  // would return false, as no `_writableState` property is attached.

  // Trying to use the custom `instanceof` for Writable here will also break the
  // Node.js LazyTransform implementation, which has a non-trivial getter for
  // `_writableState` that would lead to infinite recursion.
  if (!realHasInstance.call(Writable, this) && !(this instanceof Duplex)) {
    return new Writable(options);
  }

  this._writableState = new WritableState(options, this);

  // legacy.
  this.writable = true;

  if (options) {
    if (typeof options.write === 'function') this._write = options.write;

    if (typeof options.writev === 'function') this._writev = options.writev;

    if (typeof options.destroy === 'function') this._destroy = options.destroy;

    if (typeof options.final === 'function') this._final = options.final;
  }

  Stream.call(this);
}

// Otherwise people can pipe Writable streams, which is just wrong.
Writable.prototype.pipe = function () {
  this.emit('error', new Error('Cannot pipe, not readable'));
};

function writeAfterEnd(stream, cb) {
  var er = new Error('write after end');
  // TODO: defer error events consistently everywhere, not just the cb
  stream.emit('error', er);
  pna.nextTick(cb, er);
}

// Checks that a user-supplied chunk is valid, especially for the particular
// mode the stream is in. Currently this means that `null` is never accepted
// and undefined/non-string values are only allowed in object mode.
function validChunk(stream, state, chunk, cb) {
  var valid = true;
  var er = false;

  if (chunk === null) {
    er = new TypeError('May not write null values to stream');
  } else if (typeof chunk !== 'string' && chunk !== undefined && !state.objectMode) {
    er = new TypeError('Invalid non-string/buffer chunk');
  }
  if (er) {
    stream.emit('error', er);
    pna.nextTick(cb, er);
    valid = false;
  }
  return valid;
}

Writable.prototype.write = function (chunk, encoding, cb) {
  var state = this._writableState;
  var ret = false;
  var isBuf = !state.objectMode && _isUint8Array(chunk);

  if (isBuf && !Buffer.isBuffer(chunk)) {
    chunk = _uint8ArrayToBuffer(chunk);
  }

  if (typeof encoding === 'function') {
    cb = encoding;
    encoding = null;
  }

  if (isBuf) encoding = 'buffer';else if (!encoding) encoding = state.defaultEncoding;

  if (typeof cb !== 'function') cb = nop;

  if (state.ended) writeAfterEnd(this, cb);else if (isBuf || validChunk(this, state, chunk, cb)) {
    state.pendingcb++;
    ret = writeOrBuffer(this, state, isBuf, chunk, encoding, cb);
  }

  return ret;
};

Writable.prototype.cork = function () {
  var state = this._writableState;

  state.corked++;
};

Writable.prototype.uncork = function () {
  var state = this._writableState;

  if (state.corked) {
    state.corked--;

    if (!state.writing && !state.corked && !state.finished && !state.bufferProcessing && state.bufferedRequest) clearBuffer(this, state);
  }
};

Writable.prototype.setDefaultEncoding = function setDefaultEncoding(encoding) {
  // node::ParseEncoding() requires lower case.
  if (typeof encoding === 'string') encoding = encoding.toLowerCase();
  if (!(['hex', 'utf8', 'utf-8', 'ascii', 'binary', 'base64', 'ucs2', 'ucs-2', 'utf16le', 'utf-16le', 'raw'].indexOf((encoding + '').toLowerCase()) > -1)) throw new TypeError('Unknown encoding: ' + encoding);
  this._writableState.defaultEncoding = encoding;
  return this;
};

function decodeChunk(state, chunk, encoding) {
  if (!state.objectMode && state.decodeStrings !== false && typeof chunk === 'string') {
    chunk = Buffer.from(chunk, encoding);
  }
  return chunk;
}

Object.defineProperty(Writable.prototype, 'writableHighWaterMark', {
  // making it explicit this property is not enumerable
  // because otherwise some prototype manipulation in
  // userland will fail
  enumerable: false,
  get: function () {
    return this._writableState.highWaterMark;
  }
});

// if we're already writing something, then just put this
// in the queue, and wait our turn.  Otherwise, call _write
// If we return false, then we need a drain event, so set that flag.
function writeOrBuffer(stream, state, isBuf, chunk, encoding, cb) {
  if (!isBuf) {
    var newChunk = decodeChunk(state, chunk, encoding);
    if (chunk !== newChunk) {
      isBuf = true;
      encoding = 'buffer';
      chunk = newChunk;
    }
  }
  var len = state.objectMode ? 1 : chunk.length;

  state.length += len;

  var ret = state.length < state.highWaterMark;
  // we must ensure that previous needDrain will not be reset to false.
  if (!ret) state.needDrain = true;

  if (state.writing || state.corked) {
    var last = state.lastBufferedRequest;
    state.lastBufferedRequest = {
      chunk: chunk,
      encoding: encoding,
      isBuf: isBuf,
      callback: cb,
      next: null
    };
    if (last) {
      last.next = state.lastBufferedRequest;
    } else {
      state.bufferedRequest = state.lastBufferedRequest;
    }
    state.bufferedRequestCount += 1;
  } else {
    doWrite(stream, state, false, len, chunk, encoding, cb);
  }

  return ret;
}

function doWrite(stream, state, writev, len, chunk, encoding, cb) {
  state.writelen = len;
  state.writecb = cb;
  state.writing = true;
  state.sync = true;
  if (writev) stream._writev(chunk, state.onwrite);else stream._write(chunk, encoding, state.onwrite);
  state.sync = false;
}

function onwriteError(stream, state, sync, er, cb) {
  --state.pendingcb;

  if (sync) {
    // defer the callback if we are being called synchronously
    // to avoid piling up things on the stack
    pna.nextTick(cb, er);
    // this can emit finish, and it will always happen
    // after error
    pna.nextTick(finishMaybe, stream, state);
    stream._writableState.errorEmitted = true;
    stream.emit('error', er);
  } else {
    // the caller expect this to happen before if
    // it is async
    cb(er);
    stream._writableState.errorEmitted = true;
    stream.emit('error', er);
    // this can emit finish, but finish must
    // always follow error
    finishMaybe(stream, state);
  }
}

function onwriteStateUpdate(state) {
  state.writing = false;
  state.writecb = null;
  state.length -= state.writelen;
  state.writelen = 0;
}

function onwrite(stream, er) {
  var state = stream._writableState;
  var sync = state.sync;
  var cb = state.writecb;

  onwriteStateUpdate(state);

  if (er) onwriteError(stream, state, sync, er, cb);else {
    // Check if we're actually ready to finish, but don't emit yet
    var finished = needFinish(state);

    if (!finished && !state.corked && !state.bufferProcessing && state.bufferedRequest) {
      clearBuffer(stream, state);
    }

    if (sync) {
      /*<replacement>*/
      asyncWrite(afterWrite, stream, state, finished, cb);
      /*</replacement>*/
    } else {
      afterWrite(stream, state, finished, cb);
    }
  }
}

function afterWrite(stream, state, finished, cb) {
  if (!finished) onwriteDrain(stream, state);
  state.pendingcb--;
  cb();
  finishMaybe(stream, state);
}

// Must force callback to be called on nextTick, so that we don't
// emit 'drain' before the write() consumer gets the 'false' return
// value, and has a chance to attach a 'drain' listener.
function onwriteDrain(stream, state) {
  if (state.length === 0 && state.needDrain) {
    state.needDrain = false;
    stream.emit('drain');
  }
}

// if there's something in the buffer waiting, then process it
function clearBuffer(stream, state) {
  state.bufferProcessing = true;
  var entry = state.bufferedRequest;

  if (stream._writev && entry && entry.next) {
    // Fast case, write everything using _writev()
    var l = state.bufferedRequestCount;
    var buffer = new Array(l);
    var holder = state.corkedRequestsFree;
    holder.entry = entry;

    var count = 0;
    var allBuffers = true;
    while (entry) {
      buffer[count] = entry;
      if (!entry.isBuf) allBuffers = false;
      entry = entry.next;
      count += 1;
    }
    buffer.allBuffers = allBuffers;

    doWrite(stream, state, true, state.length, buffer, '', holder.finish);

    // doWrite is almost always async, defer these to save a bit of time
    // as the hot path ends with doWrite
    state.pendingcb++;
    state.lastBufferedRequest = null;
    if (holder.next) {
      state.corkedRequestsFree = holder.next;
      holder.next = null;
    } else {
      state.corkedRequestsFree = new CorkedRequest(state);
    }
    state.bufferedRequestCount = 0;
  } else {
    // Slow case, write chunks one-by-one
    while (entry) {
      var chunk = entry.chunk;
      var encoding = entry.encoding;
      var cb = entry.callback;
      var len = state.objectMode ? 1 : chunk.length;

      doWrite(stream, state, false, len, chunk, encoding, cb);
      entry = entry.next;
      state.bufferedRequestCount--;
      // if we didn't call the onwrite immediately, then
      // it means that we need to wait until it does.
      // also, that means that the chunk and cb are currently
      // being processed, so move the buffer counter past them.
      if (state.writing) {
        break;
      }
    }

    if (entry === null) state.lastBufferedRequest = null;
  }

  state.bufferedRequest = entry;
  state.bufferProcessing = false;
}

Writable.prototype._write = function (chunk, encoding, cb) {
  cb(new Error('_write() is not implemented'));
};

Writable.prototype._writev = null;

Writable.prototype.end = function (chunk, encoding, cb) {
  var state = this._writableState;

  if (typeof chunk === 'function') {
    cb = chunk;
    chunk = null;
    encoding = null;
  } else if (typeof encoding === 'function') {
    cb = encoding;
    encoding = null;
  }

  if (chunk !== null && chunk !== undefined) this.write(chunk, encoding);

  // .end() fully uncorks
  if (state.corked) {
    state.corked = 1;
    this.uncork();
  }

  // ignore unnecessary end() calls.
  if (!state.ending && !state.finished) endWritable(this, state, cb);
};

function needFinish(state) {
  return state.ending && state.length === 0 && state.bufferedRequest === null && !state.finished && !state.writing;
}
function callFinal(stream, state) {
  stream._final(function (err) {
    state.pendingcb--;
    if (err) {
      stream.emit('error', err);
    }
    state.prefinished = true;
    stream.emit('prefinish');
    finishMaybe(stream, state);
  });
}
function prefinish(stream, state) {
  if (!state.prefinished && !state.finalCalled) {
    if (typeof stream._final === 'function') {
      state.pendingcb++;
      state.finalCalled = true;
      pna.nextTick(callFinal, stream, state);
    } else {
      state.prefinished = true;
      stream.emit('prefinish');
    }
  }
}

function finishMaybe(stream, state) {
  var need = needFinish(state);
  if (need) {
    prefinish(stream, state);
    if (state.pendingcb === 0) {
      state.finished = true;
      stream.emit('finish');
    }
  }
  return need;
}

function endWritable(stream, state, cb) {
  state.ending = true;
  finishMaybe(stream, state);
  if (cb) {
    if (state.finished) pna.nextTick(cb);else stream.once('finish', cb);
  }
  state.ended = true;
  stream.writable = false;
}

function onCorkedFinish(corkReq, state, err) {
  var entry = corkReq.entry;
  corkReq.entry = null;
  while (entry) {
    var cb = entry.callback;
    state.pendingcb--;
    cb(err);
    entry = entry.next;
  }
  if (state.corkedRequestsFree) {
    state.corkedRequestsFree.next = corkReq;
  } else {
    state.corkedRequestsFree = corkReq;
  }
}

Object.defineProperty(Writable.prototype, 'destroyed', {
  get: function () {
    if (this._writableState === undefined) {
      return false;
    }
    return this._writableState.destroyed;
  },
  set: function (value) {
    // we ignore the value if the stream
    // has not been initialized yet
    if (!this._writableState) {
      return;
    }

    // backward compatibility, the user is explicitly
    // managing destroyed
    this._writableState.destroyed = value;
  }
});

Writable.prototype.destroy = destroyImpl.destroy;
Writable.prototype._undestroy = destroyImpl.undestroy;
Writable.prototype._destroy = function (err, cb) {
  this.end();
  cb(err);
};
}).call(this,_dereq_('_process'),typeof global !== "undefined" ? global : typeof self !== "undefined" ? self : typeof window !== "undefined" ? window : {},_dereq_("timers").setImmediate)
},{"./_stream_duplex":21,"./internal/streams/destroy":27,"./internal/streams/stream":28,"_process":19,"core-util-is":10,"inherits":14,"process-nextick-args":18,"safe-buffer":33,"timers":36,"util-deprecate":37}],26:[function(_dereq_,module,exports){
'use strict';

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

var Buffer = _dereq_('safe-buffer').Buffer;
var util = _dereq_('util');

function copyBuffer(src, target, offset) {
  src.copy(target, offset);
}

module.exports = function () {
  function BufferList() {
    _classCallCheck(this, BufferList);

    this.head = null;
    this.tail = null;
    this.length = 0;
  }

  BufferList.prototype.push = function push(v) {
    var entry = { data: v, next: null };
    if (this.length > 0) this.tail.next = entry;else this.head = entry;
    this.tail = entry;
    ++this.length;
  };

  BufferList.prototype.unshift = function unshift(v) {
    var entry = { data: v, next: this.head };
    if (this.length === 0) this.tail = entry;
    this.head = entry;
    ++this.length;
  };

  BufferList.prototype.shift = function shift() {
    if (this.length === 0) return;
    var ret = this.head.data;
    if (this.length === 1) this.head = this.tail = null;else this.head = this.head.next;
    --this.length;
    return ret;
  };

  BufferList.prototype.clear = function clear() {
    this.head = this.tail = null;
    this.length = 0;
  };

  BufferList.prototype.join = function join(s) {
    if (this.length === 0) return '';
    var p = this.head;
    var ret = '' + p.data;
    while (p = p.next) {
      ret += s + p.data;
    }return ret;
  };

  BufferList.prototype.concat = function concat(n) {
    if (this.length === 0) return Buffer.alloc(0);
    if (this.length === 1) return this.head.data;
    var ret = Buffer.allocUnsafe(n >>> 0);
    var p = this.head;
    var i = 0;
    while (p) {
      copyBuffer(p.data, ret, i);
      i += p.data.length;
      p = p.next;
    }
    return ret;
  };

  return BufferList;
}();

if (util && util.inspect && util.inspect.custom) {
  module.exports.prototype[util.inspect.custom] = function () {
    var obj = util.inspect({ length: this.length });
    return this.constructor.name + ' ' + obj;
  };
}
},{"safe-buffer":33,"util":7}],27:[function(_dereq_,module,exports){
'use strict';

/*<replacement>*/

var pna = _dereq_('process-nextick-args');
/*</replacement>*/

// undocumented cb() API, needed for core, not for public API
function destroy(err, cb) {
  var _this = this;

  var readableDestroyed = this._readableState && this._readableState.destroyed;
  var writableDestroyed = this._writableState && this._writableState.destroyed;

  if (readableDestroyed || writableDestroyed) {
    if (cb) {
      cb(err);
    } else if (err && (!this._writableState || !this._writableState.errorEmitted)) {
      pna.nextTick(emitErrorNT, this, err);
    }
    return this;
  }

  // we set destroyed to true before firing error callbacks in order
  // to make it re-entrance safe in case destroy() is called within callbacks

  if (this._readableState) {
    this._readableState.destroyed = true;
  }

  // if this is a duplex stream mark the writable part as destroyed as well
  if (this._writableState) {
    this._writableState.destroyed = true;
  }

  this._destroy(err || null, function (err) {
    if (!cb && err) {
      pna.nextTick(emitErrorNT, _this, err);
      if (_this._writableState) {
        _this._writableState.errorEmitted = true;
      }
    } else if (cb) {
      cb(err);
    }
  });

  return this;
}

function undestroy() {
  if (this._readableState) {
    this._readableState.destroyed = false;
    this._readableState.reading = false;
    this._readableState.ended = false;
    this._readableState.endEmitted = false;
  }

  if (this._writableState) {
    this._writableState.destroyed = false;
    this._writableState.ended = false;
    this._writableState.ending = false;
    this._writableState.finished = false;
    this._writableState.errorEmitted = false;
  }
}

function emitErrorNT(self, err) {
  self.emit('error', err);
}

module.exports = {
  destroy: destroy,
  undestroy: undestroy
};
},{"process-nextick-args":18}],28:[function(_dereq_,module,exports){
module.exports = _dereq_('events').EventEmitter;

},{"events":12}],29:[function(_dereq_,module,exports){
module.exports = _dereq_('./readable').PassThrough

},{"./readable":30}],30:[function(_dereq_,module,exports){
exports = module.exports = _dereq_('./lib/_stream_readable.js');
exports.Stream = exports;
exports.Readable = exports;
exports.Writable = _dereq_('./lib/_stream_writable.js');
exports.Duplex = _dereq_('./lib/_stream_duplex.js');
exports.Transform = _dereq_('./lib/_stream_transform.js');
exports.PassThrough = _dereq_('./lib/_stream_passthrough.js');

},{"./lib/_stream_duplex.js":21,"./lib/_stream_passthrough.js":22,"./lib/_stream_readable.js":23,"./lib/_stream_transform.js":24,"./lib/_stream_writable.js":25}],31:[function(_dereq_,module,exports){
module.exports = _dereq_('./readable').Transform

},{"./readable":30}],32:[function(_dereq_,module,exports){
module.exports = _dereq_('./lib/_stream_writable.js');

},{"./lib/_stream_writable.js":25}],33:[function(_dereq_,module,exports){
/* eslint-disable node/no-deprecated-api */
var buffer = _dereq_('buffer')
var Buffer = buffer.Buffer

// alternative to using Object.keys for old browsers
function copyProps (src, dst) {
  for (var key in src) {
    dst[key] = src[key]
  }
}
if (Buffer.from && Buffer.alloc && Buffer.allocUnsafe && Buffer.allocUnsafeSlow) {
  module.exports = buffer
} else {
  // Copy properties from require('buffer')
  copyProps(buffer, exports)
  exports.Buffer = SafeBuffer
}

function SafeBuffer (arg, encodingOrOffset, length) {
  return Buffer(arg, encodingOrOffset, length)
}

// Copy static methods from Buffer
copyProps(Buffer, SafeBuffer)

SafeBuffer.from = function (arg, encodingOrOffset, length) {
  if (typeof arg === 'number') {
    throw new TypeError('Argument must not be a number')
  }
  return Buffer(arg, encodingOrOffset, length)
}

SafeBuffer.alloc = function (size, fill, encoding) {
  if (typeof size !== 'number') {
    throw new TypeError('Argument must be a number')
  }
  var buf = Buffer(size)
  if (fill !== undefined) {
    if (typeof encoding === 'string') {
      buf.fill(fill, encoding)
    } else {
      buf.fill(fill)
    }
  } else {
    buf.fill(0)
  }
  return buf
}

SafeBuffer.allocUnsafe = function (size) {
  if (typeof size !== 'number') {
    throw new TypeError('Argument must be a number')
  }
  return Buffer(size)
}

SafeBuffer.allocUnsafeSlow = function (size) {
  if (typeof size !== 'number') {
    throw new TypeError('Argument must be a number')
  }
  return buffer.SlowBuffer(size)
}

},{"buffer":8}],34:[function(_dereq_,module,exports){
// Copyright Joyent, Inc. and other Node contributors.
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to permit
// persons to whom the Software is furnished to do so, subject to the
// following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
// NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
// USE OR OTHER DEALINGS IN THE SOFTWARE.

module.exports = Stream;

var EE = _dereq_('events').EventEmitter;
var inherits = _dereq_('inherits');

inherits(Stream, EE);
Stream.Readable = _dereq_('readable-stream/readable.js');
Stream.Writable = _dereq_('readable-stream/writable.js');
Stream.Duplex = _dereq_('readable-stream/duplex.js');
Stream.Transform = _dereq_('readable-stream/transform.js');
Stream.PassThrough = _dereq_('readable-stream/passthrough.js');

// Backwards-compat with node 0.4.x
Stream.Stream = Stream;



// old-style streams.  Note that the pipe method (the only relevant
// part of this class) is overridden in the Readable class.

function Stream() {
  EE.call(this);
}

Stream.prototype.pipe = function(dest, options) {
  var source = this;

  function ondata(chunk) {
    if (dest.writable) {
      if (false === dest.write(chunk) && source.pause) {
        source.pause();
      }
    }
  }

  source.on('data', ondata);

  function ondrain() {
    if (source.readable && source.resume) {
      source.resume();
    }
  }

  dest.on('drain', ondrain);

  // If the 'end' option is not supplied, dest.end() will be called when
  // source gets the 'end' or 'close' events.  Only dest.end() once.
  if (!dest._isStdio && (!options || options.end !== false)) {
    source.on('end', onend);
    source.on('close', onclose);
  }

  var didOnEnd = false;
  function onend() {
    if (didOnEnd) return;
    didOnEnd = true;

    dest.end();
  }


  function onclose() {
    if (didOnEnd) return;
    didOnEnd = true;

    if (typeof dest.destroy === 'function') dest.destroy();
  }

  // don't leave dangling pipes when there are errors.
  function onerror(er) {
    cleanup();
    if (EE.listenerCount(this, 'error') === 0) {
      throw er; // Unhandled stream error in pipe.
    }
  }

  source.on('error', onerror);
  dest.on('error', onerror);

  // remove all the event listeners that were added.
  function cleanup() {
    source.removeListener('data', ondata);
    dest.removeListener('drain', ondrain);

    source.removeListener('end', onend);
    source.removeListener('close', onclose);

    source.removeListener('error', onerror);
    dest.removeListener('error', onerror);

    source.removeListener('end', cleanup);
    source.removeListener('close', cleanup);

    dest.removeListener('close', cleanup);
  }

  source.on('end', cleanup);
  source.on('close', cleanup);

  dest.on('close', cleanup);

  dest.emit('pipe', source);

  // Allow for unix-like usage: A.pipe(B).pipe(C)
  return dest;
};

},{"events":12,"inherits":14,"readable-stream/duplex.js":20,"readable-stream/passthrough.js":29,"readable-stream/readable.js":30,"readable-stream/transform.js":31,"readable-stream/writable.js":32}],35:[function(_dereq_,module,exports){
// Copyright Joyent, Inc. and other Node contributors.
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to permit
// persons to whom the Software is furnished to do so, subject to the
// following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
// NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
// USE OR OTHER DEALINGS IN THE SOFTWARE.

'use strict';

/*<replacement>*/

var Buffer = _dereq_('safe-buffer').Buffer;
/*</replacement>*/

var isEncoding = Buffer.isEncoding || function (encoding) {
  encoding = '' + encoding;
  switch (encoding && encoding.toLowerCase()) {
    case 'hex':case 'utf8':case 'utf-8':case 'ascii':case 'binary':case 'base64':case 'ucs2':case 'ucs-2':case 'utf16le':case 'utf-16le':case 'raw':
      return true;
    default:
      return false;
  }
};

function _normalizeEncoding(enc) {
  if (!enc) return 'utf8';
  var retried;
  while (true) {
    switch (enc) {
      case 'utf8':
      case 'utf-8':
        return 'utf8';
      case 'ucs2':
      case 'ucs-2':
      case 'utf16le':
      case 'utf-16le':
        return 'utf16le';
      case 'latin1':
      case 'binary':
        return 'latin1';
      case 'base64':
      case 'ascii':
      case 'hex':
        return enc;
      default:
        if (retried) return; // undefined
        enc = ('' + enc).toLowerCase();
        retried = true;
    }
  }
};

// Do not cache `Buffer.isEncoding` when checking encoding names as some
// modules monkey-patch it to support additional encodings
function normalizeEncoding(enc) {
  var nenc = _normalizeEncoding(enc);
  if (typeof nenc !== 'string' && (Buffer.isEncoding === isEncoding || !isEncoding(enc))) throw new Error('Unknown encoding: ' + enc);
  return nenc || enc;
}

// StringDecoder provides an interface for efficiently splitting a series of
// buffers into a series of JS strings without breaking apart multi-byte
// characters.
exports.StringDecoder = StringDecoder;
function StringDecoder(encoding) {
  this.encoding = normalizeEncoding(encoding);
  var nb;
  switch (this.encoding) {
    case 'utf16le':
      this.text = utf16Text;
      this.end = utf16End;
      nb = 4;
      break;
    case 'utf8':
      this.fillLast = utf8FillLast;
      nb = 4;
      break;
    case 'base64':
      this.text = base64Text;
      this.end = base64End;
      nb = 3;
      break;
    default:
      this.write = simpleWrite;
      this.end = simpleEnd;
      return;
  }
  this.lastNeed = 0;
  this.lastTotal = 0;
  this.lastChar = Buffer.allocUnsafe(nb);
}

StringDecoder.prototype.write = function (buf) {
  if (buf.length === 0) return '';
  var r;
  var i;
  if (this.lastNeed) {
    r = this.fillLast(buf);
    if (r === undefined) return '';
    i = this.lastNeed;
    this.lastNeed = 0;
  } else {
    i = 0;
  }
  if (i < buf.length) return r ? r + this.text(buf, i) : this.text(buf, i);
  return r || '';
};

StringDecoder.prototype.end = utf8End;

// Returns only complete characters in a Buffer
StringDecoder.prototype.text = utf8Text;

// Attempts to complete a partial non-UTF-8 character using bytes from a Buffer
StringDecoder.prototype.fillLast = function (buf) {
  if (this.lastNeed <= buf.length) {
    buf.copy(this.lastChar, this.lastTotal - this.lastNeed, 0, this.lastNeed);
    return this.lastChar.toString(this.encoding, 0, this.lastTotal);
  }
  buf.copy(this.lastChar, this.lastTotal - this.lastNeed, 0, buf.length);
  this.lastNeed -= buf.length;
};

// Checks the type of a UTF-8 byte, whether it's ASCII, a leading byte, or a
// continuation byte. If an invalid byte is detected, -2 is returned.
function utf8CheckByte(byte) {
  if (byte <= 0x7F) return 0;else if (byte >> 5 === 0x06) return 2;else if (byte >> 4 === 0x0E) return 3;else if (byte >> 3 === 0x1E) return 4;
  return byte >> 6 === 0x02 ? -1 : -2;
}

// Checks at most 3 bytes at the end of a Buffer in order to detect an
// incomplete multi-byte UTF-8 character. The total number of bytes (2, 3, or 4)
// needed to complete the UTF-8 character (if applicable) are returned.
function utf8CheckIncomplete(self, buf, i) {
  var j = buf.length - 1;
  if (j < i) return 0;
  var nb = utf8CheckByte(buf[j]);
  if (nb >= 0) {
    if (nb > 0) self.lastNeed = nb - 1;
    return nb;
  }
  if (--j < i || nb === -2) return 0;
  nb = utf8CheckByte(buf[j]);
  if (nb >= 0) {
    if (nb > 0) self.lastNeed = nb - 2;
    return nb;
  }
  if (--j < i || nb === -2) return 0;
  nb = utf8CheckByte(buf[j]);
  if (nb >= 0) {
    if (nb > 0) {
      if (nb === 2) nb = 0;else self.lastNeed = nb - 3;
    }
    return nb;
  }
  return 0;
}

// Validates as many continuation bytes for a multi-byte UTF-8 character as
// needed or are available. If we see a non-continuation byte where we expect
// one, we "replace" the validated continuation bytes we've seen so far with
// a single UTF-8 replacement character ('\ufffd'), to match v8's UTF-8 decoding
// behavior. The continuation byte check is included three times in the case
// where all of the continuation bytes for a character exist in the same buffer.
// It is also done this way as a slight performance increase instead of using a
// loop.
function utf8CheckExtraBytes(self, buf, p) {
  if ((buf[0] & 0xC0) !== 0x80) {
    self.lastNeed = 0;
    return '\ufffd';
  }
  if (self.lastNeed > 1 && buf.length > 1) {
    if ((buf[1] & 0xC0) !== 0x80) {
      self.lastNeed = 1;
      return '\ufffd';
    }
    if (self.lastNeed > 2 && buf.length > 2) {
      if ((buf[2] & 0xC0) !== 0x80) {
        self.lastNeed = 2;
        return '\ufffd';
      }
    }
  }
}

// Attempts to complete a multi-byte UTF-8 character using bytes from a Buffer.
function utf8FillLast(buf) {
  var p = this.lastTotal - this.lastNeed;
  var r = utf8CheckExtraBytes(this, buf, p);
  if (r !== undefined) return r;
  if (this.lastNeed <= buf.length) {
    buf.copy(this.lastChar, p, 0, this.lastNeed);
    return this.lastChar.toString(this.encoding, 0, this.lastTotal);
  }
  buf.copy(this.lastChar, p, 0, buf.length);
  this.lastNeed -= buf.length;
}

// Returns all complete UTF-8 characters in a Buffer. If the Buffer ended on a
// partial character, the character's bytes are buffered until the required
// number of bytes are available.
function utf8Text(buf, i) {
  var total = utf8CheckIncomplete(this, buf, i);
  if (!this.lastNeed) return buf.toString('utf8', i);
  this.lastTotal = total;
  var end = buf.length - (total - this.lastNeed);
  buf.copy(this.lastChar, 0, end);
  return buf.toString('utf8', i, end);
}

// For UTF-8, a replacement character is added when ending on a partial
// character.
function utf8End(buf) {
  var r = buf && buf.length ? this.write(buf) : '';
  if (this.lastNeed) return r + '\ufffd';
  return r;
}

// UTF-16LE typically needs two bytes per character, but even if we have an even
// number of bytes available, we need to check if we end on a leading/high
// surrogate. In that case, we need to wait for the next two bytes in order to
// decode the last character properly.
function utf16Text(buf, i) {
  if ((buf.length - i) % 2 === 0) {
    var r = buf.toString('utf16le', i);
    if (r) {
      var c = r.charCodeAt(r.length - 1);
      if (c >= 0xD800 && c <= 0xDBFF) {
        this.lastNeed = 2;
        this.lastTotal = 4;
        this.lastChar[0] = buf[buf.length - 2];
        this.lastChar[1] = buf[buf.length - 1];
        return r.slice(0, -1);
      }
    }
    return r;
  }
  this.lastNeed = 1;
  this.lastTotal = 2;
  this.lastChar[0] = buf[buf.length - 1];
  return buf.toString('utf16le', i, buf.length - 1);
}

// For UTF-16LE we do not explicitly append special replacement characters if we
// end on a partial character, we simply let v8 handle that.
function utf16End(buf) {
  var r = buf && buf.length ? this.write(buf) : '';
  if (this.lastNeed) {
    var end = this.lastTotal - this.lastNeed;
    return r + this.lastChar.toString('utf16le', 0, end);
  }
  return r;
}

function base64Text(buf, i) {
  var n = (buf.length - i) % 3;
  if (n === 0) return buf.toString('base64', i);
  this.lastNeed = 3 - n;
  this.lastTotal = 3;
  if (n === 1) {
    this.lastChar[0] = buf[buf.length - 1];
  } else {
    this.lastChar[0] = buf[buf.length - 2];
    this.lastChar[1] = buf[buf.length - 1];
  }
  return buf.toString('base64', i, buf.length - n);
}

function base64End(buf) {
  var r = buf && buf.length ? this.write(buf) : '';
  if (this.lastNeed) return r + this.lastChar.toString('base64', 0, 3 - this.lastNeed);
  return r;
}

// Pass bytes on through for single-byte encodings (e.g. ascii, latin1, hex)
function simpleWrite(buf) {
  return buf.toString(this.encoding);
}

function simpleEnd(buf) {
  return buf && buf.length ? this.write(buf) : '';
}
},{"safe-buffer":33}],36:[function(_dereq_,module,exports){
(function (setImmediate,clearImmediate){
var nextTick = _dereq_('process/browser.js').nextTick;
var apply = Function.prototype.apply;
var slice = Array.prototype.slice;
var immediateIds = {};
var nextImmediateId = 0;

// DOM APIs, for completeness

exports.setTimeout = function() {
  return new Timeout(apply.call(setTimeout, window, arguments), clearTimeout);
};
exports.setInterval = function() {
  return new Timeout(apply.call(setInterval, window, arguments), clearInterval);
};
exports.clearTimeout =
exports.clearInterval = function(timeout) { timeout.close(); };

function Timeout(id, clearFn) {
  this._id = id;
  this._clearFn = clearFn;
}
Timeout.prototype.unref = Timeout.prototype.ref = function() {};
Timeout.prototype.close = function() {
  this._clearFn.call(window, this._id);
};

// Does not start the time, just sets up the members needed.
exports.enroll = function(item, msecs) {
  clearTimeout(item._idleTimeoutId);
  item._idleTimeout = msecs;
};

exports.unenroll = function(item) {
  clearTimeout(item._idleTimeoutId);
  item._idleTimeout = -1;
};

exports._unrefActive = exports.active = function(item) {
  clearTimeout(item._idleTimeoutId);

  var msecs = item._idleTimeout;
  if (msecs >= 0) {
    item._idleTimeoutId = setTimeout(function onTimeout() {
      if (item._onTimeout)
        item._onTimeout();
    }, msecs);
  }
};

// That's not how node.js implements it but the exposed api is the same.
exports.setImmediate = typeof setImmediate === "function" ? setImmediate : function(fn) {
  var id = nextImmediateId++;
  var args = arguments.length < 2 ? false : slice.call(arguments, 1);

  immediateIds[id] = true;

  nextTick(function onNextTick() {
    if (immediateIds[id]) {
      // fn.call() is faster so we optimize for the common use-case
      // @see http://jsperf.com/call-apply-segu
      if (args) {
        fn.apply(null, args);
      } else {
        fn.call(null);
      }
      // Prevent ids from leaking
      exports.clearImmediate(id);
    }
  });

  return id;
};

exports.clearImmediate = typeof clearImmediate === "function" ? clearImmediate : function(id) {
  delete immediateIds[id];
};
}).call(this,_dereq_("timers").setImmediate,_dereq_("timers").clearImmediate)
},{"process/browser.js":19,"timers":36}],37:[function(_dereq_,module,exports){
(function (global){

/**
 * Module exports.
 */

module.exports = deprecate;

/**
 * Mark that a method should not be used.
 * Returns a modified function which warns once by default.
 *
 * If `localStorage.noDeprecation = true` is set, then it is a no-op.
 *
 * If `localStorage.throwDeprecation = true` is set, then deprecated functions
 * will throw an Error when invoked.
 *
 * If `localStorage.traceDeprecation = true` is set, then deprecated functions
 * will invoke `console.trace()` instead of `console.error()`.
 *
 * @param {Function} fn - the function to deprecate
 * @param {String} msg - the string to print to the console when `fn` is invoked
 * @returns {Function} a new "deprecated" version of `fn`
 * @api public
 */

function deprecate (fn, msg) {
  if (config('noDeprecation')) {
    return fn;
  }

  var warned = false;
  function deprecated() {
    if (!warned) {
      if (config('throwDeprecation')) {
        throw new Error(msg);
      } else if (config('traceDeprecation')) {
        console.trace(msg);
      } else {
        console.warn(msg);
      }
      warned = true;
    }
    return fn.apply(this, arguments);
  }

  return deprecated;
}

/**
 * Checks `localStorage` for boolean values for the given `name`.
 *
 * @param {String} name
 * @returns {Boolean}
 * @api private
 */

function config (name) {
  // accessing global.localStorage can trigger a DOMException in sandboxed iframes
  try {
    if (!global.localStorage) return false;
  } catch (_) {
    return false;
  }
  var val = global.localStorage[name];
  if (null == val) return false;
  return String(val).toLowerCase() === 'true';
}

}).call(this,typeof global !== "undefined" ? global : typeof self !== "undefined" ? self : typeof window !== "undefined" ? window : {})
},{}],38:[function(_dereq_,module,exports){
arguments[4][4][0].apply(exports,arguments)
},{"dup":4}],39:[function(_dereq_,module,exports){
arguments[4][5][0].apply(exports,arguments)
},{"./support/isBuffer":38,"_process":19,"dup":5,"inherits":14}],40:[function(_dereq_,module,exports){
var autonomy = exports;
var ardrone = _dereq_('ar-drone');

exports.EKF = _dereq_('./lib/EKF');
exports.Camera = _dereq_('./lib/Camera');
exports.Controller = _dereq_('./lib/Controller');
exports.Mission = _dereq_('./lib/Mission');

exports.control = function(client, options) {
    return new autonomy.Controller(client, options);
}

exports.createMission = function(options, client) {
    var client  = client || ardrone.createClient(options);
    var control = new autonomy.Controller(client, options);
    var mission = new autonomy.Mission(client, control, options);

    return mission;
}


},{"./lib/Camera":41,"./lib/Controller":42,"./lib/EKF":43,"./lib/Mission":44,"ar-drone":46}],41:[function(_dereq_,module,exports){
var sylvester    = _dereq_('sylvester');
var util         = _dereq_('util');


// TODO: Extend to support roll/pitch in back projection
// TODO: Add support for front-facing camera
// TODO: Make image aspect ratio configurable

// AR Drone 2.0 Bottom Camera Intrinsic Matrix
// https://github.com/tum-vision/ardrone_autonomy/blob/master/calibrations/ardrone2_bottom/cal.ymli
var K_BOTTOM = $M([[686.994766, 0, 329.323208],
                   [0, 688.195055, 159.323007],
                   [0, 0, 1]]);

module.exports = Camera;
function Camera(options) {
    this._options = options || {};
    this._k = this._options.k || K_BOTTOM;

    // We need to compute the inverse of K to back-project 2D to 3D
    this._invK = this._k.inverse();
}

/*
 * Given (x,y) pixel coordinates (e.g. obtained from tag detection)
 * Returns a (X,Y) coordinate in drone space.
 */
Camera.prototype.p2m = function(x, y, altitude) {
    // From the SDK Documentation:
    // X and Y coordinates of detected tag or oriented roundel #i inside the picture,
    // with (0; 0) being the top-left corner, and (1000; 1000) the right-bottom corner regardless
    // the picture resolution or the source camera.
    //
    // But our camera intrinsic is built for 640 x 360 pixel grid, so we must do some mapping.
    var xratio = 640 / 1000;
    var yratio = 360 / 1000;

    // Perform a simple back projection, we assume the drone is flat (no roll/pitch)
    // for the moment. We ignore the drone translation and yaw since we want X,Y in the
    // drone coordinate system.
    var p = $V([x * xratio, y * yratio, 1]);
    var P = this._invK.multiply(p).multiply(altitude);

    // X,Y are expressed in meters, in the drone coordinate system.
    // Which is:
    //          <--- front-facing camera
    //        |
    //       / \------- X
    //       \_/
    //        |
    //        |
    //        Y
    return {x: P.e(1), y: P.e(2)};
}


},{"sylvester":88,"util":39}],42:[function(_dereq_,module,exports){
var EventEmitter = _dereq_('events').EventEmitter;
var Timers  = _dereq_('timers');
var util    = _dereq_('util');
var PID     = _dereq_('./PID');
var EKF     = _dereq_('./EKF');
var Camera  = _dereq_('./Camera');

EPS_LIN      = 0.1; // We are ok with 10 cm horizontal precision
EPS_ALT      = 0.1; // We are ok with 10 cm altitude precision
EPS_ANG      = 0.1; // We are ok with 0.1 rad precision (5 deg)
STABLE_DELAY = 200; // Time in ms to wait before declaring the drone on target

module.exports = Controller;
util.inherits(Controller, EventEmitter);
function Controller(client, options) {
    EventEmitter.call(this);

    options = options || {};

    // A ardrone client to pilot the drone
    this._client  = client;

    // The position of a roundel tag to detect
    this._tag     = options.tag || {x: 0, y: 0, yaw: 0};

    // Configure the four PID required to control the drone
    this._pid_x   = new PID(0.5, 0, 0.35);
    this._pid_y   = new PID(0.5, 0, 0.35);
    this._pid_z   = new PID(0.8, 0, 0.35);
    this._pid_yaw = new PID(1.0, 0, 0.30);

    // kalman filter is used for the drone state estimation
    this._ekf     = new EKF(options);

    // Used to process images and backproject them
    this._camera  = new Camera();

    // Control will only work if enabled
    this._enabled = false;

    // Ensure that we don't enter the processing loop twice
    this._busy = false;

    // The curretn target goal and an optional callback to trigger
    // when goal is reached
    this._goal     = null;
    this._callback = null;

    // The last known state
    this._state   = null,

    // The last time we have reached the goal (all control commands = 0)
    this._last_ok = 0;

    // Register the listener on navdata for our control loop
    var self = this;
    client.on('navdata', function(d) {
        if (!this._busy && d.demo) {
            this._busy = true;
            self._processNavdata(d);
            self._control(d);
            this._busy = false;
        }
    });
}

/*
 * Enable auto-pilot. The controller will attempt to bring
 * the drone (and maintain it) to the goal.
 */
Controller.prototype.enable = function() {
    this._pid_x.reset();
    this._pid_y.reset();
    this._pid_z.reset();
    this._pid_yaw.reset();
    this._enabled = true;
};

/*
 * Disable auto-pilot. The controller will stop all actions
 * and send a stop command to the drone.
 */
Controller.prototype.disable = function() {
    this._enabled = false;
    this._client.stop();
}

/*
 * Return the drone state (x,y,z,yaw) as estimated
 * by the Kalman Filter.
 */
Controller.prototype.state = function() {
    return this._state;
}

/*
 * Sets the goal to the current state and attempt to hover on top.
 */
Controller.prototype.hover = function() {
    this._go({x: this._state.x, y: this._state.y, z: this._state.z, yaw: this._state.yaw});
}

/*
 * Reset the kalman filter to its base state (default is x:0, y:0, yaw:0).
 *
 * This is especially usefull to set mark the drone position as the starting position
 * after takeoff. We must disable, to ensure that the zeroing does not trigger a sudden move
 * of the drone.
 */
Controller.prototype.zero = function() {
    this.disable();
    this._ekf.reset();
}

/*
 * Move forward (direction faced by the front camera) by the given
 * distance (in meters).
 */
Controller.prototype.forward = function(distance, callback) {
   // Our starting position
   var state = this.state();

   // Remap our target position in the world coordinates
   var gx = state.x + Math.cos(state.yaw) * distance;
   var gy = state.y + Math.sin(state.yaw) * distance;

   // Assign the new goal
   this._go({x: gx, y: gy, z: state.z, yaw: state.yaw}, callback);
}

/*
 * Move backward by the given distance (in meters).
 */
Controller.prototype.backward = function(distance, callback) {
    return this.forward(-distance, callback);
}

/*
 * Move right (front being the direction faced by the front camera) by the given
 * distance (in meters).
 */
Controller.prototype.right = function(distance, callback) {
   // Our starting position
   var state = this.state();

   // Remap our target position in the world coordinates
   var gx = state.x - Math.sin(state.yaw) * distance;
   var gy = state.y + Math.cos(state.yaw) * distance;

   // Assign the new goal
   this._go({x: gx, y: gy, z: state.z, yaw: state.yaw}, callback);
}

/*
 * Move left by the given distance (in meters).
 */
Controller.prototype.left = function(distance, callback) {
    return this.right(-distance, callback);
}

/*
 * Turn clockwise of the given angle. Note that this does not
 * force a clockwise motion, if the angle is > 180 then the drone
 * will turn in the other direction, taking the shortest path.
 */
Controller.prototype.cw = function(angle, callback) {
    var state = this.state();
    var yaw   = state.yaw.toDeg() + angle;

    return this._go({x: state.x, y: state.y, z: state.z, yaw: yaw.toRad()}, callback);
}

/*
 * Turn counter clockwise of the given angle (in degrees)
 */
Controller.prototype.ccw = function(angle, callback) {
    return this.cw(-angle, callback);
}

/*
 * Climb ups by the given distance (in meters).
 */
Controller.prototype.up = function(distance, callback) {
    var state = this.state();
    return this._go({x: state.x, y: state.y, z: state.z + distance, yaw: state.yaw}, callback);
}

/*
 * Lower itself by the given distance (in meters).
 */
Controller.prototype.down = function(distance, callback) {
    return this.up(-distance, callback);
}

/*
 * Go to the target altitude
 */
Controller.prototype.altitude = function(altitude, callback) {
    var state = this.state();
    return this._go({x: state.x, y: state.y, z: altitude, yaw: state.yaw}, callback);
}

/*
 * Go to the target yaw (argument in degree)
 */
Controller.prototype.yaw = function(yaw, callback) {
    var state = this.state();
    return this._go({x: state.x, y: state.y, z: state.z, yaw: yaw.toRad()}, callback);
}

/*
 * Sets a new goal and enable the controller. When the goal
 * is reached, the callback is called with the current state.
 *
 * x,y,z in meters
 * yaw in degrees
 */
Controller.prototype.go = function(goal, callback) {
    if (goal.yaw != undefined) {
        goal.yaw = goal.yaw.toRad();
    }

    return this._go(goal, callback);
}

Controller.prototype._go = function(goal, callback) {
    // Since we are going to modify goal settings, we
    // disable the controller, just in case.
    this.disable();

    // If no goal given, assume an empty goal
    goal = goal || {};

    // Normalize the yaw, to make sure we don't spin 360deg for
    // nothing :-)
    if (goal.yaw != undefined) {
        var yaw = goal.yaw;
        goal.yaw = Math.atan2(Math.sin(yaw),Math.cos(yaw));
    }

    // Make sure we don't attempt to go too low
    if (goal.z != undefined) {
        goal.z = Math.max(goal.z, 0.5);
    }

    // Update our goal
    this._goal = goal;
    this._goal.reached = false;

    // Keep track of the callback to trigger when we reach the goal
    this._callback = callback;

    // (Re)-Enable the controller
    this.enable();
}

Controller.prototype._processNavdata = function(d) {
    // EKF prediction step
    this._ekf.predict(d);

    // If a tag is detected by the bottom camera, we attempt a correction step
    // This require prior configuration of the client to detect the oriented
    // roundel and to enable the vision detect in navdata.
    // TODO: Add documentation about this
    if (d.visionDetect && d.visionDetect.nbDetected > 0) {
        // Fetch detected tag position, size and orientation
        var xc = d.visionDetect.xc[0]
          , yc = d.visionDetect.yc[0]
          , wc = d.visionDetect.width[0]
          , hc = d.visionDetect.height[0]
          , yaw = d.visionDetect.orientationAngle[0]
          , dist = d.visionDetect.dist[0] / 100 // Need meters
          ;

        // Compute measure tag position (relative to drone) by
        // back-projecting the pixel position p(x,y) to the drone
        // coordinate system P(X,Y).
        // TODO: Should we use dist or the measure altitude ?
        var camera = this._camera.p2m(xc + wc/2, yc + hc/2, dist);

        // We convert this to the controller coordinate system
        var measured = {x: -1 * camera.y, y: camera.x};

        // Rotation is provided by the drone, we convert to radians
        measured.yaw = yaw.toRad();

        // Execute the EKS correction step
        this._ekf.correct(measured, this._tag);
    }

    // Keep a local copy of the state
    this._state = this._ekf.state();
    this._state.z = d.demo.altitude;
    this._state.vx = d.demo.velocity.x / 1000 //We want m/s instead of mm/s
    this._state.vy = d.demo.velocity.y / 1000
}

Controller.prototype._control = function(d) {
    // Do not control if not enabled
    if (!this._enabled) return;

    // Do not control if no known state or no goal defines
    if (this._goal == null || this._state ==  null) return;

    // Compute error between current state and goal
    var ex   = (this._goal.x != undefined)   ? this._goal.x   - this._state.x   : 0
      , ey   = (this._goal.y != undefined)   ? this._goal.y   - this._state.y   : 0
      , ez   = (this._goal.z != undefined)   ? this._goal.z   - this._state.z   : 0
      , eyaw = (this._goal.yaw != undefined) ? this._goal.yaw - this._state.yaw : 0
      ;

    // Normalize eyaw within [-180, 180]
    while(eyaw < -Math.PI) eyaw += (2 * Math.PI);
    while(eyaw >  Math.PI) eyaw -= (2 * Math.PI);

    // Check if we are within the target area
    if ((Math.abs(ex) < EPS_LIN) && (Math.abs(ey) < EPS_LIN) && (Math.abs(ez) < EPS_ALT) && (Math.abs(eyaw) < EPS_ANG)) {
        // Have we been here before ?
        if (!this._goal.reached && this._last_ok != 0) {
            // And for long enough ?
            if ((Date.now() - this._last_ok) > STABLE_DELAY) {
                // Mark the goal has reached
                this._goal.reached = true;

                // We schedule the callback in the near future. This is to make
                // sure we finish all our work before the callback is called.
                if (this._callback != null) {
                    setTimeout(this._callback, 10);
                    this._callback = null;
                }

                // Emit a state reached
                this.emit('goalReached', this._state);
            }
        } else {
            this._last_ok = Date.now();
        }
    } else {
        // If we just left the goal, we notify
        if (this._last_ok != 0) {
            // Reset last ok since we are in motion
            this._last_ok = 0;
            this._goal.reached = false;
            this.emit('goalLeft', this._state);
        }
    }

    // Get Raw command from PID
    var ux = this._pid_x.getCommand(ex);
    var uy = this._pid_y.getCommand(ey);
    var uz = this._pid_z.getCommand(ez);
    var uyaw = this._pid_yaw.getCommand(eyaw);

    // Ceil commands and map them to drone orientation
    var yaw  = this._state.yaw;
    var cx   = within(Math.cos(yaw) * ux + Math.sin(yaw) * uy, -1, 1);
    var cy   = within(-Math.sin(yaw) * ux + Math.cos(yaw) * uy, -1, 1);
    var cz   = within(uz, -1, 1);
    var cyaw = within(uyaw, -1, 1);

    // Emit the control data for auditing
    this.emit('controlData', {
        state:   this._state,
        goal:    this._goal,
        error:   {ex: ex, ey: ey, ez: ez, eyaw: eyaw},
        control: {ux: ux, uy: uy, uz: uz, uyaw: uyaw},
        last_ok: this._last_ok,
        tag:     (d.visionDetect && d.visionDetect.nbDetected > 0) ? 1 : 0
    });

    // Send commands to drone
    if (Math.abs(cx) > 0.01) this._client.front(cx);
    if (Math.abs(cy) > 0.01) this._client.right(cy);
    if (Math.abs(cz) > 0.01) this._client.up(cz);
    if (Math.abs(cyaw) > 0.01) this._client.clockwise(cyaw);

}

function within(x, min, max) {
    if (x < min) {
        return min;
    } else if (x > max) {
        return max;
    } else {
        return x;
    }
}

},{"./Camera":41,"./EKF":43,"./PID":45,"events":12,"timers":36,"util":39}],43:[function(_dereq_,module,exports){
var sylvester    = _dereq_('sylvester');
var util         = _dereq_('util');

var Matrix       = sylvester.Matrix;
var Vector       = sylvester.Vector;

EKF.DELTA_T = 1 / 15; // In demo mode, 15 navdata per second

module.exports = EKF;
function EKF(options) {

  options = options || {};

  this._options     = options;
  this._delta_t     = options.delta_t || EKF.DELTA_T;

  this.reset();
}

EKF.prototype.state = function() {
    return this._state;
}

EKF.prototype.confidence = function() {
    return this._sigma;
}

EKF.prototype.reset = function() {
  this._state       = this._options.state   || {x: 0, y: 0, yaw: 0};
  this._sigma       = Matrix.I(3);
  this._q           = Matrix.Diagonal([0.0003, 0.0003, 0.0001]);
  this._r           = Matrix.Diagonal([0.3, 0.3, 0.3]);
  this._last_yaw    = null;
}

EKF.prototype.predict = function(data) {
    var pitch = data.demo.rotation.pitch.toRad()
      , roll  = data.demo.rotation.roll.toRad()
      , yaw   = normAngle(data.demo.rotation.yaw.toRad())
      , vx    = data.demo.velocity.x / 1000 //We want m/s instead of mm/s
      , vy    = data.demo.velocity.y / 1000
      , dt    = this._delta_t
    ;

    // We are not interested by the absolute yaw, but the yaw motion,
    // so we need at least a prior value to get started.
    if (this._last_yaw == null) {
        this._last_yaw = yaw;
        return;
    }

    // Compute the odometry by integrating the motion over delta_t
    var o = {dx: vx * dt, dy: vy * dt, dyaw: yaw - this._last_yaw};
    this._last_yaw  = yaw;

    // Update the state estimate
    var state = this._state;
    state.x   = state.x + o.dx * Math.cos(state.yaw) - o.dy * Math.sin(state.yaw);
    state.y   = state.y + o.dx * Math.sin(state.yaw) + o.dy * Math.cos(state.yaw);
    state.yaw = state.yaw + o.dyaw;

    // Normalize the yaw value
    state.yaw = Math.atan2(Math.sin(state.yaw),Math.cos(state.yaw));

    // Compute the G term (due to the Taylor approximation to linearize the function).
    var G = $M(
           [[1, 0, -1 * Math.sin(state.yaw) * o.dx - Math.cos(state.yaw) * o.dy],
            [0, 1,  Math.cos(state.yaw) * o.dx - Math.sin(state.yaw) * o.dy],
            [0, 0, 1]]
            );

    // Compute the new sigma
    this._sigma = G.multiply(this._sigma).multiply(G.transpose()).add(this._q);
}
 /*
  * measure.x:   x-position of marker in drone's xy-coordinate system (independant of roll, pitch)
  * measure.y:   y-position of marker in drone's xy-coordinate system (independant of roll, pitch)
  * measure.yaw: yaw rotation of marker, in drone's xy-coordinate system (independant of roll, pitch)
  *
  * pose.x:   x-position of marker in world-coordinate system
  * pose.y:   y-position of marker in world-coordinate system
  * pose.yaw: yaw-rotation of marker in world-coordinate system
  */
EKF.prototype.correct = function(measure, pose) {
    // Compute expected measurement given our current state and the marker pose
    var state  = this._state;
    var psi    = state.yaw;
    this._s    = {x: state.x, y: state.y, yaw: state.yaw};

    // Normalized the measure yaw
    measure.yaw = normAngle(measure.yaw);
    this._m = {x: measure.x, y: measure.y, yaw: measure.yaw};

    var z1 = Math.cos(psi) * (pose.x - state.x) + Math.sin(psi) * (pose.y - state.y);
    var z2 = -1 * Math.sin(psi) * (pose.x - state.x) + Math.cos(psi) * (pose.y - state.y);
    var z3 = pose.yaw - psi;
    this._z = {x: z1, y: z2, yaw: z3};

    // Compute the error
    var e1 = measure.x - z1;
    var e2 = measure.y - z2;
    var e3 = measure.yaw - z3;
    this._e = {x: e1, y: e2, yaw: e3};

    // Compute the H term
    var H = $M([[ -Math.cos(psi), -Math.sin(psi), Math.sin(psi) * (state.x - pose.x) - Math.cos(psi) * (state.y - pose.y)],
                [  Math.sin(psi), -Math.cos(psi), Math.cos(psi) * (state.x - pose.x) + Math.sin(psi) * (state.y - pose.y)],
                [  0, 0, -1]]);

    // Compute the Kalman Gain
    var Ht = H.transpose();
    var K = this._sigma.multiply(Ht).multiply(H.multiply(this._sigma).multiply(Ht).add(this._r).inverse())

    // Correct the pose estimate
    var err = $V([e1, e2, e3]);
    var c = K . multiply(err);
    state.x = state.x + c.e(1);
    state.y = state.y + c.e(2);

//  TODO - This does not work, need more investigation.
//  In the meanwhile, we don't correct yaw based on observation.
//  state.yaw = state.yaw + c.e(3);

    this._sigma = Matrix.I(3).subtract(K.multiply(H)).multiply(this._sigma);
};

function normAngle(rad) {
    while (rad >  Math.PI) { rad -= 2 * Math.PI;}
    while (rad < -Math.PI) { rad += 2 * Math.PI;}
    return rad;
}

/** Converts numeric degrees to radians */
if (typeof(Number.prototype.toRad) === "undefined") {
  Number.prototype.toRad = function() {
    return this * Math.PI / 180;
  }
}

/** Converts radians to numeric dregrees */
if (typeof(Number.prototype.toDeg) === "undefined") {
  Number.prototype.toDeg = function() {
    return this * 180 / Math.PI;
  }
}

},{"sylvester":88,"util":39}],44:[function(_dereq_,module,exports){
var async = _dereq_('async')
  , fs    = _dereq_('fs')
  ;

module.exports = Mission;
function Mission(client, controller, options) {

    options = options || {};

    this._options   = options;
    this._client    = client;
    this._control   = controller;

    this._steps     = [];
}

Mission.prototype.client = function() {
    return this._client;
}

Mission.prototype.control = function() {
    return this._control;
}

Mission.prototype.run = function(callback) {
    async.waterfall(this._steps, callback);
}

Mission.prototype.log = function(path) {
    var dataStream = fs.createWriteStream(path);
    var ekf = this._control._ekf;

    this._control.on('controlData', function(d) {
        var log = (d.state.x + "," +
                   d.state.y + "," +
                   d.state.z + "," +
                   d.state.yaw + "," +
                   d.state.vx + "," +
                   d.state.vy + "," +
                   d.goal.x + "," +
                   d.goal.y + "," +
                   d.goal.z + "," +
                   d.goal.yaw + "," +
                   d.error.ex + "," +
                   d.error.ey + "," +
                   d.error.ez + "," +
                   d.error.eyaw + "," +
                   d.control.ux + "," +
                   d.control.uy + "," +
                   d.control.uz + "," +
                   d.control.uyaw + "," +
                   d.last_ok + "," +
                   d.tag);

        if (d.tag > 0) {
            log = log + "," +
                  ekf._s.x + "," +
                  ekf._s.y + "," +
                  ekf._s.yaw.toDeg() + "," +
                  ekf._m.x + "," +
                  ekf._m.y + "," +
                  ekf._m.yaw.toDeg() + "," +
                  ekf._z.x + "," +
                  ekf._z.y + "," +
                  ekf._z.yaw.toDeg() + "," +
                  ekf._e.x + "," +
                  ekf._e.y + "," +
                  ekf._e.yaw.toDeg()
        } else {
            log = log + ",0,0,0,0,0,0"
        }

        log = log + "\n";

        dataStream.write(log);
    });
}

Mission.prototype.takeoff = function() {
    var self = this;
    this._steps.push(function(cb) {
        self._client.takeoff(cb);
    });

    return this;
}

Mission.prototype.land = function() {
    var self = this;
    this._steps.push(function(cb) {
        self._client.land(cb);
    });

    return this;
}

Mission.prototype.hover = function(delay) {
    var self = this;
    this._steps.push(function(cb) {
        self._control.hover();
        setTimeout(cb, delay);
    });

    return this;
}

Mission.prototype.wait = function(delay) {
    this._steps.push(function(cb) {
        setTimeout(cb, delay);
    });

    return this;
}

Mission.prototype.task = function(task) {
    this._steps.push(function(cb) {
        task(cb);
    });

    return this;
}

Mission.prototype.taskSync = function(task) {
    this._steps.push(function(cb) {
        task();
        cb();
    });

    return this;
}

Mission.prototype.zero = function() {
    var self = this;
    this._steps.push(function(cb) {
        self._control.zero();
        cb();
    });

    return this;
}

Mission.prototype.go = function(goal) {
    var self = this;
    this._steps.push(function(cb) {
        self._control.go(goal, cb);
    });

    return this;
}

Mission.prototype.forward = function(distance) {
    var self = this;
    this._steps.push(function(cb) {
        self._control.forward(distance, cb);
    });

    return this;
}

Mission.prototype.backward = function(distance) {
    var self = this;
    this._steps.push(function(cb) {
        self._control.backward(distance, cb);
    });

    return this;
}

Mission.prototype.left = function(distance) {
    var self = this;
    this._steps.push(function(cb) {
        self._control.left(distance, cb);
    });

    return this;
}

Mission.prototype.right = function(distance) {
    var self = this;
    this._steps.push(function(cb) {
        self._control.right(distance, cb);
    });

    return this;
}

Mission.prototype.up = function(distance) {
    var self = this;
    this._steps.push(function(cb) {
        self._control.up(distance, cb);
    });

    return this;
}

Mission.prototype.down = function(distance) {
    var self = this;
    this._steps.push(function(cb) {
        self._control.down(distance, cb);
    });

    return this;
}

Mission.prototype.cw = function(angle) {
    var self = this;
    this._steps.push(function(cb) {
        self._control.cw(angle, cb);
    });

    return this;
}

Mission.prototype.ccw = function(angle) {
    var self = this;
    this._steps.push(function(cb) {
        self._control.ccw(angle, cb);
    });

    return this;
}

Mission.prototype.altitude = function(altitude) {
    var self = this;
    this._steps.push(function(cb) {
        self._control.altitude(altitude, cb);
    });

    return this;
}

Mission.prototype.yaw = function(angle) {
    var self = this;
    this._steps.push(function(cb) {
        self._control.yaw(angle,cb);
    });

    return this;
}





},{"async":62,"fs":1}],45:[function(_dereq_,module,exports){
module.exports = PID;
function PID(kp, ki, kd) {
    this.configure(kp, ki, kd);
    this.reset();
}

PID.prototype.configure = function(kp,ki,kd) {
    this._kp = kp;
    this._ki = ki;
    this._kd = kd;
}

PID.prototype.reset = function() {
    this._last_time = 0;
    this._last_error = Infinity;
    this._error_sum = 0;
}

PID.prototype.getCommand = function(e) {
    // Compute dt in seconds
    var time = Date.now();
    var dt = (time - this._last_time) / 1000

    var de = 0;
    if (this._last_time != 0) {
        // Compute de (error derivation)
        if (this._last_error < Infinity) {
            de = (e - this._last_error) / dt;
        }

        // Integrate error
        this._error_sum += e * dt;
    }

    // Update our trackers
    this._last_time = time;
    this._last_error = e;

    // Compute commands
    var command = this._kp * e
                + this._ki * this._error_sum
                + this._kd * de;

    return command;
}

},{}],46:[function(_dereq_,module,exports){
var arDrone = exports;

exports.Client           = _dereq_('./lib/Client');
exports.UdpControl       = _dereq_('./lib/control/UdpControl');
exports.PngStream        = _dereq_('./lib/video/PngStream');
exports.UdpNavdataStream = _dereq_('./lib/navdata/UdpNavdataStream');

exports.createClient = function(options) {
  var client = new arDrone.Client(options);
  client.resume();
  return client;
};

exports.createUdpControl = function(options) {
  return new arDrone.UdpControl(options);
};

exports.createPngStream = function(options) {
  var stream = new arDrone.PngStream(options);
  stream.resume();
  return stream;
};

exports.createUdpNavdataStream = function(options) {
  var stream = new arDrone.UdpNavdataStream(options);
  stream.resume();
  return stream;
};

},{"./lib/Client":47,"./lib/control/UdpControl":52,"./lib/navdata/UdpNavdataStream":56,"./lib/video/PngStream":60}],47:[function(_dereq_,module,exports){
var EventEmitter = _dereq_('events').EventEmitter;
var util         = _dereq_('util');

Client.UdpControl       = _dereq_('./control/UdpControl');
Client.Repl             = _dereq_('./Repl');
Client.UdpNavdataStream = _dereq_('./navdata/UdpNavdataStream');
Client.PngStream        = _dereq_('./video/PngStream');
Client.PngEncoder       = _dereq_('./video/PngEncoder');
Client.TcpVideoStream   = _dereq_('./video/TcpVideoStream');

module.exports = Client;
util.inherits(Client, EventEmitter);
function Client(options) {
  EventEmitter.call(this);

  options = options || {};

  this._options           = options;
  this._udpControl        = options.udpControl || new Client.UdpControl(options);
  this._udpNavdatasStream = options.udpNavdataStream || new Client.UdpNavdataStream(options);
  this._pngStream         = null;
  this._tcpVideoStream    = null;
  this._interval          = null;
  this._ref               = {};
  this._pcmd              = {};
  this._repeaters         = [];
  this._afterOffset       = 0;
  this._disableEmergency  = false;
  this._lastState         = 'CTRL_LANDED';
  this._lastBattery       = 100;
  this._lastAltitude      = 0;
}

Client.prototype.after = function(duration, fn) {
  setTimeout(fn.bind(this), this._afterOffset + duration);
  this._afterOffset += duration;
  return this;
};

Client.prototype.createRepl = function() {
  var repl = new Client.Repl(this);
  repl.resume();
  return repl;
};

Client.prototype.createPngStream = function() {
  console.warn("Client.createPngStream is deprecated. Use Client.getPngStream instead.");
  return this.getPngStream();
};

Client.prototype.getPngStream = function() {
  if (this._pngStream === null) {
    this._pngStream = this._newPngStream();
  }
  return this._pngStream;
};

Client.prototype.getVideoStream = function() {
  if (!this._tcpVideoStream) {
    this._tcpVideoStream = this._newTcpVideoStream();
  }
  return this._tcpVideoStream;
};

Client.prototype._newTcpVideoStream = function() {
  var stream = new Client.TcpVideoStream(this._options);
  var callback = function(err) {
    if (err) {
      console.log('TcpVideoStream error: %s', err.message);
      setTimeout(function () {
        console.log('Attempting to reconnect to TcpVideoStream...');
        stream.connect(callback);
      }, 1000);
    }
  };

  stream.connect(callback);
  stream.on('error', callback);
  return stream;
};

Client.prototype._newPngStream = function() {
  var videoStream = this.getVideoStream();
  var pngEncoder = new Client.PngEncoder(this._options);

  videoStream.on('data', function(data) {
    pngEncoder.write(data);
  });

  return pngEncoder;
};

Client.prototype.resume = function() {
  // Reset config ACK.
  this._udpControl.ctrl(5, 0);
  // request basic navdata by default
  this.config('general:navdata_demo', 'TRUE');
  this.disableEmergency();
  this._setInterval(30);

  this._udpNavdatasStream.removeAllListeners();
  this._udpNavdatasStream.resume();
  this._udpNavdatasStream
    .on('error', this._maybeEmitError.bind(this))
    .on('data', this._handleNavdata.bind(this));
};

Client.prototype._handleNavdata = function(navdata) {
  if (navdata.droneState && navdata.droneState.emergencyLanding && this._disableEmergency) {
    this._ref.emergency = true;
  } else {
    this._ref.emergency    = false;
    this._disableEmergency = false;
  }
  if (navdata.droneState.controlCommandAck) {
    this._udpControl.ack();
  } else {
    this._udpControl.ackReset();
  }
  this.emit('navdata', navdata);
  this._processNavdata(navdata);
};

Client.prototype._processNavdata = function(navdata) {
  if (navdata.droneState && navdata.demo) {
    // controlState events
    var cstate = navdata.demo.controlState;
    var emitState = (function(e, state) {
      if (cstate === state && this._lastState !== state) {
        return this.emit(e);
      }
    }).bind(this);
    emitState('landing', 'CTRL_TRANS_LANDING');
    emitState('landed', 'CTRL_LANDED');
    emitState('takeoff', 'CTRL_TRANS_TAKEOFF');
    emitState('hovering', 'CTRL_HOVERING');
    emitState('flying', 'CTRL_FLYING');
    this._lastState = cstate;

    // battery events
    var battery = navdata.demo.batteryPercentage;
    if (navdata.droneState.lowBattery === 1) {
      this.emit('lowBattery', battery);
    }
    if (navdata.demo.batteryPercentage !== this._lastBattery) {
      this.emit('batteryChange', battery);
      this._lastBattery = battery;
    }

    // altitude events
    var altitude = navdata.demo.altitudeMeters;
    if (altitude !== this._lastAltitude) {
      this.emit('altitudeChange', altitude);
      this._lastAltitude = altitude;
    }
  }

};

// emits an 'error' event, but only if somebody is listening. This avoids
// making node's EventEmitter throwing an exception for non-critical errors
Client.prototype._maybeEmitError = function(err) {
  if (this.listeners('error').length > 0) {
    this.emit('error', err);
  }
};

Client.prototype._setInterval = function(duration) {
  clearInterval(this._interval);
  this._interval = setInterval(this._sendCommands.bind(this), duration);
};

Client.prototype._sendCommands = function() {
  this._udpControl.ref(this._ref);
  this._udpControl.pcmd(this._pcmd);
  this._udpControl.flush();

  this._repeaters
    .forEach(function(repeat) {
      repeat.times--;
      repeat.method();
    });

  this._repeaters = this._repeaters.filter(function(repeat) {
    return repeat.times > 0;
  });
};

Client.prototype.disableEmergency = function() {
  this._disableEmergency = true;
};

Client.prototype.takeoff = function(cb) {
  this.once('hovering', cb || function() {});
  this._ref.fly = true;
  return true;
};

Client.prototype.land = function(cb) {
  this.once('landed', cb || function() {});
  this._ref.fly = false;
  return true;
};

Client.prototype.stop = function() {
  this._pcmd = {};
  return true;
};

Client.prototype.calibrate = function(device_num) {
  this._udpControl.calibrate(device_num);
};

Client.prototype.ftrim = function() {
  // @TODO Figure out if we can get a ACK for this, so we don't need to
  // repeat it blindly like this

  if(this._ref.fly) {
    console.trace("You can’t ftrim when you fly");
    return false;
  }

  var self = this;
  this._repeat(10, function() {
    self._udpControl.ftrim();
  });
};

Client.prototype.config = function(key, value, callback) {
  this._udpControl.config(key, value, callback);
};

Client.prototype.ctrl = function(controlMode, otherMode) {
  this._udpControl.ctrl(controlMode, otherMode);
};

Client.prototype.animate = function(animation, duration) {
  // @TODO Figure out if we can get a ACK for this, so we don't need to
  // repeat it blindly like this
  var self = this;
  this._repeat(10, function() {
    self._udpControl.animate(animation, duration);
  });
};

Client.prototype.animateLeds = function(animation, hz, duration) {
  // @TODO Figure out if we can get a ACK for this, so we don't need to
  // repeat it blindly like this
  var self = this;
  this._repeat(10, function() {
    self._udpControl.animateLeds(animation, hz, duration);
  });
};

Client.prototype.battery = function() {
  return this._lastBattery;
};

Client.prototype._repeat = function(times, fn) {
  this._repeaters.push({times: times, method: fn});
};

var pcmdOptions = [
  ['up', 'down'],
  ['left', 'right'],
  ['front', 'back'],
  ['clockwise', 'counterClockwise'],
];

pcmdOptions.forEach(function(pair) {
  Client.prototype[pair[0]] = function(speed) {
    if (isNaN(speed)) {
      return;
    }
    speed = parseFloat(speed);

    this._pcmd[pair[0]] = speed;
    delete this._pcmd[pair[1]];

    return speed;
  };

  Client.prototype[pair[1]] = function(speed) {
    if (isNaN(speed)) {
      return;
    }

    speed = parseFloat(speed);

    this._pcmd[pair[1]] = speed;
    delete this._pcmd[pair[0]];

    return speed;
  };
});

},{"./Repl":48,"./control/UdpControl":52,"./navdata/UdpNavdataStream":56,"./video/PngEncoder":58,"./video/PngStream":60,"./video/TcpVideoStream":61,"events":12,"util":39}],48:[function(_dereq_,module,exports){
var repl       = _dereq_('repl');

module.exports = Repl;
function Repl(client) {
  this._client   = client;
  this._repl     = null;
  this._nodeEval = null;
}

Repl.prototype.resume = function() {
  this._repl = repl.start({
    prompt : 'drone> ',
  });

  /* jshint evil:true */
  this._nodeEval = this._repl.eval;

  // @TODO This does not seem to work for animate('yawShake', 2000), need
  // to fix this.
  //this._repl.eval = this._eval.bind(this);

  this._setupAutocomplete();
};

Repl.prototype._setupAutocomplete = function() {
  for (var property in this._client) {
    if (property.substr(0, 1) === '_') {
      // Skip "private" properties
      continue;
    }

    var value = this._client[property];
    if (typeof value === 'function') {
      value = value.bind(this._client);
    }

    this._repl.context[property] = value;
  }

  this._repl.context.client = this._client;
};

Repl.prototype._eval = function(code, context, filename, cb) {
  var args = code.match(/[^() \n]+/g);
  if (!args) {
    return this._nodeEval.apply(this._repl, arguments);
  }

  var property = args.shift();
  if (!this._client[property]) {
    return this._nodeEval.apply(this._repl, arguments);
  }

  var type = typeof this._client[property];

  if (type === 'function') {
    try {
      cb(null, this._client[property].apply(this._client, args));
    } catch (err) {
      cb(err);
    }
    return;
  }

  if (args.length > 1) {
    // Don't accept more than 1 argument
    cb(new Error('Cannot set property "' + property + '". Too many arguments.'));
    return;
  }

  if (args.length === 1) {
    // Set a client property
    cb(null, this._client[property] = args[1]);
    return;
  }

  // args.length === 0, return property value
  cb(null, this._client[property]);
};

},{"repl":1}],49:[function(_dereq_,module,exports){
(function (process){
// Constants required to decode/encode the network communication with the drone.
// Names taken from the official SDK are not renamed, but:
//
// * Consistent prefixes/suffixes are stripped
// * Always converted to UPPER_CASE

var constants = module.exports;

constants.getName = function(group, value) {
  group = this[group];

  for (var name in group) {
    if (group[name] === value) {
      return name;
    }
  }
};

constants.DEFAULT_DRONE_IP = process.env.DEFAULT_DRONE_IP || '192.168.1.1';

// from ARDrone_SDK_2_0/ARDroneLib/Soft/Common/config.h
constants.ports = {
  FTP            : 5551,
  AUTH           : 5552,
  VIDEO_RECORDER : 5553,
  NAVDATA        : 5554,
  VIDEO          : 5555,
  AT             : 5556,
  RAW_CAPTURE    : 5557,
  PRINTF         : 5558,
  CONTROL        : 5559
};

// from ARDrone_SDK_2_0/ARDroneLib/Soft/Common/config.h
constants.droneStates = {
  FLY_MASK            : 1 << 0,  /*!< FLY MASK : (0) ardrone is landed, (1) ardrone is flying */
  VIDEO_MASK          : 1 << 1,  /*!< VIDEO MASK : (0) video disable, (1) video enable */
  VISION_MASK         : 1 << 2,  /*!< VISION MASK : (0) vision disable, (1) vision enable */
  CONTROL_MASK        : 1 << 3,  /*!< CONTROL ALGO : (0) euler angles control, (1) angular speed control */
  ALTITUDE_MASK       : 1 << 4,  /*!< ALTITUDE CONTROL ALGO : (0) altitude control inactive (1) altitude control active */
  USER_FEEDBACK_START : 1 << 5,  /*!< USER feedback : Start button state */
  COMMAND_MASK        : 1 << 6,  /*!< Control command ACK : (0) None, (1) one received */
  CAMERA_MASK         : 1 << 7,  /*!< CAMERA MASK : (0) camera not ready, (1) Camera ready */
  TRAVELLING_MASK     : 1 << 8,  /*!< Travelling mask : (0) disable, (1) enable */
  USB_MASK            : 1 << 9,  /*!< USB key : (0) usb key not ready, (1) usb key ready */
  NAVDATA_DEMO_MASK   : 1 << 10, /*!< Navdata demo : (0) All navdata, (1) only navdata demo */
  NAVDATA_BOOTSTRAP   : 1 << 11, /*!< Navdata bootstrap : (0) options sent in all or demo mode, (1) no navdata options sent */
  MOTORS_MASK         : 1 << 12, /*!< Motors status : (0) Ok, (1) Motors problem */
  COM_LOST_MASK       : 1 << 13, /*!< Communication Lost : (1) com problem, (0) Com is ok */
  SOFTWARE_FAULT      : 1 << 14, /*!< Software fault detected - user should land as quick as possible (1) */
  VBAT_LOW            : 1 << 15, /*!< VBat low : (1) too low, (0) Ok */
  USER_EL             : 1 << 16, /*!< User Emergency Landing : (1) User EL is ON, (0) User EL is OFF*/
  TIMER_ELAPSED       : 1 << 17, /*!< Timer elapsed : (1) elapsed, (0) not elapsed */
  MAGNETO_NEEDS_CALIB : 1 << 18, /*!< Magnetometer calibration state : (0) Ok, no calibration needed, (1) not ok, calibration needed */
  ANGLES_OUT_OF_RANGE : 1 << 19, /*!< Angles : (0) Ok, (1) out of range */
  WIND_MASK           : 1 << 20, /*!< WIND MASK: (0) ok, (1) Too much wind */
  ULTRASOUND_MASK     : 1 << 21, /*!< Ultrasonic sensor : (0) Ok, (1) deaf */
  CUTOUT_MASK         : 1 << 22, /*!< Cutout system detection : (0) Not detected, (1) detected */
  PIC_VERSION_MASK    : 1 << 23, /*!< PIC Version number OK : (0) a bad version number, (1) version number is OK */
  ATCODEC_THREAD_ON   : 1 << 24, /*!< ATCodec thread ON : (0) thread OFF (1) thread ON */
  NAVDATA_THREAD_ON   : 1 << 25, /*!< Navdata thread ON : (0) thread OFF (1) thread ON */
  VIDEO_THREAD_ON     : 1 << 26, /*!< Video thread ON : (0) thread OFF (1) thread ON */
  ACQ_THREAD_ON       : 1 << 27, /*!< Acquisition thread ON : (0) thread OFF (1) thread ON */
  CTRL_WATCHDOG_MASK  : 1 << 28, /*!< CTRL watchdog : (1) delay in control execution (> 5ms), (0) control is well scheduled */
  ADC_WATCHDOG_MASK   : 1 << 29, /*!< ADC Watchdog : (1) delay in uart2 dsr (> 5ms), (0) uart2 is good */
  COM_WATCHDOG_MASK   : 1 << 30, /*!< Communication Watchdog : (1) com problem, (0) Com is ok */
  EMERGENCY_MASK      : 1 << 31  /*!< Emergency landing : (0) no emergency, (1) emergency */
};


// from ARDrone_SDK_2_0/ARDroneLib/Soft/Common/navdata_keys.h
constants.options = {
  DEMO              : 0,
  TIME              : 1,
  RAW_MEASURES      : 2,
  PHYS_MEASURES     : 3,
  GYROS_OFFSETS     : 4,
  EULER_ANGLES      : 5,
  REFERENCES        : 6,
  TRIMS             : 7,
  RC_REFERENCES     : 8,
  PWM               : 9,
  ALTITUDE          : 10,
  VISION_RAW        : 11,
  VISION_OF         : 12,
  VISION            : 13,
  VISION_PERF       : 14,
  TRACKERS_SEND     : 15,
  VISION_DETECT     : 16,
  WATCHDOG          : 17,
  ADC_DATA_FRAME    : 18,
  VIDEO_STREAM      : 19,
  GAMES             : 20,
  PRESSURE_RAW      : 21,
  MAGNETO           : 22,
  WIND_SPEED        : 23,
  KALMAN_PRESSURE   : 24,
  HDVIDEO_STREAM    : 25,
  WIFI              : 26,
  ZIMMU_3000        : 27,
  CKS               : 65535
};

constants.CAD_TYPE = {
  HORIZONTAL              : 0,   /*<! Deprecated */
  VERTICAL                : 1,   /*<! Deprecated */
  VISION                  : 2,   /*<! Detection of 2D horizontal tags on drone shells */
  NONE                    : 3,   /*<! Detection disabled */
  COCARDE                 : 4,   /*<! Detects a roundel under the drone */
  ROUNDEL                 : 4,   /*<! Detects a roundel under the drone */
  ORIENTED_COCARDE        : 5,   /*<! Detects an oriented roundel under the drone */
  ORIENTED_ROUNDEL        : 5,   /*<! Detects an oriented roundel under the drone */
  STRIPE                  : 6,   /*<! Detects a uniform stripe on the ground */
  H_COCARDE               : 7,   /*<! Detects a roundel in front of the drone */
  H_ROUNDEL               : 7,   /*<! Detects a roundel in front of the drone */
  H_ORIENTED_COCARDE      : 8,   /*<! Detects an oriented roundel in front of the drone */
  H_ORIENTED_ROUNDEL      : 8,   /*<! Detects an oriented roundel in front of the drone */
  STRIPE_V                : 9,
  MULTIPLE_DETECTION_MODE : 10,  /* The drone uses several detections at the same time */
  CAP                     : 11,  /*<! Detects a Cap orange and green in front of the drone */
  ORIENTED_COCARDE_BW     : 12,  /*<! Detects the black and white roundel */
  ORIENTED_ROUNDEL_BW     : 12,  /*<! Detects the black and white roundel */
  VISION_V2               : 13,  /*<! Detects 2nd version of shell/tag in front of the drone */
  TOWER_SIDE              : 14,  /*<! Detect a tower side with the front camera */
  NUM                     : 15   /*<! Number of possible values for CAD_TYPE */
};

constants.TAG_TYPE = {
  NONE             : 0,
  SHELL_TAG        : 1,
  ROUNDEL          : 2,
  ORIENTED_ROUNDEL : 3,
  STRIPE           : 4,
  CAP              : 5,
  SHELL_TAG_V2     : 6,
  TOWER_SIDE       : 7,
  BLACK_ROUNDEL    : 8,
  NUM              : 9
};

constants.FLYING_MODE = {
  FREE_FLIGHT                      : 0,       /**< Normal mode, commands are enabled */
  HOVER_ON_TOP_OF_ROUNDEL          : 1 << 0,  /**< Commands are disabled, drones hovers on top of a roundel - roundel detection MUST be activated by the user with 'detect_type' configuration. */
  HOVER_ON_TOP_OF_ORIENTED_ROUNDEL : 1 << 1   /**< Commands are disabled, drones hovers on top of an oriented roundel - oriented roundel detection MUST be activated by the user with 'detect_type' configuration. */
};

constants.ARDRONE_DETECTION_COLOR = {
  ORANGE_GREEN       : 1,    /*!< Cameras detect orange-green-orange tags */
  ORANGE_YELLOW      : 2,    /*!< Cameras detect orange-yellow-orange tags*/
  ORANGE_BLUE        : 3,    /*!< Cameras detect orange-blue-orange tags */
  ARRACE_FINISH_LINE : 0x10,
  ARRACE_DONUT       : 0x11
};

}).call(this,_dereq_('_process'))
},{"_process":19}],50:[function(_dereq_,module,exports){
module.exports = AtCommand;
function AtCommand(type, args, blocks, options, callback) {
  this.type   = type;
  this.args   = args;
  if (blocks === undefined) {
    blocks = false;
  }
  this.blocks = blocks;
  this.options = options || {};
  this.callback = callback;
}

AtCommand.prototype.serialize = function(sequenceNumber) {
  var args = [sequenceNumber].concat(this.args);
  return 'AT*' + this.type + '=' + args.join(',') + '\r';
};

},{}],51:[function(_dereq_,module,exports){
var AtCommand = _dereq_('./AtCommand');
var at        = _dereq_('./at');

var exports = module.exports = AtCommandCreator;
function AtCommandCreator() {
}

AtCommandCreator.prototype.raw = function(type, args, blocks, options, callback) {
  args = (Array.isArray(args))
    ? args
    : Array.prototype.slice.call(arguments, 1);

  return new AtCommand(type, args, blocks, options, callback);
};

AtCommandCreator.prototype.ctrl = function(controlMode, otherMode) {
  var args = [controlMode, otherMode];
  return this.raw('CTRL', args);
};

// Used for fly/land as well as emergency trigger/recover
AtCommandCreator.prototype.ref = function(options) {
  options = options || {};

  var args = [0];

  if (options.fly) {
    args[0] = args[0] | REF_FLAGS.takeoff;
  }

  if (options.emergency) {
    args[0] = args[0] | REF_FLAGS.emergency;
  }

  return this.raw('REF', args);
};

// Used to fly the drone around
AtCommandCreator.prototype.pcmd = function(options) {
  options = options || {};

  // flags, leftRight, frontBack, upDown, clockWise
  var args = [0, 0, 0, 0, 0];

  for (var key in options) {
    var alias = PCMD_ALIASES[key];
    var value = options[key];

    if (alias.invert) {
      value = -value;
    }

    args[alias.index] = at.floatString(value);
    args[0]           = args[0] | PCMD_FLAGS.progressive;
  }

  return this.raw('PCMD', args);
};

AtCommandCreator.prototype.calibrate = function(device_num) {
  var args = [device_num];

  return this.raw('CALIB', args);
};

// Can be called either as config(key, value, callback) or
// config(options, callback) where options has at least keys "key" and
// "value" (but may also have other keys).
AtCommandCreator.prototype.config = function(key, value, callback) {
  var options;
  if (key && typeof(key) === 'object') {
    // Called with signature (options, callback).
    options = key;
    callback = value;
    key = options.key;
    value = options.value;
  } else {
    options = {};
  }
  return this.raw('CONFIG', ['"' + key + '"', '"' + value + '"'], true, options, callback);
};

AtCommandCreator.prototype.ctrl = function(a, b) {
  return this.raw('CTRL', [a, b]);
};

// TMP: Used to send FTRIM
AtCommandCreator.prototype.ftrim = function() {
  return this.raw('FTRIM');
};

AtCommandCreator.prototype.animateLeds = function(name, hz, duration) {
  // Default animation
  name     = name || 'redSnake';
  hz       = hz || 2;
  duration = duration || 3;

  var animationId = LED_ANIMATIONS.indexOf(name);
  if (animationId < 0) {
    throw new Error('Unknown led animation: ' + name);
  }

  hz = at.floatString(hz);

  var params = [animationId, hz, duration].join(',');
  return this.config('leds:leds_anim', params);
};

AtCommandCreator.prototype.animate = function(name, duration) {
  var animationId = ANIMATIONS.indexOf(name);
  if (animationId < 0) {
    throw new Error('Unknown animation: ' + name);
  }

  var params = [animationId, duration].join(',');
  return this.config('control:flight_anim', params);
};

// Constants

var REF_FLAGS = exports.REF_FLAGS = {
  emergency : (1 << 8),
  takeoff   : (1 << 9),
};

var PCMD_FLAGS = exports.PCMD_FLAGS = {
  progressive : (1 << 0),
};

var PCMD_ALIASES = exports.PCMD_ALIASES = {
  left             : {index: 1, invert: true},
  right            : {index: 1, invert: false},
  front            : {index: 2, invert: true},
  back             : {index: 2, invert: false},
  up               : {index: 3, invert: false},
  down             : {index: 3, invert: true},
  clockwise        : {index: 4, invert: false},
  counterClockwise : {index: 4, invert: true},
};

// from ARDrone_SDK_2_0/ControlEngine/iPhone/Release/ARDroneGeneratedTypes.h
var LED_ANIMATIONS = exports.LED_ANIMATIONS = [
  'blinkGreenRed',
  'blinkGreen',
  'blinkRed',
  'blinkOrange',
  'snakeGreenRed',
  'fire',
  'standard',
  'red',
  'green',
  'redSnake',
  'blank',
  'rightMissile',
  'leftMissile',
  'doubleMissile',
  'frontLeftGreenOthersRed',
  'frontRightGreenOthersRed',
  'rearRightGreenOthersRed',
  'rearLeftGreenOthersRed',
  'leftGreenRightRed',
  'leftRedRightGreen',
  'blinkStandard',
];

// from ARDrone_SDK_2_0/ControlEngine/iPhone/Release/ARDroneGeneratedTypes.h
var ANIMATIONS = exports.ANIMATIONS = [
  'phiM30Deg',
  'phi30Deg',
  'thetaM30Deg',
  'theta30Deg',
  'theta20degYaw200deg',
  'theta20degYawM200deg',
  'turnaround',
  'turnaroundGodown',
  'yawShake',
  'yawDance',
  'phiDance',
  'thetaDance',
  'vzDance',
  'wave',
  'phiThetaMixed',
  'doublePhiThetaMixed',
  'flipAhead',
  'flipBehind',
  'flipLeft',
  'flipRight',
];

},{"./AtCommand":50,"./at":53}],52:[function(_dereq_,module,exports){
(function (Buffer){
var assert           = _dereq_('assert');
var dgram            = _dereq_('dgram');

var debug            = _dereq_('simple-debug')('udpcontrol');

var AtCommandCreator = _dereq_('./AtCommandCreator');
var meta             = _dereq_('../misc/meta');
var constants        = _dereq_('../constants');


var DEFAULT_TIMEOUT = 500;  // ms

var states = {
  NOT_BLOCKED: 'NOT_BLOCKED',
  WAITING_FOR_ACK: 'WAITING_FOR_ACK',
  WAITING_FOR_ACK_RESET: 'WAITING_FOR_ACK_RESET'
};



module.exports = UdpControl;
function UdpControl(options) {
  options = options || {};

  this._sequenceNumber = 0;
  this._socket     = options.socket || dgram.createSocket('udp4');
  this._port       = options.port || constants.ports.AT;
  this._ip         = options.ip ||  constants.DEFAULT_DRONE_IP;
  this.defaultTimeout = options.defaultTimeout || DEFAULT_TIMEOUT;
  this._cmdCreator = new AtCommandCreator();
  this._cmds       = [];
  this._blockingCmds = [];
  this._blocked = {
    state: states.NOT_BLOCKED,
    cmd: null,
    ackTimer: null
  };
}

meta.methods(AtCommandCreator).forEach(function(methodName) {
  UdpControl.prototype[methodName] = function() {
    var cmd = this._cmdCreator[methodName].apply(this._cmdCreator, arguments);
    if (cmd.blocks) {
      this._blockingCmds.push(cmd);
    } else {
      this._cmds.push(cmd);
    }
    return cmd;
  };
});

UdpControl.prototype.ack = function() {
  if (this._blocked.state === states.WAITING_FOR_ACK) {
    assert(this._blocked.cmd);
    assert(this._blocked.ackTimer);
    debug('Got ACK for ' + this._blocked.cmd);
    clearTimeout(this._blocked.ackTimer);
    this._blocked.ackTimer = null;
    if (this._blocked.cmd.callback) {
      this._blocked.cmd.callback.call(this, null);
    }
    this._blocked.state = states.WAITING_FOR_ACK_RESET;
    debug('Resetting ACK');
    this.ctrl(5, 0);
    // Send ctrl(5, 0) every 50 ms until we get an ACK reset.
    var self = this;
    this._blocked.ctrlTimer = setInterval(
      function() {
        if (self.state === states.WAITING_FOR_ACK_RESET) {
          self.ctrl(5, 0);
        } else {
          clearInterval(self._blocked.ctrlTimer);
          self._blocked.ctrlTimer = null;
        }
      },
      50);
  }
};

UdpControl.prototype.ackReset = function() {
  if (this._blocked.state === states.WAITING_FOR_ACK_RESET) {
    assert(this._blocked.cmd);
    debug('Got ACK reset');
    this._blocked.cmd = null;
    this._blocked.state = states.NOT_BLOCKED;
  }
};

UdpControl.prototype._handleAckTimeout = function() {
  assert(this._blocked.state === states.WAITING_FOR_ACK);
  assert(this._blocked.ackTimer);
  assert(this._blocked.cmd);
  this._blocked.ackTimer = null;
  debug('Blocking command timed out:');
  debug(this._blocked.cmd);
  if (this._blocked.cmd.callback) {
    this._blocked.cmd.callback.call(this, new Error('ACK Timeout'));
  }
  this._blocked.cmd = null;
  this._blocked.state = states.NOT_BLOCKED;
};


UdpControl.prototype.flush = function() {
  if (this._cmds.length) {
    this._sendCommands(this._cmds);
    this._cmds = [];
  }
  if (this._blocked.state == states.NOT_BLOCKED && this._blockingCmds.length) {
    // Send the blocking command.
    var cmd = this._blockingCmds.shift();
    this._blocked.cmd = cmd;
    this._sendCommands([cmd]);
    this._blocked.state = states.WAITING_FOR_ACK;
    this._blocked.ackTimer = setTimeout(
      this._handleAckTimeout.bind(this),
      cmd.options.timeout || this.defaultTimeout);
  }
};

UdpControl.prototype._sendCommands = function(commands) {
  var serialized = this._concat(commands);
  debug(serialized.replace('\r', '|'));
  var buffer = new Buffer(serialized);
  this._socket.send(buffer, 0, buffer.length, this._port, this._ip);
};

UdpControl.prototype._concat = function(commands) {
  var self = this;
  return commands.reduce(function(cmds, cmd) {
    return cmds + cmd.serialize(self._sequenceNumber++);
  }, '');
};

UdpControl.prototype.close = function() {
  this._socket.close();
};

}).call(this,_dereq_("buffer").Buffer)
},{"../constants":49,"../misc/meta":54,"./AtCommandCreator":51,"assert":2,"buffer":8,"dgram":1,"simple-debug":87}],53:[function(_dereq_,module,exports){
(function (Buffer){
// module for generic AT command related functionality
var at = exports;

at.floatString = function(number) {
  // Not sure if this is correct, but it works for the example provided in
  // the drone manual ... (should be revisted)
  var buffer = new Buffer(4);
  buffer.writeFloatBE(number, 0);
  return -~parseInt(buffer.toString('hex'), 16) - 1;
};

}).call(this,_dereq_("buffer").Buffer)
},{"buffer":8}],54:[function(_dereq_,module,exports){
var meta = exports;

// lists public methods of a class
meta.methods = function(Constructor) {
  var methods = Object.keys(Constructor.prototype);

  return methods.filter(function(name) {
    return !/^_/.test(name);
  });
};

},{}],55:[function(_dereq_,module,exports){
var util   = _dereq_('util');
var Reader = _dereq_('buffy').Reader;

module.exports = NavdataReader;
util.inherits(NavdataReader, Reader);
function NavdataReader(buffer) {
  Reader.call(this, buffer);
}

NavdataReader.prototype.int16 = function() {
  return this.int16LE();
};

NavdataReader.prototype.uint16 = function() {
  return this.uint16LE();
};

NavdataReader.prototype.int32 = function() {
  return this.int32LE();
};

NavdataReader.prototype.uint32 = function() {
  return this.uint32LE();
};

NavdataReader.prototype.float32 = function() {
  return this.float32LE();
};

NavdataReader.prototype.double64 = function() {
  return this.double64LE();
};

NavdataReader.prototype.char = function() {
  return this.uint8();
};

NavdataReader.prototype.bool = function() {
  return !!this.char();
};

NavdataReader.prototype.matrix33 = function() {
  return {
    m11 : this.float32(),
    m12 : this.float32(),
    m13 : this.float32(),
    m21 : this.float32(),
    m22 : this.float32(),
    m23 : this.float32(),
    m31 : this.float32(),
    m32 : this.float32(),
    m33 : this.float32()
  };
};

NavdataReader.prototype.vector31 = function() {
  return {
    x : this.float32(),
    y : this.float32(),
    z : this.float32()
  };
};

NavdataReader.prototype.screenPoint = function() {
  return {
    x : this.int32(),
    y : this.int32()
  };
};

NavdataReader.prototype.satChannel = function() {
  return {
    sat: this.uint8(),
    cn0: this.uint8()
  };
};

NavdataReader.prototype.mask32 = function(mask) {
  var value = this.uint32();
  return this._mask(mask, value);
};

NavdataReader.prototype._mask = function(mask, value) {
  var flags = {};
  for (var flag in mask) {
    flags[flag] = (value & mask[flag])
      ? 1
      : 0;
  }

  return flags;
};

},{"buffy":64,"util":39}],56:[function(_dereq_,module,exports){
(function (Buffer){
var Stream       = _dereq_('stream').Stream;
var util         = _dereq_('util');
var dgram        = _dereq_('dgram');
var constants    = _dereq_('../constants');
var parseNavdata = _dereq_('./parseNavdata');

module.exports = UdpNavdataStream;
util.inherits(UdpNavdataStream, Stream);
function UdpNavdataStream(options) {
  Stream.call(this);

  options = options || {};

  this.readable        = true;
  this._socket         = options.socket || dgram.createSocket('udp4');
  this._port           = options.port || constants.ports.NAVDATA;
  this._ip             = options.ip || constants.DEFAULT_DRONE_IP;
  this._initialized    = false;
  this._parseNavdata   = options.parser || parseNavdata;
  this._timeout        = options.timeout || 100;
  this._timer          = undefined;
  this._sequenceNumber = 0;
}

UdpNavdataStream.prototype.resume = function() {
  if (!this._initialized) {
    this._init();
    this._initialized = true;
  }

  this._requestNavdata();
};

UdpNavdataStream.prototype.destroy = function() {
  this._socket.close();
};

UdpNavdataStream.prototype._init = function() {
  this._socket.bind();
  this._socket.on('message', this._handleMessage.bind(this));
};

UdpNavdataStream.prototype._requestNavdata = function() {
  var buffer = new Buffer([1]);
  this._socket.send(buffer, 0, buffer.length, this._port, this._ip);

  this._setTimeout();

  // @TODO logging
};

UdpNavdataStream.prototype._setTimeout = function() {
  clearTimeout(this._timer);
  this._timer = setTimeout(this._requestNavdata.bind(this), this._timeout);
};

UdpNavdataStream.prototype._handleMessage = function(buffer) {
  var navdata = {};

  try {
    navdata = this._parseNavdata(buffer);
  } catch (err) {
    // avoid 'error' causing an exception when nobody is listening
    if (this.listeners('error').length > 0) {
      this.emit('error', err);
    }
    return;
  }

  // Ignore out of order messages
  if (navdata.sequenceNumber > this._sequenceNumber) {
    this._sequenceNumber = navdata.sequenceNumber;
    this.emit('data', navdata);
  }

  this._setTimeout();
};

}).call(this,_dereq_("buffer").Buffer)
},{"../constants":49,"./parseNavdata":57,"buffer":8,"dgram":1,"stream":34,"util":39}],57:[function(_dereq_,module,exports){
var NavdataReader = _dereq_('./NavdataReader');

// call `iterator` `n` times, returning an array of the results
var timesMap = function(n, iterator, context) {
  var results = [];
  for (var i = 0; i < n; i++) {
    results[i] = iterator.call(context, i);
  }
  return results;
};

var droneTimeToMilliSeconds = function (time) {
  // first 11 bits are seconds
  var seconds = time >>> 21;
  // last 21 bits are microseconds
  var microseconds = (time << 11) >>> 11;

  // Convert to ms (which is idiomatic for time in JS)
  return seconds * 1000 + microseconds / 1000;
};

var exports = module.exports = function parseNavdata(buffer) {
  var reader = new NavdataReader(buffer);

  var navdata = {};

  navdata.header = reader.uint32();
  if (navdata.header !== exports.NAVDATA_HEADER1 &&
     navdata.header !== exports.NAVDATA_HEADER2) {
    throw new Error('Invalid header: 0x' + navdata.header.toString(16));
  }

  navdata.droneState     = reader.mask32(exports.DRONE_STATES);
  navdata.sequenceNumber = reader.uint32();
  navdata.visionFlag     = reader.uint32();

  while (true) {
    var optionId   = reader.uint16();
    var optionName = exports.OPTION_IDS[optionId];
    var length     = reader.uint16();

    // length includes header size (4 bytes)
    var optionReader = new NavdataReader(reader.buffer(length - 4));

    if (optionName == 'checksum') {
      var expectedChecksum = 0;
      for (var i = 0; i < buffer.length - length; i++) {
        expectedChecksum += buffer[i];
      }

      var checksum = optionReader.uint32();

      if (checksum !== expectedChecksum) {
        throw new Error('Invalid checksum, expected: ' + expectedChecksum + ', got: ' + checksum);
      }

      // checksum is the last option
      break;
    }

    // parse option according to  ARDrone_SDK_2_0/ARDroneLib/Soft/Common/navdata_common.h)
    navdata[optionName] = (exports.OPTION_PARSERS[optionName])
      ? exports.OPTION_PARSERS[optionName](optionReader)
      : new Error('Option not implemented yet: ' + optionName + ' (0x' + optionId.toString(16) + ')');
  }

  return navdata;
};

exports.OPTION_PARSERS = {
  'demo': function(reader) {
    // simplified version of ctrl_state_str (in ARDrone_SDK_2_0/Examples/Linux/Navigation/Sources/navdata_client/navdata_ihm.c)
    var flyState          = exports.FLY_STATES[reader.uint16()];
    var controlState      = exports.CONTROL_STATES[reader.uint16()];
    var batteryPercentage = reader.uint32();
    var theta             = reader.float32() / 1000; // [mdeg]
    var phi               = reader.float32() / 1000; // [mdeg]
    var psi               = reader.float32() / 1000; // [mdeg]
    var altitude          = reader.int32()   / 1000; // [mm]
    var velocity          = reader.vector31(); // [mm/s]
    var frameIndex = reader.uint32();
    var detection = {
      camera: {
        rotation    : reader.matrix33(),
        translation : reader.vector31()
      },
      tagIndex: reader.uint32()
    };
    detection.camera.type = reader.uint32();
    var drone = {
      camera: {
        rotation    : reader.matrix33(),
        translation : reader.vector31()
      }
    };
    var rotation = {
      frontBack : theta,
      pitch     : theta,
      theta     : theta,
      y         : theta,
      leftRight : phi,
      roll      : phi,
      phi       : phi,
      x         : phi,
      clockwise : psi,
      yaw       : psi,
      psi       : psi,
      z         : psi
    };

    return {
      controlState      : controlState,
      flyState          : flyState,
      batteryPercentage : batteryPercentage,
      rotation          : rotation,
      frontBackDegrees  : theta,
      leftRightDegrees  : phi,
      clockwiseDegrees  : psi,
      altitude          : altitude,
      altitudeMeters    : altitude,
      velocity          : velocity,
      xVelocity         : velocity.x,
      yVelocity         : velocity.y,
      zVelocity         : velocity.z,
      frameIndex        : frameIndex,
      detection         : detection,
      drone             : drone
    };
  },

  'time': function(reader) {
    var time = reader.uint32();
    return droneTimeToMilliSeconds(time);
  },

  'rawMeasures': function(reader) {
    var accelerometers = {
      x: reader.uint16(), // [LSB]
      y: reader.uint16(), // [LSB]
      z: reader.uint16()  // [LSB]
    };
    var gyroscopes = {
      x: reader.int16(), // [LSB]
      y: reader.int16(), // [LSB]
      z: reader.int16()  // [LSB]
    };
    // gyroscopes  x/y 110 deg/s (@TODO figure out what that means)
    var gyroscopes110 = {
      x: reader.int16(), // [LSB]
      y: reader.int16()  // [LSB]
    };
    var batteryMilliVolt = reader.uint32(); // [mV]
    // no idea what any of those are
    var usEcho = {
      start       : reader.uint16(), // [LSB]
      end         : reader.uint16(), // [LSB]
      association : reader.uint16(), // [LSB?]
      distance    : reader.uint16()  // [LSB]
    };
    var usCurve = {
      time  : reader.uint16(), // [LSB]
      value : reader.uint16(), // [LSB]
      ref   : reader.uint16()  // [LSB]
    };
    var echo = {
      flagIni : reader.uint16(), // [LSB]
      num     : reader.uint16(), // [LSB] (is key `num` OR `name`?)
      sum     : reader.uint32()  // [LSB]
    };
    var altTemp  = reader.int32(); // [mm]

    return {
      accelerometers    : accelerometers,
      gyroscopes        : gyroscopes,
      gyrometers        : gyroscopes,
      gyroscopes110     : gyroscopes110,
      gyrometers110     : [gyroscopes110.x, gyroscopes110.y],
      batteryMilliVolt  : batteryMilliVolt,
      us                : {echo: usEcho, curve: usCurve},
      usDebutEcho       : usEcho.start,
      usFinEcho         : usEcho.end,
      usAssociationEcho : usEcho.association,
      usDistanceEcho    : usEcho.distance,
      usCourbeTemps     : usCurve.time,
      usCourbeValeur    : usCurve.value,
      usCourbeRef       : usCurve.ref,
      echo              : echo,
      flagEchoIni       : echo.flagIni,
      nbEcho            : echo.num,
      sumEcho           : echo.sum,
      altTemp           : altTemp,
      altTempRaw        : altTemp
    };
  },

  'physMeasures': function(reader) {
    return {
      temperature: {
        accelerometer: reader.float32(), // [K]
        gyroscope: reader.uint16()  // [LSB]
      },
      accelerometers : reader.vector31(), // [mg]
      gyroscopes     : reader.vector31(), // [deg/s]
      alim3V3        : reader.uint32(), // [LSB]
      vrefEpson      : reader.uint32(), // [LSB]
      vrefIDG        : reader.uint32()  // [LSB]
    };
  },

  'gyrosOffsets': function(reader) {
    return reader.vector31();  // [LSB]
  },

  'eulerAngles': function(reader) {
    return {
      theta : reader.float32(), // [mdeg?]
      phi   : reader.float32()  // [mdeg?]
    };
  },

  'references': function(reader) {
    return {
      theta    : reader.int32(), // [mdeg]
      phi      : reader.int32(), // [mdeg]
      thetaI   : reader.int32(), // [mdeg]
      phiI     : reader.int32(), // [mdeg]
      pitch    : reader.int32(), // [mdeg]
      roll     : reader.int32(), // [mdeg]
      yaw      : reader.int32(), // [mdeg/s]
      psi      : reader.int32(), // [mdeg]

      vx       : reader.float32(),
      vy       : reader.float32(),
      thetaMod : reader.float32(),
      phiMod   : reader.float32(),

      kVX      : reader.float32(),
      kVY      : reader.float32(),
      kMode    : reader.uint32(),

      ui       : {
        time        : reader.float32(),
        theta       : reader.float32(),
        phi         : reader.float32(),
        psi         : reader.float32(),
        psiAccuracy : reader.float32(),
        seq         : reader.int32()
      }
    };
  },

  'trims': function(reader) {
    return {
      angularRates : {
        r : reader.float32()
      },
      eulerAngles : {
        theta : reader.float32(), // [mdeg?]
        phi   : reader.float32()  // [mdeg?]
      }
    };
  },

  'rcReferences': function(reader) {
    return {
      pitch : reader.int32(), // [mdeg?]
      roll  : reader.int32(), // [mdeg?]
      yaw   : reader.int32(), // [mdeg/s?]
      gaz   : reader.int32(),
      ag    : reader.int32()
    };
  },

  'pwm': function(reader) {
    return {
      motors           : timesMap(4, reader.uint8, reader), // [PWM]
      satMotors        : timesMap(4, reader.uint8, reader), // [PWM]
      gazFeedForward   : reader.float32(), // [PWM]
      gazAltitude      : reader.float32(), // [PWM]
      altitudeIntegral : reader.float32(), // [mm/s]
      vzRef            : reader.float32(), // [mm/s]
      uPitch           : reader.int32(), // [PWM]
      uRoll            : reader.int32(), // [PWM]
      uYaw             : reader.int32(), // [PWM]
      yawUI            : reader.float32(), // [PWM] yaw_u_I
      uPitchPlanif     : reader.int32(), // [PWM]
      uRollPlanif      : reader.int32(), // [PWM]
      uYawPlanif       : reader.int32(), // [PWM]
      uGazPlanif       : reader.float32(), // [PWM]
      motorCurrents    : timesMap(4, reader.uint16, reader), // [mA]
      altitudeProp     : reader.float32(), // [PWM]
      altitudeDer      : reader.float32()  // [PWM]
    };
  },

  'altitude': function(reader) {
    return {
      vision   : reader.int32(),   // [mm]
      velocity : reader.float32(), // [mm/s]
      ref      : reader.int32(),   // [mm]
      raw      : reader.int32(),   // [mm]
      observer : {
        acceleration : reader.float32(), // [m/s2]
        altitude     : reader.float32(), // [m]
        x            : reader.vector31(),
        state        : reader.uint32()
      },
      estimated : {
        vb    : {
          x : reader.float32(),
          y : reader.float32()
        },
        state : reader.uint32()
      }
    };
  },

  'visionRaw': function(reader) {
    return {
      tx : reader.float32(),
      ty : reader.float32(),
      tz : reader.float32()
    };
  },

  'visionOf': function(reader) {
    return {
      dx : timesMap(5, reader.float32, reader),
      dy : timesMap(5, reader.float32, reader)
    };
  },

  'vision': function(reader) {
    return {
      state         : reader.uint32(),
      misc          : reader.int32(),
      phi: {
        trim       : reader.float32(), // [rad]
        refProp    : reader.float32()  // [rad]
      },
      theta: {
        trim     : reader.float32(), // [rad]
        refProp  : reader.float32()  // [rad]
      },
      newRawPicture : reader.int32(),
      capture       : {
        theta    : reader.float32(),
        phi      : reader.float32(),
        psi      : reader.float32(),
        altitude : reader.int32(),
        time     : droneTimeToMilliSeconds(reader.uint32())
      },
      bodyV         : reader.vector31(), // [mm/s]
      delta         : {
        phi   : reader.float32(),
        theta : reader.float32(),
        psi   : reader.float32()
      },
      gold          : {
        defined : reader.uint32(),
        reset   : reader.uint32(),
        x       : reader.float32(),
        y       : reader.float32()
      }
    };
  },

  'visionPerf': function(reader) {
    return {
      szo      : reader.float32(),
      corners  : reader.float32(),
      compute  : reader.float32(),
      tracking : reader.float32(),
      trans    : reader.float32(),
      update   : reader.float32(),
      custom   : timesMap(20, reader.float32, reader)
    };
  },

  'trackersSend': function(reader) {
    return {
      locked : timesMap(30, reader.int32, reader),
      point  : timesMap(30, reader.screenPoint, reader)
    };
  },

  'visionDetect': function(reader) {
    return {
      nbDetected       : reader.uint32(),
      type             : timesMap(4, reader.uint32, reader),
      xc               : timesMap(4, reader.uint32, reader),
      yc               : timesMap(4, reader.uint32, reader),
      width            : timesMap(4, reader.uint32, reader),
      height           : timesMap(4, reader.uint32, reader),
      dist             : timesMap(4, reader.uint32, reader),
      orientationAngle : timesMap(4, reader.float32, reader),
      rotation         : timesMap(4, reader.matrix33, reader),
      translation      : timesMap(4, reader.vector31, reader),
      cameraSource     : timesMap(4, reader.uint32, reader)
    };
  },

  'watchdog': function(reader) {
    return reader.uint32();
  },

  'adcDataFrame': function(reader) {
    return {
      version   : reader.uint32(),
      dataFrame : timesMap(32, reader.uint8, reader)
    };
  },

  'videoStream': function(reader) {
    return {
      quant          : reader.uint8(),
      frame          : {
        size   : reader.uint32(), // [bytes]
        number : reader.uint32()
      },
      atcmd          : {
        sequence : reader.uint32(),
        meanGap  : reader.uint32(), // [ms]
        varGap   : reader.float32(), // [SU]
        quality  : reader.uint32()
      },
      bitrate        : {
        out     : reader.uint32(),
        desired : reader.uint32()
      },
      data           : timesMap(5, reader.int32, reader),
      tcpQueueLevel  : reader.uint32(),
      fifoQueueLevel : reader.uint32()
    };
  },

  'games': function(reader) {
    return {
      counters: {
        doubleTap  : reader.uint32(),
        finishLine : reader.uint32()
      }
    };
  },

  'pressureRaw': function(reader) {
    return {
      up          : reader.int32(), // [LSB] (UP?)
      ut          : reader.int16(), // [LSB] (UT?)
      temperature : reader.int32(), // [0_1C]
      pressure    : reader.int32()  // [Pa]
    };
  },

  'magneto': function(reader) {
    return {
      mx        : reader.int16(), // [LSB]
      my        : reader.int16(), // [LSB]
      mz        : reader.int16(), // [LSB]
      raw       : reader.vector31(), // [mG]
      rectified : reader.vector31(), // [mG]
      offset    : reader.vector31(), // [mG]
      heading   : {
        unwrapped: reader.float32(), // [deg]
        gyroUnwrapped: reader.float32(), // [deg]
        fusionUnwrapped: reader.float32() // [mdeg]
      },
      ok     : reader.char(),
      state  : reader.uint32(), // [SU]
      radius : reader.float32(), // [mG]
      error  : {
        mean     : reader.float32(), // [mG]
        variance : reader.float32()  // [mG2]
      }
    };
  },

  'windSpeed': function(reader) {
    return {
      speed: reader.float32(), // [m/s]
      angle: reader.float32(), // [deg]
      compensation: {
        theta: reader.float32(), // [rad]
        phi: reader.float32()
      },
      stateX: timesMap(6, reader.float32, reader), // [SU]
      debug: timesMap(3, reader.float32, reader)
    };
  },

  'kalmanPressure': function(reader) {
    return {
      offsetPressure: reader.float32(), // [Pa]
      estimated: {
        altitude: reader.float32(), // [mm]
        velocity: reader.float32(), // [m/s]
        angle: {
          pwm: reader.float32(), // [m/s2]
          pressure: reader.float32() // [Pa]
        },
        us: {
          offset: reader.float32(), // [m]
          prediction: reader.float32() // [mm]
        },
        covariance: {
          alt: reader.float32(), // [m]
          pwm: reader.float32(),
          velocity: reader.float32() // [m/s]
        },
        groundEffect: reader.bool(), // [SU]
        sum: reader.float32(), // [mm]
        reject: reader.bool(), // [SU]
        uMultisinus: reader.float32(),
        gazAltitude: reader.float32(),
        flagMultisinus: reader.bool(),
        flagMultisinusStart: reader.bool()
      }
    };
  },

  'hdvideoStream': function(reader) {
    var values = {
      hdvideoState : reader.uint32(),
      storageFifo  : {
        nbPackets : reader.uint32(),
        size      : reader.uint32()
      },
      usbkey       : {
        size      : reader.uint32(),
        freespace : reader.uint32()
      },
      frameNumber  : reader.uint32()
    };
    values.usbkey.remainingTime = reader.uint32();

    return values;
  },

  'wifi': function(reader) {
    return {
      // from ARDrone_SDK_2_0/ControlEngine/iPhone/Classes/Controllers/MainViewController.m
      linkQuality: (1 - (reader.float32()))
    };
  },

  'gps': function(reader) {
    return {
      // from https://github.com/lesire/ardrone_autonomy/commit/a986b3380da8d9306407e2ebfe7e0f2cd5f97683
      latitude:             reader.double64(),
      longitude:            reader.double64(),
      elevation:            reader.double64(),
      hdop:                 reader.double64(),
      dataAvailable:        reader.int32(),
      zeroValidated:        reader.int32(),
      wptValidated:         reader.int32(),
      lat0:                 reader.double64(),
      lon0:                 reader.double64(),
      latFuse:              reader.double64(),
      lonFuse:              reader.double64(),
      gpsState:             reader.uint32(),
      xTraj:                reader.float32(),
      xRef:                 reader.float32(),
      yTraj:                reader.float32(),
      yRef:                 reader.float32(),
      thetaP:               reader.float32(),
      phiP:                 reader.float32(),
      thetaI:               reader.float32(),
      phiI:                 reader.float32(),
      thetaD:               reader.float32(),
      phiD:                 reader.float32(),
      vdop:                 reader.double64(),
      pdop:                 reader.double64(),
      speed:                reader.float32(),
      lastFrameTimestamp:   droneTimeToMilliSeconds(reader.uint32()),
      degree:               reader.float32(),
      degreeMag:            reader.float32(),
      ehpe:                 reader.float32(),
      ehve:                 reader.float32(),
      c_n0:                 reader.float32(),
      nbSatellites:         reader.uint32(),
      channels:             timesMap(12, reader.satChannel, reader),
      gpsPlugged:           reader.int32(),
      ephemerisStatus:      reader.uint32(),
      vxTraj:               reader.float32(),
      vyTraj:               reader.float32(),
      firmwareStatus:       reader.uint32()
    };
  }
};


// Constants

exports.NAVDATA_HEADER1 = 0x55667788;
exports.NAVDATA_HEADER2 = 0x55667789;

// from ARDrone_SDK_2_0/ARDroneLib/Soft/Common/config.h
exports.DRONE_STATES = {
  flying                     : 1 << 0,  /*!< FLY MASK : (0) ardrone is landed, (1) ardrone is flying */
  videoEnabled               : 1 << 1,  /*!< VIDEO MASK : (0) video disable, (1) video enable */
  visionEnabled              : 1 << 2,  /*!< VISION MASK : (0) vision disable, (1) vision enable */
  controlAlgorithm           : 1 << 3,  /*!< CONTROL ALGO : (0) euler angles control, (1) angular speed control */
  altitudeControlAlgorithm   : 1 << 4,  /*!< ALTITUDE CONTROL ALGO : (0) altitude control inactive (1) altitude control active */
  startButtonState           : 1 << 5,  /*!< USER feedback : Start button state */
  controlCommandAck          : 1 << 6,  /*!< Control command ACK : (0) None, (1) one received */
  cameraReady                : 1 << 7,  /*!< CAMERA MASK : (0) camera not ready, (1) Camera ready */
  travellingEnabled          : 1 << 8,  /*!< Travelling mask : (0) disable, (1) enable */
  usbReady                   : 1 << 9,  /*!< USB key : (0) usb key not ready, (1) usb key ready */
  navdataDemo                : 1 << 10, /*!< Navdata demo : (0) All navdata, (1) only navdata demo */
  navdataBootstrap           : 1 << 11, /*!< Navdata bootstrap : (0) options sent in all or demo mode, (1) no navdata options sent */
  motorProblem               : 1 << 12, /*!< Motors status : (0) Ok, (1) Motors problem */
  communicationLost          : 1 << 13, /*!< Communication Lost : (1) com problem, (0) Com is ok */
  softwareFault              : 1 << 14, /*!< Software fault detected - user should land as quick as possible (1) */
  lowBattery                 : 1 << 15, /*!< VBat low : (1) too low, (0) Ok */
  userEmergencyLanding       : 1 << 16, /*!< User Emergency Landing : (1) User EL is ON, (0) User EL is OFF*/
  timerElapsed               : 1 << 17, /*!< Timer elapsed : (1) elapsed, (0) not elapsed */
  MagnometerNeedsCalibration : 1 << 18, /*!< Magnetometer calibration state : (0) Ok, no calibration needed, (1) not ok, calibration needed */
  anglesOutOfRange           : 1 << 19, /*!< Angles : (0) Ok, (1) out of range */
  tooMuchWind                : 1 << 20, /*!< WIND MASK: (0) ok, (1) Too much wind */
  ultrasonicSensorDeaf       : 1 << 21, /*!< Ultrasonic sensor : (0) Ok, (1) deaf */
  cutoutDetected             : 1 << 22, /*!< Cutout system detection : (0) Not detected, (1) detected */
  picVersionNumberOk         : 1 << 23, /*!< PIC Version number OK : (0) a bad version number, (1) version number is OK */
  atCodecThreadOn            : 1 << 24, /*!< ATCodec thread ON : (0) thread OFF (1) thread ON */
  navdataThreadOn            : 1 << 25, /*!< Navdata thread ON : (0) thread OFF (1) thread ON */
  videoThreadOn              : 1 << 26, /*!< Video thread ON : (0) thread OFF (1) thread ON */
  acquisitionThreadOn        : 1 << 27, /*!< Acquisition thread ON : (0) thread OFF (1) thread ON */
  controlWatchdogDelay       : 1 << 28, /*!< CTRL watchdog : (1) delay in control execution (> 5ms), (0) control is well scheduled */
  adcWatchdogDelay           : 1 << 29, /*!< ADC Watchdog : (1) delay in uart2 dsr (> 5ms), (0) uart2 is good */
  comWatchdogProblem         : 1 << 30, /*!< Communication Watchdog : (1) com problem, (0) Com is ok */
  emergencyLanding           : 1 << 31  /*!< Emergency landing : (0) no emergency, (1) emergency */
};

exports.OPTION_IDS = {
  0     : 'demo',
  1     : 'time',
  2     : 'rawMeasures',
  3     : 'physMeasures',
  4     : 'gyrosOffsets',
  5     : 'eulerAngles',
  6     : 'references',
  7     : 'trims',
  8     : 'rcReferences',
  9     : 'pwm',
  10    : 'altitude',
  11    : 'visionRaw',
  12    : 'visionOf',
  13    : 'vision',
  14    : 'visionPerf',
  15    : 'trackersSend',
  16    : 'visionDetect',
  17    : 'watchdog',
  18    : 'adcDataFrame',
  19    : 'videoStream',
  20    : 'games',
  21    : 'pressureRaw',
  22    : 'magneto',
  23    : 'windSpeed',
  24    : 'kalmanPressure',
  25    : 'hdvideoStream',
  26    : 'wifi',
  27    : 'gps',
  65535 : 'checksum'
};

exports.CONTROL_STATES = {
  0: 'CTRL_DEFAULT',
  1: 'CTRL_INIT',
  2: 'CTRL_LANDED',
  3: 'CTRL_FLYING',
  4: 'CTRL_HOVERING',
  5: 'CTRL_TEST',
  6: 'CTRL_TRANS_TAKEOFF',
  7: 'CTRL_TRANS_GOTOFIX',
  8: 'CTRL_TRANS_LANDING',
  9: 'CTRL_TRANS_LOOPING'
};

exports.FLY_STATES = {
  0: 'FLYING_OK',
  1: 'FLYING_LOST_ALT',
  2: 'FLYING_LOST_ALT_GO_DOWN',
  3: 'FLYING_ALT_OUT_ZONE',
  4: 'FLYING_COMBINED_YAW',
  5: 'FLYING_BRAKE',
  6: 'FLYING_NO_VISION'
};

},{"./NavdataReader":55}],58:[function(_dereq_,module,exports){
// Converts a video stream into a stream of png buffers. Each 'data' event
// is guaranteed to contain exactly 1 png image.

// @TODO handle ffmpeg exit (and trigger "error" if unexpected)

var Stream      = _dereq_('stream').Stream;
var util        = _dereq_('util');
var spawn       = _dereq_('child_process').spawn;
var PngSplitter = _dereq_('./PngSplitter');

module.exports = PngEncoder;
util.inherits(PngEncoder, Stream);
function PngEncoder(options) {
  Stream.call(this);

  options = options || {};

  this.writable = true;
  this.readable = true;

  this._spawn       = options.spawn || spawn;
  this._imageSize   = options.imageSize || null;
  this._frameRate   = options.frameRate || 5;
  this._pngSplitter = options.pngSplitter || new PngSplitter();
  this._log         = options.log;
  this._ffmpeg      = undefined;
  this._ending      = false;
}

PngEncoder.prototype.write = function(buffer) {
  if (!this._ffmpeg) {
    this._initFfmpegAndPipes();
  }

  return this._ffmpeg.stdin.write(buffer);
};

PngEncoder.prototype._initFfmpegAndPipes = function() {
  this._ffmpeg = this._spawnFfmpeg();

  // @TODO: Make this more sophisticated for somehow and figure out how it
  // will work with the planned new data recording system.
  if (this._log) {
    this._ffmpeg.stderr.pipe(this._log);
  }

  this._ffmpeg.stdout.pipe(this._pngSplitter);
  this._pngSplitter.on('data', this.emit.bind(this, 'data'));

  // 'error' can be EPIPE if ffmpeg does not exist. We handle this with the
  // 'exit' event below.
  this._ffmpeg.stdin.on('error', function() {});

  this._ffmpeg.stdin.on('drain', this.emit.bind(this, 'drain'));

  var self = this;
  // Since node 0.10, spawn emits 'error' when the child can't be spawned.
  // http://nodejs.org/api/child_process.html#child_process_event_error
  this._ffmpeg.on('error', function(err) {
    if (err.code === 'ENOENT') {
      self.emit('error', new Error('Ffmpeg was not found. Have you installed the ffmpeg package?'));
    } else {
      self.emit('error', new Error('Unexpected error when launching ffmpeg:' + err.toString()));
    }
  });

  this._ffmpeg.on('exit', function(code) {
    if (code === 0) {
      // we expected ffmpeg to exit
      if (self._ending) {
        self.emit('end');
        return;
      }

      self.emit('error', new Error('Unexpected ffmpeg exit with code 0.'));
      return;
    }

    // 127 is used by the OS to indicate that ffmpeg was not found
    // required when using node < 0.10
    if (code === 127) {
      self.emit('error', new Error('Ffmpeg was not found / exit code 127.'));
      return;
    }

    self.emit('error', new Error('Ffmpeg exited with error code: ' + code));
  });
};

PngEncoder.prototype._spawnFfmpeg = function() {
  var ffmpegOptions = [];
  ffmpegOptions.push('-i', '-'); // input flag
  ffmpegOptions.push('-f', 'image2pipe'); // format flag
  if(this._imageSize){
    ffmpegOptions.push('-s', this._imageSize); // size flag
  }
  ffmpegOptions.push('-vcodec', 'png'); // codec flag
  ffmpegOptions.push('-r', this._frameRate); // framerate flag
  ffmpegOptions.push('-'); // output
  // @TODO allow people to configure the ffmpeg path
  return this._spawn('ffmpeg', ffmpegOptions);
};

PngEncoder.prototype.end = function() {
  // No data handled yet? Nothing to do for ending.
  if (!this._ffmpeg) {
    return;
  }

  this._ending = true;
  this._ffmpeg.stdin.end();
};

},{"./PngSplitter":59,"child_process":1,"stream":34,"util":39}],59:[function(_dereq_,module,exports){
(function (Buffer){
var EventEmitter = _dereq_('events').EventEmitter;
var util         = _dereq_('util');

module.exports = PngSplitter;
util.inherits(PngSplitter, EventEmitter);
function PngSplitter() {
  EventEmitter.call(this);

  this.writable        = true;
  this._buffer         = new Buffer(0);
  this._offset         = 0;
  this._pngStartOffset = null;
  this._state          = 'header';
  this._chunk          = null;
}

PngSplitter.prototype.write = function(buffer) {
  this._buffer = Buffer.concat([this._buffer, buffer]);

  while (true) {
    switch (this._state) {
      case 'header':
        this._pngStartOffset = this._offset;

        if (this.remainingBytes() < 8) {
          return;
        }

        this._offset += 8;
        this._state = 'chunk.header';
        break;
      case 'chunk.header':
        if (this.remainingBytes() < 8) {
          return;
        }

        this._chunk = {
          length : this.unsignedNumber(4),
          type   : this.array(4).join(' '),
        };

        this._state = 'chunk.data';
        break;
      case 'chunk.data':
        var chunkSize = this._chunk.length + 4; // 4 bytes = CRC
        if (this.remainingBytes() < chunkSize) {
          return;
        }

        this._offset += chunkSize;

        // IEND chunk
        if (this._chunk.type == '73 69 78 68') {
          var png = this._buffer.slice(this._pngStartOffset, this._offset);
          this.emit('data', png);

          this._buffer = this._buffer.slice(this._offset);
          this._offset = 0;
          this._state  = 'header';
          break;
        }

        this._state = 'chunk.header';
        break;
    }
  }

  return true;
};

PngSplitter.prototype.remainingBytes = function() {
  return this._buffer.length - this._offset;
};

PngSplitter.prototype.array = function(bytes) {
  var array = [];

  for (var i = 0; i < bytes; i++) {
    array.push(this._buffer[this._offset++]);
  }

  return array;
};

PngSplitter.prototype.unsignedNumber = function(bytes) {
  var bytesRead = 0;
  var value     = 0;

  while (bytesRead < bytes) {
    var byte = this._buffer[this._offset++];

    value += byte * Math.pow(256, bytes - bytesRead - 1);

    bytesRead++;
  }

  return value;
};

PngSplitter.prototype.end = function() {
};

}).call(this,_dereq_("buffer").Buffer)
},{"buffer":8,"events":12,"util":39}],60:[function(_dereq_,module,exports){
PngStream.TcpVideoStream = _dereq_('./TcpVideoStream');
PngStream.PngEncoder = _dereq_('./PngEncoder');

var Stream     = _dereq_('stream').Stream;
var util       = _dereq_('util');

module.exports = PngStream;
util.inherits(PngStream, Stream);
function PngStream(options) {
  console.warn('The PngStream class is deprecated. Use client.getPngStream() instead.');
  Stream.call(this);

  options = options || {};

  this.readable        = true;
  this._options        = options;
  this._tcpVideoStream = null;
  this._pngEncoder     = null;
}

PngStream.prototype.resume = function() {
  this._resume();
};

PngStream.prototype._resume = function() {
  var self = this;

  this._tcpVideoStream = new PngStream.TcpVideoStream(this._options);
  this._pngEncoder     = new PngStream.PngEncoder(this._options);

  this._tcpVideoStream.on('error', function(err) {
    self._pngEncoder.end();
    self._resume();

    if (self.listeners('error').length > 0) {
      self.emit('error', err);
    }
  });

  this._tcpVideoStream.connect();
  this._tcpVideoStream.pipe(this._pngEncoder);
  this._pngEncoder.on('data', this.emit.bind(this, 'data'));
  this._pngEncoder.on('error', this.emit.bind(this, 'error'));
};

},{"./PngEncoder":58,"./TcpVideoStream":61,"stream":34,"util":39}],61:[function(_dereq_,module,exports){
var Stream    = _dereq_('stream').Stream;
var util      = _dereq_('util');
var net       = _dereq_('net');
var constants = _dereq_('../constants');

module.exports = TcpVideoStream;
util.inherits(TcpVideoStream, Stream);
function TcpVideoStream(options) {
  Stream.call(this);

  options = options || {};

  this.readable   = true;
  this._socket    = options.socket || new net.Socket();
  this._port      = options.port || constants.ports.VIDEO;
  this._ip        = options.ip || constants.DEFAULT_DRONE_IP;
  this._timeout   = options.timeout || 1 * 1000;
  this._expectFIN = false;
}

TcpVideoStream.prototype.connect = function(cb) {
  cb = cb || function() {};

  this._socket.removeAllListeners();                // To avoid duplicates when re-connecting
  this._socket.connect(this._port, this._ip);
  this._socket.setTimeout(this._timeout);

  var self = this;
  this._socket
    .on('connect', function() {
      self._socket.removeListener('error', cb);
      self._socket.on('error', function(err) {
        self.emit('error', err);
      });
      cb(null);
    })
    .on('data', function(buffer) {
      self.emit('data', buffer);
    })
    .on('timeout', function() {
      var err = new Error('TcpVideoStream timeout after ' + self._timeout + ' ms.');
      self.emit('error', err);
      self.emit('close', err);
      self._socket.destroy();
    })
    .on('error', cb)
    .on('end', function() {
      if (self._expectFIN) {
        self.emit('close');
        return;
      }

      var err = new Error('TcpVideoStream received FIN unexpectedly.');
      self.emit('error', err);
      self.emit('close', err);
    });

};

TcpVideoStream.prototype.end = function() {
  this._expectFIN = true;
  this._socket.end();
};

},{"../constants":49,"net":1,"stream":34,"util":39}],62:[function(_dereq_,module,exports){
(function (process,setImmediate){
/*global setImmediate: false, setTimeout: false, console: false */
(function () {

    var async = {};

    // global on the server, window in the browser
    var root, previous_async;

    root = this;
    if (root != null) {
      previous_async = root.async;
    }

    async.noConflict = function () {
        root.async = previous_async;
        return async;
    };

    function only_once(fn) {
        var called = false;
        return function() {
            if (called) throw new Error("Callback was already called.");
            called = true;
            fn.apply(root, arguments);
        }
    }

    //// cross-browser compatiblity functions ////

    var _each = function (arr, iterator) {
        if (arr.forEach) {
            return arr.forEach(iterator);
        }
        for (var i = 0; i < arr.length; i += 1) {
            iterator(arr[i], i, arr);
        }
    };

    var _map = function (arr, iterator) {
        if (arr.map) {
            return arr.map(iterator);
        }
        var results = [];
        _each(arr, function (x, i, a) {
            results.push(iterator(x, i, a));
        });
        return results;
    };

    var _reduce = function (arr, iterator, memo) {
        if (arr.reduce) {
            return arr.reduce(iterator, memo);
        }
        _each(arr, function (x, i, a) {
            memo = iterator(memo, x, i, a);
        });
        return memo;
    };

    var _keys = function (obj) {
        if (Object.keys) {
            return Object.keys(obj);
        }
        var keys = [];
        for (var k in obj) {
            if (obj.hasOwnProperty(k)) {
                keys.push(k);
            }
        }
        return keys;
    };

    //// exported async module functions ////

    //// nextTick implementation with browser-compatible fallback ////
    if (typeof process === 'undefined' || !(process.nextTick)) {
        if (typeof setImmediate === 'function') {
            async.nextTick = function (fn) {
                // not a direct alias for IE10 compatibility
                setImmediate(fn);
            };
            async.setImmediate = async.nextTick;
        }
        else {
            async.nextTick = function (fn) {
                setTimeout(fn, 0);
            };
            async.setImmediate = async.nextTick;
        }
    }
    else {
        async.nextTick = process.nextTick;
        if (typeof setImmediate !== 'undefined') {
            async.setImmediate = setImmediate;
        }
        else {
            async.setImmediate = async.nextTick;
        }
    }

    async.each = function (arr, iterator, callback) {
        callback = callback || function () {};
        if (!arr.length) {
            return callback();
        }
        var completed = 0;
        _each(arr, function (x) {
            iterator(x, only_once(function (err) {
                if (err) {
                    callback(err);
                    callback = function () {};
                }
                else {
                    completed += 1;
                    if (completed >= arr.length) {
                        callback(null);
                    }
                }
            }));
        });
    };
    async.forEach = async.each;

    async.eachSeries = function (arr, iterator, callback) {
        callback = callback || function () {};
        if (!arr.length) {
            return callback();
        }
        var completed = 0;
        var iterate = function () {
            iterator(arr[completed], function (err) {
                if (err) {
                    callback(err);
                    callback = function () {};
                }
                else {
                    completed += 1;
                    if (completed >= arr.length) {
                        callback(null);
                    }
                    else {
                        iterate();
                    }
                }
            });
        };
        iterate();
    };
    async.forEachSeries = async.eachSeries;

    async.eachLimit = function (arr, limit, iterator, callback) {
        var fn = _eachLimit(limit);
        fn.apply(null, [arr, iterator, callback]);
    };
    async.forEachLimit = async.eachLimit;

    var _eachLimit = function (limit) {

        return function (arr, iterator, callback) {
            callback = callback || function () {};
            if (!arr.length || limit <= 0) {
                return callback();
            }
            var completed = 0;
            var started = 0;
            var running = 0;

            (function replenish () {
                if (completed >= arr.length) {
                    return callback();
                }

                while (running < limit && started < arr.length) {
                    started += 1;
                    running += 1;
                    iterator(arr[started - 1], function (err) {
                        if (err) {
                            callback(err);
                            callback = function () {};
                        }
                        else {
                            completed += 1;
                            running -= 1;
                            if (completed >= arr.length) {
                                callback();
                            }
                            else {
                                replenish();
                            }
                        }
                    });
                }
            })();
        };
    };


    var doParallel = function (fn) {
        return function () {
            var args = Array.prototype.slice.call(arguments);
            return fn.apply(null, [async.each].concat(args));
        };
    };
    var doParallelLimit = function(limit, fn) {
        return function () {
            var args = Array.prototype.slice.call(arguments);
            return fn.apply(null, [_eachLimit(limit)].concat(args));
        };
    };
    var doSeries = function (fn) {
        return function () {
            var args = Array.prototype.slice.call(arguments);
            return fn.apply(null, [async.eachSeries].concat(args));
        };
    };


    var _asyncMap = function (eachfn, arr, iterator, callback) {
        var results = [];
        arr = _map(arr, function (x, i) {
            return {index: i, value: x};
        });
        eachfn(arr, function (x, callback) {
            iterator(x.value, function (err, v) {
                results[x.index] = v;
                callback(err);
            });
        }, function (err) {
            callback(err, results);
        });
    };
    async.map = doParallel(_asyncMap);
    async.mapSeries = doSeries(_asyncMap);
    async.mapLimit = function (arr, limit, iterator, callback) {
        return _mapLimit(limit)(arr, iterator, callback);
    };

    var _mapLimit = function(limit) {
        return doParallelLimit(limit, _asyncMap);
    };

    // reduce only has a series version, as doing reduce in parallel won't
    // work in many situations.
    async.reduce = function (arr, memo, iterator, callback) {
        async.eachSeries(arr, function (x, callback) {
            iterator(memo, x, function (err, v) {
                memo = v;
                callback(err);
            });
        }, function (err) {
            callback(err, memo);
        });
    };
    // inject alias
    async.inject = async.reduce;
    // foldl alias
    async.foldl = async.reduce;

    async.reduceRight = function (arr, memo, iterator, callback) {
        var reversed = _map(arr, function (x) {
            return x;
        }).reverse();
        async.reduce(reversed, memo, iterator, callback);
    };
    // foldr alias
    async.foldr = async.reduceRight;

    var _filter = function (eachfn, arr, iterator, callback) {
        var results = [];
        arr = _map(arr, function (x, i) {
            return {index: i, value: x};
        });
        eachfn(arr, function (x, callback) {
            iterator(x.value, function (v) {
                if (v) {
                    results.push(x);
                }
                callback();
            });
        }, function (err) {
            callback(_map(results.sort(function (a, b) {
                return a.index - b.index;
            }), function (x) {
                return x.value;
            }));
        });
    };
    async.filter = doParallel(_filter);
    async.filterSeries = doSeries(_filter);
    // select alias
    async.select = async.filter;
    async.selectSeries = async.filterSeries;

    var _reject = function (eachfn, arr, iterator, callback) {
        var results = [];
        arr = _map(arr, function (x, i) {
            return {index: i, value: x};
        });
        eachfn(arr, function (x, callback) {
            iterator(x.value, function (v) {
                if (!v) {
                    results.push(x);
                }
                callback();
            });
        }, function (err) {
            callback(_map(results.sort(function (a, b) {
                return a.index - b.index;
            }), function (x) {
                return x.value;
            }));
        });
    };
    async.reject = doParallel(_reject);
    async.rejectSeries = doSeries(_reject);

    var _detect = function (eachfn, arr, iterator, main_callback) {
        eachfn(arr, function (x, callback) {
            iterator(x, function (result) {
                if (result) {
                    main_callback(x);
                    main_callback = function () {};
                }
                else {
                    callback();
                }
            });
        }, function (err) {
            main_callback();
        });
    };
    async.detect = doParallel(_detect);
    async.detectSeries = doSeries(_detect);

    async.some = function (arr, iterator, main_callback) {
        async.each(arr, function (x, callback) {
            iterator(x, function (v) {
                if (v) {
                    main_callback(true);
                    main_callback = function () {};
                }
                callback();
            });
        }, function (err) {
            main_callback(false);
        });
    };
    // any alias
    async.any = async.some;

    async.every = function (arr, iterator, main_callback) {
        async.each(arr, function (x, callback) {
            iterator(x, function (v) {
                if (!v) {
                    main_callback(false);
                    main_callback = function () {};
                }
                callback();
            });
        }, function (err) {
            main_callback(true);
        });
    };
    // all alias
    async.all = async.every;

    async.sortBy = function (arr, iterator, callback) {
        async.map(arr, function (x, callback) {
            iterator(x, function (err, criteria) {
                if (err) {
                    callback(err);
                }
                else {
                    callback(null, {value: x, criteria: criteria});
                }
            });
        }, function (err, results) {
            if (err) {
                return callback(err);
            }
            else {
                var fn = function (left, right) {
                    var a = left.criteria, b = right.criteria;
                    return a < b ? -1 : a > b ? 1 : 0;
                };
                callback(null, _map(results.sort(fn), function (x) {
                    return x.value;
                }));
            }
        });
    };

    async.auto = function (tasks, callback) {
        callback = callback || function () {};
        var keys = _keys(tasks);
        if (!keys.length) {
            return callback(null);
        }

        var results = {};

        var listeners = [];
        var addListener = function (fn) {
            listeners.unshift(fn);
        };
        var removeListener = function (fn) {
            for (var i = 0; i < listeners.length; i += 1) {
                if (listeners[i] === fn) {
                    listeners.splice(i, 1);
                    return;
                }
            }
        };
        var taskComplete = function () {
            _each(listeners.slice(0), function (fn) {
                fn();
            });
        };

        addListener(function () {
            if (_keys(results).length === keys.length) {
                callback(null, results);
                callback = function () {};
            }
        });

        _each(keys, function (k) {
            var task = (tasks[k] instanceof Function) ? [tasks[k]]: tasks[k];
            var taskCallback = function (err) {
                var args = Array.prototype.slice.call(arguments, 1);
                if (args.length <= 1) {
                    args = args[0];
                }
                if (err) {
                    var safeResults = {};
                    _each(_keys(results), function(rkey) {
                        safeResults[rkey] = results[rkey];
                    });
                    safeResults[k] = args;
                    callback(err, safeResults);
                    // stop subsequent errors hitting callback multiple times
                    callback = function () {};
                }
                else {
                    results[k] = args;
                    async.setImmediate(taskComplete);
                }
            };
            var requires = task.slice(0, Math.abs(task.length - 1)) || [];
            var ready = function () {
                return _reduce(requires, function (a, x) {
                    return (a && results.hasOwnProperty(x));
                }, true) && !results.hasOwnProperty(k);
            };
            if (ready()) {
                task[task.length - 1](taskCallback, results);
            }
            else {
                var listener = function () {
                    if (ready()) {
                        removeListener(listener);
                        task[task.length - 1](taskCallback, results);
                    }
                };
                addListener(listener);
            }
        });
    };

    async.waterfall = function (tasks, callback) {
        callback = callback || function () {};
        if (tasks.constructor !== Array) {
          var err = new Error('First argument to waterfall must be an array of functions');
          return callback(err);
        }
        if (!tasks.length) {
            return callback();
        }
        var wrapIterator = function (iterator) {
            return function (err) {
                if (err) {
                    callback.apply(null, arguments);
                    callback = function () {};
                }
                else {
                    var args = Array.prototype.slice.call(arguments, 1);
                    var next = iterator.next();
                    if (next) {
                        args.push(wrapIterator(next));
                    }
                    else {
                        args.push(callback);
                    }
                    async.setImmediate(function () {
                        iterator.apply(null, args);
                    });
                }
            };
        };
        wrapIterator(async.iterator(tasks))();
    };

    var _parallel = function(eachfn, tasks, callback) {
        callback = callback || function () {};
        if (tasks.constructor === Array) {
            eachfn.map(tasks, function (fn, callback) {
                if (fn) {
                    fn(function (err) {
                        var args = Array.prototype.slice.call(arguments, 1);
                        if (args.length <= 1) {
                            args = args[0];
                        }
                        callback.call(null, err, args);
                    });
                }
            }, callback);
        }
        else {
            var results = {};
            eachfn.each(_keys(tasks), function (k, callback) {
                tasks[k](function (err) {
                    var args = Array.prototype.slice.call(arguments, 1);
                    if (args.length <= 1) {
                        args = args[0];
                    }
                    results[k] = args;
                    callback(err);
                });
            }, function (err) {
                callback(err, results);
            });
        }
    };

    async.parallel = function (tasks, callback) {
        _parallel({ map: async.map, each: async.each }, tasks, callback);
    };

    async.parallelLimit = function(tasks, limit, callback) {
        _parallel({ map: _mapLimit(limit), each: _eachLimit(limit) }, tasks, callback);
    };

    async.series = function (tasks, callback) {
        callback = callback || function () {};
        if (tasks.constructor === Array) {
            async.mapSeries(tasks, function (fn, callback) {
                if (fn) {
                    fn(function (err) {
                        var args = Array.prototype.slice.call(arguments, 1);
                        if (args.length <= 1) {
                            args = args[0];
                        }
                        callback.call(null, err, args);
                    });
                }
            }, callback);
        }
        else {
            var results = {};
            async.eachSeries(_keys(tasks), function (k, callback) {
                tasks[k](function (err) {
                    var args = Array.prototype.slice.call(arguments, 1);
                    if (args.length <= 1) {
                        args = args[0];
                    }
                    results[k] = args;
                    callback(err);
                });
            }, function (err) {
                callback(err, results);
            });
        }
    };

    async.iterator = function (tasks) {
        var makeCallback = function (index) {
            var fn = function () {
                if (tasks.length) {
                    tasks[index].apply(null, arguments);
                }
                return fn.next();
            };
            fn.next = function () {
                return (index < tasks.length - 1) ? makeCallback(index + 1): null;
            };
            return fn;
        };
        return makeCallback(0);
    };

    async.apply = function (fn) {
        var args = Array.prototype.slice.call(arguments, 1);
        return function () {
            return fn.apply(
                null, args.concat(Array.prototype.slice.call(arguments))
            );
        };
    };

    var _concat = function (eachfn, arr, fn, callback) {
        var r = [];
        eachfn(arr, function (x, cb) {
            fn(x, function (err, y) {
                r = r.concat(y || []);
                cb(err);
            });
        }, function (err) {
            callback(err, r);
        });
    };
    async.concat = doParallel(_concat);
    async.concatSeries = doSeries(_concat);

    async.whilst = function (test, iterator, callback) {
        if (test()) {
            iterator(function (err) {
                if (err) {
                    return callback(err);
                }
                async.whilst(test, iterator, callback);
            });
        }
        else {
            callback();
        }
    };

    async.doWhilst = function (iterator, test, callback) {
        iterator(function (err) {
            if (err) {
                return callback(err);
            }
            if (test()) {
                async.doWhilst(iterator, test, callback);
            }
            else {
                callback();
            }
        });
    };

    async.until = function (test, iterator, callback) {
        if (!test()) {
            iterator(function (err) {
                if (err) {
                    return callback(err);
                }
                async.until(test, iterator, callback);
            });
        }
        else {
            callback();
        }
    };

    async.doUntil = function (iterator, test, callback) {
        iterator(function (err) {
            if (err) {
                return callback(err);
            }
            if (!test()) {
                async.doUntil(iterator, test, callback);
            }
            else {
                callback();
            }
        });
    };

    async.queue = function (worker, concurrency) {
        if (concurrency === undefined) {
            concurrency = 1;
        }
        function _insert(q, data, pos, callback) {
          if(data.constructor !== Array) {
              data = [data];
          }
          _each(data, function(task) {
              var item = {
                  data: task,
                  callback: typeof callback === 'function' ? callback : null
              };

              if (pos) {
                q.tasks.unshift(item);
              } else {
                q.tasks.push(item);
              }

              if (q.saturated && q.tasks.length === concurrency) {
                  q.saturated();
              }
              async.setImmediate(q.process);
          });
        }

        var workers = 0;
        var q = {
            tasks: [],
            concurrency: concurrency,
            saturated: null,
            empty: null,
            drain: null,
            push: function (data, callback) {
              _insert(q, data, false, callback);
            },
            unshift: function (data, callback) {
              _insert(q, data, true, callback);
            },
            process: function () {
                if (workers < q.concurrency && q.tasks.length) {
                    var task = q.tasks.shift();
                    if (q.empty && q.tasks.length === 0) {
                        q.empty();
                    }
                    workers += 1;
                    var next = function () {
                        workers -= 1;
                        if (task.callback) {
                            task.callback.apply(task, arguments);
                        }
                        if (q.drain && q.tasks.length + workers === 0) {
                            q.drain();
                        }
                        q.process();
                    };
                    var cb = only_once(next);
                    worker(task.data, cb);
                }
            },
            length: function () {
                return q.tasks.length;
            },
            running: function () {
                return workers;
            }
        };
        return q;
    };

    async.cargo = function (worker, payload) {
        var working     = false,
            tasks       = [];

        var cargo = {
            tasks: tasks,
            payload: payload,
            saturated: null,
            empty: null,
            drain: null,
            push: function (data, callback) {
                if(data.constructor !== Array) {
                    data = [data];
                }
                _each(data, function(task) {
                    tasks.push({
                        data: task,
                        callback: typeof callback === 'function' ? callback : null
                    });
                    if (cargo.saturated && tasks.length === payload) {
                        cargo.saturated();
                    }
                });
                async.setImmediate(cargo.process);
            },
            process: function process() {
                if (working) return;
                if (tasks.length === 0) {
                    if(cargo.drain) cargo.drain();
                    return;
                }

                var ts = typeof payload === 'number'
                            ? tasks.splice(0, payload)
                            : tasks.splice(0);

                var ds = _map(ts, function (task) {
                    return task.data;
                });

                if(cargo.empty) cargo.empty();
                working = true;
                worker(ds, function () {
                    working = false;

                    var args = arguments;
                    _each(ts, function (data) {
                        if (data.callback) {
                            data.callback.apply(null, args);
                        }
                    });

                    process();
                });
            },
            length: function () {
                return tasks.length;
            },
            running: function () {
                return working;
            }
        };
        return cargo;
    };

    var _console_fn = function (name) {
        return function (fn) {
            var args = Array.prototype.slice.call(arguments, 1);
            fn.apply(null, args.concat([function (err) {
                var args = Array.prototype.slice.call(arguments, 1);
                if (typeof console !== 'undefined') {
                    if (err) {
                        if (console.error) {
                            console.error(err);
                        }
                    }
                    else if (console[name]) {
                        _each(args, function (x) {
                            console[name](x);
                        });
                    }
                }
            }]));
        };
    };
    async.log = _console_fn('log');
    async.dir = _console_fn('dir');
    /*async.info = _console_fn('info');
    async.warn = _console_fn('warn');
    async.error = _console_fn('error');*/

    async.memoize = function (fn, hasher) {
        var memo = {};
        var queues = {};
        hasher = hasher || function (x) {
            return x;
        };
        var memoized = function () {
            var args = Array.prototype.slice.call(arguments);
            var callback = args.pop();
            var key = hasher.apply(null, args);
            if (key in memo) {
                callback.apply(null, memo[key]);
            }
            else if (key in queues) {
                queues[key].push(callback);
            }
            else {
                queues[key] = [callback];
                fn.apply(null, args.concat([function () {
                    memo[key] = arguments;
                    var q = queues[key];
                    delete queues[key];
                    for (var i = 0, l = q.length; i < l; i++) {
                      q[i].apply(null, arguments);
                    }
                }]));
            }
        };
        memoized.memo = memo;
        memoized.unmemoized = fn;
        return memoized;
    };

    async.unmemoize = function (fn) {
      return function () {
        return (fn.unmemoized || fn).apply(null, arguments);
      };
    };

    async.times = function (count, iterator, callback) {
        var counter = [];
        for (var i = 0; i < count; i++) {
            counter.push(i);
        }
        return async.map(counter, iterator, callback);
    };

    async.timesSeries = function (count, iterator, callback) {
        var counter = [];
        for (var i = 0; i < count; i++) {
            counter.push(i);
        }
        return async.mapSeries(counter, iterator, callback);
    };

    async.compose = function (/* functions... */) {
        var fns = Array.prototype.reverse.call(arguments);
        return function () {
            var that = this;
            var args = Array.prototype.slice.call(arguments);
            var callback = args.pop();
            async.reduce(fns, args, function (newargs, fn, cb) {
                fn.apply(that, newargs.concat([function () {
                    var err = arguments[0];
                    var nextargs = Array.prototype.slice.call(arguments, 1);
                    cb(err, nextargs);
                }]))
            },
            function (err, results) {
                callback.apply(that, [err].concat(results));
            });
        };
    };

    var _applyEach = function (eachfn, fns /*args...*/) {
        var go = function () {
            var that = this;
            var args = Array.prototype.slice.call(arguments);
            var callback = args.pop();
            return eachfn(fns, function (fn, cb) {
                fn.apply(that, args.concat([cb]));
            },
            callback);
        };
        if (arguments.length > 2) {
            var args = Array.prototype.slice.call(arguments, 2);
            return go.apply(this, args);
        }
        else {
            return go;
        }
    };
    async.applyEach = doParallel(_applyEach);
    async.applyEachSeries = doSeries(_applyEach);

    async.forever = function (fn, callback) {
        function next(err) {
            if (err) {
                if (callback) {
                    return callback(err);
                }
                throw err;
            }
            fn(next);
        }
        next();
    };

    // AMD / RequireJS
    if (typeof define !== 'undefined' && define.amd) {
        define([], function () {
            return async;
        });
    }
    // Node.js
    else if (typeof module !== 'undefined' && module.exports) {
        module.exports = async;
    }
    // included directly via <script> tag
    else {
        root.async = async;
    }

}());

}).call(this,_dereq_('_process'),_dereq_("timers").setImmediate)
},{"_process":19,"timers":36}],63:[function(_dereq_,module,exports){
(function (process,__filename){

/**
 * Module dependencies.
 */

var fs = _dereq_('fs')
  , path = _dereq_('path')
  , join = path.join
  , dirname = path.dirname
  , exists = fs.existsSync || path.existsSync
  , defaults = {
        arrow: process.env.NODE_BINDINGS_ARROW || ' → '
      , compiled: process.env.NODE_BINDINGS_COMPILED_DIR || 'compiled'
      , platform: process.platform
      , arch: process.arch
      , version: process.versions.node
      , bindings: 'bindings.node'
      , try: [
          // node-gyp's linked version in the "build" dir
          [ 'module_root', 'build', 'bindings' ]
          // node-waf and gyp_addon (a.k.a node-gyp)
        , [ 'module_root', 'build', 'Debug', 'bindings' ]
        , [ 'module_root', 'build', 'Release', 'bindings' ]
          // Debug files, for development (legacy behavior, remove for node v0.9)
        , [ 'module_root', 'out', 'Debug', 'bindings' ]
        , [ 'module_root', 'Debug', 'bindings' ]
          // Release files, but manually compiled (legacy behavior, remove for node v0.9)
        , [ 'module_root', 'out', 'Release', 'bindings' ]
        , [ 'module_root', 'Release', 'bindings' ]
          // Legacy from node-waf, node <= 0.4.x
        , [ 'module_root', 'build', 'default', 'bindings' ]
          // Production "Release" buildtype binary (meh...)
        , [ 'module_root', 'compiled', 'version', 'platform', 'arch', 'bindings' ]
        ]
    }

/**
 * The main `bindings()` function loads the compiled bindings for a given module.
 * It uses V8's Error API to determine the parent filename that this function is
 * being invoked from, which is then used to find the root directory.
 */

function bindings (opts) {

  // Argument surgery
  if (typeof opts == 'string') {
    opts = { bindings: opts }
  } else if (!opts) {
    opts = {}
  }
  opts.__proto__ = defaults

  // Get the module root
  if (!opts.module_root) {
    opts.module_root = exports.getRoot(exports.getFileName())
  }

  // Ensure the given bindings name ends with .node
  if (path.extname(opts.bindings) != '.node') {
    opts.bindings += '.node'
  }

  var tries = []
    , i = 0
    , l = opts.try.length
    , n
    , b
    , err

  for (; i<l; i++) {
    n = join.apply(null, opts.try[i].map(function (p) {
      return opts[p] || p
    }))
    tries.push(n)
    try {
      b = opts.path ? _dereq_.resolve(n) : _dereq_(n)
      if (!opts.path) {
        b.path = n
      }
      return b
    } catch (e) {
      if (!/not find/i.test(e.message)) {
        throw e
      }
    }
  }

  err = new Error('Could not locate the bindings file. Tried:\n'
    + tries.map(function (a) { return opts.arrow + a }).join('\n'))
  err.tries = tries
  throw err
}
module.exports = exports = bindings


/**
 * Gets the filename of the JavaScript file that invokes this function.
 * Used to help find the root directory of a module.
 * Optionally accepts an filename argument to skip when searching for the invoking filename
 */

exports.getFileName = function getFileName (calling_file) {
  var origPST = Error.prepareStackTrace
    , origSTL = Error.stackTraceLimit
    , dummy = {}
    , fileName

  Error.stackTraceLimit = 10

  Error.prepareStackTrace = function (e, st) {
    for (var i=0, l=st.length; i<l; i++) {
      fileName = st[i].getFileName()
      if (fileName !== __filename) {
        if (calling_file) {
            if (fileName !== calling_file) {
              return
            }
        } else {
          return
        }
      }
    }
  }

  // run the 'prepareStackTrace' function above
  Error.captureStackTrace(dummy)
  dummy.stack

  // cleanup
  Error.prepareStackTrace = origPST
  Error.stackTraceLimit = origSTL

  return fileName
}

/**
 * Gets the root directory of a module, given an arbitrary filename
 * somewhere in the module tree. The "root directory" is the directory
 * containing the `package.json` file.
 *
 *   In:  /home/nate/node-native-module/lib/index.js
 *   Out: /home/nate/node-native-module
 */

exports.getRoot = function getRoot (file) {
  var dir = dirname(file)
    , prev
  while (true) {
    if (dir === '.') {
      // Avoids an infinite loop in rare cases, like the REPL
      dir = process.cwd()
    }
    if (exists(join(dir, 'package.json')) || exists(join(dir, 'node_modules'))) {
      // Found the 'package.json' file or 'node_modules' dir; we're done
      return dir
    }
    if (prev === dir) {
      // Got to the top
      throw new Error('Could not find module root given file: "' + file
                    + '". Do you have a `package.json` file? ')
    }
    // Try the parent dir next
    prev = dir
    dir = join(dir, '..')
  }
}

}).call(this,_dereq_('_process'),"/node_modules/bindings/bindings.js")
},{"_process":19,"fs":1,"path":17}],64:[function(_dereq_,module,exports){
var buffy = exports;

buffy.Reader = _dereq_('./lib/Reader');
buffy.createReader = function(options) {
  return new buffy.Reader(options);
};

},{"./lib/Reader":65}],65:[function(_dereq_,module,exports){
(function (Buffer){
var util         = _dereq_('util');
var EventEmitter = _dereq_('events').EventEmitter;

module.exports = Reader;
util.inherits(Reader, EventEmitter);
function Reader(options) {
  EventEmitter.call(this);

  if (Buffer.isBuffer(options)) {
    options = {buffer: options};
  }

  options = options || {};

  this.writable = true;

  this._buffer = options.buffer || new Buffer(0);
  this._offset = options.offset || 0;
  this._paused = false;
}

Reader.prototype.write = function(newBuffer) {
  var bytesAhead = this.bytesAhead();

  // existing buffer has enough space in the beginning?
  var reuseBuffer = (this._offset >= newBuffer.length);

  if (reuseBuffer) {
    // move unread bytes forward to make room for the new
    this._buffer.copy(this._buffer, this._offset - newBuffer.length, this._offset);
    this._offset -= newBuffer.length;

    // add the new bytes at the end
    newBuffer.copy(this._buffer, this._buffer.length - newBuffer.length);
  } else {
    var oldBuffer = this._buffer;

    // grow a new buffer that can hold both
    this._buffer = new Buffer(bytesAhead + newBuffer.length);

    // copy the old and new buffer into it
    oldBuffer.copy(this._buffer, 0, this._offset);
    newBuffer.copy(this._buffer, bytesAhead);
    this._offset = 0;
  }

  return !this._paused;
};

Reader.prototype.pause = function() {
  this._paused = true;
};

Reader.prototype.resume = function() {
  this._paused = false;
};

Reader.prototype.bytesAhead = function() {
  return this._buffer.length - this._offset;
};

Reader.prototype.bytesBuffered = function() {
  return this._buffer.length;
};

Reader.prototype.uint8 = function() {
  return this._buffer.readUInt8(this._offset++);
};

Reader.prototype.int8 = function() {
  return this._buffer.readInt8(this._offset++);
};

Reader.prototype.uint16BE = function() {
  this._offset += 2;
  return this._buffer.readUInt16BE(this._offset - 2);
};

Reader.prototype.int16BE = function() {
  this._offset += 2;
  return this._buffer.readInt16BE(this._offset - 2);
};

Reader.prototype.uint16LE = function() {
  this._offset += 2;
  return this._buffer.readUInt16LE(this._offset - 2);
};

Reader.prototype.int16LE = function() {
  this._offset += 2;
  return this._buffer.readInt16LE(this._offset - 2);
};

Reader.prototype.uint32BE = function() {
  this._offset += 4;
  return this._buffer.readUInt32BE(this._offset - 4);
};

Reader.prototype.int32BE = function() {
  this._offset += 4;
  return this._buffer.readInt32BE(this._offset - 4);
};

Reader.prototype.uint32LE = function() {
  this._offset += 4;
  return this._buffer.readUInt32LE(this._offset - 4);
};

Reader.prototype.int32LE = function() {
  this._offset += 4;
  return this._buffer.readInt32LE(this._offset - 4);
};

Reader.prototype.float32BE = function() {
  this._offset += 4;
  return this._buffer.readFloatBE(this._offset - 4);
};

Reader.prototype.float32LE = function() {
  this._offset += 4;
  return this._buffer.readFloatLE(this._offset - 4);
};

Reader.prototype.double64BE = function() {
  this._offset += 8;
  return this._buffer.readDoubleBE(this._offset - 8);
};

Reader.prototype.double64LE = function() {
  this._offset += 8;
  return this._buffer.readDoubleLE(this._offset - 8);
};

Reader.prototype.ascii = function(bytes) {
  return this._string('ascii', bytes);
};

Reader.prototype.utf8 = function(bytes) {
  return this._string('utf8', bytes);
};

Reader.prototype._string = function(encoding, bytes) {
  var nullTerminated = (bytes === undefined);
  if (nullTerminated) {
    bytes = this._nullDistance();
  }

  var offset = this._offset;
  this._offset += bytes;

  var value = this._buffer.toString(encoding, offset, this._offset);

  if (nullTerminated) {
    this._offset++;
  }

  return value;
};

Reader.prototype._nullDistance = function() {
  for (var i = this._offset; i < this._buffer.length; i++) {
    var byte = this._buffer[i];
    if (byte === 0) {
      return i - this._offset;
    }
  }
};

Reader.prototype.buffer = function(bytes) {
  this._offset += bytes;
  var ret = new Buffer(bytes);
  this._buffer.copy(ret, 0, this._offset -  bytes, this._offset);
  return ret;
};

Reader.prototype.skip = function(bytes) {
    if (bytes > this.bytesAhead() || bytes < 0) {
        this.emit('error', new Error('tried to skip outsite of the buffer'));
    }
    this._offset += bytes;
};

}).call(this,_dereq_("buffer").Buffer)
},{"buffer":8,"events":12,"util":39}],66:[function(_dereq_,module,exports){
(function (process){
/**
 * This is the web browser implementation of `debug()`.
 *
 * Expose `debug()` as the module.
 */

exports = module.exports = _dereq_('./debug');
exports.log = log;
exports.formatArgs = formatArgs;
exports.save = save;
exports.load = load;
exports.useColors = useColors;
exports.storage = 'undefined' != typeof chrome
               && 'undefined' != typeof chrome.storage
                  ? chrome.storage.local
                  : localstorage();

/**
 * Colors.
 */

exports.colors = [
  'lightseagreen',
  'forestgreen',
  'goldenrod',
  'dodgerblue',
  'darkorchid',
  'crimson'
];

/**
 * Currently only WebKit-based Web Inspectors, Firefox >= v31,
 * and the Firebug extension (any Firefox version) are known
 * to support "%c" CSS customizations.
 *
 * TODO: add a `localStorage` variable to explicitly enable/disable colors
 */

function useColors() {
  // NB: In an Electron preload script, document will be defined but not fully
  // initialized. Since we know we're in Chrome, we'll just detect this case
  // explicitly
  if (typeof window !== 'undefined' && window.process && window.process.type === 'renderer') {
    return true;
  }

  // is webkit? http://stackoverflow.com/a/16459606/376773
  // document is undefined in react-native: https://github.com/facebook/react-native/pull/1632
  return (typeof document !== 'undefined' && document.documentElement && document.documentElement.style && document.documentElement.style.WebkitAppearance) ||
    // is firebug? http://stackoverflow.com/a/398120/376773
    (typeof window !== 'undefined' && window.console && (window.console.firebug || (window.console.exception && window.console.table))) ||
    // is firefox >= v31?
    // https://developer.mozilla.org/en-US/docs/Tools/Web_Console#Styling_messages
    (typeof navigator !== 'undefined' && navigator.userAgent && navigator.userAgent.toLowerCase().match(/firefox\/(\d+)/) && parseInt(RegExp.$1, 10) >= 31) ||
    // double check webkit in userAgent just in case we are in a worker
    (typeof navigator !== 'undefined' && navigator.userAgent && navigator.userAgent.toLowerCase().match(/applewebkit\/(\d+)/));
}

/**
 * Map %j to `JSON.stringify()`, since no Web Inspectors do that by default.
 */

exports.formatters.j = function(v) {
  try {
    return JSON.stringify(v);
  } catch (err) {
    return '[UnexpectedJSONParseError]: ' + err.message;
  }
};


/**
 * Colorize log arguments if enabled.
 *
 * @api public
 */

function formatArgs(args) {
  var useColors = this.useColors;

  args[0] = (useColors ? '%c' : '')
    + this.namespace
    + (useColors ? ' %c' : ' ')
    + args[0]
    + (useColors ? '%c ' : ' ')
    + '+' + exports.humanize(this.diff);

  if (!useColors) return;

  var c = 'color: ' + this.color;
  args.splice(1, 0, c, 'color: inherit')

  // the final "%c" is somewhat tricky, because there could be other
  // arguments passed either before or after the %c, so we need to
  // figure out the correct index to insert the CSS into
  var index = 0;
  var lastC = 0;
  args[0].replace(/%[a-zA-Z%]/g, function(match) {
    if ('%%' === match) return;
    index++;
    if ('%c' === match) {
      // we only are interested in the *last* %c
      // (the user may have provided their own)
      lastC = index;
    }
  });

  args.splice(lastC, 0, c);
}

/**
 * Invokes `console.log()` when available.
 * No-op when `console.log` is not a "function".
 *
 * @api public
 */

function log() {
  // this hackery is required for IE8/9, where
  // the `console.log` function doesn't have 'apply'
  return 'object' === typeof console
    && console.log
    && Function.prototype.apply.call(console.log, console, arguments);
}

/**
 * Save `namespaces`.
 *
 * @param {String} namespaces
 * @api private
 */

function save(namespaces) {
  try {
    if (null == namespaces) {
      exports.storage.removeItem('debug');
    } else {
      exports.storage.debug = namespaces;
    }
  } catch(e) {}
}

/**
 * Load `namespaces`.
 *
 * @return {String} returns the previously persisted debug modes
 * @api private
 */

function load() {
  var r;
  try {
    r = exports.storage.debug;
  } catch(e) {}

  // If debug isn't set in LS, and we're in Electron, try to load $DEBUG
  if (!r && typeof process !== 'undefined' && 'env' in process) {
    r = process.env.DEBUG;
  }

  return r;
}

/**
 * Enable namespaces listed in `localStorage.debug` initially.
 */

exports.enable(load());

/**
 * Localstorage attempts to return the localstorage.
 *
 * This is necessary because safari throws
 * when a user disables cookies/localstorage
 * and you attempt to access it.
 *
 * @return {LocalStorage}
 * @api private
 */

function localstorage() {
  try {
    return window.localStorage;
  } catch (e) {}
}

}).call(this,_dereq_('_process'))
},{"./debug":67,"_process":19}],67:[function(_dereq_,module,exports){

/**
 * This is the common logic for both the Node.js and web browser
 * implementations of `debug()`.
 *
 * Expose `debug()` as the module.
 */

exports = module.exports = createDebug.debug = createDebug['default'] = createDebug;
exports.coerce = coerce;
exports.disable = disable;
exports.enable = enable;
exports.enabled = enabled;
exports.humanize = _dereq_('ms');

/**
 * The currently active debug mode names, and names to skip.
 */

exports.names = [];
exports.skips = [];

/**
 * Map of special "%n" handling functions, for the debug "format" argument.
 *
 * Valid key names are a single, lower or upper-case letter, i.e. "n" and "N".
 */

exports.formatters = {};

/**
 * Previous log timestamp.
 */

var prevTime;

/**
 * Select a color.
 * @param {String} namespace
 * @return {Number}
 * @api private
 */

function selectColor(namespace) {
  var hash = 0, i;

  for (i in namespace) {
    hash  = ((hash << 5) - hash) + namespace.charCodeAt(i);
    hash |= 0; // Convert to 32bit integer
  }

  return exports.colors[Math.abs(hash) % exports.colors.length];
}

/**
 * Create a debugger with the given `namespace`.
 *
 * @param {String} namespace
 * @return {Function}
 * @api public
 */

function createDebug(namespace) {

  function debug() {
    // disabled?
    if (!debug.enabled) return;

    var self = debug;

    // set `diff` timestamp
    var curr = +new Date();
    var ms = curr - (prevTime || curr);
    self.diff = ms;
    self.prev = prevTime;
    self.curr = curr;
    prevTime = curr;

    // turn the `arguments` into a proper Array
    var args = new Array(arguments.length);
    for (var i = 0; i < args.length; i++) {
      args[i] = arguments[i];
    }

    args[0] = exports.coerce(args[0]);

    if ('string' !== typeof args[0]) {
      // anything else let's inspect with %O
      args.unshift('%O');
    }

    // apply any `formatters` transformations
    var index = 0;
    args[0] = args[0].replace(/%([a-zA-Z%])/g, function(match, format) {
      // if we encounter an escaped % then don't increase the array index
      if (match === '%%') return match;
      index++;
      var formatter = exports.formatters[format];
      if ('function' === typeof formatter) {
        var val = args[index];
        match = formatter.call(self, val);

        // now we need to remove `args[index]` since it's inlined in the `format`
        args.splice(index, 1);
        index--;
      }
      return match;
    });

    // apply env-specific formatting (colors, etc.)
    exports.formatArgs.call(self, args);

    var logFn = debug.log || exports.log || console.log.bind(console);
    logFn.apply(self, args);
  }

  debug.namespace = namespace;
  debug.enabled = exports.enabled(namespace);
  debug.useColors = exports.useColors();
  debug.color = selectColor(namespace);

  // env-specific initialization logic for debug instances
  if ('function' === typeof exports.init) {
    exports.init(debug);
  }

  return debug;
}

/**
 * Enables a debug mode by namespaces. This can include modes
 * separated by a colon and wildcards.
 *
 * @param {String} namespaces
 * @api public
 */

function enable(namespaces) {
  exports.save(namespaces);

  exports.names = [];
  exports.skips = [];

  var split = (typeof namespaces === 'string' ? namespaces : '').split(/[\s,]+/);
  var len = split.length;

  for (var i = 0; i < len; i++) {
    if (!split[i]) continue; // ignore empty strings
    namespaces = split[i].replace(/\*/g, '.*?');
    if (namespaces[0] === '-') {
      exports.skips.push(new RegExp('^' + namespaces.substr(1) + '$'));
    } else {
      exports.names.push(new RegExp('^' + namespaces + '$'));
    }
  }
}

/**
 * Disable debug output.
 *
 * @api public
 */

function disable() {
  exports.enable('');
}

/**
 * Returns true if the given mode name is enabled, false otherwise.
 *
 * @param {String} name
 * @return {Boolean}
 * @api public
 */

function enabled(name) {
  var i, len;
  for (i = 0, len = exports.skips.length; i < len; i++) {
    if (exports.skips[i].test(name)) {
      return false;
    }
  }
  for (i = 0, len = exports.names.length; i < len; i++) {
    if (exports.names[i].test(name)) {
      return true;
    }
  }
  return false;
}

/**
 * Coerce `val`.
 *
 * @param {Mixed} val
 * @return {Mixed}
 * @api private
 */

function coerce(val) {
  if (val instanceof Error) return val.stack || val.message;
  return val;
}

},{"ms":84}],68:[function(_dereq_,module,exports){
(function (process,Buffer){

/**
 * Module dependencies.
 */

var assert = _dereq_('assert')
  , debug = _dereq_('debug')('ffi:_ForeignFunction')
  , ref = _dereq_('ref')
  , bindings = _dereq_('./bindings')
  , POINTER_SIZE = ref.sizeof.pointer
  , FFI_ARG_SIZE = bindings.FFI_ARG_SIZE


function ForeignFunction (cif, funcPtr, returnType, argTypes) {
  debug('creating new ForeignFunction', funcPtr)

  var numArgs = argTypes.length
  var argsArraySize = numArgs * POINTER_SIZE

  // "result" must point to storage that is sizeof(long) or larger. For smaller
  // return value sizes, the ffi_arg or ffi_sarg integral type must be used to
  // hold the return value
  var resultSize = returnType.size >= ref.sizeof.long ? returnType.size : FFI_ARG_SIZE
  assert(resultSize > 0)

  /**
   * This is the actual JS function that gets returned.
   * It handles marshalling input arguments into C values,
   * and unmarshalling the return value back into a JS value
   */

  var proxy = function () {
    debug('invoking proxy function')

    if (arguments.length !== numArgs) {
      throw new TypeError('Expected ' + numArgs +
          ' arguments, got ' + arguments.length)
    }

    // storage buffers for input arguments and the return value
    var result = new Buffer(resultSize)
      , argsList = new Buffer(argsArraySize)

    // write arguments to storage areas
    var i, argType, val, valPtr
    try {
      for (i = 0; i < numArgs; i++) {
        argType = argTypes[i]
        val = arguments[i]
        valPtr = ref.alloc(argType, val)
        argsList.writePointer(valPtr, i * POINTER_SIZE)
      }
    } catch (e) {
      e.message = 'error setting argument ' + i + ' - ' + e.message
      throw e
    }

    // invoke the `ffi_call()` function
    bindings.ffi_call(cif, funcPtr, result, argsList)

    result.type = returnType
    return result.deref()
  }

  /**
   * The asynchronous version of the proxy function.
   */

  proxy.async = function () {
    debug('invoking async proxy function')

    var argc = arguments.length
    if (argc !== numArgs + 1) {
      throw new TypeError('Expected ' + (numArgs + 1) +
          ' arguments, got ' + argc)
    }

    var callback = arguments[argc - 1]
    if (typeof callback !== 'function') {
      throw new TypeError('Expected a callback function as argument number: ' +
          (argc - 1))
    }

    // storage buffers for input arguments and the return value
    var result = new Buffer(resultSize)
    var argsList = new Buffer(argsArraySize)

    // write arguments to storage areas
    var i, argType, val, valPtr
    try {
      for (i = 0; i < numArgs; i++) {
        argType = argTypes[i]
        val = arguments[i]
        valPtr = ref.alloc(argType, val)
        argsList.writePointer(valPtr, i * POINTER_SIZE)
      }
    } catch (e) {
      e.message = 'error setting argument ' + i + ' - ' + e.message
      return process.nextTick(callback.bind(null, e));
    }

    // invoke the `ffi_call()` function asynchronously
    bindings.ffi_call_async(cif, funcPtr, result, argsList, function (err) {
      // make sure that the 4 Buffers passed in above don't get GC'd while we're
      // doing work on the thread pool...
      cif = cif;
      funcPtr = funcPtr;
      argsList = argsList;

      // now invoke the user-provided callback function
      if (err) {
        callback(err)
      } else {
        result.type = returnType
        callback(null, result.deref())
      }
    })
  }

  return proxy
}
module.exports = ForeignFunction

}).call(this,_dereq_('_process'),_dereq_("buffer").Buffer)
},{"./bindings":69,"_process":19,"assert":2,"buffer":8,"debug":66,"ref":86}],69:[function(_dereq_,module,exports){

module.exports = _dereq_('bindings')('ffi_bindings.node')

},{"bindings":63}],70:[function(_dereq_,module,exports){
(function (process){

/**
 * Module dependencies.
 */

var ref = _dereq_('ref')
  , CIF = _dereq_('./cif')
  , assert = _dereq_('assert')
  , debug = _dereq_('debug')('ffi:Callback')
  , _Callback = _dereq_('./bindings').Callback

// Function used to report errors to the current process event loop,
// When user callback function gets gced.
function errorReportCallback (err) {
  if (err) {
    process.nextTick(function () {
      if (typeof err === 'string') {
        throw new Error(err)
      } else {
        throw err
      }
    })
  }
}

/**
 * Turns a JavaScript function into a C function pointer.
 * The function pointer may be used in other C functions that
 * accept C callback functions.
 */

function Callback (retType, argTypes, abi, func) {
  debug('creating new Callback')

  if (typeof abi === 'function') {
    func = abi
    abi = void(0)
  }

  // check args
  assert(!!retType, 'expected a return "type" object as the first argument')
  assert(Array.isArray(argTypes), 'expected Array of arg "type" objects as the second argument')
  assert.equal(typeof func, 'function', 'expected a function as the third argument')

  // normalize the "types" (they could be strings, so turn into real type
  // instances)
  retType = ref.coerceType(retType)
  argTypes = argTypes.map(ref.coerceType)

  // create the `ffi_cif *` instance
  var cif = CIF(retType, argTypes, abi)
  var argc = argTypes.length

  var callback = _Callback(cif, retType.size, argc, errorReportCallback, function (retval, params) {
    debug('Callback function being invoked')
    try {
      var args = []
      for (var i = 0; i < argc; i++) {
        var type = argTypes[ i ]
        var argPtr = params.readPointer(i * ref.sizeof.pointer, type.size)
        argPtr.type = type
        args.push(argPtr.deref())
      }

      // Invoke the user-given function
      var result = func.apply(null, args)
      try {
        ref.set(retval, 0, result, retType)
      } catch (e) {
        e.message = 'error setting return value - ' + e.message
        throw e
      }
    } catch (e) {
      return e
    }
  })

  // store reference to the CIF Buffer so that it doesn't get
  // garbage collected before the callback Buffer does
  callback._cif = cif;

  return callback
}
module.exports = Callback

}).call(this,_dereq_('_process'))
},{"./bindings":69,"./cif":71,"_process":19,"assert":2,"debug":66,"ref":86}],71:[function(_dereq_,module,exports){
(function (Buffer){

/**
 * Module dependencies.
 */

var Type = _dereq_('./type')
  , assert = _dereq_('assert')
  , debug = _dereq_('debug')('ffi:cif')
  , ref = _dereq_('ref')
  , bindings = _dereq_('./bindings')
  , POINTER_SIZE = ref.sizeof.pointer
  , ffi_prep_cif = bindings.ffi_prep_cif
  , FFI_CIF_SIZE = bindings.FFI_CIF_SIZE
  , FFI_DEFAULT_ABI = bindings.FFI_DEFAULT_ABI
  // status codes
  , FFI_OK = bindings.FFI_OK
  , FFI_BAD_TYPEDEF = bindings.FFI_BAD_TYPEDEF
  , FFI_BAD_ABI = bindings.FFI_BAD_ABI

/**
 * JS wrapper for the `ffi_prep_cif` function.
 * Returns a Buffer instance representing a `ffi_cif *` instance.
 */

function CIF (rtype, types, abi) {
  debug('creating `ffi_cif *` instance')

  // the return and arg types are expected to be coerced at this point...
  assert(!!rtype, 'expected a return "type" object as the first argument')
  assert(Array.isArray(types), 'expected an Array of arg "type" objects as the second argument')

  // the buffer that will contain the return `ffi_cif *` instance
  var cif = new Buffer(FFI_CIF_SIZE)

  var numArgs = types.length
  var _argtypesptr = new Buffer(numArgs * POINTER_SIZE)
  var _rtypeptr = Type(rtype)

  for (var i = 0; i < numArgs; i++) {
    var type = types[i]
    var ffiType = Type(type)

    _argtypesptr.writePointer(ffiType, i * POINTER_SIZE)
  }

  // prevent GC of the arg type and rtn type buffers (not sure if this is required)
  cif.rtnTypePtr = _rtypeptr
  cif.argTypesPtr = _argtypesptr

  if (typeof abi === 'undefined') {
    debug('no ABI specified (this is OK), using FFI_DEFAULT_ABI')
    abi = FFI_DEFAULT_ABI
  }

  var status = ffi_prep_cif(cif, numArgs, _rtypeptr, _argtypesptr, abi)

  if (status !== FFI_OK) {
    switch (status) {
      case FFI_BAD_TYPEDEF:
        var err = new Error('ffi_prep_cif() returned an FFI_BAD_TYPEDEF error')
        err.code = 'FFI_BAD_TYPEDEF'
        err.errno = status
        throw err
        break;
      case FFI_BAD_ABI:
        var err = new Error('ffi_prep_cif() returned an FFI_BAD_ABI error')
        err.code = 'FFI_BAD_ABI'
        err.errno = status
        throw err
        break;
      default:
        throw new Error('ffi_prep_cif() returned an error: ' + status)
        break;
    }
  }

  return cif
}
module.exports = CIF

}).call(this,_dereq_("buffer").Buffer)
},{"./bindings":69,"./type":80,"assert":2,"buffer":8,"debug":66,"ref":86}],72:[function(_dereq_,module,exports){
(function (Buffer){

/**
 * Module dependencies.
 */

var Type = _dereq_('./type')
  , assert = _dereq_('assert')
  , debug = _dereq_('debug')('ffi:cif_var')
  , ref = _dereq_('ref')
  , bindings = _dereq_('./bindings')
  , POINTER_SIZE = ref.sizeof.pointer
  , ffi_prep_cif_var = bindings.ffi_prep_cif_var
  , FFI_CIF_SIZE = bindings.FFI_CIF_SIZE
  , FFI_DEFAULT_ABI = bindings.FFI_DEFAULT_ABI
  // status codes
  , FFI_OK = bindings.FFI_OK
  , FFI_BAD_TYPEDEF = bindings.FFI_BAD_TYPEDEF
  , FFI_BAD_ABI = bindings.FFI_BAD_ABI

/**
 * JS wrapper for the `ffi_prep_cif_var` function.
 * Returns a Buffer instance representing a variadic `ffi_cif *` instance.
 */

function CIF_var (rtype, types, numFixedArgs, abi) {
  debug('creating `ffi_cif *` instance with `ffi_prep_cif_var()`')

  // the return and arg types are expected to be coerced at this point...
  assert(!!rtype, 'expected a return "type" object as the first argument')
  assert(Array.isArray(types), 'expected an Array of arg "type" objects as the second argument')
  assert(numFixedArgs >= 1, 'expected the number of fixed arguments to be at least 1')

  // the buffer that will contain the return `ffi_cif *` instance
  var cif = new Buffer(FFI_CIF_SIZE)

  var numTotalArgs = types.length
  var _argtypesptr = new Buffer(numTotalArgs * POINTER_SIZE)
  var _rtypeptr = Type(rtype)

  for (var i = 0; i < numTotalArgs; i++) {
    var ffiType = Type(types[i])
    _argtypesptr.writePointer(ffiType, i * POINTER_SIZE)
  }

  // prevent GC of the arg type and rtn type buffers (not sure if this is required)
  cif.rtnTypePtr = _rtypeptr
  cif.argTypesPtr = _argtypesptr

  if (typeof abi === 'undefined') {
    debug('no ABI specified (this is OK), using FFI_DEFAULT_ABI')
    abi = FFI_DEFAULT_ABI
  }

  var status = ffi_prep_cif_var(cif, numFixedArgs, numTotalArgs, _rtypeptr, _argtypesptr, abi)

  if (status !== FFI_OK) {
    switch (status) {
      case FFI_BAD_TYPEDEF:
        var err = new Error('ffi_prep_cif_var() returned an FFI_BAD_TYPEDEF error')
        err.code = 'FFI_BAD_TYPEDEF'
        err.errno = status
        throw err
        break;
      case FFI_BAD_ABI:
        var err = new Error('ffi_prep_cif_var() returned an FFI_BAD_ABI error')
        err.code = 'FFI_BAD_ABI'
        err.errno = status
        throw err
        break;
      default:
        var err = new Error('ffi_prep_cif_var() returned an error: ' + status)
        err.errno = status
        throw err
        break;
    }
  }

  return cif
}
module.exports = CIF_var

}).call(this,_dereq_("buffer").Buffer)
},{"./bindings":69,"./type":80,"assert":2,"buffer":8,"debug":66,"ref":86}],73:[function(_dereq_,module,exports){
(function (Buffer){

/**
 * Module dependencies.
 */

var ForeignFunction = _dereq_('./foreign_function')
  , assert = _dereq_('assert')
  , debug = _dereq_('debug')('ffi:DynamicLibrary')
  , bindings = _dereq_('./bindings')
  , funcs = bindings.StaticFunctions
  , ref = _dereq_('ref')
  , read  = _dereq_('fs').readFileSync

// typedefs
var int = ref.types.int
  , voidPtr = ref.refType(ref.types.void)

var dlopen  = ForeignFunction(funcs.dlopen,  voidPtr, [ 'string', int ])
  , dlclose = ForeignFunction(funcs.dlclose, int,     [ voidPtr ])
  , dlsym   = ForeignFunction(funcs.dlsym,   voidPtr, [ voidPtr, 'string' ])
  , dlerror = ForeignFunction(funcs.dlerror, 'string', [ ])

/**
 * `DynamicLibrary` loads and fetches function pointers for dynamic libraries
 * (.so, .dylib, etc). After the libray's function pointer is acquired, then you
 * call `get(symbol)` to retreive a pointer to an exported symbol. You need to
 * call `get___()` on the pointer to dereference it into its actual value, or
 * turn the pointer into a callable function with `ForeignFunction`.
 */

function DynamicLibrary (path, mode) {
  if (!(this instanceof DynamicLibrary)) {
    return new DynamicLibrary(path, mode)
  }
  debug('new DynamicLibrary()', path, mode)

  if (null == mode) {
    mode = DynamicLibrary.FLAGS.RTLD_LAZY
  }

  this._handle = dlopen(path, mode)
  assert(Buffer.isBuffer(this._handle), 'expected a Buffer instance to be returned from `dlopen()`')

  if (this._handle.isNull()) {
    var err = this.error()

    // THIS CODE IS BASED ON GHC Trac ticket #2615
    // http://hackage.haskell.org/trac/ghc/attachment/ticket/2615

    // On some systems (e.g., Gentoo Linux) dynamic files (e.g. libc.so)
    // contain linker scripts rather than ELF-format object code. This
    // code handles the situation by recognizing the real object code
    // file name given in the linker script.

    // If an "invalid ELF header" error occurs, it is assumed that the
    // .so file contains a linker script instead of ELF object code.
    // In this case, the code looks for the GROUP ( ... ) linker
    // directive. If one is found, the first file name inside the
    // parentheses is treated as the name of a dynamic library and the
    // code attempts to dlopen that file. If this is also unsuccessful,
    // an error message is returned.

    // see if the error message is due to an invalid ELF header
    var match

    if (match = err.match(/^(([^ \t()])+\.so([^ \t:()])*):([ \t])*/)) {
      var content = read(match[1], 'ascii')
      // try to find a GROUP ( ... ) command
      if (match = content.match(/GROUP *\( *(([^ )])+)/)){
        return DynamicLibrary.call(this, match[1], mode)
      }
    }

    throw new Error('Dynamic Linking Error: ' + err)
  }
}
module.exports = DynamicLibrary

/**
 * Set the exported flags from "dlfcn.h"
 */

DynamicLibrary.FLAGS = {};
Object.keys(bindings).forEach(function (k) {
  if (!/^RTLD_/.test(k)) return;
  var desc = Object.getOwnPropertyDescriptor(bindings, k)
  Object.defineProperty(DynamicLibrary.FLAGS, k, desc)
});


/**
 * Close this library, returns the result of the dlclose() system function.
 */

DynamicLibrary.prototype.close = function () {
  debug('dlclose()')
  return dlclose(this._handle)
}

/**
 * Get a symbol from this library, returns a Pointer for (memory address of) the symbol
 */

DynamicLibrary.prototype.get = function (symbol) {
  debug('dlsym()', symbol)
  assert.equal('string', typeof symbol)

  var ptr = dlsym(this._handle, symbol)
  assert(Buffer.isBuffer(ptr))

  if (ptr.isNull()) {
    throw new Error('Dynamic Symbol Retrieval Error: ' + this.error())
  }

  ptr.name = symbol

  return ptr
}

/**
 * Returns the result of the dlerror() system function
 */

DynamicLibrary.prototype.error = function error () {
  debug('dlerror()')
  return dlerror()
}

}).call(this,{"isBuffer":_dereq_("C:/Users/Lorenzo/AppData/Roaming/nvm/v8.11.2/node_modules/browserify/node_modules/is-buffer/index.js")})
},{"./bindings":69,"./foreign_function":76,"C:/Users/Lorenzo/AppData/Roaming/nvm/v8.11.2/node_modules/browserify/node_modules/is-buffer/index.js":15,"assert":2,"debug":66,"fs":1,"ref":86}],74:[function(_dereq_,module,exports){
(function (process){
var DynamicLibrary = _dereq_('./dynamic_library')
  , ForeignFunction = _dereq_('./foreign_function')
  , bindings = _dereq_('./bindings')
  , funcs = bindings.StaticFunctions
  , ref = _dereq_('ref')
  , int = ref.types.int
  , intPtr = ref.refType(int)
  , errno = null

if (process.platform == 'win32') {
  var _errno = DynamicLibrary('msvcrt.dll').get('_errno')
  var errnoPtr = ForeignFunction(_errno, intPtr, [])
  errno = function() {
    return errnoPtr().deref()
  }
} else {
  errno = ForeignFunction(funcs._errno, 'int', [])
}


module.exports = errno

}).call(this,_dereq_('_process'))
},{"./bindings":69,"./dynamic_library":73,"./foreign_function":76,"_process":19,"ref":86}],75:[function(_dereq_,module,exports){

/**
 * Module dependencies.
 */

var ref = _dereq_('ref')
var assert = _dereq_('assert')
var debug = _dereq_('debug')('ffi:ffi')
var Struct = _dereq_('ref-struct')
var bindings = _dereq_('./bindings')

/**
 * Export some of the properties from the "bindings" file.
 */

;['HAS_OBJC', 'FFI_TYPES',
, 'FFI_OK', 'FFI_BAD_TYPEDEF', 'FFI_BAD_ABI'
, 'FFI_DEFAULT_ABI', 'FFI_FIRST_ABI', 'FFI_LAST_ABI', 'FFI_SYSV', 'FFI_UNIX64'
, 'FFI_WIN64', 'FFI_VFP', 'FFI_STDCALL', 'FFI_THISCALL', 'FFI_FASTCALL'
, 'RTLD_LAZY', 'RTLD_NOW', 'RTLD_LOCAL', 'RTLD_GLOBAL', 'RTLD_NOLOAD'
, 'RTLD_NODELETE', 'RTLD_FIRST', 'RTLD_NEXT', 'RTLD_DEFAULT', 'RTLD_SELF'
, 'RTLD_MAIN_ONLY', 'FFI_MS_CDECL'].forEach(function (prop) {
  if (!bindings.hasOwnProperty(prop)) {
    return debug('skipping exporting of non-existant property', prop)
  }
  var desc = Object.getOwnPropertyDescriptor(bindings, prop)
  Object.defineProperty(exports, prop, desc)
})

/**
 * Set the `ffi_type` property on the built-in types.
 */

Object.keys(bindings.FFI_TYPES).forEach(function (name) {
  var type = bindings.FFI_TYPES[name]
  type.name = name
  if (name === 'pointer') return // there is no "pointer" type...
  ref.types[name].ffi_type = type
})

// make `size_t` use the "ffi_type_pointer"
ref.types.size_t.ffi_type = bindings.FFI_TYPES.pointer

// make `Utf8String` use "ffi_type_pointer"
var CString = ref.types.CString || ref.types.Utf8String
CString.ffi_type = bindings.FFI_TYPES.pointer

// make `Object` use the "ffi_type_pointer"
ref.types.Object.ffi_type = bindings.FFI_TYPES.pointer


// libffi is weird when it comes to long data types (defaults to 64-bit),
// so we emulate here, since some platforms have 32-bit longs and some
// platforms have 64-bit longs.
switch (ref.sizeof.long) {
  case 4:
    ref.types.ulong.ffi_type = bindings.FFI_TYPES.uint32
    ref.types.long.ffi_type = bindings.FFI_TYPES.int32
    break;
  case 8:
    ref.types.ulong.ffi_type = bindings.FFI_TYPES.uint64
    ref.types.long.ffi_type = bindings.FFI_TYPES.int64
    break;
  default:
    throw new Error('unsupported "long" size: ' + ref.sizeof.long)
}

/**
 * Alias the "ref" types onto ffi's exports, for convenience...
 */

exports.types = ref.types

// Include our other modules
exports.version = bindings.version
exports.CIF = _dereq_('./cif')
exports.CIF_var = _dereq_('./cif_var')
exports.Function = _dereq_('./function')
exports.ForeignFunction = _dereq_('./foreign_function')
exports.VariadicForeignFunction = _dereq_('./foreign_function_var')
exports.DynamicLibrary = _dereq_('./dynamic_library')
exports.Library = _dereq_('./library')
exports.Callback = _dereq_('./callback')
exports.errno = _dereq_('./errno')
exports.ffiType = _dereq_('./type')

// the shared library extension for this platform
exports.LIB_EXT = exports.Library.EXT

// the FFI_TYPE struct definition
exports.FFI_TYPE = exports.ffiType.FFI_TYPE

},{"./bindings":69,"./callback":70,"./cif":71,"./cif_var":72,"./dynamic_library":73,"./errno":74,"./foreign_function":76,"./foreign_function_var":77,"./function":78,"./library":79,"./type":80,"assert":2,"debug":66,"ref":86,"ref-struct":85}],76:[function(_dereq_,module,exports){
(function (Buffer){

/**
 * Module dependencies.
 */

var CIF = _dereq_('./cif')
  , _ForeignFunction = _dereq_('./_foreign_function')
  , debug = _dereq_('debug')('ffi:ForeignFunction')
  , assert = _dereq_('assert')
  , ref = _dereq_('ref')

/**
 * Represents a foreign function in another library. Manages all of the aspects
 * of function execution, including marshalling the data parameters for the
 * function into native types and also unmarshalling the return from function
 * execution.
 */

function ForeignFunction (funcPtr, returnType, argTypes, abi) {
  debug('creating new ForeignFunction', funcPtr)

  // check args
  assert(Buffer.isBuffer(funcPtr), 'expected Buffer as first argument')
  assert(!!returnType, 'expected a return "type" object as the second argument')
  assert(Array.isArray(argTypes), 'expected Array of arg "type" objects as the third argument')

  // normalize the "types" (they could be strings,
  // so turn into real type instances)
  returnType = ref.coerceType(returnType)
  argTypes = argTypes.map(ref.coerceType)

  // create the `ffi_cif *` instance
  var cif = CIF(returnType, argTypes, abi)

  // create and return the JS proxy function
  return _ForeignFunction(cif, funcPtr, returnType, argTypes)
}
module.exports = ForeignFunction

}).call(this,{"isBuffer":_dereq_("C:/Users/Lorenzo/AppData/Roaming/nvm/v8.11.2/node_modules/browserify/node_modules/is-buffer/index.js")})
},{"./_foreign_function":68,"./cif":71,"C:/Users/Lorenzo/AppData/Roaming/nvm/v8.11.2/node_modules/browserify/node_modules/is-buffer/index.js":15,"assert":2,"debug":66,"ref":86}],77:[function(_dereq_,module,exports){
(function (Buffer){

/**
 * Module dependencies.
 */

var CIF_var = _dereq_('./cif_var')
  , Type = _dereq_('./type')
  , _ForeignFunction = _dereq_('./_foreign_function')
  , assert = _dereq_('assert')
  , debug = _dereq_('debug')('ffi:VariadicForeignFunction')
  , ref = _dereq_('ref')
  , bindings = _dereq_('./bindings')
  , POINTER_SIZE = ref.sizeof.pointer
  , FFI_ARG_SIZE = bindings.FFI_ARG_SIZE

/**
 * For when you want to call to a C function with variable amount of arguments.
 * i.e. `printf()`.
 *
 * This function takes care of caching and reusing ForeignFunction instances that
 * contain the same ffi_type argument signature.
 */

function VariadicForeignFunction (funcPtr, returnType, fixedArgTypes, abi) {
  debug('creating new VariadicForeignFunction', funcPtr)

  // the cache of ForeignFunction instances that this
  // VariadicForeignFunction has created so far
  var cache = {}

  // check args
  assert(Buffer.isBuffer(funcPtr), 'expected Buffer as first argument')
  assert(!!returnType, 'expected a return "type" object as the second argument')
  assert(Array.isArray(fixedArgTypes), 'expected Array of arg "type" objects as the third argument')

  var numFixedArgs = fixedArgTypes.length

  // normalize the "types" (they could be strings,
  // so turn into real type instances)
  fixedArgTypes = fixedArgTypes.map(ref.coerceType)

  // get the names of the fixed arg types
  var fixedKey = fixedArgTypes.map(function (type) {
    return getId(type)
  })


  // what gets returned is another function that needs to be invoked with the rest
  // of the variadic types that are being invoked from the function.
  function variadic_function_generator () {
    debug('variadic_function_generator invoked')

    // first get the types of variadic args we are working with
    var argTypes = fixedArgTypes.slice()
    var key = fixedKey.slice()

    for (var i = 0; i < arguments.length; i++) {
      var type = ref.coerceType(arguments[i])
      argTypes.push(type)

      var ffi_type = Type(type)
      assert(ffi_type.name)
      key.push(getId(type))
    }

    // now figure out the return type
    var rtnType = ref.coerceType(variadic_function_generator.returnType)
    var rtnName = getId(rtnType)
    assert(rtnName)

    // first let's generate the key and see if we got a cache-hit
    key = rtnName + key.join('')

    var func = cache[key]
    if (func) {
      debug('cache hit for key:', key)
    } else {
      // create the `ffi_cif *` instance
      debug('creating the variadic ffi_cif instance for key:', key)
      var cif = CIF_var(returnType, argTypes, numFixedArgs, abi)
      func = cache[key] = _ForeignFunction(cif, funcPtr, rtnType, argTypes)
    }
    return func
  }

  // set the return type. we set it as a property of the function generator to
  // allow for monkey patching the return value in the very rare case where the
  // return type is variadic as well
  variadic_function_generator.returnType = returnType

  return variadic_function_generator
}

module.exports = VariadicForeignFunction

var idKey = '_ffiId'
function getId (type) {
  if (!type.hasOwnProperty(idKey)) {
    type[idKey] = (((1+Math.random())*0x10000)|0).toString(16)
  }
  return type[idKey]
}

}).call(this,{"isBuffer":_dereq_("C:/Users/Lorenzo/AppData/Roaming/nvm/v8.11.2/node_modules/browserify/node_modules/is-buffer/index.js")})
},{"./_foreign_function":68,"./bindings":69,"./cif_var":72,"./type":80,"C:/Users/Lorenzo/AppData/Roaming/nvm/v8.11.2/node_modules/browserify/node_modules/is-buffer/index.js":15,"assert":2,"debug":66,"ref":86}],78:[function(_dereq_,module,exports){
(function (Buffer){

/**
 * Module dependencies.
 */

var ref = _dereq_('ref')
  , assert = _dereq_('assert')
  , bindings = _dereq_('./bindings')
  , Callback = _dereq_('./callback')
  , ForeignFunction = _dereq_('./foreign_function')
  , debug = _dereq_('debug')('ffi:FunctionType')
 
/**
 * Module exports.
 */

module.exports = Function

/**
 * Creates and returns a "type" object for a C "function pointer".
 *
 * @api public
 */

function Function (retType, argTypes, abi) {
  if (!(this instanceof Function)) {
    return new Function(retType, argTypes, abi)
  }

  debug('creating new FunctionType')

  // check args
  assert(!!retType, 'expected a return "type" object as the first argument')
  assert(Array.isArray(argTypes), 'expected Array of arg "type" objects as the second argument')

  // normalize the "types" (they could be strings, so turn into real type
  // instances)
  this.retType = ref.coerceType(retType)
  this.argTypes = argTypes.map(ref.coerceType)
  this.abi = null == abi ? bindings.FFI_DEFAULT_ABI : abi
}

/**
 * The "ffi_type" is set for node-ffi functions.
 */

Function.prototype.ffi_type = bindings.FFI_TYPES.pointer

/**
 * The "size" is always pointer-sized.
 */

Function.prototype.size = ref.sizeof.pointer

/**
 * The "alignment" is always pointer-aligned.
 */

Function.prototype.alignment = ref.alignof.pointer

/**
 * The "indirection" is always 1 to ensure that our get()/set() get called.
 */

Function.prototype.indirection = 1

/**
 * Returns a ffi.Callback pointer (Buffer) of this function type for the
 * given `fn` Function.
 */

Function.prototype.toPointer = function toPointer (fn) {
  return Callback(this.retType, this.argTypes, this.abi, fn)
}

/**
 * Returns a ffi.ForeignFunction (Function) of this function type for the
 * given `buf` Buffer.
 */

Function.prototype.toFunction = function toFunction (buf) {
  return ForeignFunction(buf, this.retType, this.argTypes, this.abi)
}

/**
 * get function; return a ForeignFunction instance.
 */

Function.prototype.get = function get (buffer, offset) {
  debug('ffi FunctionType "get" function')
  var ptr = buffer.readPointer(offset)
  return this.toFunction(ptr)
}

/**
 * set function; return a Callback buffer.
 */

Function.prototype.set = function set (buffer, offset, value) {
  debug('ffi FunctionType "set" function')
  var ptr
  if ('function' == typeof value) {
    ptr = this.toPointer(value)
  } else if (Buffer.isBuffer(value)) {
    ptr = value
  } else {
    throw new Error('don\'t know how to set callback function for: ' + value)
  }
  buffer.writePointer(ptr, offset)
}

}).call(this,{"isBuffer":_dereq_("C:/Users/Lorenzo/AppData/Roaming/nvm/v8.11.2/node_modules/browserify/node_modules/is-buffer/index.js")})
},{"./bindings":69,"./callback":70,"./foreign_function":76,"C:/Users/Lorenzo/AppData/Roaming/nvm/v8.11.2/node_modules/browserify/node_modules/is-buffer/index.js":15,"assert":2,"debug":66,"ref":86}],79:[function(_dereq_,module,exports){
(function (process){

/**
 * Module dependencies.
 */

var DynamicLibrary = _dereq_('./dynamic_library')
  , ForeignFunction = _dereq_('./foreign_function')
  , VariadicForeignFunction = _dereq_('./foreign_function_var')
  , debug = _dereq_('debug')('ffi:Library')
  , RTLD_NOW = DynamicLibrary.FLAGS.RTLD_NOW

/**
 * The extension to use on libraries.
 * i.e.  libm  ->  libm.so   on linux
 */

var EXT = Library.EXT = {
    'linux':  '.so'
  , 'linux2': '.so'
  , 'sunos':  '.so'
  , 'solaris':'.so'
  , 'freebsd':'.so'
  , 'openbsd':'.so'
  , 'darwin': '.dylib'
  , 'mac':    '.dylib'
  , 'win32':  '.dll'
}[process.platform]

/**
 * Provides a friendly abstraction/API on-top of DynamicLibrary and
 * ForeignFunction.
 */

function Library (libfile, funcs, lib) {
  debug('creating Library object for', libfile)

  if (libfile && libfile.indexOf(EXT) === -1) {
    debug('appending library extension to library name', EXT)
    libfile += EXT
  }

  if (!lib) {
    lib = {}
  }
  var dl = new DynamicLibrary(libfile || null, RTLD_NOW)

  Object.keys(funcs || {}).forEach(function (func) {
    debug('defining function', func)

    var fptr = dl.get(func)
      , info = funcs[func]

    if (fptr.isNull()) {
      throw new Error('Library: "' + libfile
        + '" returned NULL function pointer for "' + func + '"')
    }

    var resultType = info[0]
      , paramTypes = info[1]
      , fopts = info[2]
      , abi = fopts && fopts.abi
      , async = fopts && fopts.async
      , varargs = fopts && fopts.varargs

    if (varargs) {
      lib[func] = VariadicForeignFunction(fptr, resultType, paramTypes, abi)
    } else {
      var ff = ForeignFunction(fptr, resultType, paramTypes, abi)
      lib[func] = async ? ff.async : ff
    }
  })

  return lib
}
module.exports = Library

}).call(this,_dereq_('_process'))
},{"./dynamic_library":73,"./foreign_function":76,"./foreign_function_var":77,"_process":19,"debug":66}],80:[function(_dereq_,module,exports){
(function (Buffer){

/**
 * Module dependencies.
 */

var ref = _dereq_('ref')
var assert = _dereq_('assert')
var debug = _dereq_('debug')('ffi:types')
var Struct = _dereq_('ref-struct')
var bindings = _dereq_('./bindings')

/**
 * Define the `ffi_type` struct (see deps/libffi/include/ffi.h) for use in JS.
 * This struct type is used internally to define custom struct rtn/arg types.
 */

var FFI_TYPE = Type.FFI_TYPE = Struct()
FFI_TYPE.defineProperty('size',      ref.types.size_t)
FFI_TYPE.defineProperty('alignment', ref.types.ushort)
FFI_TYPE.defineProperty('type',      ref.types.ushort)
// this last prop is a C Array of `ffi_type *` elements, so this is `ffi_type **`
var ffi_type_ptr_array = ref.refType(ref.refType(FFI_TYPE))
FFI_TYPE.defineProperty('elements',  ffi_type_ptr_array)
assert.equal(bindings.FFI_TYPE_SIZE, FFI_TYPE.size)

/**
 * Returns a `ffi_type *` Buffer appropriate for the given "type".
 *
 * @param {Type|String} type A "ref" type (or string) to get the `ffi_type` for
 * @return {Buffer} A buffer pointing to a `ffi_type` instance for "type"
 * @api private
 */

function Type (type) {
  type = ref.coerceType(type)
  debug('Type()', type.name || type)
  assert(type.indirection >= 1, 'invalid "type" given: ' + (type.name || type))
  var rtn

  // first we assume it's a regular "type". if the "indirection" is greater than
  // 1, then we can just use "pointer" ffi_type, otherwise we hope "ffi_type" is
  // set
  if (type.indirection === 1) {
    rtn = type.ffi_type
  } else {
    rtn = bindings.FFI_TYPES.pointer
  }

  // if "rtn" isn't set (ffi_type was not set) then we check for "ref-array" type
  if (!rtn && type.type) {
    // got a "ref-array" type
    // passing arrays to C functions are always by reference, so we use "pointer"
    rtn = bindings.FFI_TYPES.pointer
  }

  if (!rtn && type.fields) {
    // got a "ref-struct" type
    // need to create the `ffi_type` instance manually
    debug('creating an `ffi_type` for given "ref-struct" type')
    var fields = type.fields
      , fieldNames = Object.keys(fields)
      , numFields = fieldNames.length
      , numElements = 0
      , ffi_type = new FFI_TYPE
      , i = 0
      , field
      , ffi_type_ptr

    // these are the "ffi_type" values expected for a struct
    ffi_type.size = 0
    ffi_type.alignment = 0
    ffi_type.type = 13 // FFI_TYPE_STRUCT

    // first we have to figure out the number of "elements" that will go in the
    // array. this would normally just be "numFields" but we also have to account
    // for arrays, which each act as their own element
    for (i = 0; i < numFields; i++) {
      field = fields[fieldNames[i]]
      if (field.type.fixedLength > 0) {
        // a fixed-length "ref-array" type
        numElements += field.type.fixedLength
      } else {
        numElements += 1
      }
    }

    // hand-crafting a null-terminated array here.
    // XXX: use "ref-array"?
    var size = ref.sizeof.pointer * (numElements + 1) // +1 because of the NULL terminator
    var elements = ffi_type.elements = new Buffer(size)
    var index = 0
    for (i = 0; i < numFields; i++) {
      field = fields[fieldNames[i]]
      if (field.type.fixedLength > 0) {
        // a fixed-length "ref-array" type
        ffi_type_ptr = Type(field.type.type)
        for (var j = 0; j < field.type.fixedLength; j++) {
          elements.writePointer(ffi_type_ptr, (index++) * ref.sizeof.pointer)
        }
      } else {
        ffi_type_ptr = Type(field.type)
        elements.writePointer(ffi_type_ptr, (index++) * ref.sizeof.pointer)
      }
    }
    // final NULL pointer to terminate the Array
    elements.writePointer(ref.NULL, index * ref.sizeof.pointer)
    // also set the `ffi_type` property to that it's cached for next time
    rtn = type.ffi_type = ffi_type.ref()
  }

  if (!rtn && type.name) {
    // handle "ref" types other than the set that node-ffi is using (i.e.
    // a separate copy)
    if ('CString' == type.name) {
      rtn = type.ffi_type = bindings.FFI_TYPES.pointer
    } else {
      var cur = type
      while (!rtn && cur) {
        rtn = cur.ffi_type = bindings.FFI_TYPES[cur.name]
        cur = Object.getPrototypeOf(cur)
      }
    }
  }

  assert(rtn, 'Could not determine the `ffi_type` instance for type: ' + (type.name || type))
  debug('returning `ffi_type`', rtn.name)
  return rtn
}
module.exports = Type

}).call(this,_dereq_("buffer").Buffer)
},{"./bindings":69,"assert":2,"buffer":8,"debug":66,"ref":86,"ref-struct":85}],81:[function(_dereq_,module,exports){
(function (Buffer){
/*
Copyright (c) 2011, Chris Umbel

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

var FFI = _dereq_('ffi');

function fortranArrayToJSMatrix(fortranArray, m, n, elementSize) {
    var op = elementSize == 8 ? 'readDoubleLE' : 'readFloatLE';
    var array = [];
    var rowWidth = elementSize * n;
    var columnOffset = m * elementSize;

    for(var i = 0; i < m; i++) {
        var row = [];
        var rowStart = i * elementSize;

        for(var j = 0; j < n; j++) {
            row.push(fortranArray[op](columnOffset * j + rowStart));
        }

        array.push(row);
    }

    return array;
}

function jsMatrixToFortranArray(array, elementSize) {
    var op = elementSize == 8 ? 'writeDoubleLE' : 'writeFloatLE';
    var m = array.length;
    var n = array[0].length;
    var fortranArrayStart = fortranArray = new Buffer(m * n * elementSize);
    for(var j = 0; j < n; j++) {
        for(var i = 0; i < m; i++) {
            fortranArray[op](array[i][j], elementSize * (j * m + i));
        }
    }

    return fortranArrayStart;
}

function fortranArrayToJSArray(fortranArray, n, op, elementSize) {
    var array = [];
    
    for(var i = 0; i < n; i++) {
        array.push(fortranArray[op](i * elementSize));
    }

    return array;
}

module.exports.fortranArrayToJSMatrix = fortranArrayToJSMatrix;
module.exports.jsMatrixToFortranArray = jsMatrixToFortranArray;
module.exports.fortranArrayToJSArray = fortranArrayToJSArray;

}).call(this,_dereq_("buffer").Buffer)
},{"buffer":8,"ffi":75}],82:[function(_dereq_,module,exports){
/*
Copyright (c) 2011, Chris Umbel

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

var lapack = _dereq_('./lapack.js');
exports.sgeqrf = lapack.sgeqrf;
exports.dgeqrf = lapack.dgeqrf;
exports.sgesvd = lapack.sgesvd;
exports.qr = lapack.qr;
exports.lu = lapack.lu;
exports.sgetrf = lapack.sgetrf;
exports.sgesv = lapack.sgesv;

},{"./lapack.js":83}],83:[function(_dereq_,module,exports){
(function (Buffer){
/*
Copyright (c) 2011, Chris Umbel

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

var fortranArray = _dereq_('./fortranArray');
var FFI = _dereq_('ffi');

var LAPACK;

try {
    LAPACK = new FFI.Library('liblapack', {
	"sgeqrf_": ["void", ["pointer", "pointer", "pointer", "pointer", "pointer", 
			    "pointer", "pointer", "pointer"]],
	"dgeqrf_": ["void", ["pointer", "pointer", "pointer", "pointer", "pointer", 
			    "pointer", "pointer", "pointer"]],
	"sorgqr_": ["void", ["pointer", "pointer", "pointer", "pointer", "pointer", "pointer", 
			    "pointer", "pointer", "pointer"]],
	"sgesvd_": ["void", ["pointer", "pointer", "pointer", "pointer", "pointer", 
			    "pointer", "pointer", "pointer", "pointer", "pointer", 
			    "pointer", "pointer", "pointer", "pointer", ]],
	"sgetrf_": ["void", ["pointer", "pointer", "pointer", "pointer", "pointer", "pointer"]],
	"dgetrf_": ["void", ["pointer", "pointer", "pointer", "pointer", "pointer", "pointer"]],
	"sgesv_": ["void", ["pointer", "pointer", "pointer", "pointer", "pointer", "pointer", "pointer", "pointer"]]
	
    });
} catch(e) {
    console.log("!!! node-lapack requires the native lapack to be built as a shared lib.");
    console.log(e);
}

var FORTRAN_INT = 4;
var FORTRAN_CHAR = 1;
var FORTRAN_FLOAT = 4;
var FORTRAN_DOUBLE = 8;

function eye(m) {
    var matrix = [];

    for(var i = 0; i < m; i++) {
	var row = [];
	matrix.push(row);
	
	for(var j = 0; j < m; j++) {
	    if(i == j)
		row.push(1);
	    else
		row.push(0);
	}
    }

    return matrix;
}

function matrixOp(matrix, elementSize, callback) {
    var m = matrix.length;
    var n = matrix[0].length;    
    var f_m = new Buffer(FORTRAN_INT);
    var f_n = new Buffer(FORTRAN_INT);
    var f_a = fortranArray.jsMatrixToFortranArray(matrix, elementSize);
    var f_lda = new Buffer(FORTRAN_INT);
    
    f_m.writeInt32LE(m, 0);
    f_n.writeInt32LE(n, 0);
    f_lda.writeInt32LE(Math.max(1, m), 0);
    
    callback(m, n, f_m, f_n, f_a, f_lda);
}

function zeroBottomLeft(matrix) {
    // zero out bottom left forming an upper right triangle matrix                
    for(var i = 1; i < matrix.length; i++) {
        for(var j = 0; j < i && j < matrix[0].length; j++)
            matrix[i][j] = 0;
    }

    return matrix
}

function sgesv(a, b) {
    var f_info = new Buffer(FORTRAN_INT);
    var result = {};

    matrixOp(a, FORTRAN_FLOAT, function(am, an, af_m, af_n, f_a) {
	var f_ipiv = new Buffer(am  * FORTRAN_INT);

	matrixOp(b, FORTRAN_FLOAT, function(bm, bn, bf_m, bf_n, f_b) {
	    LAPACK.sgesv_(af_m, bf_n, f_a, af_n, f_ipiv, f_b, bf_m, f_info);
	    result.X = fortranArray.fortranArrayToJSMatrix(f_b, bm, bn, FORTRAN_FLOAT);
	    result.P = ipivToP(bm, fortranArray.fortranArrayToJSArray(f_ipiv, bm, 'readInt32LE', FORTRAN_FLOAT));
	});
    });

    return result;
}

function qr(matrix) {
    var result;

    sgeqrf(matrix, function(qr, m, n, f_m, f_n, f_a, f_lda, f_tau, f_work, f_lwork, f_info) {
	var f_k = new Buffer(FORTRAN_INT);
	f_k.writeInt32LE(Math.min(m, n), 0);
	LAPACK.sorgqr_(f_m, f_n, f_k, f_a, f_lda, f_tau, f_work, f_lwork, f_info);
	qr.Q = fortranArray.fortranArrayToJSMatrix(f_a, m, n, FORTRAN_FLOAT);
	qr.R = zeroBottomLeft(qr.R);
	result = qr;
    });
    
    return result;
}

function sgeqrf(matrix, callback) {
    return geqrf(matrix, 'readFloatLE', LAPACK.sgeqrf_, FORTRAN_FLOAT, callback);
}

function dgeqrf(matrix, callback) {
    return geqrf(matrix, 'readDoubleLE', LAPACK.dgeqrf_, FORTRAN_DOUBLE, callback);
}

function geqrf(matrix, op, lapackFunc, elementSize, callback) {
    var qr;
    
    matrixOp(matrix, elementSize, function(m, n, f_m, f_n, f_a, f_lda) {
	var f_tau = new Buffer(m * n * elementSize);
	var f_info = new Buffer(FORTRAN_INT);
	var f_lwork = new Buffer(FORTRAN_INT);
	var f_work;
	f_lwork.writeInt32LE(-1, 0);
	
	// get optimal size of workspace
	f_work = new Buffer(FORTRAN_DOUBLE);
	lapackFunc(f_m, f_n, f_a, f_lda, f_tau, f_work, f_lwork, f_info);
	lwork = f_work[op](0);
	
	// allocate workspace
	f_work = new Buffer(lwork * elementSize);
	f_lwork.writeInt32LE(lwork, 0);
	
	// perform QR decomp
	lapackFunc(f_m, f_n, f_a, f_lda, f_tau, f_work, f_lwork, f_info); 
	
	qr = {
	    R: fortranArray.fortranArrayToJSMatrix(f_a, m, n, elementSize),
	    tau: fortranArray.fortranArrayToJSArray(f_tau, Math.min(m, n), op, elementSize)
	};
	
	if(callback)
	    qr = callback(qr, m, n, f_m, f_n, f_a, f_lda, f_tau, f_work, f_lwork, f_info);
    });
    
    return qr;
}

function cloneMatrix(matrix, height, width) {
    var clone = [];

    height = height || matrix.length;
    width = width || matrix[0].length;

    for(var i = 0; i < height; i++) {
	var row = [];
	clone.push(row);

	for(var j = 0; j < width; j++) {
	    row.push(matrix[i][j]);
	}
    }

    return clone;
}

function swapRows(matrix, i, j) {
    var tmp = matrix[j];
    matrix[j] = matrix[i];
    matrix[i] = tmp;
    return matrix;
}

function lu(matrix) {
    var result = sgetrf(matrix);
    var P = ipivToP(matrix.length, result.IPIV);
    var L = cloneMatrix(result.LU);
    var m = n = Math.min(matrix.length, matrix[0].length);

    for(var i = 0; i < L.length; i++) {
	for(var j = i; j < L[i].length; j++) {
	    if(i == j)
		L[i][j] = 1;
	    else
		L[i][j] = 0;
	}
    }

    return {
	L: L,
	U: zeroBottomLeft(cloneMatrix(result.LU, n, n)),
	P: P,
	IPIV: result.IPIV
    };
}

function ipivToP(m, ipiv){
    var P = eye(m);

    for(var i = 0; i < ipiv.length; i++) {
	if(i != ipiv[i] - 1)
            swapRows(P, i, ipiv[i] - 1);
    }

    return P;
}

function sgetrf(matrix) {
    return getrf(matrix, LAPACK.sgetrf_, FORTRAN_FLOAT);
}

function dgetrf(matrix) {
    return getrf(matrix, LAPACK.dgetrf_, FORTRAN_DOUBLE);
}

function getrf(matrix, lapackFunc, elementSize) {
    var result = {};

    matrixOp(matrix, elementSize, function(m, n, f_m, f_n, f_a, f_lda) {
	var f_ipiv = new Buffer(Math.min(m, n) * FORTRAN_INT);
	var f_info = new Buffer(FORTRAN_INT);
	lapackFunc(f_m, f_n, f_a, f_m, f_ipiv, f_info);
	result.LU = fortranArray.fortranArrayToJSMatrix(f_a, m, n, elementSize);
	result.IPIV = fortranArray.fortranArrayToJSArray(f_ipiv, Math.min(m, n), 'readInt32LE', elementSize);
    });

    return result;
}

function sgesvd(jobu, jobvt, matrix) {
    var f_jobu = new Buffer(FORTRAN_CHAR);
    var f_jobvt = new Buffer(FORTRAN_CHAR);
    f_jobu.writeUInt8(jobu.charCodeAt(0), 0);
    f_jobvt.writeUInt8(jobvt.charCodeAt(0), 0);
    var svd;

    matrixOp(matrix, FORTRAN_FLOAT, function(m, n, f_m, f_n, f_a, f_lda) {
	var f_s = new Buffer(Math.pow(Math.min(m, n), 2) * FORTRAN_FLOAT);
	var f_u = new Buffer(Math.pow(m, 2) * FORTRAN_FLOAT);
	var f_ldu = new Buffer(FORTRAN_INT);
	f_ldu.writeInt32LE(m, 0);

	// TODO: punting on dims for now. revisit with http://www.netlib.org/lapack/single/sgesvd.f
	var f_vt = new Buffer(Math.pow(n, 2) * FORTRAN_FLOAT);
	var f_ldvt = new Buffer(FORTRAN_INT);
	f_ldvt.writeInt32LE(n, 0);
	
	var lwork = -1;
	var f_work = new Buffer(FORTRAN_FLOAT);
	var f_lwork = new Buffer(FORTRAN_INT);
	f_lwork.writeInt32LE(lwork, 0);
	var f_info = new Buffer(FORTRAN_INT);

	LAPACK.sgesvd_(f_jobu, f_jobvt, f_m, f_n, f_a, f_lda, f_s, f_u, f_ldu, f_vt, f_ldvt, 
		      f_work, f_lwork, f_info);

	lwork = f_work.readFloatLE(0);
	f_work = new Buffer(lwork * FORTRAN_FLOAT);
	f_lwork.writeInt32LE(lwork, 0);

	LAPACK.sgesvd_(f_jobu, f_jobvt, f_m, f_n, f_a, f_lda, f_s, f_u, f_ldu, f_vt, f_ldvt, 
		      f_work, f_lwork, f_info);

	svd = {
	    U: fortranArray.fortranArrayToJSMatrix(f_u, m, m, FORTRAN_FLOAT),
	    S: fortranArray.fortranArrayToJSMatrix(f_s, n, n, FORTRAN_FLOAT),
	    VT: fortranArray.fortranArrayToJSMatrix(f_vt, n, n, FORTRAN_FLOAT)
	};
    });

    return svd;
}

exports.sgeqrf = sgeqrf;
exports.dgeqrf = dgeqrf;
exports.sgesvd = sgesvd;
exports.sgetrf = sgetrf;
exports.dgetrf = sgetrf;
exports.sgesv = sgesv;
exports.qr = qr;
exports.lu = lu;

}).call(this,_dereq_("buffer").Buffer)
},{"./fortranArray":81,"buffer":8,"ffi":75}],84:[function(_dereq_,module,exports){
/**
 * Helpers.
 */

var s = 1000;
var m = s * 60;
var h = m * 60;
var d = h * 24;
var y = d * 365.25;

/**
 * Parse or format the given `val`.
 *
 * Options:
 *
 *  - `long` verbose formatting [false]
 *
 * @param {String|Number} val
 * @param {Object} [options]
 * @throws {Error} throw an error if val is not a non-empty string or a number
 * @return {String|Number}
 * @api public
 */

module.exports = function(val, options) {
  options = options || {};
  var type = typeof val;
  if (type === 'string' && val.length > 0) {
    return parse(val);
  } else if (type === 'number' && isNaN(val) === false) {
    return options.long ? fmtLong(val) : fmtShort(val);
  }
  throw new Error(
    'val is not a non-empty string or a valid number. val=' +
      JSON.stringify(val)
  );
};

/**
 * Parse the given `str` and return milliseconds.
 *
 * @param {String} str
 * @return {Number}
 * @api private
 */

function parse(str) {
  str = String(str);
  if (str.length > 100) {
    return;
  }
  var match = /^((?:\d+)?\.?\d+) *(milliseconds?|msecs?|ms|seconds?|secs?|s|minutes?|mins?|m|hours?|hrs?|h|days?|d|years?|yrs?|y)?$/i.exec(
    str
  );
  if (!match) {
    return;
  }
  var n = parseFloat(match[1]);
  var type = (match[2] || 'ms').toLowerCase();
  switch (type) {
    case 'years':
    case 'year':
    case 'yrs':
    case 'yr':
    case 'y':
      return n * y;
    case 'days':
    case 'day':
    case 'd':
      return n * d;
    case 'hours':
    case 'hour':
    case 'hrs':
    case 'hr':
    case 'h':
      return n * h;
    case 'minutes':
    case 'minute':
    case 'mins':
    case 'min':
    case 'm':
      return n * m;
    case 'seconds':
    case 'second':
    case 'secs':
    case 'sec':
    case 's':
      return n * s;
    case 'milliseconds':
    case 'millisecond':
    case 'msecs':
    case 'msec':
    case 'ms':
      return n;
    default:
      return undefined;
  }
}

/**
 * Short format for `ms`.
 *
 * @param {Number} ms
 * @return {String}
 * @api private
 */

function fmtShort(ms) {
  if (ms >= d) {
    return Math.round(ms / d) + 'd';
  }
  if (ms >= h) {
    return Math.round(ms / h) + 'h';
  }
  if (ms >= m) {
    return Math.round(ms / m) + 'm';
  }
  if (ms >= s) {
    return Math.round(ms / s) + 's';
  }
  return ms + 'ms';
}

/**
 * Long format for `ms`.
 *
 * @param {Number} ms
 * @return {String}
 * @api private
 */

function fmtLong(ms) {
  return plural(ms, d, 'day') ||
    plural(ms, h, 'hour') ||
    plural(ms, m, 'minute') ||
    plural(ms, s, 'second') ||
    ms + ' ms';
}

/**
 * Pluralization helper.
 */

function plural(ms, n, name) {
  if (ms < n) {
    return;
  }
  if (ms < n * 1.5) {
    return Math.floor(ms / n) + ' ' + name;
  }
  return Math.ceil(ms / n) + ' ' + name + 's';
}

},{}],85:[function(_dereq_,module,exports){
(function (Buffer){

/**
 * An interface for modeling and instantiating C-style data structures. This is
 * not a constructor per-say, but a constructor generator. It takes an array of
 * tuples, the left side being the type, and the right side being a field name.
 * The order should be the same order it would appear in the C-style struct
 * definition. It returns a function that can be used to construct an object that
 * reads and writes to the data structure using properties specified by the
 * initial field list.
 *
 * The only verboten field names are "ref", which is used used on struct
 * instances as a function to retrieve the backing Buffer instance of the
 * struct, and "ref.buffer" which contains the backing Buffer instance.
 *
 *
 * Example:
 *
 * ``` javascript
 * var ref = require('ref')
 * var Struct = require('ref-struct')
 *
 * // create the `char *` type
 * var charPtr = ref.refType(ref.types.char)
 * var int = ref.types.int
 *
 * // create the struct "type" / constructor
 * var PasswordEntry = Struct({
 *     'username': 'string'
 *   , 'password': 'string'
 *   , 'salt':     int
 * })
 *
 * // create an instance of the struct, backed a Buffer instance
 * var pwd = new PasswordEntry()
 * pwd.username = 'ricky'
 * pwd.password = 'rbransonlovesnode.js'
 * pwd.salt = (Math.random() * 1000000) | 0
 *
 * pwd.username // → 'ricky'
 * pwd.password // → 'rbransonlovesnode.js'
 * pwd.salt     // → 820088
 * ```
 */

/**
 * Module dependencies.
 */

var ref = _dereq_('ref')
var util = _dereq_('util')
var assert = _dereq_('assert')
var debug = _dereq_('debug')('ref:struct')

/**
 * Module exports.
 */

module.exports = Struct

/**
 * The Struct "type" meta-constructor.
 */

function Struct () {
  debug('defining new struct "type"')

  /**
   * This is the "constructor" of the Struct type that gets returned.
   *
   * Invoke it with `new` to create a new Buffer instance backing the struct.
   * Pass it an existing Buffer instance to use that as the backing buffer.
   * Pass in an Object containing the struct fields to auto-populate the
   * struct with the data.
   */

  function StructType (arg, data) {
    if (!(this instanceof StructType)) {
      return new StructType(arg, data)
    }
    debug('creating new struct instance')
    var store
    if (Buffer.isBuffer(arg)) {
      debug('using passed-in Buffer instance to back the struct', arg)
      assert(arg.length >= StructType.size, 'Buffer instance must be at least ' +
          StructType.size + ' bytes to back this struct type')
      store = arg
      arg = data
    } else {
      debug('creating new Buffer instance to back the struct (size: %d)', StructType.size)
      store = new Buffer(StructType.size)
    }

    // set the backing Buffer store
    store.type = StructType
    this['ref.buffer'] = store

    if (arg) {
      for (var key in arg) {
        // hopefully hit the struct setters
        this[key] = arg[key]
      }
    }
    StructType._instanceCreated = true
  }

  // make instances inherit from the `proto`
  StructType.prototype = Object.create(proto, {
    constructor: {
        value: StructType
      , enumerable: false
      , writable: true
      , configurable: true
    }
  })

  StructType.defineProperty = defineProperty
  StructType.toString = toString
  StructType.fields = {}

  var opt = (arguments.length > 0 && arguments[1]) ? arguments[1] : {};
  // Setup the ref "type" interface. The constructor doubles as the "type" object
  StructType.size = 0
  StructType.alignment = 0
  StructType.indirection = 1
  StructType.isPacked = opt.packed ? Boolean(opt.packed) : false
  StructType.get = get
  StructType.set = set

  // Read the fields list and apply all the fields to the struct
  // TODO: Better arg handling... (maybe look at ES6 binary data API?)
  var arg = arguments[0]
  if (Array.isArray(arg)) {
    // legacy API
    arg.forEach(function (a) {
      var type = a[0]
      var name = a[1]
      StructType.defineProperty(name, type)
    })
  } else if (typeof arg === 'object') {
    Object.keys(arg).forEach(function (name) {
      var type = arg[name]
      StructType.defineProperty(name, type)
    })
  }

  return StructType
}

/**
 * The "get" function of the Struct "type" interface
 */

function get (buffer, offset) {
  debug('Struct "type" getter for buffer at offset', buffer, offset)
  if (offset > 0) {
    buffer = buffer.slice(offset)
  }
  return new this(buffer)
}

/**
 * The "set" function of the Struct "type" interface
 */

function set (buffer, offset, value) {
  debug('Struct "type" setter for buffer at offset', buffer, offset, value)
  var isStruct = value instanceof this
  if (isStruct) {
    // optimization: copy the buffer contents directly rather
    // than going through the ref-struct constructor
    value['ref.buffer'].copy(buffer, offset, 0, this.size)
  } else {
    if (offset > 0) {
      buffer = buffer.slice(offset)
    }
    new this(buffer, value)
  }
}

/**
 * Custom `toString()` override for struct type instances.
 */

function toString () {
  return '[StructType]'
}

/**
 * Adds a new field to the struct instance with the given name and type.
 * Note that this function will throw an Error if any instances of the struct
 * type have already been created, therefore this function must be called at the
 * beginning, before any instances are created.
 */

function defineProperty (name, type) {
  debug('defining new struct type field', name)

  // allow string types for convenience
  type = ref.coerceType(type)

  assert(!this._instanceCreated, 'an instance of this Struct type has already ' +
      'been created, cannot add new "fields" anymore')
  assert.equal('string', typeof name, 'expected a "string" field name')
  assert(type && /object|function/i.test(typeof type) && 'size' in type &&
      'indirection' in type
      , 'expected a "type" object describing the field type: "' + type + '"')
  assert(type.indirection > 1 || type.size > 0,
      '"type" object must have a size greater than 0')
  assert(!(name in this.prototype), 'the field "' + name +
      '" already exists in this Struct type')

  var field = {
    type: type
  }
  this.fields[name] = field

  // define the getter/setter property
  var desc = { enumerable: true , configurable: true }
  desc.get = function () {
    debug('getting "%s" struct field (offset: %d)', name, field.offset)
    return ref.get(this['ref.buffer'], field.offset, type)
  }
  desc.set = function (value) {
    debug('setting "%s" struct field (offset: %d)', name, field.offset, value)
    return ref.set(this['ref.buffer'], field.offset, value, type)
  }

  // calculate the new size and field offsets
  recalc(this)

  Object.defineProperty(this.prototype, name, desc)
}

function recalc (struct) {

  // reset size and alignment
  struct.size = 0
  struct.alignment = 0

  var fieldNames = Object.keys(struct.fields)

  // first loop through is to determine the `alignment` of this struct
  fieldNames.forEach(function (name) {
    var field = struct.fields[name]
    var type = field.type
    var alignment = type.alignment || ref.alignof.pointer
    if (type.indirection > 1) {
      alignment = ref.alignof.pointer
    }
    if (struct.isPacked) {
      struct.alignment = Math.min(struct.alignment || alignment, alignment)
    } else {
      struct.alignment = Math.max(struct.alignment, alignment)
    }
  })

  // second loop through sets the `offset` property on each "field"
  // object, and sets the `struct.size` as we go along
  fieldNames.forEach(function (name) {
    var field = struct.fields[name]
    var type = field.type

    if (null != type.fixedLength) {
      // "ref-array" types set the "fixedLength" prop. don't treat arrays like one
      // contiguous entity. instead, treat them like individual elements in the
      // struct. doing this makes the padding end up being calculated correctly.
      field.offset = addType(type.type)
      for (var i = 1; i < type.fixedLength; i++) {
        addType(type.type)
      }
    } else {
      field.offset = addType(type)
    }
  })

  function addType (type) {
    var offset = struct.size
    var align = type.indirection === 1 ? type.alignment : ref.alignof.pointer
    var padding = struct.isPacked ? 0 : (align - (offset % align)) % align
    var size = type.indirection === 1 ? type.size : ref.sizeof.pointer

    offset += padding

    if (!struct.isPacked) {
      assert.equal(offset % align, 0, "offset should align")
    }

    // adjust the "size" of the struct type
    struct.size = offset + size

    // return the calulated offset
    return offset
  }

  // any final padding?
  var left = struct.size % struct.alignment
  if (left > 0) {
    debug('additional padding to the end of struct:', struct.alignment - left)
    struct.size += struct.alignment - left
  }
}

/**
 * this is the custom prototype of Struct type instances.
 */

var proto = {}

/**
 * set a placeholder variable on the prototype so that defineProperty() will
 * throw an error if you try to define a struct field with the name "buffer".
 */

proto['ref.buffer'] = ref.NULL

/**
 * Flattens the Struct instance into a regular JavaScript Object. This function
 * "gets" all the defined properties.
 *
 * @api public
 */

proto.toObject = function toObject () {
  var obj = {}
  Object.keys(this.constructor.fields).forEach(function (k) {
    obj[k] = this[k]
  }, this)
  return obj
}

/**
 * Basic `JSON.stringify(struct)` support.
 */

proto.toJSON = function toJSON () {
  return this.toObject()
}

/**
 * `.inspect()` override. For the REPL.
 *
 * @api public
 */

proto.inspect = function inspect () {
  var obj = this.toObject()
  // add instance's "own properties"
  Object.keys(this).forEach(function (k) {
    obj[k] = this[k]
  }, this)
  return util.inspect(obj)
}

/**
 * returns a Buffer pointing to this struct data structure.
 */

proto.ref = function ref () {
  return this['ref.buffer']
}

}).call(this,_dereq_("buffer").Buffer)
},{"assert":2,"buffer":8,"debug":66,"ref":86,"util":39}],86:[function(_dereq_,module,exports){
(function (Buffer){
var assert = _dereq_('assert')
var inspect = _dereq_('util').inspect
var debug = _dereq_('debug')('ref')

exports = module.exports = _dereq_('bindings')('binding')

/**
 * A `Buffer` that references the C NULL pointer. That is, its memory address
 * points to 0. Its `length` is 0 because accessing any data from this buffer
 * would cause a _segmentation fault_.
 *
 * ```
 * console.log(ref.NULL);
 * <SlowBuffer@0x0 >
 * ```
 *
 * @name NULL
 * @type Buffer
 */

/**
 * A string that represents the native endianness of the machine's processor.
 * The possible values are either `"LE"` or `"BE"`.
 *
 * ```
 * console.log(ref.endianness);
 * 'LE'
 * ```
 *
 * @name endianness
 * @type String
 */

/**
 * Accepts a `Buffer` instance and returns the memory address of the buffer
 * instance.
 *
 * ```
 * console.log(ref.address(new Buffer(1)));
 * 4320233616
 *
 * console.log(ref.address(ref.NULL)));
 * 0
 * ```
 *
 * @param {Buffer} buffer The buffer to get the memory address of.
 * @return {Number} The memory address the buffer instance.
 * @name address
 * @type method
 */

/**
 * Accepts a `Buffer` instance and returns _true_ if the buffer represents the
 * NULL pointer, _false_ otherwise.
 *
 * ```
 * console.log(ref.isNull(new Buffer(1)));
 * false
 *
 * console.log(ref.isNull(ref.NULL));
 * true
 * ```
 *
 * @param {Buffer} buffer The buffer to check for NULL.
 * @return {Boolean} true or false.
 * @name isNull
 * @type method
 */

/**
 * Reads a JavaScript Object that has previously been written to the given
 * _buffer_ at the given _offset_.
 *
 * ```
 * var obj = { foo: 'bar' };
 * var buf = ref.alloc('Object', obj);
 *
 * var obj2 = ref.readObject(buf, 0);
 * console.log(obj === obj2);
 * true
 * ```
 *
 * @param {Buffer} buffer The buffer to read an Object from.
 * @param {Number} offset The offset to begin reading from.
 * @return {Object} The Object that was read from _buffer_.
 * @name readObject
 * @type method
 */

/**
 * Reads a Buffer instance from the given _buffer_ at the given _offset_.
 * The _size_ parameter specifies the `length` of the returned Buffer instance,
 * which defaults to __0__.
 *
 * ```
 * var buf = new Buffer('hello world');
 * var pointer = ref.alloc('pointer');
 *
 * var buf2 = ref.readPointer(pointer, 0, buf.length);
 * console.log(buf.toString());
 * 'hello world'
 * ```
 *
 * @param {Buffer} buffer The buffer to read a Buffer from.
 * @param {Number} offset The offset to begin reading from.
 * @param {Number} length (optional) The length of the returned Buffer. Defaults to 0.
 * @return {Buffer} The Buffer instance that was read from _buffer_.
 * @name readPointer
 * @type method
 */

/**
 * Returns a JavaScript String read from _buffer_ at the given _offset_. The
 * C String is read until the first NULL byte, which indicates the end of the
 * String.
 *
 * This function can read beyond the `length` of a Buffer.
 *
 * ```
 * var buf = new Buffer('hello\0world\0');
 *
 * var str = ref.readCString(buf, 0);
 * console.log(str);
 * 'hello'
 * ```
 *
 * @param {Buffer} buffer The buffer to read a Buffer from.
 * @param {Number} offset The offset to begin reading from.
 * @return {String} The String that was read from _buffer_.
 * @name readCString
 * @type method
 */

/**
 * Returns a big-endian signed 64-bit int read from _buffer_ at the given
 * _offset_.
 *
 * If the returned value will fit inside a JavaScript Number without losing
 * precision, then a Number is returned, otherwise a String is returned.
 *
 * ```
 * var buf = ref.alloc('int64');
 * ref.writeInt64BE(buf, 0, '9223372036854775807');
 *
 * var val = ref.readInt64BE(buf, 0)
 * console.log(val)
 * '9223372036854775807'
 * ```
 *
 * @param {Buffer} buffer The buffer to read a Buffer from.
 * @param {Number} offset The offset to begin reading from.
 * @return {Number|String} The Number or String that was read from _buffer_.
 * @name readInt64BE
 * @type method
 */

/**
 * Returns a little-endian signed 64-bit int read from _buffer_ at the given
 * _offset_.
 *
 * If the returned value will fit inside a JavaScript Number without losing
 * precision, then a Number is returned, otherwise a String is returned.
 *
 * ```
 * var buf = ref.alloc('int64');
 * ref.writeInt64LE(buf, 0, '9223372036854775807');
 *
 * var val = ref.readInt64LE(buf, 0)
 * console.log(val)
 * '9223372036854775807'
 * ```
 *
 * @param {Buffer} buffer The buffer to read a Buffer from.
 * @param {Number} offset The offset to begin reading from.
 * @return {Number|String} The Number or String that was read from _buffer_.
 * @name readInt64LE
 * @type method
 */

/**
 * Returns a big-endian unsigned 64-bit int read from _buffer_ at the given
 * _offset_.
 *
 * If the returned value will fit inside a JavaScript Number without losing
 * precision, then a Number is returned, otherwise a String is returned.
 *
 * ```
 * var buf = ref.alloc('uint64');
 * ref.writeUInt64BE(buf, 0, '18446744073709551615');
 *
 * var val = ref.readUInt64BE(buf, 0)
 * console.log(val)
 * '18446744073709551615'
 * ```
 *
 * @param {Buffer} buffer The buffer to read a Buffer from.
 * @param {Number} offset The offset to begin reading from.
 * @return {Number|String} The Number or String that was read from _buffer_.
 * @name readUInt64BE
 * @type method
 */

/**
 * Returns a little-endian unsigned 64-bit int read from _buffer_ at the given
 * _offset_.
 *
 * If the returned value will fit inside a JavaScript Number without losing
 * precision, then a Number is returned, otherwise a String is returned.
 *
 * ```
 * var buf = ref.alloc('uint64');
 * ref.writeUInt64LE(buf, 0, '18446744073709551615');
 *
 * var val = ref.readUInt64LE(buf, 0)
 * console.log(val)
 * '18446744073709551615'
 * ```
 *
 * @param {Buffer} buffer The buffer to read a Buffer from.
 * @param {Number} offset The offset to begin reading from.
 * @return {Number|String} The Number or String that was read from _buffer_.
 * @name readUInt64LE
 * @type method
 */

/**
 * Writes the _input_ Number or String as a big-endian signed 64-bit int into
 * _buffer_ at the given _offset_.
 *
 * ```
 * var buf = ref.alloc('int64');
 * ref.writeInt64BE(buf, 0, '9223372036854775807');
 * ```
 *
 * @param {Buffer} buffer The buffer to write to.
 * @param {Number} offset The offset to begin writing from.
 * @param {Number|String} input This String or Number which gets written.
 * @name writeInt64BE
 * @type method
 */

/**
 * Writes the _input_ Number or String as a little-endian signed 64-bit int into
 * _buffer_ at the given _offset_.
 *
 * ```
 * var buf = ref.alloc('int64');
 * ref.writeInt64LE(buf, 0, '9223372036854775807');
 * ```
 *
 * @param {Buffer} buffer The buffer to write to.
 * @param {Number} offset The offset to begin writing from.
 * @param {Number|String} input This String or Number which gets written.
 * @name writeInt64LE
 * @type method
 */

/**
 * Writes the _input_ Number or String as a big-endian unsigned 64-bit int into
 * _buffer_ at the given _offset_.
 *
 * ```
 * var buf = ref.alloc('uint64');
 * ref.writeUInt64BE(buf, 0, '18446744073709551615');
 * ```
 *
 * @param {Buffer} buffer The buffer to write to.
 * @param {Number} offset The offset to begin writing from.
 * @param {Number|String} input This String or Number which gets written.
 * @name writeUInt64BE
 * @type method
 */

/**
 * Writes the _input_ Number or String as a little-endian unsigned 64-bit int
 * into _buffer_ at the given _offset_.
 *
 * ```
 * var buf = ref.alloc('uint64');
 * ref.writeUInt64LE(buf, 0, '18446744073709551615');
 * ```
 *
 * @param {Buffer} buffer The buffer to write to.
 * @param {Number} offset The offset to begin writing from.
 * @param {Number|String} input This String or Number which gets written.
 * @name writeUInt64LE
 * @type method
 */

/**
 * Returns a new clone of the given "type" object, with its
 * `indirection` level incremented by **1**.
 *
 * Say you wanted to create a type representing a `void *`:
 *
 * ```
 * var voidPtrType = ref.refType(ref.types.void);
 * ```
 *
 * @param {Object|String} type The "type" object to create a reference type from. Strings get coerced first.
 * @return {Object} The new "type" object with its `indirection` incremented by 1.
 */

exports.refType = function refType (type) {
  var _type = exports.coerceType(type)
  var rtn = Object.create(_type)
  rtn.indirection++
  if (_type.name) {
    Object.defineProperty(rtn, 'name', {
      value: _type.name + '*',
      configurable: true,
      enumerable: true,
      writable: true
    })
  }
  return rtn
}

/**
 * Returns a new clone of the given "type" object, with its
 * `indirection` level decremented by 1.
 *
 * @param {Object|String} type The "type" object to create a dereference type from. Strings get coerced first.
 * @return {Object} The new "type" object with its `indirection` decremented by 1.
 */

exports.derefType = function derefType (type) {
  var _type = exports.coerceType(type)
  if (_type.indirection === 1) {
    throw new Error('Cannot create deref\'d type for type with indirection 1')
  }
  var rtn = Object.getPrototypeOf(_type)
  if (rtn.indirection !== _type.indirection - 1) {
    // slow case
    rtn = Object.create(_type)
    rtn.indirection--
  }
  return rtn
}

/**
 * Coerces a "type" object from a String or an actual "type" object. String values
 * are looked up from the `ref.types` Object. So:
 *
 *   * `"int"` gets coerced into `ref.types.int`.
 *   * `"int *"` gets translated into `ref.refType(ref.types.int)`
 *   * `ref.types.int` gets translated into `ref.types.int` (returns itself)
 *
 * Throws an Error if no valid "type" object could be determined. Most `ref`
 * functions use this function under the hood, so anywhere a "type" object is
 * expected, a String may be passed as well, including simply setting the
 * `buffer.type` property.
 *
 * ```
 * var type = ref.coerceType('int **');
 *
 * console.log(type.indirection);
 * 3
 * ```
 *
 * @param {Object|String} type The "type" Object or String to coerce.
 * @return {Object} A "type" object
 */

exports.coerceType = function coerceType (type) {
  var rtn = type
  if (typeof rtn === 'string') {
    rtn = exports.types[type]
    if (rtn) return rtn

    // strip whitespace
    rtn = type.replace(/\s+/g, '').toLowerCase()
    if (rtn === 'pointer') {
      // legacy "pointer" being used :(
      rtn = exports.refType(exports.types.void) // void *
    } else if (rtn === 'string') {
      rtn = exports.types.CString // special char * type
    } else {
      var refCount = 0
      rtn = rtn.replace(/\*/g, function () {
        refCount++
        return ''
      })
      // allow string names to be passed in
      rtn = exports.types[rtn]
      if (refCount > 0) {
        if (!(rtn && 'size' in rtn && 'indirection' in rtn)) {
          throw new TypeError('could not determine a proper "type" from: ' + JSON.stringify(type))
        }
        for (var i = 0; i < refCount; i++) {
          rtn = exports.refType(rtn)
        }
      }
    }
  }
  if (!(rtn && 'size' in rtn && 'indirection' in rtn)) {
    throw new TypeError('could not determine a proper "type" from: ' + JSON.stringify(type))
  }
  return rtn
}

/**
 * Returns the "type" property of the given Buffer.
 * Creates a default type for the buffer when none exists.
 *
 * @param {Buffer} buffer The Buffer instance to get the "type" object from.
 * @return {Object} The "type" object from the given Buffer.
 */

exports.getType = function getType (buffer) {
  if (!buffer.type) {
    debug('WARN: no "type" found on buffer, setting default "type"', buffer)
    buffer.type = {}
    buffer.type.size = buffer.length
    buffer.type.indirection = 1
    buffer.type.get = function get () {
      throw new Error('unknown "type"; cannot get()')
    }
    buffer.type.set = function set () {
      throw new Error('unknown "type"; cannot set()')
    }
  }
  return exports.coerceType(buffer.type)
}

/**
 * Calls the `get()` function of the Buffer's current "type" (or the
 * passed in _type_ if present) at the given _offset_.
 *
 * This function handles checking the "indirection" level and returning a
 * proper "dereferenced" Bufffer instance when necessary.
 *
 * @param {Buffer} buffer The Buffer instance to read from.
 * @param {Number} offset (optional) The offset on the Buffer to start reading from. Defaults to 0.
 * @param {Object|String} type (optional) The "type" object to use when reading. Defaults to calling `getType()` on the buffer.
 * @return {?} Whatever value the "type" used when reading returns.
 */

exports.get = function get (buffer, offset, type) {
  if (!offset) {
    offset = 0
  }
  if (type) {
    type = exports.coerceType(type)
  } else {
    type = exports.getType(buffer)
  }
  debug('get(): (offset: %d)', offset, buffer)
  assert(type.indirection > 0, '"indirection" level must be at least 1')
  if (type.indirection === 1) {
    // need to check "type"
    return type.get(buffer, offset)
  } else {
    // need to create a deref'd Buffer
    var size = type.indirection === 2 ? type.size : exports.sizeof.pointer
    var reference = exports.readPointer(buffer, offset, size)
    reference.type = exports.derefType(type)
    return reference
  }
}

/**
 * Calls the `set()` function of the Buffer's current "type" (or the
 * passed in _type_ if present) at the given _offset_.
 *
 * This function handles checking the "indirection" level writing a pointer rather
 * than calling the `set()` function if the indirection is greater than 1.
 *
 * @param {Buffer} buffer The Buffer instance to write to.
 * @param {Number} offset The offset on the Buffer to start writing to.
 * @param {?} value The value to write to the Buffer instance.
 * @param {Object|String} type (optional) The "type" object to use when reading. Defaults to calling `getType()` on the buffer.
 */

exports.set = function set (buffer, offset, value, type) {
  if (!offset) {
    offset = 0
  }
  if (type) {
    type = exports.coerceType(type)
  } else {
    type = exports.getType(buffer)
  }
  debug('set(): (offset: %d)', offset, buffer, value)
  assert(type.indirection >= 1, '"indirection" level must be at least 1')
  if (type.indirection === 1) {
    type.set(buffer, offset, value)
  } else {
    exports.writePointer(buffer, offset, value)
  }
}


/**
 * Returns a new Buffer instance big enough to hold `type`,
 * with the given `value` written to it.
 *
 * ``` js
 * var intBuf = ref.alloc(ref.types.int)
 * var int_with_4 = ref.alloc(ref.types.int, 4)
 * ```
 *
 * @param {Object|String} type The "type" object to allocate. Strings get coerced first.
 * @param {?} value (optional) The initial value set on the returned Buffer, using _type_'s `set()` function.
 * @return {Buffer} A new Buffer instance with it's `type` set to "type", and (optionally) "value" written to it.
 */

exports.alloc = function alloc (_type, value) {
  var type = exports.coerceType(_type)
  debug('allocating Buffer for type with "size"', type.size)
  var size
  if (type.indirection === 1) {
    size = type.size
  } else {
    size = exports.sizeof.pointer
  }
  var buffer = new Buffer(size)
  buffer.type = type
  if (arguments.length >= 2) {
    debug('setting value on allocated buffer', value)
    exports.set(buffer, 0, value, type)
  }
  return buffer
}

/**
 * Returns a new `Buffer` instance with the given String written to it with the
 * given encoding (defaults to __'utf8'__). The buffer is 1 byte longer than the
 * string itself, and is NUL terminated.
 *
 * ```
 * var buf = ref.allocCString('hello world');
 *
 * console.log(buf.toString());
 * 'hello world\u0000'
 * ```
 *
 * @param {String} string The JavaScript string to be converted to a C string.
 * @param {String} encoding (optional) The encoding to use for the C string. Defaults to __'utf8'__.
 * @return {Buffer} The new `Buffer` instance with the specified String wrtten to it, and a trailing NUL byte.
 */

exports.allocCString = function allocCString (string, encoding) {
  if (null == string || (Buffer.isBuffer(string) && exports.isNull(string))) {
    return exports.NULL
  }
  var size = Buffer.byteLength(string, encoding) + 1
  var buffer = new Buffer(size)
  exports.writeCString(buffer, 0, string, encoding)
  buffer.type = charPtrType
  return buffer
}

/**
 * Writes the given string as a C String (NULL terminated) to the given buffer
 * at the given offset. "encoding" is optional and defaults to __'utf8'__.
 *
 * Unlike `readCString()`, this function requires the buffer to actually have the
 * proper length.
 *
 * @param {Buffer} buffer The Buffer instance to write to.
 * @param {Number} offset The offset of the buffer to begin writing at.
 * @param {String} string The JavaScript String to write that will be written to the buffer.
 * @param {String} encoding (optional) The encoding to read the C string as. Defaults to __'utf8'__.
 */

exports.writeCString = function writeCString (buffer, offset, string, encoding) {
  assert(Buffer.isBuffer(buffer), 'expected a Buffer as the first argument')
  assert.equal('string', typeof string, 'expected a "string" as the third argument')
  if (!offset) {
    offset = 0
  }
  if (!encoding) {
    encoding = 'utf8'
  }
  var size = buffer.length - offset
  var len = buffer.write(string, offset, size, encoding)
  buffer.writeUInt8(0, offset + len)  // NUL terminate
}

exports['readInt64' + exports.endianness] = exports.readInt64
exports['readUInt64' + exports.endianness] = exports.readUInt64
exports['writeInt64' + exports.endianness] = exports.writeInt64
exports['writeUInt64' + exports.endianness] = exports.writeUInt64

var opposite = exports.endianness == 'LE' ? 'BE' : 'LE'
var int64temp = new Buffer(exports.sizeof.int64)
var uint64temp = new Buffer(exports.sizeof.uint64)

exports['readInt64' + opposite] = function (buffer, offset) {
  for (var i = 0; i < exports.sizeof.int64; i++) {
    int64temp[i] = buffer[offset + exports.sizeof.int64 - i - 1]
  }
  return exports.readInt64(int64temp, 0)
}
exports['readUInt64' + opposite] = function (buffer, offset) {
  for (var i = 0; i < exports.sizeof.uint64; i++) {
    uint64temp[i] = buffer[offset + exports.sizeof.uint64 - i - 1]
  }
  return exports.readUInt64(uint64temp, 0)
}
exports['writeInt64' + opposite] = function (buffer, offset, value) {
  exports.writeInt64(int64temp, 0, value)
  for (var i = 0; i < exports.sizeof.int64; i++) {
    buffer[offset + i] = int64temp[exports.sizeof.int64 - i - 1]
  }
}
exports['writeUInt64' + opposite] = function (buffer, offset, value) {
  exports.writeUInt64(uint64temp, 0, value)
  for (var i = 0; i < exports.sizeof.uint64; i++) {
    buffer[offset + i] = uint64temp[exports.sizeof.uint64 - i - 1]
  }
}

/**
 * `ref()` accepts a Buffer instance and returns a new Buffer
 * instance that is "pointer" sized and has its data pointing to the given
 * Buffer instance. Essentially the created Buffer is a "reference" to the
 * original pointer, equivalent to the following C code:
 *
 * ``` c
 * char *buf = buffer;
 * char **ref = &buf;
 * ```
 *
 * @param {Buffer} buffer A Buffer instance to create a reference to.
 * @return {Buffer} A new Buffer instance pointing to _buffer_.
 */

exports.ref = function ref (buffer) {
  debug('creating a reference to buffer', buffer)
  var type = exports.refType(exports.getType(buffer))
  return exports.alloc(type, buffer)
}

/**
 * Accepts a Buffer instance and attempts to "dereference" it.
 * That is, first it checks the `indirection` count of _buffer_'s "type", and if
 * it's greater than __1__ then it merely returns another Buffer, but with one
 * level less `indirection`.
 *
 * When _buffer_'s indirection is at __1__, then it checks for `buffer.type`
 * which should be an Object with its own `get()` function.
 *
 * ```
 * var buf = ref.alloc('int', 6);
 *
 * var val = ref.deref(buf);
 * console.log(val);
 * 6
 * ```
 *
 *
 * @param {Buffer} buffer A Buffer instance to dereference.
 * @return {?} The returned value after dereferencing _buffer_.
 */

exports.deref = function deref (buffer) {
  debug('dereferencing buffer', buffer)
  return exports.get(buffer)
}

/**
 * Attaches _object_ to _buffer_ such that it prevents _object_ from being garbage
 * collected until _buffer_ does.
 *
 * @param {Buffer} buffer A Buffer instance to attach _object_ to.
 * @param {Object|Buffer} object An Object or Buffer to prevent from being garbage collected until _buffer_ does.
 * @api private
 */

exports._attach = function _attach (buf, obj) {
  if (!buf._refs) {
    buf._refs = []
  }
  buf._refs.push(obj)
}

/**
 * Same as `ref.writeObject()`, except that this version does not _attach_ the
 * Object to the Buffer, which is potentially unsafe if the garbage collector
 * runs.
 *
 * @param {Buffer} buffer A Buffer instance to write _object_ to.
 * @param {Number} offset The offset on the Buffer to start writing at.
 * @param {Object} object The Object to be written into _buffer_.
 * @api private
 */

exports._writeObject = exports.writeObject

/**
 * Writes a pointer to _object_ into _buffer_ at the specified _offset.
 *
 * This function "attaches" _object_ to _buffer_ to prevent it from being garbage
 * collected.
 *
 * ```
 * var buf = ref.alloc('Object');
 * ref.writeObject(buf, 0, { foo: 'bar' });
 *
 * ```
 *
 * @param {Buffer} buffer A Buffer instance to write _object_ to.
 * @param {Number} offset The offset on the Buffer to start writing at.
 * @param {Object} object The Object to be written into _buffer_.
 */

exports.writeObject = function writeObject (buf, offset, obj, persistent) {
  debug('writing Object to buffer', buf, offset, obj, persistent)
  exports._writeObject(buf, offset, obj, persistent)
  exports._attach(buf, obj)
}

/**
 * Same as `ref.writePointer()`, except that this version does not attach
 * _pointer_ to _buffer_, which is potentially unsafe if the garbage collector
 * runs.
 *
 * @param {Buffer} buffer A Buffer instance to write _pointer to.
 * @param {Number} offset The offset on the Buffer to start writing at.
 * @param {Buffer} pointer The Buffer instance whose memory address will be written to _buffer_.
 * @api private
 */

exports._writePointer = exports.writePointer

/**
 * Writes the memory address of _pointer_ to _buffer_ at the specified _offset_.
 *
 * This function "attaches" _object_ to _buffer_ to prevent it from being garbage
 * collected.
 *
 * ```
 * var someBuffer = new Buffer('whatever');
 * var buf = ref.alloc('pointer');
 * ref.writePointer(buf, 0, someBuffer);
 * ```
 *
 * @param {Buffer} buffer A Buffer instance to write _pointer to.
 * @param {Number} offset The offset on the Buffer to start writing at.
 * @param {Buffer} pointer The Buffer instance whose memory address will be written to _buffer_.
 */

exports.writePointer = function writePointer (buf, offset, ptr) {
  debug('writing pointer to buffer', buf, offset, ptr)
  exports._writePointer(buf, offset, ptr)
  exports._attach(buf, ptr)
}

/**
 * Same as `ref.reinterpret()`, except that this version does not attach
 * _buffer_ to the returned Buffer, which is potentially unsafe if the
 * garbage collector runs.
 *
 * @param {Buffer} buffer A Buffer instance to base the returned Buffer off of.
 * @param {Number} size The `length` property of the returned Buffer.
 * @param {Number} offset The offset of the Buffer to begin from.
 * @return {Buffer} A new Buffer instance with the same memory address as _buffer_, and the requested _size_.
 * @api private
 */

exports._reinterpret = exports.reinterpret

/**
 * Returns a new Buffer instance with the specified _size_, with the same memory
 * address as _buffer_.
 *
 * This function "attaches" _buffer_ to the returned Buffer to prevent it from
 * being garbage collected.
 *
 * @param {Buffer} buffer A Buffer instance to base the returned Buffer off of.
 * @param {Number} size The `length` property of the returned Buffer.
 * @param {Number} offset The offset of the Buffer to begin from.
 * @return {Buffer} A new Buffer instance with the same memory address as _buffer_, and the requested _size_.
 */

exports.reinterpret = function reinterpret (buffer, size, offset) {
  debug('reinterpreting buffer to "%d" bytes', size)
  var rtn = exports._reinterpret(buffer, size, offset || 0)
  exports._attach(rtn, buffer)
  return rtn
}

/**
 * Same as `ref.reinterpretUntilZeros()`, except that this version does not
 * attach _buffer_ to the returned Buffer, which is potentially unsafe if the
 * garbage collector runs.
 *
 * @param {Buffer} buffer A Buffer instance to base the returned Buffer off of.
 * @param {Number} size The number of sequential, aligned `NULL` bytes that are required to terminate the buffer.
 * @param {Number} offset The offset of the Buffer to begin from.
 * @return {Buffer} A new Buffer instance with the same memory address as _buffer_, and a variable `length` that is terminated by _size_ NUL bytes.
 * @api private
 */

exports._reinterpretUntilZeros = exports.reinterpretUntilZeros

/**
 * Accepts a `Buffer` instance and a number of `NULL` bytes to read from the
 * pointer. This function will scan past the boundary of the Buffer's `length`
 * until it finds `size` number of aligned `NULL` bytes.
 *
 * This is useful for finding the end of NUL-termintated array or C string. For
 * example, the `readCString()` function _could_ be implemented like:
 *
 * ```
 * function readCString (buf) {
 *   return ref.reinterpretUntilZeros(buf, 1).toString('utf8')
 * }
 * ```
 *
 * This function "attaches" _buffer_ to the returned Buffer to prevent it from
 * being garbage collected.
 *
 * @param {Buffer} buffer A Buffer instance to base the returned Buffer off of.
 * @param {Number} size The number of sequential, aligned `NULL` bytes are required to terminate the buffer.
 * @param {Number} offset The offset of the Buffer to begin from.
 * @return {Buffer} A new Buffer instance with the same memory address as _buffer_, and a variable `length` that is terminated by _size_ NUL bytes.
 */

exports.reinterpretUntilZeros = function reinterpretUntilZeros (buffer, size, offset) {
  debug('reinterpreting buffer to until "%d" NULL (0) bytes are found', size)
  var rtn = exports._reinterpretUntilZeros(buffer, size, offset || 0)
  exports._attach(rtn, buffer)
  return rtn
}


// the built-in "types"
var types = exports.types = {}

/**
 * The `void` type.
 *
 * @section types
 */

types.void = {
    size: 0
  , indirection: 1
  , get: function get (buf, offset) {
      debug('getting `void` type (returns `null`)')
      return null
    }
  , set: function set (buf, offset, val) {
      debug('setting `void` type (no-op)')
    }
}

/**
 * The `int8` type.
 */

types.int8 = {
    size: exports.sizeof.int8
  , indirection: 1
  , get: function get (buf, offset) {
      return buf.readInt8(offset || 0)
    }
  , set: function set (buf, offset, val) {
      if (typeof val === 'string') {
        val = val.charCodeAt(0)
      }
      return buf.writeInt8(val, offset || 0)
    }
}

/**
 * The `uint8` type.
 */

types.uint8 = {
    size: exports.sizeof.uint8
  , indirection: 1
  , get: function get (buf, offset) {
      return buf.readUInt8(offset || 0)
    }
  , set: function set (buf, offset, val) {
      if (typeof val === 'string') {
        val = val.charCodeAt(0)
      }
      return buf.writeUInt8(val, offset || 0)
    }
}

/**
 * The `int16` type.
 */

types.int16 = {
    size: exports.sizeof.int16
  , indirection: 1
  , get: function get (buf, offset) {
      return buf['readInt16' + exports.endianness](offset || 0)
    }
  , set: function set (buf, offset, val) {
      return buf['writeInt16' + exports.endianness](val, offset || 0)
    }
}

/**
 * The `uint16` type.
 */

types.uint16 = {
    size: exports.sizeof.uint16
  , indirection: 1
  , get: function get (buf, offset) {
      return buf['readUInt16' + exports.endianness](offset || 0)
    }
  , set: function set (buf, offset, val) {
      return buf['writeUInt16' + exports.endianness](val, offset || 0)
    }
}

/**
 * The `int32` type.
 */

types.int32 = {
    size: exports.sizeof.int32
  , indirection: 1
  , get: function get (buf, offset) {
      return buf['readInt32' + exports.endianness](offset || 0)
    }
  , set: function set (buf, offset, val) {
      return buf['writeInt32' + exports.endianness](val, offset || 0)
    }
}

/**
 * The `uint32` type.
 */

types.uint32 = {
    size: exports.sizeof.uint32
  , indirection: 1
  , get: function get (buf, offset) {
      return buf['readUInt32' + exports.endianness](offset || 0)
    }
  , set: function set (buf, offset, val) {
      return buf['writeUInt32' + exports.endianness](val, offset || 0)
    }
}

/**
 * The `int64` type.
 */

types.int64 = {
    size: exports.sizeof.int64
  , indirection: 1
  , get: function get (buf, offset) {
      return buf['readInt64' + exports.endianness](offset || 0)
    }
  , set: function set (buf, offset, val) {
      return buf['writeInt64' + exports.endianness](val, offset || 0)
    }
}

/**
 * The `uint64` type.
 */

types.uint64 = {
    size: exports.sizeof.uint64
  , indirection: 1
  , get: function get (buf, offset) {
      return buf['readUInt64' + exports.endianness](offset || 0)
    }
  , set: function set (buf, offset, val) {
      return buf['writeUInt64' + exports.endianness](val, offset || 0)
    }
}

/**
 * The `float` type.
 */

types.float = {
    size: exports.sizeof.float
  , indirection: 1
  , get: function get (buf, offset) {
      return buf['readFloat' + exports.endianness](offset || 0)
    }
  , set: function set (buf, offset, val) {
      return buf['writeFloat' + exports.endianness](val, offset || 0)
    }
}

/**
 * The `double` type.
 */

types.double = {
    size: exports.sizeof.double
  , indirection: 1
  , get: function get (buf, offset) {
      return buf['readDouble' + exports.endianness](offset || 0)
    }
  , set: function set (buf, offset, val) {
      return buf['writeDouble' + exports.endianness](val, offset || 0)
    }
}

/**
 * The `Object` type. This can be used to read/write regular JS Objects
 * into raw memory.
 */

types.Object = {
    size: exports.sizeof.Object
  , indirection: 1
  , get: function get (buf, offset) {
      return buf.readObject(offset || 0)
    }
  , set: function set (buf, offset, val) {
      return buf.writeObject(val, offset || 0)
    }
}

/**
 * The `CString` (a.k.a `"string"`) type.
 *
 * CStrings are a kind of weird thing. We say it's `sizeof(char *)`, and
 * `indirection` level of 1, which means that we have to return a Buffer that
 * is pointer sized, and points to a some utf8 string data, so we have to create
 * a 2nd "in-between" buffer.
 */

types.CString = {
    size: exports.sizeof.pointer
  , alignment: exports.alignof.pointer
  , indirection: 1
  , get: function get (buf, offset) {
      var _buf = exports.readPointer(buf, offset)
      if (exports.isNull(_buf)) {
        return null
      }
      return exports.readCString(_buf, 0)
    }
  , set: function set (buf, offset, val) {
      var _buf
      if (Buffer.isBuffer(val)) {
        _buf = val
      } else {
        // assume string
        _buf = exports.allocCString(val)
      }
      return exports.writePointer(buf, offset, _buf)
    }
}

// alias Utf8String
var utfstringwarned = false
Object.defineProperty(types, 'Utf8String', {
    enumerable: false
  , configurable: true
  , get: function () {
      if (!utfstringwarned) {
        utfstringwarned = true
        console.error('"Utf8String" type is deprecated, use "CString" instead')
      }
      return types.CString
    }
})

/**
 * The `bool` type.
 *
 * Wrapper type around `types.uint8` that accepts/returns `true` or
 * `false` Boolean JavaScript values.
 *
 * @name bool
 *
 */

/**
 * The `byte` type.
 *
 * @name byte
 */

/**
 * The `char` type.
 *
 * @name char
 */

/**
 * The `uchar` type.
 *
 * @name uchar
 */

/**
 * The `short` type.
 *
 * @name short
 */

/**
 * The `ushort` type.
 *
 * @name ushort
 */

/**
 * The `int` type.
 *
 * @name int
 */

/**
 * The `uint` type.
 *
 * @name uint
 */

/**
 * The `long` type.
 *
 * @name long
 */

/**
 * The `ulong` type.
 *
 * @name ulong
 */

/**
 * The `longlong` type.
 *
 * @name longlong
 */

/**
 * The `ulonglong` type.
 *
 * @name ulonglong
 */

/**
 * The `size_t` type.
 *
 * @name size_t
 */

// "typedef"s for the variable-sized types
;[ 'bool', 'byte', 'char', 'uchar', 'short', 'ushort', 'int', 'uint', 'long'
, 'ulong', 'longlong', 'ulonglong', 'size_t' ].forEach(function (name) {
  var unsigned = name === 'bool'
              || name === 'byte'
              || name === 'size_t'
              || name[0] === 'u'
  var size = exports.sizeof[name]
  assert(size >= 1 && size <= 8)
  var typeName = 'int' + (size * 8)
  if (unsigned) {
    typeName = 'u' + typeName
  }
  var type = exports.types[typeName]
  assert(type)
  exports.types[name] = Object.create(type)
})

// set the "alignment" property on the built-in types
Object.keys(exports.alignof).forEach(function (name) {
  if (name === 'pointer') return
  exports.types[name].alignment = exports.alignof[name]
  assert(exports.types[name].alignment > 0)
})

// make the `bool` type work with JS true/false values
exports.types.bool.get = (function (_get) {
  return function get (buf, offset) {
    return _get(buf, offset) ? true : false
  }
})(exports.types.bool.get)
exports.types.bool.set = (function (_set) {
  return function set (buf, offset, val) {
    if (typeof val !== 'number') {
      val = val ? 1 : 0
    }
    return _set(buf, offset, val)
  }
})(exports.types.bool.set)

/*!
 * Set the `name` property of the types. Used for debugging...
 */

Object.keys(exports.types).forEach(function (name) {
  exports.types[name].name = name
})

/*!
 * This `char *` type is used by "allocCString()" above.
 */

var charPtrType = exports.refType(exports.types.char)

/*!
 * Set the `type` property of the `NULL` pointer Buffer object.
 */

exports.NULL.type = exports.types.void

/**
 * `NULL_POINTER` is a pointer-sized `Buffer` instance pointing to `NULL`.
 * Conceptually, it's equivalent to the following C code:
 *
 * ``` c
 * char *null_pointer;
 * null_pointer = NULL;
 * ```
 *
 * @type Buffer
 */

exports.NULL_POINTER = exports.ref(exports.NULL)

/**
 * All these '...' comment blocks below are for the documentation generator.
 *
 * @section buffer
 */

Buffer.prototype.address = function address () {
  return exports.address(this, 0)
}

/**
 * ...
 */

Buffer.prototype.hexAddress = function hexAddress () {
  return exports.hexAddress(this, 0)
}

/**
 * ...
 */

Buffer.prototype.isNull = function isNull () {
  return exports.isNull(this, 0)
}

/**
 * ...
 */

Buffer.prototype.ref = function ref () {
  return exports.ref(this)
}

/**
 * ...
 */

Buffer.prototype.deref = function deref () {
  return exports.deref(this)
}

/**
 * ...
 */

Buffer.prototype.readObject = function readObject (offset) {
  return exports.readObject(this, offset)
}

/**
 * ...
 */

Buffer.prototype.writeObject = function writeObject (obj, offset) {
  return exports.writeObject(this, offset, obj)
}

/**
 * ...
 */

Buffer.prototype.readPointer = function readPointer (offset, size) {
  return exports.readPointer(this, offset, size)
}

/**
 * ...
 */

Buffer.prototype.writePointer = function writePointer (ptr, offset) {
  return exports.writePointer(this, offset, ptr)
}

/**
 * ...
 */

Buffer.prototype.readCString = function readCString (offset) {
  return exports.readCString(this, offset)
}

/**
 * ...
 */

Buffer.prototype.writeCString = function writeCString (string, offset, encoding) {
  return exports.writeCString(this, offset, string, encoding)
}

/**
 * ...
 */

Buffer.prototype.readInt64BE = function readInt64BE (offset) {
  return exports.readInt64BE(this, offset)
}

/**
 * ...
 */

Buffer.prototype.writeInt64BE = function writeInt64BE (val, offset) {
  return exports.writeInt64BE(this, offset, val)
}

/**
 * ...
 */

Buffer.prototype.readUInt64BE = function readUInt64BE (offset) {
  return exports.readUInt64BE(this, offset)
}

/**
 * ...
 */

Buffer.prototype.writeUInt64BE = function writeUInt64BE (val, offset) {
  return exports.writeUInt64BE(this, offset, val)
}

/**
 * ...
 */

Buffer.prototype.readInt64LE = function readInt64LE (offset) {
  return exports.readInt64LE(this, offset)
}

/**
 * ...
 */

Buffer.prototype.writeInt64LE = function writeInt64LE (val, offset) {
  return exports.writeInt64LE(this, offset, val)
}

/**
 * ...
 */

Buffer.prototype.readUInt64LE = function readUInt64LE (offset) {
  return exports.readUInt64LE(this, offset)
}

/**
 * ...
 */

Buffer.prototype.writeUInt64LE = function writeUInt64LE (val, offset) {
  return exports.writeUInt64LE(this, offset, val)
}

/**
 * ...
 */

Buffer.prototype.reinterpret = function reinterpret (size, offset) {
  return exports.reinterpret(this, size, offset)
}

/**
 * ...
 */

Buffer.prototype.reinterpretUntilZeros = function reinterpretUntilZeros (size, offset) {
  return exports.reinterpretUntilZeros(this, size, offset)
}

/**
 * `ref` overwrites the default `Buffer#inspect()` function to include the
 * hex-encoded memory address of the Buffer instance when invoked.
 *
 * This is simply a nice-to-have.
 *
 * **Before**:
 *
 * ``` js
 * console.log(new Buffer('ref'));
 * <Buffer 72 65 66>
 * ```
 *
 * **After**:
 *
 * ``` js
 * console.log(new Buffer('ref'));
 * <Buffer@0x103015490 72 65 66>
 * ```
 */

var inspectSym = inspect.custom || 'inspect'
Buffer.prototype[inspectSym] = overwriteInspect(Buffer.prototype[inspectSym])


// does SlowBuffer inherit from Buffer? (node >= v0.7.9)
if (!(exports.NULL instanceof Buffer)) {
  debug('extending SlowBuffer\'s prototype since it doesn\'t inherit from Buffer.prototype')

  /*!
   * SlowBuffer convenience methods.
   */

  var SlowBuffer = _dereq_('buffer').SlowBuffer

  SlowBuffer.prototype.address = Buffer.prototype.address
  SlowBuffer.prototype.hexAddress = Buffer.prototype.hexAddress
  SlowBuffer.prototype.isNull = Buffer.prototype.isNull
  SlowBuffer.prototype.ref = Buffer.prototype.ref
  SlowBuffer.prototype.deref = Buffer.prototype.deref
  SlowBuffer.prototype.readObject = Buffer.prototype.readObject
  SlowBuffer.prototype.writeObject = Buffer.prototype.writeObject
  SlowBuffer.prototype.readPointer = Buffer.prototype.readPointer
  SlowBuffer.prototype.writePointer = Buffer.prototype.writePointer
  SlowBuffer.prototype.readCString = Buffer.prototype.readCString
  SlowBuffer.prototype.writeCString = Buffer.prototype.writeCString
  SlowBuffer.prototype.reinterpret = Buffer.prototype.reinterpret
  SlowBuffer.prototype.reinterpretUntilZeros = Buffer.prototype.reinterpretUntilZeros
  SlowBuffer.prototype.readInt64BE = Buffer.prototype.readInt64BE
  SlowBuffer.prototype.writeInt64BE = Buffer.prototype.writeInt64BE
  SlowBuffer.prototype.readUInt64BE = Buffer.prototype.readUInt64BE
  SlowBuffer.prototype.writeUInt64BE = Buffer.prototype.writeUInt64BE
  SlowBuffer.prototype.readInt64LE = Buffer.prototype.readInt64LE
  SlowBuffer.prototype.writeInt64LE = Buffer.prototype.writeInt64LE
  SlowBuffer.prototype.readUInt64LE = Buffer.prototype.readUInt64LE
  SlowBuffer.prototype.writeUInt64LE = Buffer.prototype.writeUInt64LE
  SlowBuffer.prototype[inspectSym] = overwriteInspect(SlowBuffer.prototype[inspectSym])
}

function overwriteInspect (inspect) {
  if (inspect.name === 'refinspect') {
    return inspect
  } else {
    return function refinspect () {
      var v = inspect.apply(this, arguments)
      return v.replace('Buffer', 'Buffer@0x' + this.hexAddress())
    }
  }
}

}).call(this,_dereq_("buffer").Buffer)
},{"assert":2,"bindings":63,"buffer":8,"debug":66,"util":39}],87:[function(_dereq_,module,exports){
(function (process){
/**
 * Module dependencies.
 */

var console = _dereq_('console');

exports = module.exports = debug;


/**
 * Expose log function. Default to `console.log`.
 */

exports.log = console.log.bind(console);


/**
 * Enabled debuggers.
 */

var names = []
  , skips = []
  , config;

/**
 * Create a debugger with the given `name`.
 *
 * @param {String} name
 * @return {Type}
 * @api public
 */

function debug(name) {
  function disabled(){}
  disabled.enabled = false;

  var match = skips.some(function(re){
    return re.test(name);
  });

  if (match) return disabled;

  match = names.some(function(re){
    return re.test(name);
  });

  if (!match) return disabled;

  function logger(msg) {
    msg = (msg instanceof Error) ? (msg.stack || msg.message) : msg;
    Function.prototype.apply.call(logger.log, this, arguments);
  }

  logger.log = exports.log;
  logger.enabled = true;

  return logger;
}

exports.enable = function(name) {
  name = name.replace('*', '.*?');
  names.push(new RegExp('^' + name + '$'));
};

exports.enable.all = function() {
  exports.clear();
  exports.enable("*");
}

exports.disable = function(name) {
  name = name.replace('*', '.*?');
  skips.push(new RegExp('^' + name + '$'));
}

exports.disable.all = function() {
  exports.clear();
  exports.disable("*");
}

exports.clear = function() {
  skips = [];
  names = [];
}

exports.configure = function(val) {
  try {
    localStorage.debug = val;
  } catch(e){}

  config = val;
  exports.reset();
}

exports.reset = function() {
  exports.clear();

  config
    .split(/[\s,]+/)
    .forEach(function(name){
      if (name.length === 0 || name === '-') return;
      if (name[0] === '-') {
        exports.disable(name.substr(1));
      } else {
        exports.enable(name);
      }
    });
}

if (this.window) {
  if (!window.localStorage) return;
  exports.configure(localStorage.debug || '');
} else {
  exports.configure(process.env.DEBUG || '');
}

}).call(this,_dereq_('_process'))
},{"_process":19,"console":9}],88:[function(_dereq_,module,exports){
(function (global){
// Copyright (c) 2011, Chris Umbel

exports.Vector = _dereq_('./vector');
global.$V = exports.Vector.create;
exports.Matrix = _dereq_('./matrix');
global.$M = exports.Matrix.create;
exports.Line = _dereq_('./line');
global.$L = exports.Line.create;
exports.Plane = _dereq_('./plane');
global.$P = exports.Plane.create;
exports.Line.Segment = _dereq_('./line.segment');
exports.Sylvester = _dereq_('./sylvester');

}).call(this,typeof global !== "undefined" ? global : typeof self !== "undefined" ? self : typeof window !== "undefined" ? window : {})
},{"./line":89,"./line.segment":90,"./matrix":91,"./plane":92,"./sylvester":93,"./vector":94}],89:[function(_dereq_,module,exports){
// Copyright (c) 2011, Chris Umbel, James Coglan
var Vector = _dereq_('./vector');
var Matrix = _dereq_('./matrix');
var Plane = _dereq_('./plane');
var Sylvester = _dereq_('./sylvester');

// Line class - depends on Vector, and some methods require Matrix and Plane.

function Line() {}
Line.prototype = {

  // Returns true if the argument occupies the same space as the line
  eql: function(line) {
    return (this.isParallelTo(line) && this.contains(line.anchor));
  },

  // Returns a copy of the line
  dup: function() {
    return Line.create(this.anchor, this.direction);
  },

  // Returns the result of translating the line by the given vector/array
  translate: function(vector) {
    var V = vector.elements || vector;
    return Line.create([
      this.anchor.elements[0] + V[0],
      this.anchor.elements[1] + V[1],
      this.anchor.elements[2] + (V[2] || 0)
    ], this.direction);
  },

  // Returns true if the line is parallel to the argument. Here, 'parallel to'
  // means that the argument's direction is either parallel or antiparallel to
  // the line's own direction. A line is parallel to a plane if the two do not
  // have a unique intersection.
  isParallelTo: function(obj) {
    if (obj.normal || (obj.start && obj.end)) { return obj.isParallelTo(this); }
    var theta = this.direction.angleFrom(obj.direction);
    return (Math.abs(theta) <= Sylvester.precision || Math.abs(theta - Math.PI) <= Sylvester.precision);
  },

  // Returns the line's perpendicular distance from the argument,
  // which can be a point, a line or a plane
  distanceFrom: function(obj) {
    if (obj.normal || (obj.start && obj.end)) { return obj.distanceFrom(this); }
    if (obj.direction) {
      // obj is a line
      if (this.isParallelTo(obj)) { return this.distanceFrom(obj.anchor); }
      var N = this.direction.cross(obj.direction).toUnitVector().elements;
      var A = this.anchor.elements, B = obj.anchor.elements;
      return Math.abs((A[0] - B[0]) * N[0] + (A[1] - B[1]) * N[1] + (A[2] - B[2]) * N[2]);
    } else {
      // obj is a point
      var P = obj.elements || obj;
      var A = this.anchor.elements, D = this.direction.elements;
      var PA1 = P[0] - A[0], PA2 = P[1] - A[1], PA3 = (P[2] || 0) - A[2];
      var modPA = Math.sqrt(PA1*PA1 + PA2*PA2 + PA3*PA3);
      if (modPA === 0) return 0;
      // Assumes direction vector is normalized
      var cosTheta = (PA1 * D[0] + PA2 * D[1] + PA3 * D[2]) / modPA;
      var sin2 = 1 - cosTheta*cosTheta;
      return Math.abs(modPA * Math.sqrt(sin2 < 0 ? 0 : sin2));
    }
  },

  // Returns true iff the argument is a point on the line, or if the argument
  // is a line segment lying within the receiver
  contains: function(obj) {
    if (obj.start && obj.end) { return this.contains(obj.start) && this.contains(obj.end); }
    var dist = this.distanceFrom(obj);
    return (dist !== null && dist <= Sylvester.precision);
  },

  // Returns the distance from the anchor of the given point. Negative values are
  // returned for points that are in the opposite direction to the line's direction from
  // the line's anchor point.
  positionOf: function(point) {
    if (!this.contains(point)) { return null; }
    var P = point.elements || point;
    var A = this.anchor.elements, D = this.direction.elements;
    return (P[0] - A[0]) * D[0] + (P[1] - A[1]) * D[1] + ((P[2] || 0) - A[2]) * D[2];
  },

  // Returns true iff the line lies in the given plane
  liesIn: function(plane) {
    return plane.contains(this);
  },

  // Returns true iff the line has a unique point of intersection with the argument
  intersects: function(obj) {
    if (obj.normal) { return obj.intersects(this); }
    return (!this.isParallelTo(obj) && this.distanceFrom(obj) <= Sylvester.precision);
  },

  // Returns the unique intersection point with the argument, if one exists
  intersectionWith: function(obj) {
    if (obj.normal || (obj.start && obj.end)) { return obj.intersectionWith(this); }
    if (!this.intersects(obj)) { return null; }
    var P = this.anchor.elements, X = this.direction.elements,
        Q = obj.anchor.elements, Y = obj.direction.elements;
    var X1 = X[0], X2 = X[1], X3 = X[2], Y1 = Y[0], Y2 = Y[1], Y3 = Y[2];
    var PsubQ1 = P[0] - Q[0], PsubQ2 = P[1] - Q[1], PsubQ3 = P[2] - Q[2];
    var XdotQsubP = - X1*PsubQ1 - X2*PsubQ2 - X3*PsubQ3;
    var YdotPsubQ = Y1*PsubQ1 + Y2*PsubQ2 + Y3*PsubQ3;
    var XdotX = X1*X1 + X2*X2 + X3*X3;
    var YdotY = Y1*Y1 + Y2*Y2 + Y3*Y3;
    var XdotY = X1*Y1 + X2*Y2 + X3*Y3;
    var k = (XdotQsubP * YdotY / XdotX + XdotY * YdotPsubQ) / (YdotY - XdotY * XdotY);
    return Vector.create([P[0] + k*X1, P[1] + k*X2, P[2] + k*X3]);
  },

  // Returns the point on the line that is closest to the given point or line/line segment
  pointClosestTo: function(obj) {
    if (obj.start && obj.end) {
      // obj is a line segment
      var P = obj.pointClosestTo(this);
      return (P === null) ? null : this.pointClosestTo(P);
    } else if (obj.direction) {
      // obj is a line
      if (this.intersects(obj)) { return this.intersectionWith(obj); }
      if (this.isParallelTo(obj)) { return null; }
      var D = this.direction.elements, E = obj.direction.elements;
      var D1 = D[0], D2 = D[1], D3 = D[2], E1 = E[0], E2 = E[1], E3 = E[2];
      // Create plane containing obj and the shared normal and intersect this with it
      // Thank you: http://www.cgafaq.info/wiki/Line-line_distance
      var x = (D3 * E1 - D1 * E3), y = (D1 * E2 - D2 * E1), z = (D2 * E3 - D3 * E2);
      var N = [x * E3 - y * E2, y * E1 - z * E3, z * E2 - x * E1];
      var P = Plane.create(obj.anchor, N);
      return P.intersectionWith(this);
    } else {
      // obj is a point
      var P = obj.elements || obj;
      if (this.contains(P)) { return Vector.create(P); }
      var A = this.anchor.elements, D = this.direction.elements;
      var D1 = D[0], D2 = D[1], D3 = D[2], A1 = A[0], A2 = A[1], A3 = A[2];
      var x = D1 * (P[1]-A2) - D2 * (P[0]-A1), y = D2 * ((P[2] || 0) - A3) - D3 * (P[1]-A2),
          z = D3 * (P[0]-A1) - D1 * ((P[2] || 0) - A3);
      var V = Vector.create([D2 * x - D3 * z, D3 * y - D1 * x, D1 * z - D2 * y]);
      var k = this.distanceFrom(P) / V.modulus();
      return Vector.create([
        P[0] + V.elements[0] * k,
        P[1] + V.elements[1] * k,
        (P[2] || 0) + V.elements[2] * k
      ]);
    }
  },

  // Returns a copy of the line rotated by t radians about the given line. Works by
  // finding the argument's closest point to this line's anchor point (call this C) and
  // rotating the anchor about C. Also rotates the line's direction about the argument's.
  // Be careful with this - the rotation axis' direction affects the outcome!
  rotate: function(t, line) {
    // If we're working in 2D
    if (typeof(line.direction) == 'undefined') { line = Line.create(line.to3D(), Vector.k); }
    var R = Matrix.Rotation(t, line.direction).elements;
    var C = line.pointClosestTo(this.anchor).elements;
    var A = this.anchor.elements, D = this.direction.elements;
    var C1 = C[0], C2 = C[1], C3 = C[2], A1 = A[0], A2 = A[1], A3 = A[2];
    var x = A1 - C1, y = A2 - C2, z = A3 - C3;
    return Line.create([
      C1 + R[0][0] * x + R[0][1] * y + R[0][2] * z,
      C2 + R[1][0] * x + R[1][1] * y + R[1][2] * z,
      C3 + R[2][0] * x + R[2][1] * y + R[2][2] * z
    ], [
      R[0][0] * D[0] + R[0][1] * D[1] + R[0][2] * D[2],
      R[1][0] * D[0] + R[1][1] * D[1] + R[1][2] * D[2],
      R[2][0] * D[0] + R[2][1] * D[1] + R[2][2] * D[2]
    ]);
  },

  // Returns a copy of the line with its direction vector reversed.
  // Useful when using lines for rotations.
  reverse: function() {
    return Line.create(this.anchor, this.direction.x(-1));
  },

  // Returns the line's reflection in the given point or line
  reflectionIn: function(obj) {
    if (obj.normal) {
      // obj is a plane
      var A = this.anchor.elements, D = this.direction.elements;
      var A1 = A[0], A2 = A[1], A3 = A[2], D1 = D[0], D2 = D[1], D3 = D[2];
      var newA = this.anchor.reflectionIn(obj).elements;
      // Add the line's direction vector to its anchor, then mirror that in the plane
      var AD1 = A1 + D1, AD2 = A2 + D2, AD3 = A3 + D3;
      var Q = obj.pointClosestTo([AD1, AD2, AD3]).elements;
      var newD = [Q[0] + (Q[0] - AD1) - newA[0], Q[1] + (Q[1] - AD2) - newA[1], Q[2] + (Q[2] - AD3) - newA[2]];
      return Line.create(newA, newD);
    } else if (obj.direction) {
      // obj is a line - reflection obtained by rotating PI radians about obj
      return this.rotate(Math.PI, obj);
    } else {
      // obj is a point - just reflect the line's anchor in it
      var P = obj.elements || obj;
      return Line.create(this.anchor.reflectionIn([P[0], P[1], (P[2] || 0)]), this.direction);
    }
  },

  // Set the line's anchor point and direction.
  setVectors: function(anchor, direction) {
    // Need to do this so that line's properties are not
    // references to the arguments passed in
    anchor = Vector.create(anchor);
    direction = Vector.create(direction);
    if (anchor.elements.length == 2) {anchor.elements.push(0); }
    if (direction.elements.length == 2) { direction.elements.push(0); }
    if (anchor.elements.length > 3 || direction.elements.length > 3) { return null; }
    var mod = direction.modulus();
    if (mod === 0) { return null; }
    this.anchor = anchor;
    this.direction = Vector.create([
      direction.elements[0] / mod,
      direction.elements[1] / mod,
      direction.elements[2] / mod
    ]);
    return this;
  }
};

// Constructor function
Line.create = function(anchor, direction) {
  var L = new Line();
  return L.setVectors(anchor, direction);
};

// Axes
Line.X = Line.create(Vector.Zero(3), Vector.i);
Line.Y = Line.create(Vector.Zero(3), Vector.j);
Line.Z = Line.create(Vector.Zero(3), Vector.k);

module.exports = Line;

},{"./matrix":91,"./plane":92,"./sylvester":93,"./vector":94}],90:[function(_dereq_,module,exports){
// Copyright (c) 2011, Chris Umbel, James Coglan
// Line.Segment class - depends on Line and its dependencies.

var Line = _dereq_('./line');
var Vector = _dereq_('./vector');

Line.Segment = function() {};
Line.Segment.prototype = {

  // Returns true iff the line segment is equal to the argument
  eql: function(segment) {
    return (this.start.eql(segment.start) && this.end.eql(segment.end)) ||
        (this.start.eql(segment.end) && this.end.eql(segment.start));
  },

  // Returns a copy of the line segment
  dup: function() {
    return Line.Segment.create(this.start, this.end);
  },

  // Returns the length of the line segment
  length: function() {
    var A = this.start.elements, B = this.end.elements;
    var C1 = B[0] - A[0], C2 = B[1] - A[1], C3 = B[2] - A[2];
    return Math.sqrt(C1*C1 + C2*C2 + C3*C3);
  },

  // Returns the line segment as a vector equal to its
  // end point relative to its endpoint
  toVector: function() {
    var A = this.start.elements, B = this.end.elements;
    return Vector.create([B[0] - A[0], B[1] - A[1], B[2] - A[2]]);
  },

  // Returns the segment's midpoint as a vector
  midpoint: function() {
    var A = this.start.elements, B = this.end.elements;
    return Vector.create([(B[0] + A[0])/2, (B[1] + A[1])/2, (B[2] + A[2])/2]);
  },

  // Returns the plane that bisects the segment
  bisectingPlane: function() {
    return Plane.create(this.midpoint(), this.toVector());
  },

  // Returns the result of translating the line by the given vector/array
  translate: function(vector) {
    var V = vector.elements || vector;
    var S = this.start.elements, E = this.end.elements;
    return Line.Segment.create(
      [S[0] + V[0], S[1] + V[1], S[2] + (V[2] || 0)],
      [E[0] + V[0], E[1] + V[1], E[2] + (V[2] || 0)]
    );
  },

  // Returns true iff the line segment is parallel to the argument. It simply forwards
  // the method call onto its line property.
  isParallelTo: function(obj) {
    return this.line.isParallelTo(obj);
  },

  // Returns the distance between the argument and the line segment's closest point to the argument
  distanceFrom: function(obj) {
    var P = this.pointClosestTo(obj);
    return (P === null) ? null : P.distanceFrom(obj);
  },

  // Returns true iff the given point lies on the segment
  contains: function(obj) {
    if (obj.start && obj.end) { return this.contains(obj.start) && this.contains(obj.end); }
    var P = (obj.elements || obj).slice();
    if (P.length == 2) { P.push(0); }
    if (this.start.eql(P)) { return true; }
    var S = this.start.elements;
    var V = Vector.create([S[0] - P[0], S[1] - P[1], S[2] - (P[2] || 0)]);
    var vect = this.toVector();
    return V.isAntiparallelTo(vect) && V.modulus() <= vect.modulus();
  },

  // Returns true iff the line segment intersects the argument
  intersects: function(obj) {
    return (this.intersectionWith(obj) !== null);
  },

  // Returns the unique point of intersection with the argument
  intersectionWith: function(obj) {
    if (!this.line.intersects(obj)) { return null; }
    var P = this.line.intersectionWith(obj);
    return (this.contains(P) ? P : null);
  },

  // Returns the point on the line segment closest to the given object
  pointClosestTo: function(obj) {
    if (obj.normal) {
      // obj is a plane
      var V = this.line.intersectionWith(obj);
      if (V === null) { return null; }
      return this.pointClosestTo(V);
    } else {
      // obj is a line (segment) or point
      var P = this.line.pointClosestTo(obj);
      if (P === null) { return null; }
      if (this.contains(P)) { return P; }
      return (this.line.positionOf(P) < 0 ? this.start : this.end).dup();
    }
  },

  // Set the start and end-points of the segment
  setPoints: function(startPoint, endPoint) {
    startPoint = Vector.create(startPoint).to3D();
    endPoint = Vector.create(endPoint).to3D();
    if (startPoint === null || endPoint === null) { return null; }
    this.line = Line.create(startPoint, endPoint.subtract(startPoint));
    this.start = startPoint;
    this.end = endPoint;
    return this;
  }
};

// Constructor function
Line.Segment.create = function(v1, v2) {
  var S = new Line.Segment();
  return S.setPoints(v1, v2);
};

module.exports = Line.Segment;

},{"./line":89,"./vector":94}],91:[function(_dereq_,module,exports){
// Copyright (c) 2011, Chris Umbel, James Coglan
// Matrix class - depends on Vector.

var fs = _dereq_('fs');
var Sylvester = _dereq_('./sylvester');
var Vector = _dereq_('./vector');

// augment a matrix M with identity rows/cols
function identSize(M, m, n, k) {
    var e = M.elements;
    var i = k - 1;

    while(i--) {
	var row = [];
	
	for(var j = 0; j < n; j++)
	    row.push(j == i ? 1 : 0);
	
        e.unshift(row);
    }
    
    for(var i = k - 1; i < m; i++) {
        while(e[i].length < n)
            e[i].unshift(0);
    }

    return $M(e);
}

function pca(X) {
    var Sigma = X.transpose().x(X).x(1 / X.rows());
    var svd = Sigma.svd();
    return {U: svd.U, S: svd.S};
}

// singular value decomposition in pure javascript
function svdJs() {
    var A = this;
    var V = Matrix.I(A.rows());
    var S = A.transpose();
    var U = Matrix.I(A.cols());
    var err = Number.MAX_VALUE;
    var i = 0;
    var maxLoop = 100;

    while(err > 2.2737e-13 && i < maxLoop) {
        var qr = S.transpose().qrJs();
        S = qr.R;
        V = V.x(qr.Q);
        qr = S.transpose().qrJs();
        U = U.x(qr.Q);
        S = qr.R;

        var e = S.triu(1).unroll().norm();
        var f = S.diagonal().norm();

        if(f == 0)
            f = 1;

        err = e / f;

        i++;
    }

    var ss = S.diagonal();
    var s = [];

    for(var i = 1; i <= ss.cols(); i++) {
        var ssn = ss.e(i);
        s.push(Math.abs(ssn));

        if(ssn < 0) {
            for(var j = 0; j < U.rows(); j++) {
                V.elements[j][i - 1] = -(V.elements[j][i - 1]);
            }
        }
    }

    return {U: U, S: $V(s).toDiagonalMatrix(), V: V};
}

// singular value decomposition using LAPACK
function svdPack() {
    var result = lapack.sgesvd('A', 'A', this.elements);

    return {
        U: $M(result.U),
        S: $M(result.S).column(1).toDiagonalMatrix(),
	V: $M(result.VT).transpose()
    };
}

// QR decomposition in pure javascript
function qrJs() {
    var m = this.rows();
    var n = this.cols();
    var Q = Matrix.I(m);
    var A = this;
    
    for(var k = 1; k < Math.min(m, n); k++) {
	var ak = A.slice(k, 0, k, k).col(1);
	var oneZero = [1];
	
	while(oneZero.length <=  m - k)
	    oneZero.push(0);
	
	oneZero = $V(oneZero);
	var vk = ak.add(oneZero.x(ak.norm() * Math.sign(ak.e(1))));
	var Vk = $M(vk);
	var Hk = Matrix.I(m - k + 1).subtract(Vk.x(2).x(Vk.transpose()).div(Vk.transpose().x(Vk).e(1, 1)));
	var Qk = identSize(Hk, m, n, k);
	A = Qk.x(A);
	// slow way to compute Q
	Q = Q.x(Qk);
    }
    
    return {Q: Q, R: A};
}

// QR decomposition using LAPACK
function qrPack() {
    var qr = lapack.qr(this.elements);

    return {
	Q: $M(qr.Q),
	R: $M(qr.R)
    };
}

function Matrix() {}
Matrix.prototype = {
    // solve a system of linear equations (work in progress)
    solve: function(b) {
	var lu = this.lu();
	b = lu.P.x(b);
	var y = lu.L.forwardSubstitute(b);
	var x = lu.U.backSubstitute(y);
	return lu.P.x(x);
	//return this.inv().x(b);
    },

    // project a matrix onto a lower dim
    pcaProject: function(k, U) {
	var U = U || pca(this).U;
	var Ureduce= U.slice(1, U.rows(), 1, k);
	return {Z: this.x(Ureduce), U: U};
    },

    // recover a matrix to a higher dimension
    pcaRecover: function(U) {
	var k = this.cols();
	var Ureduce = U.slice(1, U.rows(), 1, k);
	return this.x(Ureduce.transpose());
    },    

    // grab the upper triangular part of the matrix
    triu: function(k) {
	if(!k)
	    k = 0;
	
	return this.map(function(x, i, j) {
	    return j - i >= k ? x : 0;
	});
    },

    // unroll a matrix into a vector
    unroll: function() {
	var v = [];
	
	for(var i = 1; i <= this.cols(); i++) {
	    for(var j = 1; j <= this.rows(); j++) {
		v.push(this.e(j, i));
	    }
	}

	return $V(v);
    },

    // return a sub-block of the matrix
    slice: function(startRow, endRow, startCol, endCol) {
	var x = [];
	
	if(endRow == 0)
	    endRow = this.rows();
	
	if(endCol == 0)
	    endCol = this.cols();

	for(i = startRow; i <= endRow; i++) {
	    var row = [];

	    for(j = startCol; j <= endCol; j++) {
		row.push(this.e(i, j));
	    }

	    x.push(row);
	}

	return $M(x);
    },

    // Returns element (i,j) of the matrix
    e: function(i,j) {
	if (i < 1 || i > this.elements.length || j < 1 || j > this.elements[0].length) { return null; }
	return this.elements[i - 1][j - 1];
    },

    // Returns row k of the matrix as a vector
    row: function(i) {
	if (i > this.elements.length) { return null; }
	return $V(this.elements[i - 1]);
    },

    // Returns column k of the matrix as a vector
    col: function(j) {
	if (j > this.elements[0].length) { return null; }
	var col = [], n = this.elements.length;
	for (var i = 0; i < n; i++) { col.push(this.elements[i][j - 1]); }
	return $V(col);
    },

    // Returns the number of rows/columns the matrix has
    dimensions: function() {
	return {rows: this.elements.length, cols: this.elements[0].length};
    },

    // Returns the number of rows in the matrix
    rows: function() {
	return this.elements.length;
    },

    // Returns the number of columns in the matrix
    cols: function() {
	return this.elements[0].length;
    },

    approxEql: function(matrix) {
	return this.eql(matrix, Sylvester.approxPrecision);
    },

    // Returns true iff the matrix is equal to the argument. You can supply
    // a vector as the argument, in which case the receiver must be a
    // one-column matrix equal to the vector.
    eql: function(matrix, precision) {
	var M = matrix.elements || matrix;
	if (typeof(M[0][0]) == 'undefined') { M = Matrix.create(M).elements; }
	if (this.elements.length != M.length ||
            this.elements[0].length != M[0].length) { return false; }
	var i = this.elements.length, nj = this.elements[0].length, j;
	while (i--) { j = nj;
		      while (j--) {
			  if (Math.abs(this.elements[i][j] - M[i][j]) > (precision || Sylvester.precision)) { return false; }
		      }
		    }
	return true;
    },

    // Returns a copy of the matrix
    dup: function() {
	return Matrix.create(this.elements);
    },

    // Maps the matrix to another matrix (of the same dimensions) according to the given function
    map: function(fn) {
    var els = [], i = this.elements.length, nj = this.elements[0].length, j;
	while (i--) { j = nj;
		      els[i] = [];
		      while (j--) {
			  els[i][j] = fn(this.elements[i][j], i + 1, j + 1);
		      }
		    }
	return Matrix.create(els);
    },

    // Returns true iff the argument has the same dimensions as the matrix
    isSameSizeAs: function(matrix) {
	var M = matrix.elements || matrix;
	if (typeof(M[0][0]) == 'undefined') { M = Matrix.create(M).elements; }
	return (this.elements.length == M.length &&
		this.elements[0].length == M[0].length);
    },

    // Returns the result of adding the argument to the matrix
    add: function(matrix) {
	if(typeof(matrix) == 'number') {
	    return this.map(function(x, i, j) { return x + matrix});
	} else {
	    var M = matrix.elements || matrix;
	    if (typeof(M[0][0]) == 'undefined') { M = Matrix.create(M).elements; }
	    if (!this.isSameSizeAs(M)) { return null; }
	    return this.map(function(x, i, j) { return x + M[i - 1][j - 1]; });
	}
    },

    // Returns the result of subtracting the argument from the matrix
    subtract: function(matrix) {
	if(typeof(matrix) == 'number') {
	    return this.map(function(x, i, j) { return x - matrix});
	} else {
	    var M = matrix.elements || matrix;
	    if (typeof(M[0][0]) == 'undefined') { M = Matrix.create(M).elements; }
	    if (!this.isSameSizeAs(M)) { return null; }
	    return this.map(function(x, i, j) { return x - M[i - 1][j - 1]; });
	}
    },

    // Returns true iff the matrix can multiply the argument from the left
    canMultiplyFromLeft: function(matrix) {
	var M = matrix.elements || matrix;
	if (typeof(M[0][0]) == 'undefined') { M = Matrix.create(M).elements; }
	// this.columns should equal matrix.rows
	return (this.elements[0].length == M.length);
    },

    // Returns the result of a multiplication-style operation the matrix from the right by the argument.
    // If the argument is a scalar then just operate on all the elements. If the argument is
    // a vector, a vector is returned, which saves you having to remember calling
    // col(1) on the result.
    mulOp: function(matrix, op) {
	if (!matrix.elements) {
	    return this.map(function(x) { return op(x, matrix); });
	}

	var returnVector = matrix.modulus ? true : false;
	var M = matrix.elements || matrix;
	if (typeof(M[0][0]) == 'undefined') 
	    M = Matrix.create(M).elements;
	if (!this.canMultiplyFromLeft(M)) 
	    return null; 
	var e = this.elements, rowThis, rowElem, elements = [],
        sum, m = e.length, n = M[0].length, o = e[0].length, i = m, j, k;

	while (i--) {
            rowElem = [];
            rowThis = e[i];
            j = n;

            while (j--) {
		sum = 0;
		k = o;

		while (k--) {
                    sum += op(rowThis[k], M[k][j]);
		}

		rowElem[j] = sum;
            }

            elements[i] = rowElem;
	}

	var M = Matrix.create(elements);
	return returnVector ? M.col(1) : M;
    },

    // Returns the result of dividing the matrix from the right by the argument.
    // If the argument is a scalar then just divide all the elements. If the argument is
    // a vector, a vector is returned, which saves you having to remember calling
    // col(1) on the result.
    div: function(matrix) {
	return this.mulOp(matrix, function(x, y) { return x / y});
    },

    // Returns the result of multiplying the matrix from the right by the argument.
    // If the argument is a scalar then just multiply all the elements. If the argument is
    // a vector, a vector is returned, which saves you having to remember calling
    // col(1) on the result.
    multiply: function(matrix) {
	return this.mulOp(matrix, function(x, y) { return x * y});
    },

    x: function(matrix) { return this.multiply(matrix); },

    elementMultiply: function(v) {
        return this.map(function(k, i, j) {
            return v.e(i, j) * k;
        });
    },

    // sum all elements in the matrix
    sum: function() {
        var sum = 0;

        this.map(function(x) { sum += x;});

        return sum;
    },

    // Returns a Vector of each colum averaged.
    mean: function() {
      var dim = this.dimensions();
      var r = [];
      for (var i = 1; i <= dim.cols; i++) {
        r.push(this.col(i).sum() / dim.rows);
      }
      return $V(r);
    },

    column: function(n) {
	return this.col(n);
    },

    // element-wise log
    log: function() {
	return this.map(function(x) { return Math.log(x); });
    },

    // Returns a submatrix taken from the matrix
    // Argument order is: start row, start col, nrows, ncols
    // Element selection wraps if the required index is outside the matrix's bounds, so you could
    // use this to perform row/column cycling or copy-augmenting.
    minor: function(a, b, c, d) {
	var elements = [], ni = c, i, nj, j;
	var rows = this.elements.length, cols = this.elements[0].length;
	while (ni--) {
	    i = c - ni - 1;
	    elements[i] = [];
	    nj = d;
	    while (nj--) {
		j = d - nj - 1;
		elements[i][j] = this.elements[(a + i - 1) % rows][(b + j - 1) % cols];
	    }
	}
	return Matrix.create(elements);
    },

    // Returns the transpose of the matrix
    transpose: function() {
    var rows = this.elements.length, i, cols = this.elements[0].length, j;
	var elements = [], i = cols;
	while (i--) {
	    j = rows;
	    elements[i] = [];
	    while (j--) {
		elements[i][j] = this.elements[j][i];
	    }
	}
	return Matrix.create(elements);
    },

    // Returns true iff the matrix is square
    isSquare: function() {
	return (this.elements.length == this.elements[0].length);
    },

    // Returns the (absolute) largest element of the matrix
    max: function() {
	var m = 0, i = this.elements.length, nj = this.elements[0].length, j;
	while (i--) {
	    j = nj;
	    while (j--) {
		if (Math.abs(this.elements[i][j]) > Math.abs(m)) { m = this.elements[i][j]; }
	    }
	}
	return m;
    },

    // Returns the indeces of the first match found by reading row-by-row from left to right
    indexOf: function(x) {
	var index = null, ni = this.elements.length, i, nj = this.elements[0].length, j;
	for (i = 0; i < ni; i++) {
	    for (j = 0; j < nj; j++) {
		if (this.elements[i][j] == x) { return {i: i + 1, j: j + 1}; }
	    }
	}
	return null;
    },

    // If the matrix is square, returns the diagonal elements as a vector.
    // Otherwise, returns null.
    diagonal: function() {
	if (!this.isSquare) { return null; }
	var els = [], n = this.elements.length;
	for (var i = 0; i < n; i++) {
	    els.push(this.elements[i][i]);
	}
	return $V(els);
    },

    // Make the matrix upper (right) triangular by Gaussian elimination.
    // This method only adds multiples of rows to other rows. No rows are
    // scaled up or switched, and the determinant is preserved.
    toRightTriangular: function() {
	var M = this.dup(), els;
	var n = this.elements.length, i, j, np = this.elements[0].length, p;
	for (i = 0; i < n; i++) {
	    if (M.elements[i][i] == 0) {
		for (j = i + 1; j < n; j++) {
		    if (M.elements[j][i] != 0) {
			els = [];
			for (p = 0; p < np; p++) { els.push(M.elements[i][p] + M.elements[j][p]); }
			M.elements[i] = els;
			break;
		    }
		}
	    }
	    if (M.elements[i][i] != 0) {
		for (j = i + 1; j < n; j++) {
		    var multiplier = M.elements[j][i] / M.elements[i][i];
		    els = [];
		    for (p = 0; p < np; p++) {
			// Elements with column numbers up to an including the number
			// of the row that we're subtracting can safely be set straight to
			// zero, since that's the point of this routine and it avoids having
			// to loop over and correct rounding errors later
			els.push(p <= i ? 0 : M.elements[j][p] - M.elements[i][p] * multiplier);
		    }
		    M.elements[j] = els;
		}
	    }
	}
	return M;
    },

    toUpperTriangular: function() { return this.toRightTriangular(); },

    // Returns the determinant for square matrices
    determinant: function() {
	if (!this.isSquare()) { return null; }
	if (this.cols == 1 && this.rows == 1) { return this.row(1); }
	if (this.cols == 0 && this.rows == 0) { return 1; }
	var M = this.toRightTriangular();
	var det = M.elements[0][0], n = M.elements.length;
	for (var i = 1; i < n; i++) {
	    det = det * M.elements[i][i];
	}
	return det;
    },
    det: function() { return this.determinant(); },

    // Returns true iff the matrix is singular
    isSingular: function() {
	return (this.isSquare() && this.determinant() === 0);
    },

    // Returns the trace for square matrices
    trace: function() {
	if (!this.isSquare()) { return null; }
	var tr = this.elements[0][0], n = this.elements.length;
	for (var i = 1; i < n; i++) {
	    tr += this.elements[i][i];
	}
	return tr;
    },

    tr: function() { return this.trace(); },

    // Returns the rank of the matrix
    rank: function() {
	var M = this.toRightTriangular(), rank = 0;
	var i = this.elements.length, nj = this.elements[0].length, j;
	while (i--) {
	    j = nj;
	    while (j--) {
		if (Math.abs(M.elements[i][j]) > Sylvester.precision) { rank++; break; }
	    }
	}
	return rank;
    },

    rk: function() { return this.rank(); },

    // Returns the result of attaching the given argument to the right-hand side of the matrix
    augment: function(matrix) {
	var M = matrix.elements || matrix;
	if (typeof(M[0][0]) == 'undefined') { M = Matrix.create(M).elements; }
	var T = this.dup(), cols = T.elements[0].length;
	var i = T.elements.length, nj = M[0].length, j;
	if (i != M.length) { return null; }
	while (i--) {
	    j = nj;
	    while (j--) {
		T.elements[i][cols + j] = M[i][j];
	    }
	}
	return T;
    },

    // Returns the inverse (if one exists) using Gauss-Jordan
    inverse: function() {
	if (!this.isSquare() || this.isSingular()) { return null; }
	var n = this.elements.length, i = n, j;
	var M = this.augment(Matrix.I(n)).toRightTriangular();
	var np = M.elements[0].length, p, els, divisor;
	var inverse_elements = [], new_element;
	// Matrix is non-singular so there will be no zeros on the diagonal
	// Cycle through rows from last to first
	while (i--) {
	    // First, normalise diagonal elements to 1
	    els = [];
	    inverse_elements[i] = [];
	    divisor = M.elements[i][i];
	    for (p = 0; p < np; p++) {
        new_element = M.elements[i][p] / divisor;
		els.push(new_element);
		// Shuffle off the current row of the right hand side into the results
		// array as it will not be modified by later runs through this loop
		if (p >= n) { inverse_elements[i].push(new_element); }
	    }
	    M.elements[i] = els;
	    // Then, subtract this row from those above it to
	    // give the identity matrix on the left hand side
	    j = i;
	    while (j--) {
		els = [];
		for (p = 0; p < np; p++) {
		    els.push(M.elements[j][p] - M.elements[i][p] * M.elements[j][i]);
		}
		M.elements[j] = els;
	    }
	}
	return Matrix.create(inverse_elements);
    },

    inv: function() { return this.inverse(); },

    // Returns the result of rounding all the elements
    round: function() {
	return this.map(function(x) { return Math.round(x); });
    },

    // Returns a copy of the matrix with elements set to the given value if they
    // differ from it by less than Sylvester.precision
    snapTo: function(x) {
	return this.map(function(p) {
	    return (Math.abs(p - x) <= Sylvester.precision) ? x : p;
	});
    },

    // Returns a string representation of the matrix
    inspect: function() {
	var matrix_rows = [];
	var n = this.elements.length;
	for (var i = 0; i < n; i++) {
	    matrix_rows.push($V(this.elements[i]).inspect());
	}
	return matrix_rows.join('\n');
    },

    // Returns a array representation of the matrix
    toArray: function() {
    	var matrix_rows = [];
    	var n = this.elements.length;
    	for (var i = 0; i < n; i++) {
        matrix_rows.push(this.elements[i]);
    	}
      return matrix_rows;
    },


    // Set the matrix's elements from an array. If the argument passed
    // is a vector, the resulting matrix will be a single column.
    setElements: function(els) {
	var i, j, elements = els.elements || els;
	if (typeof(elements[0][0]) != 'undefined') {
	    i = elements.length;
	    this.elements = [];
	    while (i--) {
		j = elements[i].length;
		this.elements[i] = [];
		while (j--) {
		    this.elements[i][j] = elements[i][j];
		}
	    }
	    return this;
	}
	var n = elements.length;
	this.elements = [];
	for (i = 0; i < n; i++) {
	    this.elements.push([elements[i]]);
	}
	return this;
    },

    // return the indexes of the columns with the largest value
    // for each row
    maxColumnIndexes: function() {
	var maxes = [];

	for(var i = 1; i <= this.rows(); i++) {
	    var max = null;
	    var maxIndex = -1;

	    for(var j = 1; j <= this.cols(); j++) {
		if(max === null || this.e(i, j) > max) {
		    max = this.e(i, j);
		    maxIndex = j;
		}
	    }

	    maxes.push(maxIndex);
	}

	return $V(maxes);
    },

    // return the largest values in each row
    maxColumns: function() {
	var maxes = [];

	for(var i = 1; i <= this.rows(); i++) {
	    var max = null;

	    for(var j = 1; j <= this.cols(); j++) {
		if(max === null || this.e(i, j) > max) {
		    max = this.e(i, j);
		}
	    }

	    maxes.push(max);
	}

	return $V(maxes);
    },

    // return the indexes of the columns with the smallest values
    // for each row
    minColumnIndexes: function() {
	var mins = [];

	for(var i = 1; i <= this.rows(); i++) {
	    var min = null;
	    var minIndex = -1;

	    for(var j = 1; j <= this.cols(); j++) {
		if(min === null || this.e(i, j) < min) {
		    min = this.e(i, j);
		    minIndex = j;
		}
	    }

	    mins.push(minIndex);
	}

	return $V(mins);
    },

    // return the smallest values in each row
    minColumns: function() {
	var mins = [];

	for(var i = 1; i <= this.rows(); i++) {
	    var min = null;

	    for(var j = 1; j <= this.cols(); j++) {
		if(min === null || this.e(i, j) < min) {
		    min = this.e(i, j);
		}
	    }

	    mins.push(min);
	}

	return $V(mins);
    },
    
    // perorm a partial pivot on the matrix. essentially move the largest
    // row below-or-including the pivot and replace the pivot's row with it.
    // a pivot matrix is returned so multiplication can perform the transform.
    partialPivot: function(k, j, P, A, L) {
	var maxIndex = 0;
	var maxValue = 0;

	for(var i = k; i <= A.rows(); i++) {
	    if(Math.abs(A.e(i, j)) > maxValue) {
		maxValue = Math.abs(A.e(k, j));
		maxIndex = i;
	    }
	}

	if(maxIndex != k) {
	    var tmp = A.elements[k - 1];
	    A.elements[k - 1] = A.elements[maxIndex - 1];
	    A.elements[maxIndex - 1] = tmp;
	    
	    P.elements[k - 1][k - 1] = 0;
	    P.elements[k - 1][maxIndex - 1] = 1;
	    P.elements[maxIndex - 1][maxIndex - 1] = 0;
	    P.elements[maxIndex - 1][k - 1] = 1;
	}
	
	return P;
    },

    // solve lower-triangular matrix * x = b via forward substitution
    forwardSubstitute: function(b) {
	var xa = [];

	for(var i = 1; i <= this.rows(); i++) {
	    var w = 0;

	    for(var j = 1; j < i; j++) {
		w += this.e(i, j) * xa[j - 1];
	    }

	    xa.push((b.e(i) - w) / this.e(i, i));
	}

	return $V(xa);
    },

    // solve an upper-triangular matrix * x = b via back substitution
    backSubstitute: function(b) {
	var xa = [];

	for(var i = this.rows(); i > 0; i--) {
	    var w = 0;

	    for(var j = this.cols(); j > i; j--) {
		w += this.e(i, j) * xa[this.rows() - j];
	    }

	    xa.push((b.e(i) - w) / this.e(i, i));
	}

	return $V(xa.reverse());
    },
    
    luPack: luPack,
    luJs: luJs,
    svdJs: svdJs,
    svdPack: svdPack,
    qrJs: qrJs,
    qrPack: qrPack
};

// LU factorization from LAPACK
function luPack() {
    var lu = lapack.lu(this.elements);
    return {
	L: $M(lu.L),
	U: $M(lu.U),
	P: $M(lu.P)
	// don't pass back IPIV
    };
}

var tolerance =  1.4901e-08;

// pure Javascript LU factorization
function luJs() {
    var A = this.dup();
    var L = Matrix.I(A.rows());
    var P = Matrix.I(A.rows());
    var U = Matrix.Zeros(A.rows(), A.cols());
    var p = 1;

    for(var k = 1; k <= Math.min(A.cols(), A.rows()); k++) {
	P = A.partialPivot(k, p, P, A, L);
	
	for(var i = k + 1; i <= A.rows(); i++) {
	    var l = A.e(i, p) / A.e(k, p);
	    L.elements[i - 1][k - 1] = l;
	    
	    for(var j = k + 1 ; j <= A.cols(); j++) {
		A.elements[i - 1][j - 1] -= A.e(k, j) * l;
	    }
	}
	
	for(var j = k; j <= A.cols(); j++) {
	    U.elements[k - 1][j - 1] = A.e(k, j);
	}

	if(p < A.cols())
	    p++;
    }    
    
    return {L: L, U: U, P: P};
}

function getLapack() {
    try {
	return _dereq_('lapack');
    } catch(e) {}
}

var lapack;

// if node-lapack is installed use the fast, native fortran routines
if(lapack = getLapack()) {
    Matrix.prototype.svd = svdPack;
    Matrix.prototype.qr = qrPack;
    Matrix.prototype.lu = luPack;
} else {
    // otherwise use the slower pure Javascript versions
    Matrix.prototype.svd = svdJs;
    Matrix.prototype.qr = qrJs;
    Matrix.prototype.lu = luJs;
}

// Constructor function
Matrix.create = function(aElements, ignoreLapack) {
    var M = new Matrix().setElements(aElements);
    return M;
};

// Identity matrix of size n
Matrix.I = function(n) {
    var els = [], i = n, j;
    while (i--) {
	j = n;
	els[i] = [];
	while (j--) {
	    els[i][j] = (i == j) ? 1 : 0;
	}
    }
    return Matrix.create(els);
};

Matrix.loadFile = function(file) {
    var contents = fs.readFileSync(file, 'utf-8');
    var matrix = [];

    var rowArray = contents.split('\n');
    for (var i = 0; i < rowArray.length; i++) {
	var d = rowArray[i].split(',');
	if (d.length > 1) {
	    matrix.push(d);
	}
    }

    var M = new Matrix();
    return M.setElements(matrix);
};

// Diagonal matrix - all off-diagonal elements are zero
Matrix.Diagonal = function(elements) {
    var i = elements.length;
    var M = Matrix.I(i);
    while (i--) {
	M.elements[i][i] = elements[i];
    }
    return M;
};

// Rotation matrix about some axis. If no axis is
// supplied, assume we're after a 2D transform
Matrix.Rotation = function(theta, a) {
    if (!a) {
	return Matrix.create([
	    [Math.cos(theta), -Math.sin(theta)],
	    [Math.sin(theta), Math.cos(theta)]
	]);
    }
    var axis = a.dup();
    if (axis.elements.length != 3) { return null; }
    var mod = axis.modulus();
    var x = axis.elements[0] / mod, y = axis.elements[1] / mod, z = axis.elements[2] / mod;
    var s = Math.sin(theta), c = Math.cos(theta), t = 1 - c;
    // Formula derived here: http://www.gamedev.net/reference/articles/article1199.asp
    // That proof rotates the co-ordinate system so theta
    // becomes -theta and sin becomes -sin here.
    return Matrix.create([
	[t * x * x + c, t * x * y - s * z, t * x * z + s * y],
	[t * x * y + s * z, t * y * y + c, t * y * z - s * x],
	[t * x * z - s * y, t * y * z + s * x, t * z * z + c]
    ]);
};

// Special case rotations
Matrix.RotationX = function(t) {
    var c = Math.cos(t), s = Math.sin(t);
    return Matrix.create([
	[1, 0, 0],
	[0, c, -s],
	[0, s, c]
    ]);
};

Matrix.RotationY = function(t) {
    var c = Math.cos(t), s = Math.sin(t);
    return Matrix.create([
	[c, 0, s],
	[0, 1, 0],
	[-s, 0, c]
    ]);
};

Matrix.RotationZ = function(t) {
    var c = Math.cos(t), s = Math.sin(t);
    return Matrix.create([
	[c, -s, 0],
	[s, c, 0],
	[0, 0, 1]
    ]);
};

// Random matrix of n rows, m columns
Matrix.Random = function(n, m) {
    if (arguments.length === 1) m = n;
    return Matrix.Zero(n, m).map(
	function() { return Math.random(); }
  );
};

Matrix.Fill = function(n, m, v) {
    if (arguments.length === 2) {
	v = m;
	m = n;
    }

    var els = [], i = n, j;

    while (i--) {
	j = m;
	els[i] = [];

	while (j--) {
	    els[i][j] = v;
	}
    }

    return Matrix.create(els);
};

// Matrix filled with zeros
Matrix.Zero = function(n, m) {
    return Matrix.Fill(n, m, 0);
};

// Matrix filled with zeros
Matrix.Zeros = function(n, m) {
    return Matrix.Zero(n, m);
};

// Matrix filled with ones
Matrix.One = function(n, m) {
    return Matrix.Fill(n, m, 1);
};

// Matrix filled with ones
Matrix.Ones = function(n, m) {
    return Matrix.One(n, m);
};

module.exports = Matrix;

},{"./sylvester":93,"./vector":94,"fs":1,"lapack":82}],92:[function(_dereq_,module,exports){
// Copyright (c) 2011, Chris Umbel, James Coglan
// Plane class - depends on Vector. Some methods require Matrix and Line.
var Vector = _dereq_('./vector');
var Matrix = _dereq_('./matrix');
var Line = _dereq_('./line');

var Sylvester = _dereq_('./sylvester');

function Plane() {}
Plane.prototype = {

  // Returns true iff the plane occupies the same space as the argument
  eql: function(plane) {
    return (this.contains(plane.anchor) && this.isParallelTo(plane));
  },

  // Returns a copy of the plane
  dup: function() {
    return Plane.create(this.anchor, this.normal);
  },

  // Returns the result of translating the plane by the given vector
  translate: function(vector) {
    var V = vector.elements || vector;
    return Plane.create([
      this.anchor.elements[0] + V[0],
      this.anchor.elements[1] + V[1],
      this.anchor.elements[2] + (V[2] || 0)
    ], this.normal);
  },

  // Returns true iff the plane is parallel to the argument. Will return true
  // if the planes are equal, or if you give a line and it lies in the plane.
  isParallelTo: function(obj) {
    var theta;
    if (obj.normal) {
      // obj is a plane
      theta = this.normal.angleFrom(obj.normal);
      return (Math.abs(theta) <= Sylvester.precision || Math.abs(Math.PI - theta) <= Sylvester.precision);
    } else if (obj.direction) {
      // obj is a line
      return this.normal.isPerpendicularTo(obj.direction);
    }
    return null;
  },

  // Returns true iff the receiver is perpendicular to the argument
  isPerpendicularTo: function(plane) {
    var theta = this.normal.angleFrom(plane.normal);
    return (Math.abs(Math.PI/2 - theta) <= Sylvester.precision);
  },

  // Returns the plane's distance from the given object (point, line or plane)
  distanceFrom: function(obj) {
    if (this.intersects(obj) || this.contains(obj)) { return 0; }
    if (obj.anchor) {
      // obj is a plane or line
      var A = this.anchor.elements, B = obj.anchor.elements, N = this.normal.elements;
      return Math.abs((A[0] - B[0]) * N[0] + (A[1] - B[1]) * N[1] + (A[2] - B[2]) * N[2]);
    } else {
      // obj is a point
      var P = obj.elements || obj;
      var A = this.anchor.elements, N = this.normal.elements;
      return Math.abs((A[0] - P[0]) * N[0] + (A[1] - P[1]) * N[1] + (A[2] - (P[2] || 0)) * N[2]);
    }
  },

  // Returns true iff the plane contains the given point or line
  contains: function(obj) {
    if (obj.normal) { return null; }
    if (obj.direction) {
      return (this.contains(obj.anchor) && this.contains(obj.anchor.add(obj.direction)));
    } else {
      var P = obj.elements || obj;
      var A = this.anchor.elements, N = this.normal.elements;
      var diff = Math.abs(N[0]*(A[0] - P[0]) + N[1]*(A[1] - P[1]) + N[2]*(A[2] - (P[2] || 0)));
      return (diff <= Sylvester.precision);
    }
  },

  // Returns true iff the plane has a unique point/line of intersection with the argument
  intersects: function(obj) {
    if (typeof(obj.direction) == 'undefined' && typeof(obj.normal) == 'undefined') { return null; }
    return !this.isParallelTo(obj);
  },

  // Returns the unique intersection with the argument, if one exists. The result
  // will be a vector if a line is supplied, and a line if a plane is supplied.
  intersectionWith: function(obj) {
    if (!this.intersects(obj)) { return null; }
    if (obj.direction) {
      // obj is a line
      var A = obj.anchor.elements, D = obj.direction.elements,
          P = this.anchor.elements, N = this.normal.elements;
      var multiplier = (N[0]*(P[0]-A[0]) + N[1]*(P[1]-A[1]) + N[2]*(P[2]-A[2])) / (N[0]*D[0] + N[1]*D[1] + N[2]*D[2]);
      return Vector.create([A[0] + D[0]*multiplier, A[1] + D[1]*multiplier, A[2] + D[2]*multiplier]);
    } else if (obj.normal) {
      // obj is a plane
      var direction = this.normal.cross(obj.normal).toUnitVector();
      // To find an anchor point, we find one co-ordinate that has a value
      // of zero somewhere on the intersection, and remember which one we picked
      var N = this.normal.elements, A = this.anchor.elements,
          O = obj.normal.elements, B = obj.anchor.elements;
      var solver = Matrix.Zero(2,2), i = 0;
      while (solver.isSingular()) {
        i++;
        solver = Matrix.create([
          [ N[i%3], N[(i+1)%3] ],
          [ O[i%3], O[(i+1)%3]  ]
        ]);
      }
      // Then we solve the simultaneous equations in the remaining dimensions
      var inverse = solver.inverse().elements;
      var x = N[0]*A[0] + N[1]*A[1] + N[2]*A[2];
      var y = O[0]*B[0] + O[1]*B[1] + O[2]*B[2];
      var intersection = [
        inverse[0][0] * x + inverse[0][1] * y,
        inverse[1][0] * x + inverse[1][1] * y
      ];
      var anchor = [];
      for (var j = 1; j <= 3; j++) {
        // This formula picks the right element from intersection by
        // cycling depending on which element we set to zero above
        anchor.push((i == j) ? 0 : intersection[(j + (5 - i)%3)%3]);
      }
      return Line.create(anchor, direction);
    }
  },

  // Returns the point in the plane closest to the given point
  pointClosestTo: function(point) {
    var P = point.elements || point;
    var A = this.anchor.elements, N = this.normal.elements;
    var dot = (A[0] - P[0]) * N[0] + (A[1] - P[1]) * N[1] + (A[2] - (P[2] || 0)) * N[2];
    return Vector.create([P[0] + N[0] * dot, P[1] + N[1] * dot, (P[2] || 0) + N[2] * dot]);
  },

  // Returns a copy of the plane, rotated by t radians about the given line
  // See notes on Line#rotate.
  rotate: function(t, line) {
    var R = t.determinant ? t.elements : Matrix.Rotation(t, line.direction).elements;
    var C = line.pointClosestTo(this.anchor).elements;
    var A = this.anchor.elements, N = this.normal.elements;
    var C1 = C[0], C2 = C[1], C3 = C[2], A1 = A[0], A2 = A[1], A3 = A[2];
    var x = A1 - C1, y = A2 - C2, z = A3 - C3;
    return Plane.create([
      C1 + R[0][0] * x + R[0][1] * y + R[0][2] * z,
      C2 + R[1][0] * x + R[1][1] * y + R[1][2] * z,
      C3 + R[2][0] * x + R[2][1] * y + R[2][2] * z
    ], [
      R[0][0] * N[0] + R[0][1] * N[1] + R[0][2] * N[2],
      R[1][0] * N[0] + R[1][1] * N[1] + R[1][2] * N[2],
      R[2][0] * N[0] + R[2][1] * N[1] + R[2][2] * N[2]
    ]);
  },

  // Returns the reflection of the plane in the given point, line or plane.
  reflectionIn: function(obj) {
    if (obj.normal) {
      // obj is a plane
      var A = this.anchor.elements, N = this.normal.elements;
      var A1 = A[0], A2 = A[1], A3 = A[2], N1 = N[0], N2 = N[1], N3 = N[2];
      var newA = this.anchor.reflectionIn(obj).elements;
      // Add the plane's normal to its anchor, then mirror that in the other plane
      var AN1 = A1 + N1, AN2 = A2 + N2, AN3 = A3 + N3;
      var Q = obj.pointClosestTo([AN1, AN2, AN3]).elements;
      var newN = [Q[0] + (Q[0] - AN1) - newA[0], Q[1] + (Q[1] - AN2) - newA[1], Q[2] + (Q[2] - AN3) - newA[2]];
      return Plane.create(newA, newN);
    } else if (obj.direction) {
      // obj is a line
      return this.rotate(Math.PI, obj);
    } else {
      // obj is a point
      var P = obj.elements || obj;
      return Plane.create(this.anchor.reflectionIn([P[0], P[1], (P[2] || 0)]), this.normal);
    }
  },

  // Sets the anchor point and normal to the plane. If three arguments are specified,
  // the normal is calculated by assuming the three points should lie in the same plane.
  // If only two are sepcified, the second is taken to be the normal. Normal vector is
  // normalised before storage.
  setVectors: function(anchor, v1, v2) {
    anchor = Vector.create(anchor);
    anchor = anchor.to3D(); if (anchor === null) { return null; }
    v1 = Vector.create(v1);
    v1 = v1.to3D(); if (v1 === null) { return null; }
    if (typeof(v2) == 'undefined') {
      v2 = null;
    } else {
      v2 = Vector.create(v2);
      v2 = v2.to3D(); if (v2 === null) { return null; }
    }
    var A1 = anchor.elements[0], A2 = anchor.elements[1], A3 = anchor.elements[2];
    var v11 = v1.elements[0], v12 = v1.elements[1], v13 = v1.elements[2];
    var normal, mod;
    if (v2 !== null) {
      var v21 = v2.elements[0], v22 = v2.elements[1], v23 = v2.elements[2];
      normal = Vector.create([
        (v12 - A2) * (v23 - A3) - (v13 - A3) * (v22 - A2),
        (v13 - A3) * (v21 - A1) - (v11 - A1) * (v23 - A3),
        (v11 - A1) * (v22 - A2) - (v12 - A2) * (v21 - A1)
      ]);
      mod = normal.modulus();
      if (mod === 0) { return null; }
      normal = Vector.create([normal.elements[0] / mod, normal.elements[1] / mod, normal.elements[2] / mod]);
    } else {
      mod = Math.sqrt(v11*v11 + v12*v12 + v13*v13);
      if (mod === 0) { return null; }
      normal = Vector.create([v1.elements[0] / mod, v1.elements[1] / mod, v1.elements[2] / mod]);
    }
    this.anchor = anchor;
    this.normal = normal;
    return this;
  }
};

// Constructor function
Plane.create = function(anchor, v1, v2) {
  var P = new Plane();
  return P.setVectors(anchor, v1, v2);
};

// X-Y-Z planes
Plane.XY = Plane.create(Vector.Zero(3), Vector.k);
Plane.YZ = Plane.create(Vector.Zero(3), Vector.i);
Plane.ZX = Plane.create(Vector.Zero(3), Vector.j);
Plane.YX = Plane.XY; Plane.ZY = Plane.YZ; Plane.XZ = Plane.ZX;

// Returns the plane containing the given points (can be arrays as
// well as vectors). If the points are not coplanar, returns null.
Plane.fromPoints = function(points) {
  var np = points.length, list = [], i, P, n, N, A, B, C, D, theta, prevN, totalN = Vector.Zero(3);
  for (i = 0; i < np; i++) {
    P = Vector.create(points[i]).to3D();
    if (P === null) { return null; }
    list.push(P);
    n = list.length;
    if (n > 2) {
      // Compute plane normal for the latest three points
      A = list[n-1].elements; B = list[n-2].elements; C = list[n-3].elements;
      N = Vector.create([
        (A[1] - B[1]) * (C[2] - B[2]) - (A[2] - B[2]) * (C[1] - B[1]),
        (A[2] - B[2]) * (C[0] - B[0]) - (A[0] - B[0]) * (C[2] - B[2]),
        (A[0] - B[0]) * (C[1] - B[1]) - (A[1] - B[1]) * (C[0] - B[0])
      ]).toUnitVector();
      if (n > 3) {
        // If the latest normal is not (anti)parallel to the previous one, we've strayed off the plane.
        // This might be a slightly long-winded way of doing things, but we need the sum of all the normals
        // to find which way the plane normal should point so that the points form an anticlockwise list.
        theta = N.angleFrom(prevN);
        if (theta !== null) {
          if (!(Math.abs(theta) <= Sylvester.precision || Math.abs(theta - Math.PI) <= Sylvester.precision)) { return null; }
        }
      }
      totalN = totalN.add(N);
      prevN = N;
    }
  }
  // We need to add in the normals at the start and end points, which the above misses out
  A = list[1].elements; B = list[0].elements; C = list[n-1].elements; D = list[n-2].elements;
  totalN = totalN.add(Vector.create([
    (A[1] - B[1]) * (C[2] - B[2]) - (A[2] - B[2]) * (C[1] - B[1]),
    (A[2] - B[2]) * (C[0] - B[0]) - (A[0] - B[0]) * (C[2] - B[2]),
    (A[0] - B[0]) * (C[1] - B[1]) - (A[1] - B[1]) * (C[0] - B[0])
  ]).toUnitVector()).add(Vector.create([
    (B[1] - C[1]) * (D[2] - C[2]) - (B[2] - C[2]) * (D[1] - C[1]),
    (B[2] - C[2]) * (D[0] - C[0]) - (B[0] - C[0]) * (D[2] - C[2]),
    (B[0] - C[0]) * (D[1] - C[1]) - (B[1] - C[1]) * (D[0] - C[0])
  ]).toUnitVector());
  return Plane.create(list[0], totalN);
};

module.exports = Plane;

},{"./line":89,"./matrix":91,"./sylvester":93,"./vector":94}],93:[function(_dereq_,module,exports){
// Copyright (c) 2011, Chris Umbel, James Coglan
// This file is required in order for any other classes to work. Some Vector methods work with the
// other Sylvester classes and are useless unless they are included. Other classes such as Line and
// Plane will not function at all without Vector being loaded first.           

Math.sign = function(x) {
    return x < 0 ? -1: 1;
}
                                              
var Sylvester = {
    precision: 1e-6,
    approxPrecision: 1e-5
};

module.exports = Sylvester;

},{}],94:[function(_dereq_,module,exports){
// Copyright (c) 2011, Chris Umbel, James Coglan
// This file is required in order for any other classes to work. Some Vector methods work with the
// other Sylvester classes and are useless unless they are included. Other classes such as Line and
// Plane will not function at all without Vector being loaded first.

var Sylvester = _dereq_('./sylvester'),
Matrix = _dereq_('./matrix');

function Vector() {}
Vector.prototype = {

    norm: function() {
	var n = this.elements.length;
	var sum = 0;

	while (n--) {
	    sum += Math.pow(this.elements[n], 2);
	}

	return Math.sqrt(sum);
    },

    // Returns element i of the vector
    e: function(i) {
      return (i < 1 || i > this.elements.length) ? null : this.elements[i - 1];
    },

    // Returns the number of rows/columns the vector has
    dimensions: function() {
      return {rows: 1, cols: this.elements.length};
    },

    // Returns the number of rows in the vector
    rows: function() {
      return 1;
    },

    // Returns the number of columns in the vector
    cols: function() {
      return this.elements.length;
    },

    // Returns the modulus ('length') of the vector
    modulus: function() {
      return Math.sqrt(this.dot(this));
    },

    // Returns true iff the vector is equal to the argument
    eql: function(vector) {
    	var n = this.elements.length;
    	var V = vector.elements || vector;
    	if (n != V.length) { return false; }
    	while (n--) {
    	    if (Math.abs(this.elements[n] - V[n]) > Sylvester.precision) { return false; }
    	}
    	return true;
    },

    // Returns a copy of the vector
    dup: function() {
	    return Vector.create(this.elements);
    },

    // Maps the vector to another vector according to the given function
    map: function(fn) {
	var elements = [];
	this.each(function(x, i) {
	    elements.push(fn(x, i));
	});
	return Vector.create(elements);
    },

    // Calls the iterator for each element of the vector in turn
    each: function(fn) {
	var n = this.elements.length;
	for (var i = 0; i < n; i++) {
	    fn(this.elements[i], i + 1);
	}
    },

    // Returns a new vector created by normalizing the receiver
    toUnitVector: function() {
	var r = this.modulus();
	if (r === 0) { return this.dup(); }
	return this.map(function(x) { return x / r; });
    },

    // Returns the angle between the vector and the argument (also a vector)
    angleFrom: function(vector) {
	var V = vector.elements || vector;
	var n = this.elements.length, k = n, i;
	if (n != V.length) { return null; }
	var dot = 0, mod1 = 0, mod2 = 0;
	// Work things out in parallel to save time
	this.each(function(x, i) {
	    dot += x * V[i - 1];
	    mod1 += x * x;
	    mod2 += V[i - 1] * V[i - 1];
	});
	mod1 = Math.sqrt(mod1); mod2 = Math.sqrt(mod2);
	if (mod1 * mod2 === 0) { return null; }
	var theta = dot / (mod1 * mod2);
	if (theta < -1) { theta = -1; }
	if (theta > 1) { theta = 1; }
	return Math.acos(theta);
    },

    // Returns true iff the vector is parallel to the argument
    isParallelTo: function(vector) {
	var angle = this.angleFrom(vector);
	return (angle === null) ? null : (angle <= Sylvester.precision);
    },

    // Returns true iff the vector is antiparallel to the argument
    isAntiparallelTo: function(vector) {
	var angle = this.angleFrom(vector);
	return (angle === null) ? null : (Math.abs(angle - Math.PI) <= Sylvester.precision);
    },

    // Returns true iff the vector is perpendicular to the argument
    isPerpendicularTo: function(vector) {
	var dot = this.dot(vector);
	return (dot === null) ? null : (Math.abs(dot) <= Sylvester.precision);
    },

    // Returns the result of adding the argument to the vector
    add: function(value) {
	var V = value.elements || value;

	if (this.elements.length != V.length) 
	    return this.map(function(v) { return v + value });
	else
	    return this.map(function(x, i) { return x + V[i - 1]; });
    },

    // Returns the result of subtracting the argument from the vector
    subtract: function(v) {
	if (typeof(v) == 'number')
	    return this.map(function(k) { return k - v; });

	var V = v.elements || v;
	if (this.elements.length != V.length) { return null; }
	return this.map(function(x, i) { return x - V[i - 1]; });
    },

    // Returns the result of multiplying the elements of the vector by the argument
    multiply: function(k) {
	return this.map(function(x) { return x * k; });
    },

    elementMultiply: function(v) {
	return this.map(function(k, i) {
	    return v.e(i) * k;
	});
    },

    sum: function() {
	var sum = 0;
	this.map(function(x) { sum += x;});
	return sum;
    },

    chomp: function(n) {
	var elements = [];

	for (var i = n; i < this.elements.length; i++) {
	    elements.push(this.elements[i]);
	}

	return Vector.create(elements);
    },

    top: function(n) {
	var elements = [];

	for (var i = 0; i < n; i++) {
	    elements.push(this.elements[i]);
	}

	return Vector.create(elements);
    },

    augment: function(elements) {
	var newElements = this.elements;

	for (var i = 0; i < elements.length; i++) {
	    newElements.push(elements[i]);
	}

	return Vector.create(newElements);
    },

    x: function(k) { return this.multiply(k); },

    log: function() {
	return Vector.log(this);
    },

    elementDivide: function(vector) {
	return this.map(function(v, i) {
	    return v / vector.e(i);
	});
    },

    product: function() {
	var p = 1;

	this.map(function(v) {
	    p *= v;
	});

	return p;
    },

    // Returns the scalar product of the vector with the argument
    // Both vectors must have equal dimensionality
    dot: function(vector) {
	var V = vector.elements || vector;
	var i, product = 0, n = this.elements.length;	
	if (n != V.length) { return null; }
	while (n--) { product += this.elements[n] * V[n]; }
	return product;
    },

    // Returns the vector product of the vector with the argument
    // Both vectors must have dimensionality 3
    cross: function(vector) {
	var B = vector.elements || vector;
	if (this.elements.length != 3 || B.length != 3) { return null; }
	var A = this.elements;
	return Vector.create([
	    (A[1] * B[2]) - (A[2] * B[1]),
	    (A[2] * B[0]) - (A[0] * B[2]),
	    (A[0] * B[1]) - (A[1] * B[0])
	]);
    },

    // Returns the (absolute) largest element of the vector
    max: function() {
	var m = 0, i = this.elements.length;
	while (i--) {
	    if (Math.abs(this.elements[i]) > Math.abs(m)) { m = this.elements[i]; }
	}
	return m;
    },


    maxIndex: function() {
	var m = 0, i = this.elements.length;
	var maxIndex = -1;

	while (i--) {
	    if (Math.abs(this.elements[i]) > Math.abs(m)) { 
		m = this.elements[i]; 
		maxIndex = i + 1;
	    }
	}

	return maxIndex;
    },


    // Returns the index of the first match found
    indexOf: function(x) {
	var index = null, n = this.elements.length;
	for (var i = 0; i < n; i++) {
	    if (index === null && this.elements[i] == x) {
		index = i + 1;
	    }
	}
	return index;
    },

    // Returns a diagonal matrix with the vector's elements as its diagonal elements
    toDiagonalMatrix: function() {
	return Matrix.Diagonal(this.elements);
    },

    // Returns the result of rounding the elements of the vector
    round: function() {
	return this.map(function(x) { return Math.round(x); });
    },

    // Transpose a Vector, return a 1xn Matrix
    transpose: function() {
	var rows = this.elements.length;
	var elements = [];

	for (var i = 0; i < rows; i++) {
	    elements.push([this.elements[i]]);
	}
	return Matrix.create(elements);
    },

    // Returns a copy of the vector with elements set to the given value if they
    // differ from it by less than Sylvester.precision
    snapTo: function(x) {
	return this.map(function(y) {
	    return (Math.abs(y - x) <= Sylvester.precision) ? x : y;
	});
    },

    // Returns the vector's distance from the argument, when considered as a point in space
    distanceFrom: function(obj) {
	if (obj.anchor || (obj.start && obj.end)) { return obj.distanceFrom(this); }
	var V = obj.elements || obj;
	if (V.length != this.elements.length) { return null; }
	var sum = 0, part;
	this.each(function(x, i) {
	    part = x - V[i - 1];
	    sum += part * part;
	});
	return Math.sqrt(sum);
    },

    // Returns true if the vector is point on the given line
    liesOn: function(line) {
	return line.contains(this);
    },

    // Return true iff the vector is a point in the given plane
    liesIn: function(plane) {
	return plane.contains(this);
    },

    // Rotates the vector about the given object. The object should be a
    // point if the vector is 2D, and a line if it is 3D. Be careful with line directions!
    rotate: function(t, obj) {
	var V, R = null, x, y, z;
	if (t.determinant) { R = t.elements; }
	switch (this.elements.length) {
	case 2:
            V = obj.elements || obj;
            if (V.length != 2) { return null; }
            if (!R) { R = Matrix.Rotation(t).elements; }
            x = this.elements[0] - V[0];
            y = this.elements[1] - V[1];
            return Vector.create([
		V[0] + R[0][0] * x + R[0][1] * y,
		V[1] + R[1][0] * x + R[1][1] * y
            ]);
            break;
	case 3:
            if (!obj.direction) { return null; }
            var C = obj.pointClosestTo(this).elements;
            if (!R) { R = Matrix.Rotation(t, obj.direction).elements; }
            x = this.elements[0] - C[0];
            y = this.elements[1] - C[1];
            z = this.elements[2] - C[2];
            return Vector.create([
		C[0] + R[0][0] * x + R[0][1] * y + R[0][2] * z,
		C[1] + R[1][0] * x + R[1][1] * y + R[1][2] * z,
		C[2] + R[2][0] * x + R[2][1] * y + R[2][2] * z
            ]);
            break;
	default:
            return null;
	}
    },

    // Returns the result of reflecting the point in the given point, line or plane
    reflectionIn: function(obj) {
	if (obj.anchor) {
	    // obj is a plane or line
	    var P = this.elements.slice();
	    var C = obj.pointClosestTo(P).elements;
	    return Vector.create([C[0] + (C[0] - P[0]), C[1] + (C[1] - P[1]), C[2] + (C[2] - (P[2] || 0))]);
	} else {
	    // obj is a point
	    var Q = obj.elements || obj;
	    if (this.elements.length != Q.length) { return null; }
	    return this.map(function(x, i) { return Q[i - 1] + (Q[i - 1] - x); });
	}
    },

    // Utility to make sure vectors are 3D. If they are 2D, a zero z-component is added
    to3D: function() {
	var V = this.dup();
	switch (V.elements.length) {
	case 3: break;
	case 2: V.elements.push(0); break;
	default: return null;
	}
	return V;
    },

    // Returns a string representation of the vector
    inspect: function() {
	return '[' + this.elements.join(', ') + ']';
    },

    // Set vector's elements from an array
    setElements: function(els) {
	this.elements = (els.elements || els).slice();
	return this;
    }
};

// Constructor function
Vector.create = function(elements) {
    var V = new Vector();
    return V.setElements(elements);
};

// i, j, k unit vectors
Vector.i = Vector.create([1, 0, 0]);
Vector.j = Vector.create([0, 1, 0]);
Vector.k = Vector.create([0, 0, 1]);

// Random vector of size n
Vector.Random = function(n) {
    var elements = [];
    while (n--) { elements.push(Math.random()); }
    return Vector.create(elements);
};

Vector.Fill = function(n, v) {
    var elements = [];
    while (n--) { elements.push(v); }
    return Vector.create(elements);
};

// Vector filled with zeros
Vector.Zero = function(n) {
    return Vector.Fill(n, 0);
};

Vector.One = function(n) {
    return Vector.Fill(n, 1);
};

Vector.log = function(v) {
    return v.map(function(x) {
	return Math.log(x);
    });
};

module.exports = Vector;

},{"./matrix":91,"./sylvester":93}]},{},[40])(40)
});
