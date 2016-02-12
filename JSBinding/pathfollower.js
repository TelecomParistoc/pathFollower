var ffi = require('ffi');
var Struct = require('ref-struct');
var simpleCallback = ffi.Function('void', []);
var robotPoint = Struct({
  'x': 'double',
  'y': 'double'
});
var robotPointPtr = ref.refType(robotPoint);

var lib = ffi.Library('libpathfollower', {
    'setCurrentLocation': [ 'void', ['double', 'double'] ],
    'setCurrentX': [ 'void', ['double'] ],
    'setCurrentY': [ 'void', ['double'] ],
    'setCruiseSpeed': [ 'void', ['double'] ],
    'followPath': ['void', [robotPointPtr, 'int', 'double', simpleCallback]],
});
var endCallback;

module.exports = {
    location: lib.setCurrentLocation,
    x: lib.setCurrentX,
    y: lib.setCurrentY,
    cruiseSpeed: lib.setCruiseSpeed,
    followPath: function (path, endSpeed, callback) {
        endCallback = callback;
        var points = [];
        for(var i in path) {
            points.push(new robotPoint({x: path[i].x, y: path[i].y,}));
        }
        var cbck = ffi.Callback('void', [], endCallback);
        lib.followPath(points, path.length, endSpeed, cbck);
    }
};
