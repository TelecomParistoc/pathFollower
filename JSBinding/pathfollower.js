var ffi = require('ffi');
var simpleCallback = ffi.Function('void', []);

var lib = ffi.Library('./../build/librobotdriver', {
    'setCurrentLocation': [ 'void', ['double', 'double'] ],
    'setCurrentX': [ 'void', ['double'] ],
    'setCurrentY': [ 'void', ['double'] ],
    'setCruiseSpeed': [ 'void', ['double'] ],
    'followPath': ['void', ['struct robotPoint *', 'int', 'double', simpleCallback]],
});
var endCallback;

module.exports = {
    location: lib.setCurrentLocation,
    x: lib.setCurrentX,
    y: lib.setCurrentY,
    cruiseSpeed: lib.setCruiseSpeed,
    followPath: function (path, endSpeed, callback) {
        endCallback = callback;
        // TODO: some magic here
        var cbck = ffi.Callback('void', [], endCallback);
        lib.followPath(path, path.length, endSpeed, cbck);
    }
};
