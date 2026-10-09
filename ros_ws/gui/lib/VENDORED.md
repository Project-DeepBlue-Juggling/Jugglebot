# Vendored libraries

Plain files served as-is (no bundler, no npm). Byte sizes and SHA-256 as shipped.

| file | library | version | bytes | sha256 |
|---|---|---|---|---|
| `roslib.min.js` | roslibjs | as vendored (pre-existing) | 66541 | `5d530ddf1cdeb2a39864989faf52ed31ccf53fd1cce929a218dafb69095d33ab` |
| `uPlot.iife.min.js` | uPlot | as vendored (pre-existing) | 50312 | `2d27e8ad3d228164525ce213f9dc716f39b4e3aee0cc773fb3491c96cf4921a2` |
| `uPlot.min.css` | uPlot | as vendored (pre-existing) | 1857 | `df630c6a8d6f8eeaff264b50f73ce5b114f646ffd9a0bb74f049b0a00135fa04` |
| `msgpack.min.js` | @msgpack/msgpack (ISC) | 2.8.0, `dist.es5+umd/msgpack.min.js` | 31572 | `43f39b184e9aebefca2b258ddfcdf03ce382190bca3e2e9e78d0b9b432d181b8` |

`msgpack.min.js` is fetched from
`https://cdn.jsdelivr.net/npm/@msgpack/msgpack@2.8.0/dist.es5+umd/msgpack.min.js`
(UMD; exposes the global `MessagePack`). It decodes the replay cache chunks
(`ros_ws/gui/js/replay/chunk.js`); `tests/ros/test_gui_replay_feed.py` loads
these same bytes under node.
