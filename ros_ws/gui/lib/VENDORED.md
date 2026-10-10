# Vendored libraries

Plain files served as-is (no bundler, no npm). Byte sizes and SHA-256 as shipped.

| file | library | version | bytes | sha256 |
|---|---|---|---|---|
| `roslib.min.js` | roslibjs | as vendored (pre-existing) | 66541 | `5d530ddf1cdeb2a39864989faf52ed31ccf53fd1cce929a218dafb69095d33ab` |
| `uPlot.iife.min.js` | uPlot | as vendored (pre-existing) | 50312 | `2d27e8ad3d228164525ce213f9dc716f39b4e3aee0cc773fb3491c96cf4921a2` |
| `uPlot.min.css` | uPlot | as vendored (pre-existing) | 1857 | `df630c6a8d6f8eeaff264b50f73ce5b114f646ffd9a0bb74f049b0a00135fa04` |
| `mcap-bundle.min.js` | @mcap/core 2.3.0, @foxglove/rosmsg 5.0.5, @foxglove/rosmsg2-serialization 3.1.2 (MIT, plus transitive deps pinned by `tools/gui_vendor/package-lock.json`) | esbuild 0.28.2 ESM bundle; exports `McapIndexedReader`, `parse`, `MessageReader` | 119335 | `bcb05709938b86ae5dba7c97b3ba53084c211c3915dd9e9e3d61e23c7c4bf36d` |

`mcap-bundle.min.js` is an ES module (not a classic script). Rebuild with
`tools/gui_vendor/build.sh` (`npm ci` from the committed lockfile, then esbuild
`--bundle --minify --format=esm --platform=browser --legal-comments=eof`); it is
byte-reproducible for a fixed lock. After a rebuild, update the bytes and sha256 above;
`tests/ros/test_gui_vendor_pin.py` fails if they disagree.
