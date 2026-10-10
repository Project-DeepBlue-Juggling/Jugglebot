#!/usr/bin/env bash
# Rebuild ros_ws/gui/lib/mcap-bundle.min.js from the pinned lockfile.
# Then update the bytes + sha256 row in ros_ws/gui/lib/VENDORED.md.
set -euo pipefail
cd "$(dirname "$0")"
npm ci
npx esbuild entry.mjs --bundle --minify --format=esm --platform=browser --legal-comments=eof \
  --outfile=../../ros_ws/gui/lib/mcap-bundle.min.js
f=../../ros_ws/gui/lib/mcap-bundle.min.js
echo "bytes: $(wc -c < "$f")"
sha256sum "$f"
