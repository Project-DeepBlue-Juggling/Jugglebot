export { McapIndexedReader } from '@mcap/core';
export { parse } from '@foxglove/rosmsg';
export { MessageReader } from '@foxglove/rosmsg2-serialization';
import { decompress as zstdDecompress } from 'fzstd';
// @mcap/core DecompressHandlers: { [compression]: (buffer, decompressedSize) => Uint8Array }.
export const decompressHandlers = { zstd: (buf, size) => zstdDecompress(buf, new Uint8Array(Number(size))) };
