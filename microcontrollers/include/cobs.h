#ifndef COBS_H
#define COBS_H

#include <cstddef>
#include <cstdint>

// Encodes a raw buffer (src) of length src_length into dst using COBS.
// The worst-case output size is src_length + 1 bytes.
// Returns the number of bytes written to dst.
size_t cobs_encode(uint8_t* dst, const uint8_t* src, size_t src_length);
size_t cobs_decode(uint8_t* dst, const uint8_t* src, size_t src_length);
#endif // COBS_H