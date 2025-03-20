#include "cobs.h"

size_t cobs_encode(uint8_t* dst, const uint8_t* src, size_t src_length) {
    size_t read_index = 0, write_index = 1, code_index = 0;
    uint8_t code = 1;

    while (read_index < src_length) {
        if (src[read_index] == 0) {
            dst[code_index] = code;
            code = 1;
            code_index = write_index++;
            read_index++;
        } else {
            dst[write_index++] = src[read_index++];
            code++;
            if (code == 0xFF) {
                dst[code_index] = code;
                code = 1;
                code_index = write_index++;
            }
        }
    }
    dst[code_index] = code;
    return write_index;
}

size_t cobs_decode(uint8_t* dst, const uint8_t* src, size_t src_length) {
    size_t read_index = 0, write_index = 0;
    while (read_index < src_length) {
        uint8_t code = src[read_index];
        if (code == 0) break; // Delimiter reached
        read_index++;
        // Copy (code - 1) non-zero bytes
        for (uint8_t i = 1; i < code; i++) {
            dst[write_index++] = src[read_index++];
        }
        // If code is less than 0xFF, append a zero byte (unless at the end)
        if (code < 0xFF && read_index < src_length) {
            dst[write_index++] = 0;
        }
    }
    return write_index;
}
