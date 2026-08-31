#include "en_dc.h"
#include <stdlib.h>

/*****************************************************************************
 * Defines
 ****************************************************************************/

#ifndef FALSE
#define FALSE (0)
#endif

#ifndef TRUE
#define TRUE (!FALSE)
#endif

/* A length code covers itself plus at most 254 payload bytes, so 0xFF is the
 * largest code that can be emitted and 0x00 stays reserved as the frame
 * delimiter. */
#define CODE_MAX (0xFFu)
#define RUN_MAX (0xFEu)

/*****************************************************************************
 * Functions
 ****************************************************************************/

/* Encode
 *
 * Consistent Overhead Byte Stuffing. Every zero byte in the source is removed
 * and replaced by the distance to the next zero, so the encoded frame contains
 * no 0x00 at all and 0x00 remains free to delimit frames on the wire.
 *
 * A code byte is written ahead of every run. The run currently being measured
 * has its code byte reserved at dst_code_write_ptr and back-filled once the
 * length of that run is known. Overhead is one byte per 254 payload bytes,
 * which is exactly what ENCODE_DST_BUF_LEN_MAX() budgets for.
 */
encode_result frame_encode(void *dst_buf_ptr, size_t dst_buf_len,
                               const void *src_ptr, size_t src_len) {
  encode_result result = {0u, ENCODE_OK};
  const uint8_t *src_read_ptr = src_ptr;
  const uint8_t *src_end_ptr = src_read_ptr + src_len;
  uint8_t *dst_buf_start_ptr = dst_buf_ptr;
  uint8_t *dst_buf_end_ptr = dst_buf_start_ptr + dst_buf_len;
  uint8_t *dst_code_write_ptr = dst_buf_ptr;
  uint8_t *dst_write_ptr = dst_code_write_ptr + 1u;
  uint8_t src_byte = 0u;
  uint8_t search_len = 1u;

  if ((dst_buf_ptr == NULL) || (src_ptr == NULL)) {
    result.status = ENCODE_NULL_POINTER;
    return result;
  }

  if (src_len != 0u) {
    for (;;) {
      /* Stop before writing past the end of the caller's buffer. */
      if (dst_write_ptr >= dst_buf_end_ptr) {
        result.status |= ENCODE_OUT_BUFFER_OVERFLOW;
        break;
      }

      src_byte = *src_read_ptr++;
      if (src_byte == 0u) {
        /* End of a run: back-fill its length code and reserve the code byte
         * of the next run. The zero itself is never copied out. */
        *dst_code_write_ptr = search_len;
        dst_code_write_ptr = dst_write_ptr++;
        search_len = 1u;
        if (src_read_ptr >= src_end_ptr) {
          break;
        }
      } else {
        *dst_write_ptr++ = src_byte;
        search_len++;
        if (src_read_ptr >= src_end_ptr) {
          break;
        }
        if (search_len == CODE_MAX) {
          /* 254 non-zero bytes in a row: a length code cannot reach any
           * further, so close this block off and start a new one. */
          *dst_code_write_ptr = search_len;
          dst_code_write_ptr = dst_write_ptr++;
          search_len = 1u;
        }
      }
    }
  }

  /* Write the code byte of the final run. A zero-length source lands here
   * directly and produces the single byte 0x01. */
  if (dst_code_write_ptr >= dst_buf_end_ptr) {
    result.status |= ENCODE_OUT_BUFFER_OVERFLOW;
    dst_write_ptr = dst_buf_end_ptr;
  } else {
    *dst_code_write_ptr = search_len;
  }

  result.out_len = (size_t)(dst_write_ptr - dst_buf_start_ptr);

  return result;
}

/* Decode
 *
 * Walks the frame one block at a time. Each code byte says how many bytes
 * follow before the zero that encoding removed, so the zero is put back after
 * every block except the last one, and except a full 0xFF block, which was a
 * length-driven split rather than a real zero.
 *
 * Every copy is bounded by both the bytes left in the input and the space left
 * in the output, so a truncated or corrupted frame is reported through the
 * status flags instead of over-reading or over-writing.
 */
decode_result frame_decode(void *dst_buf_ptr, size_t dst_buf_len,
                               const void *src_ptr, size_t src_len) {
  decode_result result = {0u, DECODE_OK};
  const uint8_t *src_read_ptr = src_ptr;
  const uint8_t *src_end_ptr = src_read_ptr + src_len;
  uint8_t *dst_buf_start_ptr = dst_buf_ptr;
  uint8_t *dst_buf_end_ptr = dst_buf_start_ptr + dst_buf_len;
  uint8_t *dst_write_ptr = dst_buf_ptr;
  size_t remaining_bytes;
  uint8_t src_byte;
  uint8_t i;
  uint8_t len_code;

  if ((dst_buf_ptr == NULL) || (src_ptr == NULL)) {
    result.status = DECODE_NULL_POINTER;
    return result;
  }

  if (src_len != 0u) {
    for (;;) {
      len_code = *src_read_ptr++;
      if (len_code == 0u) {
        /* 0x00 is the delimiter and must never appear inside a frame. */
        result.status |= DECODE_ZERO_BYTE_IN_INPUT;
        break;
      }
      len_code--;

      /* Clamp the block against the bytes actually left in the input. */
      remaining_bytes = (size_t)(src_end_ptr - src_read_ptr);
      if (len_code > remaining_bytes) {
        result.status |= DECODE_INPUT_TOO_SHORT;
        len_code = (uint8_t)remaining_bytes;
      }

      /* Clamp the block against the space left in the output buffer. */
      remaining_bytes = (size_t)(dst_buf_end_ptr - dst_write_ptr);
      if (len_code > remaining_bytes) {
        result.status |= DECODE_OUT_BUFFER_OVERFLOW;
        len_code = (uint8_t)remaining_bytes;
      }

      for (i = len_code; i != 0u; i--) {
        src_byte = *src_read_ptr++;
        if (src_byte == 0u) {
          result.status |= DECODE_ZERO_BYTE_IN_INPUT;
        }
        *dst_write_ptr++ = src_byte;
      }

      /* The last block ends the frame, and no zero was removed after it. */
      if (src_read_ptr >= src_end_ptr) {
        break;
      }

      /* Restore the zero this block stood in for, unless the block was a full
       * 0xFE-byte run that encoding split purely for length. */
      if (len_code != RUN_MAX) {
        if (dst_write_ptr >= dst_buf_end_ptr) {
          result.status |= DECODE_OUT_BUFFER_OVERFLOW;
          break;
        }
        *dst_write_ptr++ = 0u;
      }
    }
  }

  result.out_len = (size_t)(dst_write_ptr - dst_buf_start_ptr);

  return result;
}
