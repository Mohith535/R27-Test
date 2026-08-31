/*
 * Round-trip and edge-case checks for the framing in src/en_dc.c.
 *
 * Task 1 asks for the decoder to return the original data and for buffer
 * limits and invalid inputs to be handled, so this asserts both rather than
 * relying on the four rover testcases to notice a codec bug indirectly.
 *
 * Built as a second executable:  ./build/en_dc_test
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

#include "en_dc.h"

static int checks_run = 0;
static int checks_failed = 0;

static void check(int condition, const char *what) {
  checks_run++;
  if (!condition) {
    checks_failed++;
    printf("  FAIL: %s\n", what);
  }
}

/*
 * Encode then decode a buffer and require the bytes to come back unchanged.
 * Also asserts the two properties the framing exists to provide: the encoded
 * frame contains no 0x00, and it never exceeds the size the header promises.
 */
static void check_round_trip(const uint8_t *src, size_t src_len, const char *what) {
  size_t encoded_cap = ENCODE_DST_BUF_LEN_MAX(src_len);
  uint8_t *encoded = malloc(encoded_cap ? encoded_cap : 1u);
  uint8_t *decoded = NULL;
  size_t decoded_cap;
  encode_result enc;
  decode_result dec;

  if (encoded == NULL) {
    check(0, "allocation");
    return;
  }

  enc = frame_encode(encoded, encoded_cap, src, src_len);
  check(enc.status == ENCODE_OK, what);
  check(enc.out_len <= encoded_cap, what);

  for (size_t i = 0; i < enc.out_len; i++) {
    if (encoded[i] == 0u) {
      check(0, "encoded frame must contain no zero byte");
      break;
    }
  }

  decoded_cap = DECODE_DST_BUF_LEN_MAX(enc.out_len);
  decoded = malloc(decoded_cap ? decoded_cap : 1u);
  if (decoded == NULL) {
    check(0, "allocation");
    free(encoded);
    return;
  }

  dec = frame_decode(decoded, decoded_cap, encoded, enc.out_len);
  check(dec.status == DECODE_OK, what);
  check(dec.out_len == src_len, what);
  check(src_len == 0u || memcmp(decoded, src, src_len) == 0, what);

  free(decoded);
  free(encoded);
}

/* Deterministic pseudo-random source, so a failure is reproducible. */
static uint32_t rng_state = 0x13572468u;

static uint8_t next_byte(void) {
  rng_state = (rng_state * 1103515245u) + 12345u;
  return (uint8_t)(rng_state >> 16);
}

int main(void) {
  static uint8_t buffer[2048];
  uint8_t small[8];
  uint8_t encoded[32];
  uint8_t decoded[32];
  encode_result enc;
  decode_result dec;

  printf("framing tests\n");

  /* Empty input: encodes to the single code byte 0x01 and decodes back to
   * nothing at all. */
  {
    uint8_t nothing = 0u;
    enc = frame_encode(encoded, sizeof encoded, &nothing, 0u);
    check(enc.status == ENCODE_OK, "empty encode status");
    check(enc.out_len == 1u, "empty encode length");
    check(encoded[0] == 0x01u, "empty encode value");

    dec = frame_decode(decoded, sizeof decoded, encoded, enc.out_len);
    check(dec.status == DECODE_OK, "empty decode status");
    check(dec.out_len == 0u, "empty decode length");
  }

  /* A coordinate pair of 0.0 is eight zero bytes: the case the framing is
   * really there for, since a raw zero cannot go on the wire. */
  {
    float zero_lat = 0.0f;
    float zero_lon = 0.0f;
    memcpy(small, &zero_lat, sizeof zero_lat);
    memcpy(small + sizeof zero_lat, &zero_lon, sizeof zero_lon);
    check_round_trip(small, sizeof small, "all-zero coordinate payload");
  }

  /* Real coordinates from the testcases. */
  {
    float pairs[8][2] = {{0.0f, 0.0f},   {0.2f, 0.0f},  {0.5f, -0.2f},
                         {-0.5f, 0.3f},  {1.0f, 1.0f},  {-1.0f, -1.0f},
                         {2.0f, -2.0f},  {-2.0f, 2.0f}};
    for (int i = 0; i < 8; i++) {
      float lat = pairs[i][0];
      float lon = pairs[i][1];
      float back_lat = 1.0f;
      float back_lon = 1.0f;
      uint8_t payload[8];
      uint8_t frame[16];
      uint8_t out[16];

      memcpy(payload, &lat, sizeof lat);
      memcpy(payload + sizeof lat, &lon, sizeof lon);

      enc = frame_encode(frame, sizeof frame, payload, sizeof payload);
      check(enc.status == ENCODE_OK, "coordinate encode");

      dec = frame_decode(out, sizeof out, frame, enc.out_len);
      check(dec.status == DECODE_OK, "coordinate decode");
      check(dec.out_len == sizeof payload, "coordinate length");

      memcpy(&back_lat, out, sizeof back_lat);
      memcpy(&back_lon, out + sizeof back_lat, sizeof back_lon);
      check(back_lat == lat && back_lon == lon, "coordinate value survives");
    }
  }

  /* Patterns around the 254-byte block boundary, where the encoder has to
   * split a run and the decoder has to know not to reinsert a zero. */
  {
    size_t lengths[] = {1, 2, 253, 254, 255, 256, 507, 508, 509, 1020};
    for (size_t n = 0; n < sizeof lengths / sizeof lengths[0]; n++) {
      size_t len = lengths[n];

      memset(buffer, 0xAB, len);
      check_round_trip(buffer, len, "run of non-zero bytes");

      memset(buffer, 0x00, len);
      check_round_trip(buffer, len, "run of zero bytes");

      memset(buffer, 0xAB, len);
      buffer[0] = 0x00;
      buffer[len - 1] = 0x00;
      check_round_trip(buffer, len, "zero at both ends");

      for (size_t i = 0; i < len; i++) {
        buffer[i] = (uint8_t)((i % 3u) == 0u ? 0x00u : 0xC3u);
      }
      check_round_trip(buffer, len, "frequent zeros");
    }
  }

  /* Randomised round trips across a wide spread of lengths. */
  for (int trial = 0; trial < 500; trial++) {
    size_t len = (size_t)(next_byte()) * 4u + 1u;
    for (size_t i = 0; i < len; i++) {
      /* Bias towards zeros so block splitting is exercised hard. */
      buffer[i] = (next_byte() < 96u) ? 0u : next_byte();
    }
    check_round_trip(buffer, len, "random round trip");
  }

  /* Invalid inputs and buffer limits. */
  {
    enc = frame_encode(NULL, 16u, small, sizeof small);
    check(enc.status == ENCODE_NULL_POINTER, "encode rejects null destination");

    enc = frame_encode(encoded, sizeof encoded, NULL, 4u);
    check(enc.status == ENCODE_NULL_POINTER, "encode rejects null source");

    dec = frame_decode(NULL, 16u, encoded, 4u);
    check(dec.status == DECODE_NULL_POINTER, "decode rejects null destination");

    dec = frame_decode(decoded, sizeof decoded, NULL, 4u);
    check(dec.status == DECODE_NULL_POINTER, "decode rejects null source");
  }

  {
    /* Output buffer deliberately too small for the frame. */
    uint8_t payload[16];
    uint8_t tiny[4];

    memset(payload, 0x5A, sizeof payload);
    enc = frame_encode(tiny, sizeof tiny, payload, sizeof payload);
    check((enc.status & ENCODE_OUT_BUFFER_OVERFLOW) != 0,
          "encode reports output overflow");
    check(enc.out_len <= sizeof tiny, "encode stays inside a short buffer");

    enc = frame_encode(encoded, sizeof encoded, payload, sizeof payload);
    check(enc.status == ENCODE_OK, "encode into a large enough buffer");

    dec = frame_decode(tiny, sizeof tiny, encoded, enc.out_len);
    check((dec.status & DECODE_OUT_BUFFER_OVERFLOW) != 0,
          "decode reports output overflow");
    check(dec.out_len <= sizeof tiny, "decode stays inside a short buffer");
  }

  {
    /* A zero byte inside a frame is impossible in valid data. */
    uint8_t corrupt[5] = {0x03u, 0x11u, 0x22u, 0x00u, 0x33u};
    dec = frame_decode(decoded, sizeof decoded, corrupt, sizeof corrupt);
    check((dec.status & DECODE_ZERO_BYTE_IN_INPUT) != 0,
          "decode flags a zero byte inside the frame");
  }

  {
    /* Frame claiming more payload than the input actually holds. */
    uint8_t truncated[3] = {0x09u, 0x11u, 0x22u};
    dec = frame_decode(decoded, sizeof decoded, truncated, sizeof truncated);
    check((dec.status & DECODE_INPUT_TOO_SHORT) != 0,
          "decode flags a truncated frame");
    check(dec.out_len <= 2u, "decode does not read past the input");
  }

  printf("%d checks, %d failed\n", checks_run, checks_failed);
  if (checks_failed == 0) {
    printf("all framing tests passed\n");
    return 0;
  }
  return 1;
}
