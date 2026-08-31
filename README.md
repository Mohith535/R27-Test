# R27 Test — Rudra recruitment submission

[![build and check](https://github.com/Mohith535/R27-Test/actions/workflows/ci.yml/badge.svg)](https://github.com/Mohith535/R27-Test/actions/workflows/ci.yml)

<p align="center">
  <img src="https://raw.githubusercontent.com/teamrudra/r26_test/main/misc/rover.webp" width="360" height="360"/>
</p>

A threaded communication pipeline that reads coordinates from a file, frames
them, passes them safely between threads, decodes them, and drives a
differential-drive rover to each one.

**All five tasks are done, and the four files the program writes are
byte-for-byte identical to the reference output this repo ships.**

| Task | | Where |
|---|---|---|
| 0 — Fork, public | done | this repo |
| 1 — Encode / decode | done | [src/en_dc.c](src/en_dc.c), verified by [tests/en_dc_test.c](tests/en_dc_test.c) |
| 2 — Threads and synchronisation | done | [src/queue.c](src/queue.c), [src/mutex.c](src/mutex.c), [src/main.c](src/main.c) |
| 3 — Drive to target | done | [src/drive.c](src/drive.c) |
| 4 — Compile and run | done | green on Linux CI with gcc and clang |

The brief I was given is preserved verbatim in **[TASK.md](TASK.md)**.

## Check it in three commands

```
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

```
    Start 1: framing
1/2 Test #1: framing ..........................   Passed
    Start 2: pipeline
2/2 Test #2: pipeline .........................   Passed

100% tests passed, 0 tests failed out of 2
```

`framing` round-trips the codec. `pipeline` runs the rover end to end and diffs
all four result files against the reference. The R26 test had a `make check`
that answered this question outright; this repo dropped it, so I put it back —
see [Suggestions](#suggestions-for-the-test-repo).

The original instructions still work exactly as written:
`cmake -S . -B build`, `cmake --build build --verbose`, `./build/queue_test`.

---

## Understanding

The repo is one pipeline cut into stages that hand data to each other across
threads. Nothing in it stands alone — a bug in the framing shows up as a rover
driving to the wrong place — which is why I read all of it before changing any
of it.

```
input/testcaseN.txt
   |
   |  producer thread     read "x y", pack the two floats, frame them
   v
shared_buffer             one buffer, guarded by a reader-writer lock
   |
   |  3 consumer threads  one of them claims each message, unframes it
   v
Message_Queue             bounded circular buffer, 2 semaphores + 1 mutex
   |
   |  drive thread        pop, drive the rover to it, write one row
   v
result/resultN.txt
```

There are really three different synchronisation problems here, and the
skeleton deliberately uses a different primitive for each:

- one shared buffer many threads may read but only one may write → a
  reader-writer lock ([src/mutex.c](src/mutex.c));
- a "there is a new message" event → mutex + condition variable + a generation
  counter ([src/main.c](src/main.c));
- a bounded hand-off queue → counting semaphores + a mutex
  ([src/queue.c](src/queue.c)).

**`src/en_dc.c` is the transport layer, and it is COBS.** I did not have to
guess that. [lib/en_dc.h](lib/en_dc.h) budgets a worst case of
`SRC_LEN + ceil(SRC_LEN/254)`, uses 254 and `0xFF` as its magic numbers, and
defines a `DECODE_ZERO_BYTE_IN_INPUT` status. That is the signature of
Consistent Overhead Byte Stuffing: it removes every `0x00` from a payload and
replaces it with the distance to the next one, so `0x00` stays reserved as a
frame delimiter, at a cost of one byte per 254 bytes of payload.

That matters more here than it looks. A coordinate of `0.0 0.0` is eight zero
bytes, and `0.0 0.0` appears in every single testcase. Without the framing, the
most common message in the whole test would be indistinguishable on the wire
from an empty frame.

**`src/drive.c` is a differential-drive rover.** You hand it a left and a right
wheel velocity and it integrates a unicycle model on a fixed 20 ms timestep.
Latitude is north, longitude is east, heading 0 points east and grows
counter-clockwise. Reading `apply_wheel_velocities()` backwards gives the two
equations the controller has to invert:

```
v = R (wl + wr) / 2          w = R (wr - wl) / L
```

## Thought Process

### Step 1: make it fail before trying to make it pass

The first thing I did was build the untouched repo, so I would be looking at
what is actually there rather than what I assumed. It does not compile:
`frame_encode()` uses a `dst_code_write_ptr` that is never declared, and
`drive_write()` in `main.c` uses `coordinate_target`, `rover`, `result_status`
and `status`, none of which exist. So this is not a hunt for one subtle bug —
whole stages are missing, and the parts that are present have real logic errors
hiding behind the parts that are absent.

### Step 2: en_dc.c, once I knew it was COBS

With the algorithm named, each bug became a specific deviation from it rather
than a puzzle:

| Where | What was wrong |
|---|---|
| `frame_encode` | `dst_buf_end_ptr` was set to `dst_buf_start_ptr`, so the buffer looked zero bytes long |
| `frame_encode` | `dst_write_ptr` started at `dst_code_write_ptr + 3`, leaving a three-byte hole; COBS reserves exactly one |
| `frame_encode` | the loop was `for (i = 0; i < search_len; i++)` — a byte counter used as a loop bound |
| `frame_encode` | it tested `src_byte == 0xFF` and did nothing in that branch; the byte COBS cares about is `0x00`, and that branch is where a run gets closed |
| `frame_encode` | the block split tested `search_len == 0` instead of `== 0xFF`, so a 254-byte run of non-zero bytes was never split |
| `frame_encode` | no output-space check inside the loop at all |
| `frame_decode` | the loop was `for (i = 0; i < len_code; i++)` with `len_code` **read uninitialised** on the first test |
| `frame_decode` | the inner copy loop read each source byte and threw it away — it never wrote to the destination, so `out_len` was always 0 |
| `frame_decode` | no logic to restore the zero encoding removed, and no handling of the `0xFE` case where a block was split for length and there was no zero to restore |

The `0xFE` case is the one that is easy to get wrong and impossible to notice on
short messages. An eight-byte coordinate never reaches that path — which is
exactly why I did not want the four rover testcases to be the only thing
standing behind the codec.

### Step 3: drive.c, where there was no specification

This was the interesting one. The comments say path planning and PID are not
required and list constraints, but the actual controller is never specified —
and yet `result/expected_result*.txt` pins down 40 rows to two decimal places.
So the expected results *are* the specification, and the honest way to write the
function was to read them rather than guess and hope.

Four things fell straight out of the numbers:

1. **Testcase 1 row 1** — target `0.0 0.0`, output `0.00 0.00 0.00 0`. The rover
   starts at the origin and the distance test runs *before* the first step,
   otherwise it would have moved.

2. **Testcase 3 rows 1 and 6** — targets `1.0 0.0` and `-1.0 0.0`, outputs
   `0.97 0.09` and `-0.97 0.09`. Mirror-image targets, and the sideways drift is
   `+0.09` in **both**. That only happens if the rover always starts pointing
   east and curves onto the target while driving. If it turned on the spot
   first the drift would be zero; if heading carried over between messages the
   two rows could not be symmetric. So the rover resets to the origin, facing
   east, for every message.

3. **Testcase 2 row 1** — target `0.0 0.5`, straight ahead, output `0.00 0.40`.
   Exactly `0.5 - 0.10`. Full speed in a straight line, stopping the moment the
   remaining distance reaches `TARGET_TOLERANCE`.

4. **The error column is `0.10` on every row that is not already at the
   target.** This told me the most. At full speed the rover covers
   `1.0 × 0.02 = 0.02` per step, so a constant-speed controller would stop
   somewhere in `(0.08, 0.10]` and that column would show `0.08` and `0.09` too.
   It never does. So the forward speed has to taper as the target gets close,
   landing the last step just inside the tolerance every time.

What I could not read off the numbers was how the wheel limit should behave
when the controller asks for more than the wheels can give: scale both wheels by
one factor, or clamp each independently?

Rather than pick one and hope, I wrote a throwaway program that ran the
simulator over 384 combinations of everything I was unsure about — forward speed
policy, wheel-limit policy, the 180-degree tie-break, `<` versus `<=` on the
stop test, whether the angular command is clamped, and whether the wheel
conversion divides by the wheel radius — and scored each against all 40 expected
rows.

Exactly one combination scored 40/40. That is the controller in
[src/drive.c](src/drive.c). The search program was scaffolding and is not in the
repo; what it was checking is, in the sense that the four result files have to
match.

The wheel-limit question mattered for that search even though it turned out not
to matter for the answer. Candidates that drive at full speed while turning do
blow through the limit — I measured the constant-speed candidate peaking at
11.80 rad/s against a limit of 10.0, clamping on 143228 steps — and the two
limiting policies send those candidates down visibly different paths. The
controller that actually matched never gets near the limit, because gating
forward speed by `cos(heading error)` means it is barely driving forward at the
moment it is turning hardest: measured across all 40 drives its peak commanded
wheel velocity is **7.4547 rad/s** and the limiter clamps on **zero** steps. So
in the shipped code the limiter is a safety net rather than something these
testcases exercise. I kept proportional scaling because clamping the wheels
independently changes the ratio between them, and that ratio is the turn radius.

The tie-break deserves a note, because it is the one place I knowingly did not
reuse a provided helper. Testcase 2 rows 2 and 4 are targets due *west* and both
outputs end up slightly *south* (`-0.09`, `-0.10`). An exact about-turn is a
tie: `+PI` and `-PI` are the same heading error and the two arcs are equally
short. `normalize_angle()` folds onto `(-PI, PI]`, which puts that tie exactly on
the boundary, where the direction the rover spins depends on rounding. The
expected output resolves it clockwise, so the controller folds the *heading
error* onto `[-PI, PI)` in its own helper. This is the only detail I fitted to
their data rather than derived; the controller reaches any target either way.
`normalize_angle()` itself is untouched — it is marked as a provided simulator
helper and it still integrates the rover's heading.

### Step 4: the threading, and the duplicate problem

One producer, three consumers, one shared buffer, and the TODOs say twice to
make sure a message is handled only once. Three things go wrong if you write the
obvious version:

- **Duplicates.** Signal a condition variable three consumers are waiting on and
  all three wake, read the same buffer, and queue the same coordinate three
  times.
- **Lost messages.** There is exactly one `shared_buffer`. If the producer may
  write the next coordinate as soon as it has signalled, it can overwrite a
  message nobody has read.
- **Order.** Nobody states it, but the expected results enforce it:
  `result1.txt` line 3 must correspond to `testcase1.txt` line 3. Three
  consumers racing will not preserve that on their own.

All three fall to one mechanism — three generation counters under
`message_mutex`. The producer publishes and bumps `message_generation`;
whichever consumer wins the wake-up raises `claimed_generation`, which sends the
other two back to waiting instead of reprocessing; that consumer raises
`consumed_generation` when it has finished reading, releasing the producer to
reuse the buffer. Ordering then follows for free, because message *n* is fully
through the shared buffer before *n+1* enters it.

Worth being straight about the trade-off: this puts producer and consumers in
lock-step, so the three consumers buy no throughput. With a single shared buffer
and results that must come out in input order, no other design buys it either —
you would need a ring of buffers plus a resequencer at the end. I kept the
architecture the test asked for and made it correct.

## Implementation

### `src/en_dc.c` — Task 1

Both functions rewritten as actual COBS. The encoder reserves a code byte,
copies non-zero bytes, and back-fills that code byte on hitting a zero or a
254-byte run. The decoder walks block by block and restores the zero each block
stood in for, except after the last block and except after a full `0xFE` block.
Every read is bounded by remaining input and every write by remaining output, so
all four status flags are now reachable and OR-ed the way the header's bit
values imply.

### `src/queue.c` — Task 2

`message_queue_push()` was an empty body. It is now the mirror of the pop that
was already there: wait on `empty`, take the mutex, write at `tail`, advance
modulo the capacity, release, post `full`. The semaphores do the blocking, so
the mutex is never held across a wait, and the existing fixed array is used as a
circular buffer with no extra memory. The `50` was a magic number in two files
and is now `QUEUE_CAPACITY` beside the array it sizes. `current` was declared
but never initialised or maintained; it now tracks live depth. Both entry points
reject NULL.

### `src/mutex.c` — Task 2

The real bug was one character. `reader_enter()` had `if (lock->reader != 1)`
with an empty body, so the **first** reader — the only one that should take
`resource` on behalf of the group — was the one reader that did not, and writers
were never locked out by readers at all.

The `writer_count` turnstile and `reader_exit()` were already correct, so I left
them alone and documented why they are right; "review and correct" also means
not breaking what works. Worth recording that both paths take their locks in the
same order (`writer_count` → `resource`), which is what makes the pair
deadlock-free.

### `src/drive.c` and `lib/drive.h` — Task 3

`drive_to_target()` rejects NULL and non-finite inputs, then loops on the
simulator's timestep: measure the vector to the target and stop inside
tolerance; take the bearing with `atan2f(north, east)`; fold the heading error
onto `[-PI, PI)`; command an angular velocity proportional to it and clamped to
`MAX_ANGULAR_VELOCITY`; scale forward speed by `cos(heading error)` floored at
zero so the rover never reverses while turning, and cap it at the distance
remaining so it does not overshoot the tolerance band. That `(v, w)` pair is
inverted into wheel velocities, both scaled by a common factor if either exceeds
`MAX_WHEEL_VELOCITY`. The loop is bounded by `MAX_DRIVE_STEPS`, so it always
terminates.

`lib/drive.h` needed fixing too. It declared six helpers `static` in a header
`main.c` includes — a promise every including file makes and none can keep — and
one of them (`normlize_angle`) was spelled differently from the function in
`drive.c`. Including the original header and nothing else produces six
`-Wall` warnings that are not the candidate's fault. Those declarations are gone
and the helpers are `static` in `drive.c` where they belong. The rover
parameters moved into the header, since both files need `TARGET_TOLERANCE` and
the brief describes that header as the one holding the parameters, and `drive.c`
now includes it instead of redeclaring all four types locally.

`normalize_angle()` and `apply_wheel_velocities()` are byte-for-byte unchanged.

### `src/main.c` — Tasks 2 and 4

All three thread bodies filled in as described above. Also fixed in here:

- `message_mutex` was **never initialised** — only the condition variable was.
  Everything using it was undefined behaviour.
- `drive_write()`'s status logic contradicted itself: the second `if` fired on
  `result_status == DRIVE_REACHED_TARGET`, so success forced `status = 1` and
  broke out of the loop, and its tolerances (`0.7`, `0.07`) matched neither each
  other nor `TARGET_TOLERANCE`.
- `status` was tested after the loop it was declared inside, which does not
  compile.
- `drive_write()` cast its `FileArgs *` to `int *` to read an id out of it.
- `int producers_id[NUM_PRODUCERS] = {1,2,3}` gave three initialisers to a
  one-element array; it and `writer_id` were unused.
- The fixed `for (i = 0; i < 10; i++)` assumed every input file has ten lines.
  All four do, so it would have passed — but an eleventh line would be silently
  dropped and a ninth would hang the program forever on an empty queue. The
  producer now signals end of stream and one zero-length marker is queued, so
  the pipeline follows the file instead of a constant.

### `CMakeLists.txt`

`find_package(Threads REQUIRED)` and a link to `Threads::Threads`. The code uses
pthreads and POSIX semaphores but nothing linked a thread library — that works
by accident on glibc 2.34 and newer, where those symbols moved into libc, and
fails to link on anything older. Plus the test executable and the `ctest`
wiring.

## Verification

I would rather show what I checked than assert that it works.

**Linux, in CI** — [.github/workflows/ci.yml](.github/workflows/ci.yml), green
on every push: gcc 13.3.0 and clang 18.1.3 on Ubuntu, CMake 3.31.6.
**Windows, locally** — GCC 15.1.0 (UCRT64), CMake 4.4.3, Ninja 1.13.2.

1. **Output matches the reference exactly.** All four `result/resultN.txt` are
   byte-for-byte identical to `result/expected_resultN.txt` — `cmp` with no
   arguments, on both platforms — and the program prints `Success` for all four
   inputs. 40 of 40 rows, position and residual error to two decimals.

2. **Builds clean, and stays clean.** Zero warnings under
   `-Wall -Wextra -Wshadow -Wconversion`, and CI builds that set with `-Werror`
   so one cannot creep back in.

3. **No races.** A threading bug that appears one run in ten passes a single run
   happily, so one green run proves very little. 25 consecutive local runs and
   20 more in CI, every one byte-identical, none hanging.

4. **The codec round-trips.** [tests/en_dc_test.c](tests/en_dc_test.c) runs
   **2754 checks**: empty input, an all-zero coordinate payload, the real
   testcase coordinates compared as floats after the round trip, patterns at
   253/254/255/256/507/508/509/1020 bytes to hit the block boundary from both
   sides, 500 randomised payloads biased towards zeros, and the error paths —
   NULL pointers, undersized buffers, a zero byte planted inside a frame, a
   truncated frame. It also asserts the two properties the framing exists for:
   an encoded frame never contains `0x00`, and never exceeds
   `ENCODE_DST_BUF_LEN_MAX`.

5. **The tests actually bite.** A suite that passes on broken code is worthless,
   so I broke the code on purpose. Removing the zero-restore step from the
   decoder: **1068 checks fail**. Putting the original `src_byte == 0xFF` back
   into the encoder: **1379 checks fail**. I applied the same standard to the
   `pipeline` check — corrupted a reference file and confirmed `ctest` goes red.

6. **The wheel limiter was measured, not assumed.** Instrumented the controller
   and drove all 40 targets: peak commanded wheel velocity 7.4547 rad/s against
   a 10.0 limit, limiter clamping on zero steps. It is there for safety, and I
   would rather say that than imply these testcases prove it.

7. **Odd inputs do not hang it.** Input files of 0, 1, 3, 25 and 120 lines. All
   terminated, each writing exactly as many rows as its file had. The 120-line
   run also pushes the queue indices past the 50-element capacity, exercising
   the wrap-around.

## Where I got stuck

- **The 180-degree targets.** I had the controller matching 37 of 40 rows and
  could not see why three were mirrored. It took a while to notice the three
  failures were exactly the targets pointing due west, and that the problem was
  not the controller but a tie between two equally correct answers being broken
  inconsistently.

- **The error column.** I stared at that column of `0.10`s for a long time
  assuming it was a rounding coincidence, before working out that a
  constant-speed rover cannot physically produce it. Until then I had a
  controller that reached every target and matched not one row.

- **The duplicate messages.** My first threading version pushed every coordinate
  three times, once per consumer, and the result file came out with 30 rows.
  Switching from broadcast to signal made it *look* fixed, which was worse — it
  was still a race, it just usually won. The counter a consumer has to claim is
  what actually makes it correct.

- **Not being able to test on Linux.** I built this on Windows and the task is
  POSIX, and for most of the exercise I was reasoning about the Linux build
  rather than running it. That bothered me enough that I wired up CI, which
  turned the assumption into a fact and found nothing — which is the good
  outcome, but I would not have known without checking.

## Issues I found

Beyond the bugs the tasks point at, all verified rather than guessed:

- **Neither image in the brief renders.** Both `<img>` tags use
  `github.com/.../blob/...` URLs, which serve an HTML page, not an image — I
  checked the responses: `text/html` for the `blob` form, `image/webp` and
  `image/jpeg` for the `raw.githubusercontent.com` form. The
  `<p align="center">` above the first was also never closed. My README uses the
  working URLs; [TASK.md](TASK.md) keeps the original verbatim, broken links and
  all, since that is the evidence.
- **The images also live in other repositories.** This repo has no `misc/`
  directory; the brief pulls its artwork from `r26_test` and `r25-test`. If
  either is renamed or made private, every fork of this test loses its images.
- **`lib/drive.h` warns on a bare include** — six `-Wall` warnings from the
  `static` declarations, before a candidate writes a line.
- **`lib/read.h` declares an API nothing implements**: a `Reader` struct and
  `void *reader_thread(void *)` that is defined nowhere in the repo. It looks
  like the skeleton of an alternative design. I left it — removing things from a
  header I was not asked to touch seemed worse than leaving them.
- **`input_file_read()` returns a comparison**, so it is a "did I get two
  floats" flag rather than the count its `int` return suggests. Used correctly
  as a loop condition, so I left it alone.

## Suggestions for the test repo

Offered as someone who just spent a while inside it, not as criticism.

1. **Bring back a `check` step.** The R26 test had `make check` printing
   `*** Success: ***`; this one has candidates diffing four files by hand, and a
   candidate who is nearly right cannot easily tell. The
   [ctest wiring](cmake/run_pipeline_test.cmake) here is about forty lines and
   drops into the existing CMake with no new dependency — take it if it is
   useful.
2. **Put CI on the reference solution.** None of the six Rudra repos I looked at
   have a `.github/workflows` directory. The skeleton is meant not to compile,
   so CI on `main` would be wrong — but a private branch holding the reference
   solution, built and diffed against `expected_result*.txt` on every push,
   would tell you next August that the test still works before candidates fork
   it. Twenty-six people forked this one.
3. **Vendor the images** into `misc/` and use `raw.githubusercontent.com` URLs,
   so the brief does not depend on two other repositories staying put.
4. **Fix `lib/drive.h`.** The `static`-in-a-header declarations hand every
   candidate six warnings that are not theirs, and `normlize_angle` versus
   `normalize_angle` is a spelling trap with no teaching value.
5. **Consider stating the ordering requirement.** That `resultN.txt` line *k*
   must match `testcaseN.txt` line *k* is the constraint that makes the
   three-consumer design genuinely hard, and it is only discoverable by
   inference. It is a good requirement — it is worth being explicit that it is
   one.

## What I would want to work on at Rudra

Reading the org's public repos, the through-line is clear: `core` is C on a
Raspberry Pi talking to hardware over raw I2C, SPI, UDP and serial with Arduino
sketches on the other side (including `arduino/diff_drive`), `arrow_infer` is
ROS2 and YOLOv5 for detection, and the R24 test shipped Unity unit tests. This
test is a small version of the same stack: bytes over a link, threads passing
them safely, and a differential drive at the end.

Concretely, where I think I could be useful:

- **A framed link between the Pi and the microcontrollers.** `core/raw/SERIAL`
  moves bytes; what I implemented here is the layer above it — COBS framing that
  makes `0x00` a reliable packet boundary, which is exactly what you want on a
  noisy serial line where a rover can drop a byte and needs to resynchronise on
  the next frame rather than mis-parse everything after it. Adding a checksum
  and a sequence number to what is in `src/en_dc.c` is a small step from here.
- **The reliability layer.** No repo in the org has CI, and the unit tests in
  the R24 test did not carry forward. For a team where a failure costs a
  competition run rather than a rebuild, that is the cheapest reliability there
  is. I would start by making `core` build in CI at all, then add tests around
  the parsing and control paths, which is the same thing I did to this repo
  today.
- **Autonomy.** `arrow_infer` is the part I would most like to work on, and the
  controller in `src/drive.c` is the primitive that sits under it — a detection
  gives you a target, and something has to turn that target into wheel
  velocities without overshooting or spinning the wrong way round.

I am aware Rudra is in the 2026 URC finals, and that a recruitment test is not
the same as doing the work. But the way I approached this — read it all first,
treat the expected output as a specification, then prove each claim rather than
assert it — is how I would want to work on something that has one shot at a
field run.

## AI, Google and other resources

Instruction 6 asks for this, so being specific rather than vague.

- **AI assistant.** Used throughout, and it did real work. Concretely:
  confirming my reading of the COBS block-splitting rule and the `0xFE` special
  case; reviewing the reader-writer and producer-consumer handshake for
  deadlocks and lost wake-ups, which is where "the producer must wait until the
  message is consumed" came from; helping build the brute-force search that
  recovered the drive controller from the expected results; and reviewing the
  finished code. Two things in this README came directly out of that review —
  the mutation testing in Verification point 5, on the argument that a test
  suite I had not tried to break was not yet evidence of anything, and the
  correction in point 6, because I had claimed the wheel limiter fires on most
  rows and measuring it showed it never fires at all.
- **Reference documentation** for COBS, and for the POSIX semantics of
  `sem_init`, `pthread_cond_wait` and spurious wake-ups.
- **The classic readers-writers problem** for the turnstile pattern already in
  `mutex.c` — I wanted to confirm the given `writer_count` structure was the
  standard writer-starvation fix before concluding it was correct and leaving it
  alone.
- **Rudra's own repositories** — `r25-test`, `r26_test`, the two R24 tests,
  `core` and `arrow_infer` — read for the Suggestions section above and to see
  what the R26 `make check` did before this test dropped it.

What I did not do is paste the task in and commit whatever came back. Every
claim above was checked by running something: the results are diffed against the
reference on two platforms, the codec is fuzzed, and the tests were verified to
fail on broken code before I trusted them to pass on this one.
