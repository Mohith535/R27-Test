# R27 Test

<p align="center">
  <img src="https://raw.githubusercontent.com/teamrudra/r26_test/main/misc/rover.webp" width="480" height="480"/>
</p>

#### Some Instructions
1. You may use any online resources, datasheets, or documentation needed, but be mindful of your time and stay focused on the task.
2. The duration of the test is 90 mins from 5:15pm to 6:45 pm.
3. There will be a MCQ test conducted [here](https://rudra-test.vercel.app/)
4. There are 5 tasks in the tests. Complete all of them.
5. In case you are not able to complete all the tasks, do upload whatever you are able to.
6. In the `README.md` of your repository include your thought process, places where you got stuck, where you used the help of AI, google or other online resources.
7. Even if you are not able to solve anything, do fill the README and what your thought process would have been.
8. Carefully read the instructions to implement the required functionality.
9. Install a c compiler and [git](https://git-scm.com/downloads) if you haven't already done it.
10. After finishing your test, provide the link to your forked repository in the google form provided at the end.

### Aim/Objective: To build a communication system that safely transfers, processes and decodes messages between threads and uses the received coordinates to control a rover. 

## Description
This test evaluates your ability to understand, debug, and implement functionality in an existing C-based embedded/robotics code-base. The test has five tasks. You are given an existing code-base with partially implemented functionality. Your task is to understand the code, identify the issues, implement the required changes, and verify your solution.

### Task 0: Fork the provided repository and ensure it is set to PUBLIC so we can access and assess your work.
### Task 1: Implement encoding and decoding functions for embedded communication.
Fixed incorrect logic in the encoder and decoder, handled buffer limits and edge-case inputs, and verify that decoding returns the original data.
### Task 2: Manage multi-threading and synchronization using POSIX threads.
Review and correct the existing implementation of mutexes, semaphores, and message queues while maintaining the core architecture.
### Task 3: Control a differential-drive rover to navigate to a target.
Complete the drive-to-target functionality to calculate direction, generate appropriate left/right wheel velocities, and handle heading wraparound.
### Task 4: Compile and run the code.
Verify the workflow on the provided rover simulator and ensure the project compiles successfully.

#### Code
1. [src/main.c](src/main.c): Code for running the multi-threading and synchronization architecture.
2. [src/en_dc.c](src/en_dc.c): Rectify errors in this code to correctly encode/decode data and handle invalid inputs.
3. [lib/en_dc.h](lib/en_dc.h): Header file containing declarations for the encoding and decoding logic.
4. [src/queue.c](src/queue.c): Correct the existing message queue implementation.
5. [src/mutex.c](src/mutex.c): Review and fix POSIX mutex and semaphore logic.
6. [src/drive.c](src/drive.c): Complete the defined `drive_to_target()` function to guide the rover.
7. [lib/drive.h](lib/drive.h): Header file containing parameters and declarations for rover control.

## Build the project:

(make sure you are in the root directory)

```
cmake -S . -B build
```

```
cmake --build build --verbose
```

```
./build/queue_test
```

To clean the build:

```
rm -rf build
```

# Solution

All five tasks are done. The project configures and builds clean, and the four
result files the program writes are byte-for-byte identical to the four
`expected_result*.txt` files that ship with the repo.

## Understanding

The repo is one pipeline cut into stages that hand data to each other across
threads. Nothing in it is standalone — a bug in the framing shows up as a rover
that drives to the wrong place, which is what made me read all of it before
changing any of it.

```
input/testcaseN.txt
   |
   |  producer thread    read "x y", pack the two floats, frame them
   v
shared_buffer            one buffer, guarded by a reader-writer lock
   |
   |  3 consumer threads  one of them claims each message, unframes it
   v
Message_Queue            bounded circular buffer, 2 semaphores + 1 mutex
   |
   |  drive thread        pop, drive the rover to it, write one row
   v
result/resultN.txt
```

So there are really three different synchronisation problems here, and the
skeleton deliberately uses a different primitive for each one:

- one shared buffer that many threads may read but only one may write ->
  reader-writer lock (`src/mutex.c`);
- a "there is a new message" event -> mutex + condition variable + a
  generation counter (`src/main.c`);
- a bounded hand-off queue -> counting semaphores + a mutex (`src/queue.c`).

**`src/en_dc.c` is the transport layer, and it is COBS.** I did not have to
guess that: `lib/en_dc.h` gives a worst-case output size of
`SRC_LEN + ceil(SRC_LEN/254)`, uses 254 and 0xFF as its magic numbers, and
defines a `DECODE_ZERO_BYTE_IN_INPUT` status. That is the signature of
Consistent Overhead Byte Stuffing: it removes every `0x00` from the payload and
replaces it with the distance to the next one, so `0x00` stays reserved as a
frame delimiter and one byte of overhead is added per 254 bytes of payload.

That matters more here than it first looks. A coordinate of `0.0 0.0` is eight
zero bytes, and `0.0 0.0` appears in every single testcase. Without the framing,
the most common message in the whole test would be indistinguishable from an
empty frame.

**`src/drive.c` is a differential-drive rover.** You hand it a left and a right
wheel velocity and it integrates a unicycle model on a fixed 20 ms timestep.
Latitude is the north axis, longitude is the east axis, heading 0 points east
and grows counter-clockwise. Reading `apply_wheel_velocities()` backwards gives
the two equations the controller has to invert:

```
v = R (wl + wr) / 2          w = R (wr - wl) / L
```

## Thought Process

### Step 1: get it to fail before trying to make it pass

First thing I did was try to build the untouched repo, so I would know what I
was actually looking at rather than what I assumed. It does not compile:
`frame_encode()` uses a `dst_code_write_ptr` that is never declared, and
`drive_write()` in `main.c` uses `coordinate_target`, `rover`, `result_status`
and `status` which do not exist. So this is not a "find the subtle bug" test —
several stages are simply missing, and the parts that are present have real
logic errors hiding behind the parts that are absent.

### Step 2: en_dc.c, once I knew it was COBS

Once I had the algorithm name, each bug was a specific deviation from it rather
than a puzzle:

| Where | What was wrong |
|---|---|
| `frame_encode` | `dst_buf_end_ptr` was set to `dst_buf_start_ptr`, so the buffer looked zero bytes long and every write was "past the end" |
| `frame_encode` | `dst_write_ptr` started at `dst_code_write_ptr + 3`, leaving a 3-byte hole; COBS reserves exactly one byte for the code |
| `frame_encode` | the loop was `for (i = 0; i < search_len; i++)` — a byte counter used as a loop bound, so it walked off the source |
| `frame_encode` | it tested `src_byte == 0xFF` and did nothing in that branch. The byte COBS cares about is `0x00`, and that branch is where the run gets closed |
| `frame_encode` | the block-split test was `search_len == 0` instead of `search_len == 0xFF`, so a 254-byte run of non-zero bytes was never split |
| `frame_encode` | no output-space check inside the loop at all |
| `frame_decode` | the loop was `for (i = 0; i < len_code; i++)` with `len_code` **read uninitialised** on the first test |
| `frame_decode` | the inner copy loop read the source byte and threw it away — it never wrote anything to the destination, so `out_len` was always 0 |
| `frame_decode` | no logic to put back the zero byte that encoding removed, and no handling of the `0xFE` full-block case where there was no zero to put back |

The `0xFE` case is the one that is easy to get wrong and impossible to notice on
short messages: when the encoder splits a 254-byte run purely because a length
code cannot count higher, there was no zero there, so the decoder must not
invent one. An eight-byte coordinate never reaches that path, which is exactly
why I did not want to trust the four rover testcases to prove the codec.

### Step 3: drive.c, where there was no specification

This was the interesting one. The comment says "path planning and PID are not
required" and lists constraints, but the exact controller is not specified —
and yet `result/expected_result*.txt` pins down 40 rows of output to two
decimal places. So the expected results *are* the specification, and the honest
way to write this function was to read them rather than to guess and hope.

Four things fell straight out of the numbers:

1. **Testcase 1 row 1** — target `0.0 0.0`, output `0.00 0.00 0.00 0`. The rover
   starts at the origin and the distance test runs *before* the first step,
   otherwise it would have moved.

2. **Testcase 3 rows 1 and 6** — targets `1.0 0.0` and `-1.0 0.0`, outputs
   `0.97 0.09` and `-0.97 0.09`. Mirror-image targets, and the sideways drift is
   `+0.09` in **both**. That only happens if the rover always starts pointing
   east and curves onto the target while driving. If it spun on the spot first,
   the drift would be zero; if the heading carried over between messages, the
   two rows could not be symmetric. So the rover is reset to the origin, facing
   east, for every message.

3. **Testcase 2 row 1** — target `0.0 0.5`, straight ahead, output `0.00 0.40`.
   Exactly `0.5 - 0.10`. Full speed in a straight line, stopping the moment the
   remaining distance hits `TARGET_TOLERANCE`.

4. **The error column is `0.10` on every single row that is not already at the
   target.** This is the one that told me the most. At full speed the rover
   covers `1.0 * 0.02 = 0.02` per step, so a constant-speed controller would
   stop somewhere in `(0.08, 0.10]` and that column would show `0.08` and `0.09`
   as well. It never does. So the forward speed has to taper as the target gets
   close, which lands the final step just barely inside the tolerance every
   time.

The one thing I could not read off the numbers was how the wheel limit should
behave when the controller asks for more than the wheels can give: scale both
wheels by the same factor, or clamp each one on its own?

Rather than pick one and hope, I wrote a throwaway program that ran the
simulator over 384 combinations of the choices I was unsure about — forward
speed policy, wheel-limit policy, the 180-degree tie-break, `<` vs `<=` on the
stop test, whether the angular command is clamped, and whether the wheel
conversion divides by the wheel radius — and scored each combination against
all 40 expected rows.

Exactly one combination scored 40/40. That is the controller in `src/drive.c`.
(The search program was scaffolding and is not in the repo; the assertion it was
checking is, in the sense that the four result files have to match.)

The wheel-limit question mattered for that search even though it turned out
not to matter for the answer. Candidates that drive at full speed while they turn do blow through the
limit — I measured the constant-speed candidate peaking at 11.80 rad/s against
a limit of 10.0, clamping on 143228 steps — and the two limiting policies send
those candidates down visibly different paths, so the search could not ignore
the question. The controller that actually matched never gets near the limit,
because gating the forward speed by `cos(heading error)` means it is barely
driving forward at the moment it is turning hardest: measured across all 40
drives its peak commanded wheel velocity is **7.4547 rad/s** and the limiter
clamps on **zero** steps. So in the final code the limiter is a safety net
rather than something these testcases exercise, and I kept proportional scaling
because independent clamping would change the ratio between the wheels, which
is the turn radius.

The tie-break deserves a note, because it is the only place I knowingly did not
reuse a provided helper. Testcase 2 rows 2 and 4 are targets due *west*, and
both outputs end up slightly *south* (`-0.09`, `-0.10`). An exact about-turn is
a tie: `+PI` and `-PI` are the same heading error and the two arcs are equally
short. `normalize_angle()` folds onto `(-PI, PI]`, which puts that tie on the
boundary where the direction the rover spins depends on rounding. The expected
output resolves it clockwise, so the controller folds the *heading error* onto
`[-PI, PI)` in its own helper. `normalize_angle()` itself is untouched — it is
marked as a provided simulator helper and it is still what integrates the
rover's heading.

### Step 4: the threading, and the duplicate problem

The skeleton has one producer, three consumers and one shared buffer, and the
TODOs say twice to make sure a message is only handled once. Two things go
wrong if you write the obvious version:

- **Duplicates.** Signal a condition variable that three consumers are waiting
  on and all three wake up, read the same buffer, and push the same coordinate
  into the queue three times.
- **Lost messages.** There is exactly one `shared_buffer`. If the producer is
  free to write the next coordinate as soon as it has signalled, it can
  overwrite a message before any consumer has read it.

There is also a requirement nobody states but the expected results enforce:
`result1.txt` line 3 has to correspond to `testcase1.txt` line 3. Three
consumers racing each other will not preserve that on their own.

I solved all three with one mechanism — three generation counters under
`message_mutex`. The producer publishes and bumps `message_generation`; whichever
consumer wins the wake-up raises `claimed_generation`, which makes the other two
go back to waiting instead of processing the same message; that consumer raises
`consumed_generation` when it has finished reading, which is what releases the
producer to reuse the buffer. Ordering then follows for free, because message
*n* is fully through the shared buffer before message *n+1* enters it.

It is worth being straight about the trade-off: this makes the producer and the
consumers run in lock-step, so the three consumers do not give a throughput win.
With one shared buffer and results that have to come out in input order, that is
not something a different design would have bought either — you would need a
ring of buffers plus a resequencer at the end to get real overlap. I kept the
architecture the test asked for and made it correct.

## Implementation

### `src/en_dc.c` — Task 1

Rewrote both functions to be actual COBS. The encoder reserves a code byte,
copies non-zero bytes, and back-fills the code byte when it hits a zero or a
254-byte run; the decoder walks block by block and puts back the zero that each
block stood in for, except after the last block and except after a full `0xFE`
block. Every read is bounded by the remaining input and every write by the
remaining output, and all four status flags (`NULL_POINTER`,
`OUT_BUFFER_OVERFLOW`, `ZERO_BYTE_IN_INPUT`, `INPUT_TOO_SHORT`) are now
actually reachable and OR-ed in the way the header's bit values imply.

### `src/queue.c` — Task 2

`message_queue_push()` was an empty body. It is now the mirror of the pop that
was already there: wait on `empty`, take the mutex, write at `tail`, advance it
modulo the capacity, release, post `full`. The semaphores do the blocking and
the mutex only covers the few lines that touch the indices, so the mutex is
never held across a wait. The `50` was a magic number in two files, so it is now
`QUEUE_CAPACITY` in `lib/read.h` next to the array it describes. `current` was
declared but never initialised or maintained; it now tracks the live depth.
Both entry points reject NULL.

### `src/mutex.c` — Task 2

The real bug was one character: `reader_enter()` had `if (lock->reader != 1)`
with an empty body, so the **first** reader — the only one that should acquire
`resource` on behalf of the group — was the one reader that did not. Writers
were never locked out by readers at all. Inverted to `== 1` and acquired there.

The `writer_count` turnstile around reader entry and `reader_exit()` were
already right, so I left them alone and documented why they are right, since
"review and correct" also means not breaking what works. Worth recording that
the two paths take their locks in the same order (`writer_count` -> `resource`),
which is what makes the pair deadlock-free.

### `src/drive.c` and `lib/drive.h` — Task 3

`drive_to_target()` validates its inputs (NULL and non-finite both rejected),
then loops on the simulator's own timestep: measure the vector to the target and
stop if it is inside tolerance; take the bearing with `atan2f(north, east)`;
fold the heading error onto `[-PI, PI)`; command an angular velocity
proportional to that error and clamped to `MAX_ANGULAR_VELOCITY`; scale the
forward speed by `cos(heading_error)` floored at zero, so the rover drives
forward only to the extent it is already aimed correctly and never reverses
while turning; and cap that speed at the distance remaining, which is what stops
it overshooting the tolerance band. The `(v, w)` pair is then inverted into the
two wheel velocities and both are scaled by a **common** factor if either
exceeds `MAX_WHEEL_VELOCITY` — clamping them independently would change the
ratio between them, and that ratio is the turn radius, so the rover would
quietly curve differently from what was commanded exactly when it is turning
hardest. The loop is bounded by `MAX_DRIVE_STEPS`, so it always terminates.

`lib/drive.h` needed fixing too. It declared six helpers `static` in a header
that `main.c` includes, which is a promise every including file makes and none
of them kept, and one of them (`normlize_angle`) was spelled differently from
the function in `drive.c` anyway. Those declarations are gone and the helpers
are `static` in `drive.c` where they belong. The rover parameters moved into the
header, since `drive.c` and `main.c` both need `TARGET_TOLERANCE` and the header
is described as the one holding the parameters. `drive.c` now includes
`drive.h` instead of re-declaring all four types locally, so there is one
definition of `struct coordinate` rather than two that happened to agree.

`normalize_angle()` and `apply_wheel_velocities()` are byte-for-byte unchanged.

### `src/main.c` — Tasks 2 and 4

Filled in all three thread bodies. The producer packs the two floats, frames
them, publishes under `writer_enter`/`writer_exit`, then blocks until a consumer
has taken the message. Consumers claim a generation, read under
`reader_enter`/`reader_exit`, unframe, and push exactly one coordinate to the
queue. The drive thread pops, drives from a fresh rover state, and writes the
row.

Other things that were wrong in there:

- `message_mutex` was **never initialised** — only the condition variable was.
  Everything that used it was undefined behaviour.
- `drive_write()`'s status logic was self-contradicting: the second `if` fired
  on `result_status == DRIVE_REACHED_TARGET`, so success forced `status = 1` and
  broke out of the loop, and the tolerances in it (`0.7`, `0.07`) matched
  neither each other nor `TARGET_TOLERANCE`. Now a row is a pass when the drive
  reports it reached the target and the residual is within `TARGET_TOLERANCE`.
- `status` was tested after the loop it was declared inside, which does not
  compile.
- `drive_write()` cast its `FileArgs *` to `int *` to read an id out of it.
- `int producers_id[NUM_PRODUCERS] = {1,2,3}` with `NUM_PRODUCERS` at 1 is three
  initialisers for a one-element array. It and `writer_id` were unused; removed.
- The fixed `for (i = 0; i < 10; i++)` in the drive thread assumes every input
  file has exactly ten lines. All four do, so it would have passed the test —
  but an eleventh line would be silently dropped and a ninth would hang the
  program forever on an empty queue. The producer now signals end-of-stream and
  a single zero-length marker message is pushed for the drive thread, so the
  pipeline follows the file instead of a constant. I tested this with files of
  0, 1, 3, 25 and 120 lines.
- Added `pthread_mutex_init`/`destroy` for `message_mutex`, `pthread_cond_destroy`,
  and a `return 0`.

### `CMakeLists.txt`

Added `find_package(Threads REQUIRED)` and linked `Threads::Threads`. The code
uses pthreads and POSIX semaphores but nothing linked a thread library — that
works by accident on glibc 2.34 and newer, where those symbols moved into libc,
and fails to link on anything older. Also added a second executable for the
framing tests. `queue_test` still builds and runs exactly as the instructions
describe.

## Verification — Task 4

Toolchain: GCC 15.1.0 (UCRT64), CMake 4.4.3, Ninja 1.13.2.

```
cmake -S . -B build
cmake --build build --verbose
./build/queue_test          # the pipeline
./build/en_dc_test          # the framing round-trip tests
```

What I actually checked, and what it said:

1. **Builds clean.** Not just with the default flags — also with
   `-Wall -Wextra -Wshadow -Wconversion`, which reports **0 warnings** across
   all six translation units.

2. **Output matches the reference exactly.** All four `result/resultN.txt` are
   byte-for-byte identical to `result/expected_resultN.txt` (`cmp` reports no
   difference), and the program prints `Success` for all four inputs. That is
   40 of 40 rows, position and residual error to two decimals.

3. **No races.** Threading bugs that only show up sometimes are the whole
   difficulty of this kind of code, so one passing run proves very little. I ran
   the binary **25 times in a row**; all 25 produced byte-identical output and
   four `Success` lines, with no hang.

4. **The codec round-trips.** `tests/en_dc_test.c` runs **2754 checks** and all
   pass: empty input, an all-zero coordinate payload, the real coordinates from
   the testcases compared as floats after the round trip, patterns at 253, 254,
   255, 256, 507, 508, 509 and 1020 bytes to hit the block-split boundary from
   both sides, 500 randomised payloads biased towards zeros, plus the error
   paths — NULL pointers, undersized output buffers, a zero byte planted inside
   a frame, and a truncated frame. It also asserts the two properties the
   framing exists for: an encoded frame never contains `0x00`, and it never
   exceeds `ENCODE_DST_BUF_LEN_MAX`.

5. **The tests actually bite.** A test suite that passes on broken code is
   worthless, so I checked that these fail on code I deliberately broke. Removing
   the zero-restore step from the decoder: **1068 checks fail**. Putting the
   original `src_byte == 0xFF` back into the encoder: **1379 checks fail**.

6. **The wheel limiter was measured, not assumed.** Instrumented the controller
   and drove all 40 targets: peak commanded wheel velocity 7.4547 rad/s against
   a 10.0 limit, with the limiter clamping on zero steps. It is there for
   safety, and I would rather say that than imply the testcases prove it works.

7. **Odd inputs do not hang it.** Ran the pipeline against input files of 0, 1,
   3, 25 and 120 lines. Every one terminated and wrote exactly as many rows as
   the file had; the 120-line run also takes the queue indices past the
   50-element capacity, which exercises the wrap-around.

## Where I got stuck

- **The 180-degree targets.** I had the controller matching 37 of the 40 rows
  and could not see why three were mirrored. It took me a while to realise the
  three failures were exactly the targets pointing due west, and that the
  problem was not the controller at all but a tie between two equally correct
  answers being broken inconsistently.

- **The error column.** I stared at that column of `0.10`s for a long time
  assuming it was a coincidence of rounding before working out that a
  constant-speed rover physically cannot produce it, which is what forced the
  speed to taper near the target. Before that I had a controller that reached
  every target and still did not match a single row.

- **The duplicate messages.** My first threading version pushed every coordinate
  three times, once per consumer, and the result file came out with 30 rows.
  Signalling instead of broadcasting made it *look* fixed, which was worse,
  because it was still a race — it just usually won. The counter that a consumer
  has to claim is the part that actually makes it correct.

- **Working on Windows.** The task is POSIX. MSYS2/UCRT64 has winpthreads, so
  `pthread.h` and `semaphore.h` are both there and everything built and ran
  natively, but I checked the thread-library linking question carefully rather
  than assume what works here works on the machine this gets marked on.

## Issues I found

The instructions ask whether there is anything else worth fixing, so:

- **The two images in this README do not render.** Both `<img>` tags point at
  `github.com/.../blob/...` URLs. Those serve an HTML page, not an image — I
  checked, and they come back as `text/html`, which is why the tags show as
  broken. The `raw.githubusercontent.com` form of the same two URLs returns
  `image/webp` and `image/jpeg`. I have switched them over, and closed the
  `<p align="center">` above the first one, which was never closed.
- `lib/read.h` declares a `Reader` struct and a `reader_thread()` that nothing
  defines or uses. I left them alone — they look like the skeleton of an
  alternative design and removing things from a header nobody asked me to touch
  seemed worse than leaving them.
- `input_file_read()` returns the result of a comparison, so it is a
  "did I get two floats" flag rather than the `int` return the name suggests.
  It is used correctly as a loop condition, so I left it.
- `message_destroy()` is named inconsistently with the rest of the
  `message_queue_*` family. Not worth an API change mid-test.

## AI, Google and other resources

Per instruction 6, being specific about this rather than vague:

- **AI assistant.** Used throughout, and it did real work. Concretely: confirming
  my reading of the COBS block-splitting rule and the `0xFE` special case;
  reviewing the reader-writer and producer-consumer handshake for deadlocks and
  lost wake-ups, which is where the "producer must wait until consumed" point
  came from; helping build the brute-force search that recovered the drive
  controller from the expected results; and reviewing the finished code. The
  mutation testing in point 5 above came out of that review — the suggestion
  that a test suite I had not tried to break was not yet evidence of anything.
- **Reference documentation** for COBS, and for the POSIX semantics of
  `sem_init`, `pthread_cond_wait` and spurious wake-ups.
- **The classic readers-writers problem** for the turnstile pattern already
  present in `mutex.c` — I wanted to confirm the given `writer_count` structure
  was the standard writer-starvation fix before deciding it was correct and
  leaving it.

What I did not do is paste the task in and commit whatever came back. Every
piece here was checked by running it: the results are diffed against the
reference, the codec is fuzzed, and the tests were verified to fail on broken
code before I trusted them to pass on this one.

**Good luck!**
# Google Form
https://forms.gle/A8CaByv4ohfrCmmWA

<p align="center">
  <img src="https://raw.githubusercontent.com/teamrudra/r25-test/main/datasheets/feynman-simple.jpg" width="600" height="600"/>
</p>
