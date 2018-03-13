# MemoryBenchmarkLinux

Memory benchmark for Linux 32/64.
GCC and FASM used for translation.
Use Makefile_32 as Makefile when build for ia32 platforms,
Use Makefile_64 as Makefile when build for x64 platforms.

run:

"./memorybench [options]"

options list:

operation = memory operation, values: read, write, copy

method = select CPU instruction set, yet available: sse128 (ia32, x64), avx256 (for x64 only)

min = minimum block size, starts from this value: numeric value + K/M/G

max = maximum block size, stops at this value: numeric value + K/M/G

step = step for block size, adds this value: numeric value + K/M/G

pages = paging option, values: normal, huge

nontemporal = select data usage mode, values: 0 or 1 (NOT SUPPORTED YET)

threads = select number of execution threads: numeric value (NOT SUPPORTED YET)

precision = time or precision priority, values: fast, slow (NOT SUPPORTED YET)

machinereadable = make output machine readable (hex data), values: 0 or 1 (NOT SUPPORTED YET)


run examples (default and custom):

"./memorybenchmark"

"./memorybenchmark min=1M max=1M pages=huge"

One test with 1 Mbyte file

Use huge pages option.









