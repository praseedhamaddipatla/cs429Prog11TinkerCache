#include "tcache.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

/* ---------------------------------------------------------------
 * Helpers
 * ------------------------------------------------------------- */

/* write_memory / read_memory are provided by tcache_backend.c */
extern uint8_t memory[HW11_MEM_SIZE];

static void reset_memory(void) {
    memset(memory, 0, HW11_MEM_SIZE);
}

static void print_stats(const char *label) {
    cache_stats_t li = get_l1_instr_stats();
    cache_stats_t ld = get_l1_data_stats();
    cache_stats_t l2 = get_l2_stats();

    printf("  [%s]\n", label);
    printf("    L1-I  accesses=%-8llu misses=%-6llu miss-rate=%.2f%%\n",
           (unsigned long long)li.accesses, (unsigned long long)li.misses,
           li.accesses ? 100.0 * li.misses / li.accesses : 0.0);
    printf("    L1-D  accesses=%-8llu misses=%-6llu miss-rate=%.2f%%\n",
           (unsigned long long)ld.accesses, (unsigned long long)ld.misses,
           ld.accesses ? 100.0 * ld.misses / ld.accesses : 0.0);
    printf("    L2    accesses=%-8llu misses=%-6llu miss-rate=%.2f%%\n",
           (unsigned long long)l2.accesses, (unsigned long long)l2.misses,
           l2.accesses ? 100.0 * l2.misses / l2.accesses : 0.0);
}

typedef struct {
    uint64_t l1d_misses;
    uint64_t l2_misses;
} miss_result_t;

static miss_result_t run_data_pattern(const uint64_t *pattern, int pattern_len,
                                      int repeats, replacement_policy_e policy,
                                      unsigned seed) {
    reset_memory();
    init_cache(policy);

    /* The cache library seeds RANDOM deterministically for grader stability.
     * For comparison experiments, override that seed so multiple Random runs
     * actually sample different victim choices. */
    if (policy == RANDOM) {
        srand(seed);
    }

    for (int rep = 0; rep < repeats; rep++) {
        for (int i = 0; i < pattern_len; i++) {
            read_cache(pattern[i], DATA);
        }
    }

    miss_result_t result = {
        .l1d_misses = get_l1_data_stats().misses,
        .l2_misses = get_l2_stats().misses
    };
    return result;
}

static miss_result_t average_random_pattern(const uint64_t *pattern, int pattern_len,
                                            int repeats, int runs,
                                            unsigned seed_base) {
    miss_result_t total = {0, 0};

    for (int r = 0; r < runs; r++) {
        miss_result_t run = run_data_pattern(pattern, pattern_len, repeats,
                                             RANDOM, seed_base + (unsigned)r);
        total.l1d_misses += run.l1d_misses;
        total.l2_misses += run.l2_misses;
    }

    miss_result_t avg = {
        .l1d_misses = total.l1d_misses / (uint64_t)runs,
        .l2_misses = total.l2_misses / (uint64_t)runs
    };
    return avg;
}

static miss_result_t run_data_scan(uint64_t base, uint64_t bytes, int passes,
                                   replacement_policy_e policy, unsigned seed) {
    reset_memory();
    init_cache(policy);

    if (policy == RANDOM) {
        srand(seed);
    }

    for (int p = 0; p < passes; p++) {
        for (uint64_t i = 0; i < bytes; i++) {
            read_cache(base + i, DATA);
        }
    }

    miss_result_t result = {
        .l1d_misses = get_l1_data_stats().misses,
        .l2_misses = get_l2_stats().misses
    };
    return result;
}

static miss_result_t average_random_scan(uint64_t base, uint64_t bytes, int passes,
                                         int runs, unsigned seed_base) {
    miss_result_t total = {0, 0};

    for (int r = 0; r < runs; r++) {
        miss_result_t run = run_data_scan(base, bytes, passes,
                                          RANDOM, seed_base + (unsigned)r);
        total.l1d_misses += run.l1d_misses;
        total.l2_misses += run.l2_misses;
    }

    miss_result_t avg = {
        .l1d_misses = total.l1d_misses / (uint64_t)runs,
        .l2_misses = total.l2_misses / (uint64_t)runs
    };
    return avg;
}

/* ---------------------------------------------------------------
 * TEST 1 – Basic read/write correctness
 *
 * Write a value via write_cache, read it back via read_cache.
 * Verify the returned byte matches.
 * ------------------------------------------------------------- */
static void test_basic_rw(void) {
    printf("\n=== TEST 1: Basic read/write correctness ===\n");

    for (int policy = 0; policy <= 1; policy++) {
        reset_memory();
        init_cache(policy ? RANDOM : LRU);

        /* write a known pattern into 256 bytes at address 0x1000 */
        for (int i = 0; i < 256; i++)
            write_cache(0x1000 + i, (uint8_t)(i & 0xFF), DATA);

        /* read it back */
        int errors = 0;
        for (int i = 0; i < 256; i++) {
            uint8_t got = read_cache(0x1000 + i, DATA);
            if (got != (uint8_t)(i & 0xFF)) errors++;
        }

        printf("  Policy=%-6s  errors=%d  %s\n",
               policy ? "RANDOM" : "LRU", errors,
               errors == 0 ? "PASS" : "FAIL");
    }
}

/* ---------------------------------------------------------------
 * TEST 2 – get_l1_data_cache_line / get_l2_cache_line
 *
 * After a read, the line must appear in L1-D (and L2).
 * After an unrelated eviction, it may no longer appear in L1.
 * ------------------------------------------------------------- */
static void test_cache_line_pointers(void) {
    printf("\n=== TEST 2: Cache line pointer lookups ===\n");

    reset_memory();
    init_cache(LRU);

    /* Load address 0x0000 */
    read_cache(0x0000, DATA);

    cache_line_t *cl = get_l1_data_cache_line(0x0000);
    printf("  After read(0x0000): L1-D ptr=%s  PASS=%s\n",
           cl ? "non-null" : "null",
           cl != NULL ? "yes" : "NO");

    cache_line_t *cl2 = get_l2_cache_line(0x0000);
    printf("  After read(0x0000): L2   ptr=%s  PASS=%s\n",
           cl2 ? "non-null" : "null",
           cl2 != NULL ? "yes" : "NO");

    /* address not yet loaded */
    cache_line_t *cl3 = get_l1_data_cache_line(0x20000);
    printf("  Unloaded addr 0x20000: L1-D ptr=%s  PASS=%s\n",
           cl3 ? "non-null" : "null",
           cl3 == NULL ? "yes" : "NO");
}

/* ---------------------------------------------------------------
 * TEST 3 – Instruction vs data separation
 *
 * Reads via INSTR type must only count in L1-I stats, not L1-D.
 * ------------------------------------------------------------- */
static void test_instr_data_separation(void) {
    printf("\n=== TEST 3: INSTR vs DATA separation ===\n");

    reset_memory();
    init_cache(LRU);

    /* 64 instruction reads (one full cache line) */
    for (int i = 0; i < 64; i++)
        read_cache((uint64_t)i, INSTR);

    /* 64 data reads to a different address */
    for (int i = 0; i < 64; i++)
        read_cache(0x10000 + i, DATA);

    cache_stats_t li = get_l1_instr_stats();
    cache_stats_t ld = get_l1_data_stats();

    printf("  L1-I accesses=%llu (expected 64)  PASS=%s\n",
           (unsigned long long)li.accesses, li.accesses == 64 ? "yes" : "NO");
    printf("  L1-D accesses=%llu (expected 64)  PASS=%s\n",
           (unsigned long long)ld.accesses, ld.accesses == 64 ? "yes" : "NO");
    printf("  L1-I misses=%llu (expected 1, one cold miss per 64-byte line)\n",
           (unsigned long long)li.misses);
    printf("  L1-D misses=%llu (expected 1)\n",
           (unsigned long long)ld.misses);
}

/* ---------------------------------------------------------------
 * TEST 4 – Write-back: dirty data must reach main memory
 *
 * Write to an address, force eviction by filling the same set
 * with conflicting addresses, then read main memory directly.
 * ------------------------------------------------------------- */
static void test_writeback(void) {
    printf("\n=== TEST 4: Write-back to main memory ===\n");

    reset_memory();
    init_cache(LRU);

    /* Write 0xAB to address 0 */
    write_cache(0x0000, 0xAB, DATA);

    /* Force eviction of the set that contains 0x0000 by loading
     * HW11_L1_DATA_ASSOC+1 = 3 lines that all map to the same L1-D set.
     * L1-D has 256 sets, stride = 256*64 = 16384 = 0x4000 bytes
     * touches set 0 each time.                                    */
    uint64_t stride = (uint64_t)256 * 64;  /* L1D_SETS * LINE_SIZE */
    for (int i = 1; i <= HW11_L1_DATA_ASSOC + 1; i++)
        read_cache((uint64_t)i * stride, DATA);

    /* 0x0000 should have been evicted; with LRU it will be gone. */
    /* The dirty data must have propagated to L2 and then to main memory
     * (L2 is also small enough but here we check via read_memory).
     * Actually L2 holds it until *it* is evicted. Let us instead
     * force it out of L2 as well. L2 has 8192 sets, stride = 8192*64. */
    uint64_t l2_stride = (uint64_t)8192 * 64;
    for (int i = 1; i <= HW11_L2_ASSOC + 1; i++)
        read_cache((uint64_t)i * l2_stride, DATA);

    uint8_t mem_val = read_memory(0x0000);
    printf("  mem[0x0000] = 0x%02X (expected 0xAB)  PASS=%s\n",
           mem_val, mem_val == 0xAB ? "yes" : "NO");
}

/* ---------------------------------------------------------------
 * TEST 5A – LRU-friendly locality pattern.
 *
 * Pattern: A, B, A, C repeated on a single 2-way L1-D set.
 * A is reused often enough that LRU protects it, while Random
 * sometimes evicts A and pays extra misses later.
 * ------------------------------------------------------------- */
#define L1D_SET_STRIDE ((uint64_t)256 * 64)

static void test_lru_friendly_pattern(void) {
    printf("\n=== TEST 5A: LRU vs Random – hot set with occasional spoiler ===\n");

    uint64_t pattern[] = {
        0 * L1D_SET_STRIDE,  /* A */
        1 * L1D_SET_STRIDE,  /* B */
        0 * L1D_SET_STRIDE,  /* A again */
        2 * L1D_SET_STRIDE   /* C spoiler */
    };
    int repeats = 4000;
    int runs = 20;

    miss_result_t lru = run_data_pattern(pattern, 4, repeats, LRU, 0);
    miss_result_t rnd = average_random_pattern(pattern, 4, repeats, runs, 100);

    printf("  Pattern: A, B, A, C on one 2-way set (%d total accesses)\n",
           repeats * 4);
    printf("  LRU   L1-D misses=%-8llu L2 misses=%-6llu\n",
           (unsigned long long)lru.l1d_misses,
           (unsigned long long)lru.l2_misses);
    printf("  RND   L1-D misses=%-8llu L2 misses=%-6llu  (avg over %d seeds)\n",
           (unsigned long long)rnd.l1d_misses,
           (unsigned long long)rnd.l2_misses,
           runs);
    printf("  Expected: LRU <= Random because recent lines are reused before the spoiler returns.\n");
}

/* ---------------------------------------------------------------
 * TEST 5B – Neutral working-set test.
 *
 * Scan 16 KB repeatedly. The footprint fits inside the 32 KB L1-D,
 * so after the first pass both policies should behave nearly the same.
 * ------------------------------------------------------------- */
#define FIT_SCAN_BYTES  (16 * 1024)
#define FIT_SCAN_BASE   0x100000ULL
#define FIT_SCAN_PASSES 4

static void test_fit_working_set(void) {
    printf("\n=== TEST 5B: LRU vs Random – working set fits in L1-D ===\n");

    int runs = 20;
    miss_result_t lru = run_data_scan(FIT_SCAN_BASE, FIT_SCAN_BYTES,
                                      FIT_SCAN_PASSES, LRU, 0);
    miss_result_t rnd = average_random_scan(FIT_SCAN_BASE, FIT_SCAN_BYTES,
                                            FIT_SCAN_PASSES, runs, 500);

    printf("  Scan: %d KB x %d passes\n", FIT_SCAN_BYTES / 1024, FIT_SCAN_PASSES);
    printf("  LRU   L1-D misses=%-8llu L2 misses=%-6llu\n",
           (unsigned long long)lru.l1d_misses,
           (unsigned long long)lru.l2_misses);
    printf("  RND   L1-D misses=%-8llu L2 misses=%-6llu  (avg over %d seeds)\n",
           (unsigned long long)rnd.l1d_misses,
           (unsigned long long)rnd.l2_misses,
           runs);
    printf("  Expected: both policies are similar once the working set fits after warmup.\n");
}

/* ---------------------------------------------------------------
 * TEST 6 – Random-friendly strided thrash pattern.
 *
 * Access pattern: repeatedly cycle through HW11_L1_DATA_ASSOC+1 addresses
 * that all map to the same L1-D set. This is adversarial to LRU:
 * it deterministically evicts the line needed next, while Random
 * occasionally keeps a useful line by chance.
 * ------------------------------------------------------------- */
static void test_thrash_pattern(void) {
    printf("\n=== TEST 6: LRU vs Random – thrashing pattern ===\n");

    uint64_t pattern[] = {
        0 * L1D_SET_STRIDE,
        1 * L1D_SET_STRIDE,
        2 * L1D_SET_STRIDE
    };
    int repeats = 2000;
    int runs = 20;

    miss_result_t lru = run_data_pattern(pattern, 3, repeats, LRU, 0);
    miss_result_t rnd = average_random_pattern(pattern, 3, repeats, runs, 900);

    printf("  Pattern: A, B, C on one 2-way set (%d total accesses)\n",
           repeats * 3);
    printf("  LRU   L1-D misses=%-8llu L2 misses=%-6llu\n",
           (unsigned long long)lru.l1d_misses,
           (unsigned long long)lru.l2_misses);
    printf("  RND   L1-D misses=%-8llu L2 misses=%-6llu  (avg over %d seeds)\n",
           (unsigned long long)rnd.l1d_misses,
           (unsigned long long)rnd.l2_misses,
           runs);
    printf("  Expected: Random <= LRU because this pattern is adversarial to recency.\n");
}

/* ---------------------------------------------------------------
 * TEST 7 – Modified bit: written lines must have modified=1,
 *          clean lines must have modified=0.
 * ------------------------------------------------------------- */
static void test_modified_bit(void) {
    printf("\n=== TEST 7: Modified (dirty) bit ===\n");

    reset_memory();
    init_cache(LRU);

    /* clean read */
    read_cache(0x5000, DATA);
    cache_line_t *cl = get_l1_data_cache_line(0x5000);
    printf("  After read:  modified=%d (expected 0)  PASS=%s\n",
           cl ? cl->modified : -1,
           (cl && cl->modified == 0) ? "yes" : "NO");

    /* dirty write */
    write_cache(0x6000, 0xFF, DATA);
    cache_line_t *cl2 = get_l1_data_cache_line(0x6000);
    printf("  After write: modified=%d (expected 1)  PASS=%s\n",
           cl2 ? cl2->modified : -1,
           (cl2 && cl2->modified == 1) ? "yes" : "NO");
}

/* ---------------------------------------------------------------
 * Entry point
 * ------------------------------------------------------------- */
int main(int argc, char *argv[]) {
    printf("============================================\n");
    printf("  TCache Test Suite\n");
    printf("============================================\n");

    test_basic_rw();
    test_cache_line_pointers();
    test_instr_data_separation();
    test_writeback();
    test_lru_friendly_pattern();
    test_fit_working_set();
    test_thrash_pattern();
    test_modified_bit();

    printf("\n============================================\n");
    printf("  All tests complete.\n");
    printf("============================================\n");
    return 0;
}
