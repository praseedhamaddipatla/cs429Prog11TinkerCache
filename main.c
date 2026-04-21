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
 * TEST 5 – LRU vs Random miss comparison on a large sequential
 *          scan that overflows L1-D.
 *
 * Scan a 64 KB array (> 32 KB L1-D) forward 4 times.
 * LRU should yield fewer or equal L1-D misses than Random on
 * average for sequential access patterns.
 * We run Random 10 times and take the average to reduce noise.
 * ------------------------------------------------------------- */
#define SCAN_BYTES  (64 * 1024)   /* 64 KB, overflows 32 KB L1-D */
#define SCAN_BASE   0x100000ULL   /* well within 16 MB            */
#define SCAN_PASSES 4

static void test_lru_vs_random(void) {
    printf("\n=== TEST 5: LRU vs Random – sequential scan (%d KB x %d passes) ===\n",
           SCAN_BYTES / 1024, SCAN_PASSES);

    /* ---- LRU ---- */
    reset_memory();
    init_cache(LRU);
    for (int p = 0; p < SCAN_PASSES; p++)
        for (uint64_t i = 0; i < SCAN_BYTES; i++)
            read_cache(SCAN_BASE + i, DATA);

    cache_stats_t lru_ld = get_l1_data_stats();
    cache_stats_t lru_l2 = get_l2_stats();
    printf("  LRU   L1-D misses=%-8llu L2 misses=%-6llu\n",
           (unsigned long long)lru_ld.misses,
           (unsigned long long)lru_l2.misses);

    /* ---- Random (average over 10 runs) ---- */
    uint64_t total_l1d = 0, total_l2 = 0;
    int runs = 10;
    for (int r = 0; r < runs; r++) {
        reset_memory();
        init_cache(RANDOM);
        for (int p = 0; p < SCAN_PASSES; p++)
            for (uint64_t i = 0; i < SCAN_BYTES; i++)
                read_cache(SCAN_BASE + i, DATA);
        total_l1d += get_l1_data_stats().misses;
        total_l2  += get_l2_stats().misses;
    }
    printf("  RND   L1-D misses=%-8llu L2 misses=%-6llu  (avg over %d runs)\n",
           (unsigned long long)(total_l1d / runs),
           (unsigned long long)(total_l2  / runs),
           runs);

    printf("  LRU %s Random for sequential scan (expected: LRU >= for overflowing scan)\n",
           lru_ld.misses >= total_l1d / runs ? ">= " : "< ");
}

/* ---------------------------------------------------------------
 * TEST 6 – LRU vs Random on a strided / thrashing access pattern.
 *
 * Access pattern: repeatedly cycle through HW11_L1_DATA_ASSOC+1 addresses
 * that all map to the same L1-D set.  This is the classic
 * "Belady's anomaly" / LRU-thrash scenario.  LRU will miss on
 * every access; Random will occasionally keep a useful line.
 * ------------------------------------------------------------- */
static void test_thrash_pattern(void) {
    printf("\n=== TEST 6: LRU vs Random – thrashing pattern ===\n");

    /* Addresses that all map to L1-D set 0:
     * set 0 is selected by bits [13:6] = 0, so addresses spaced
     * by L1D_SETS * LINE_SIZE = 256 * 64 = 16384 bytes apart.   */
    uint64_t stride  = (uint64_t)256 * 64;
    int      n_addrs = HW11_L1_DATA_ASSOC + 1;   /* one more than ways = thrash */
    int      iters   = 2000;

    /* ---- LRU ---- */
    reset_memory();
    init_cache(LRU);
    for (int i = 0; i < iters; i++)
        for (int a = 0; a < n_addrs; a++)
            read_cache((uint64_t)a * stride, DATA);

    uint64_t lru_misses = get_l1_data_stats().misses;
    printf("  LRU  thrash L1-D misses = %llu / %d accesses\n",
           (unsigned long long)lru_misses, iters * n_addrs);

    /* ---- Random (avg 10 runs) ---- */
    uint64_t total = 0;
    for (int r = 0; r < 10; r++) {
        reset_memory();
        init_cache(RANDOM);
        for (int i = 0; i < iters; i++)
            for (int a = 0; a < n_addrs; a++)
                read_cache((uint64_t)a * stride, DATA);
        total += get_l1_data_stats().misses;
    }
    uint64_t rnd_misses = total / 10;
    printf("  RND  thrash L1-D misses = %llu / %d accesses  (avg 10 runs)\n",
           (unsigned long long)rnd_misses, iters * n_addrs);
    printf("  Random %s LRU on thrash pattern (expected: Random <=)\n",
           rnd_misses <= lru_misses ? "<=" : ">");
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
    test_lru_vs_random();
    test_thrash_pattern();
    test_modified_bit();

    printf("\n============================================\n");
    printf("  All tests complete.\n");
    printf("============================================\n");
    return 0;
}