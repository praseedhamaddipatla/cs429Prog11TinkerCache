#include "tcache.h"
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ==============================================================
 * CACHE GEOMETRY
 *
 * Cache line data payload: 64 bytes  → 6 offset bits
 *
 * L1-I:  32 KB, 1-way (direct mapped)
 *        sets = 32768 / (64 * 1) = 512   → 9 set-index bits
 *        tag  = addr[63:15]
 *
 * L1-D:  32 KB, 2-way set-associative
 *        sets = 32768 / (64 * 2) = 256   → 8 set-index bits
 *        tag  = addr[63:14]
 *
 * L2:    2 MB,  4-way set-associative
 *        sets = 2097152 / (64 * 4) = 8192 → 13 set-index bits
 *        tag  = addr[63:19]
 *
 * Address breakdown (64-bit):
 *   [63 .. tag_msb] [set-index bits] [5:0 offset]
 * ============================================================== */

#define LINE_SIZE      64
#define LINE_BITS       6   /* log2(LINE_SIZE) */
#define LINE_MASK      (LINE_SIZE - 1)

/* L1-I */
#define L1I_WAYS        HW11_L1_INSTR_ASSOC   /* 1  */
#define L1I_SETS       (HW11_L1_SIZE / (LINE_SIZE * L1I_WAYS))  /* 512 */
#define L1I_SET_BITS    9   /* log2(512) */

/* L1-D */
#define L1D_WAYS        HW11_L1_DATA_ASSOC    /* 2  */
#define L1D_SETS       (HW11_L1_SIZE / (LINE_SIZE * L1D_WAYS))  /* 256 */
#define L1D_SET_BITS    8   /* log2(256) */

/* L2 */
#define L2_WAYS         HW11_L2_ASSOC         /* 4  */
#define L2_SETS        (HW11_L2_SIZE / (LINE_SIZE * L2_WAYS))   /* 8192 */
#define L2_SET_BITS    13   /* log2(8192) */

/* ==============================================================
 * INTERNAL CACHE LINE  (extends the public cache_line_t with a tag
 * and an LRU counter; the grader only sees cache_line_t so we wrap)
 * ============================================================== */
typedef struct {
    cache_line_t pub;        /* valid, modified, data[64] — visible to grader */
    uint64_t     tag;
    uint32_t     lru_stamp;  /* larger = more recently used */
} internal_line_t;

/* ==============================================================
 * CACHE ARRAYS
 * ============================================================== */
static internal_line_t l1i_lines[L1I_SETS][L1I_WAYS];
static internal_line_t l1d_lines[L1D_SETS][L1D_WAYS];
static internal_line_t l2_lines [L2_SETS ][L2_WAYS ];

/* stats */
static cache_stats_t l1i_stats;
static cache_stats_t l1d_stats;
static cache_stats_t l2_stats_g;

/* replacement policy */
static replacement_policy_e g_policy;

/* global LRU clock */
static uint32_t lru_clock = 0;

/* ==============================================================
 * ADDRESS DECOMPOSITION HELPERS
 * ============================================================== */
static inline uint64_t offset_of(uint64_t addr)             { return addr & LINE_MASK; }
static inline uint64_t l1i_set_of(uint64_t addr)            { return (addr >> LINE_BITS) & (L1I_SETS - 1); }
static inline uint64_t l1i_tag_of(uint64_t addr)            { return addr >> (LINE_BITS + L1I_SET_BITS); }
static inline uint64_t l1d_set_of(uint64_t addr)            { return (addr >> LINE_BITS) & (L1D_SETS - 1); }
static inline uint64_t l1d_tag_of(uint64_t addr)            { return addr >> (LINE_BITS + L1D_SET_BITS); }
static inline uint64_t l2_set_of(uint64_t addr)             { return (addr >> LINE_BITS) & (L2_SETS - 1); }
static inline uint64_t l2_tag_of(uint64_t addr)             { return addr >> (LINE_BITS + L2_SET_BITS); }

/* reconstruct the base address of a cache line given its set and tag */
static inline uint64_t l1i_base_addr(uint64_t set, uint64_t tag) {
    return (tag << (LINE_BITS + L1I_SET_BITS)) | (set << LINE_BITS);
}
static inline uint64_t l1d_base_addr(uint64_t set, uint64_t tag) {
    return (tag << (LINE_BITS + L1D_SET_BITS)) | (set << LINE_BITS);
}
static inline uint64_t l2_base_addr(uint64_t set, uint64_t tag) {
    return (tag << (LINE_BITS + L2_SET_BITS)) | (set << LINE_BITS);
}

/* ==============================================================
 * REPLACEMENT POLICY
 * ============================================================== */
static int pick_victim(internal_line_t *ways, int num_ways) {
    /* always prefer an invalid (empty) slot */
    for (int w = 0; w < num_ways; w++)
        if (!ways[w].pub.valid) return w;

    if (g_policy == RANDOM) {
        return rand() % num_ways;
    } else {
        /* LRU: evict the way with the smallest stamp */
        int victim = 0;
        for (int w = 1; w < num_ways; w++)
            if (ways[w].lru_stamp < ways[victim].lru_stamp)
                victim = w;
        return victim;
    }
}

/* ==============================================================
 * L2 FILL / WRITEBACK HELPERS
 * ============================================================== */

/*
 * Write a dirty L1 cache line back into L2 (or main memory if not in L2).
 * Called before evicting a modified L1 line.
 */
static void writeback_to_l2(uint64_t base_addr, uint8_t *data) {
    uint64_t set = l2_set_of(base_addr);
    uint64_t tag = l2_tag_of(base_addr);

    /* search L2 */
    for (int w = 0; w < L2_WAYS; w++) {
        internal_line_t *lc = &l2_lines[set][w];
        if (lc->pub.valid && lc->tag == tag) {
            memcpy(lc->pub.data, data, LINE_SIZE);
            lc->pub.modified = 1;
            lc->lru_stamp = ++lru_clock;
            return;
        }
    }

    /* not in L2 — install it */
    int victim = pick_victim(l2_lines[set], L2_WAYS);
    internal_line_t *lc = &l2_lines[set][victim];

    /* if the L2 victim is dirty, write it all the way to main memory */
    if (lc->pub.valid && lc->pub.modified) {
        uint64_t evict_base = l2_base_addr(set, lc->tag);
        for (int b = 0; b < LINE_SIZE; b++)
            write_memory(evict_base + b, lc->pub.data[b]);
    }

    lc->pub.valid    = 1;
    lc->pub.modified = 1;
    lc->tag          = tag;
    lc->lru_stamp    = ++lru_clock;
    memcpy(lc->pub.data, data, LINE_SIZE);
}

/*
 * Fill a cache line from L2 (and if not there, from main memory).
 * Installs the line into L2 if it wasn't already present.
 * Returns pointer to the (now-present) L2 internal line.
 */
static internal_line_t *fill_from_l2(uint64_t mem_addr) {
    uint64_t base = mem_addr & ~(uint64_t)LINE_MASK;
    uint64_t set  = l2_set_of(base);
    uint64_t tag  = l2_tag_of(base);

    /* search L2 */
    for (int w = 0; w < L2_WAYS; w++) {
        internal_line_t *lc = &l2_lines[set][w];
        if (lc->pub.valid && lc->tag == tag) {
            l2_stats_g.accesses++;
            /* hit */
            lc->lru_stamp = ++lru_clock;
            return lc;
        }
    }

    /* L2 miss — fetch line from main memory */
    l2_stats_g.accesses++;
    l2_stats_g.misses++;

    int victim = pick_victim(l2_lines[set], L2_WAYS);
    internal_line_t *lc = &l2_lines[set][victim];

    /* evict dirty L2 line to main memory */
    if (lc->pub.valid && lc->pub.modified) {
        uint64_t evict_base = l2_base_addr(set, lc->tag);
        for (int b = 0; b < LINE_SIZE; b++)
            write_memory(evict_base + b, lc->pub.data[b]);
    }

    /* load from main memory */
    lc->pub.valid    = 1;
    lc->pub.modified = 0;
    lc->tag          = tag;
    lc->lru_stamp    = ++lru_clock;
    for (int b = 0; b < LINE_SIZE; b++)
        lc->pub.data[b] = read_memory(base + b);

    return lc;
}

/* ==============================================================
 * PUBLIC API
 * ============================================================== */

void init_cache(replacement_policy_e policy) {
    g_policy = policy;
    lru_clock = 0;
    srand((unsigned)time(NULL));

    memset(l1i_lines, 0, sizeof(l1i_lines));
    memset(l1d_lines, 0, sizeof(l1d_lines));
    memset(l2_lines,  0, sizeof(l2_lines));

    memset(&l1i_stats,   0, sizeof(l1i_stats));
    memset(&l1d_stats,   0, sizeof(l1d_stats));
    memset(&l2_stats_g,  0, sizeof(l2_stats_g));
}

/* -------------------------------------------------------------- */
uint8_t read_cache(uint64_t mem_addr, mem_type_t type) {
    uint64_t off = offset_of(mem_addr);

    if (type == INSTR) {
        /* ---- L1-I (direct mapped) ---- */
        uint64_t set = l1i_set_of(mem_addr);
        uint64_t tag = l1i_tag_of(mem_addr);
        l1i_stats.accesses++;

        internal_line_t *cl = &l1i_lines[set][0];
        if (cl->pub.valid && cl->tag == tag) {
            /* L1-I hit */
            cl->lru_stamp = ++lru_clock;
            return cl->pub.data[off];
        }

        /* L1-I miss */
        l1i_stats.misses++;

        /* fetch line from L2 (updates L2 stats internally) */
        internal_line_t *l2_line = fill_from_l2(mem_addr);

        /* instructions are read-only — no writeback needed on eviction */
        cl->pub.valid    = 1;
        cl->pub.modified = 0;
        cl->tag          = tag;
        cl->lru_stamp    = ++lru_clock;
        memcpy(cl->pub.data, l2_line->pub.data, LINE_SIZE);

        return cl->pub.data[off];

    } else {
        /* ---- L1-D (2-way) ---- */
        uint64_t set = l1d_set_of(mem_addr);
        uint64_t tag = l1d_tag_of(mem_addr);
        l1d_stats.accesses++;

        for (int w = 0; w < L1D_WAYS; w++) {
            internal_line_t *cl = &l1d_lines[set][w];
            if (cl->pub.valid && cl->tag == tag) {
                /* L1-D hit */
                cl->lru_stamp = ++lru_clock;
                return cl->pub.data[off];
            }
        }

        /* L1-D miss */
        l1d_stats.misses++;

        internal_line_t *l2_line = fill_from_l2(mem_addr);

        int victim = pick_victim(l1d_lines[set], L1D_WAYS);
        internal_line_t *cl = &l1d_lines[set][victim];

        /* writeback dirty evictee to L2 */
        if (cl->pub.valid && cl->pub.modified) {
            uint64_t evict_base = l1d_base_addr(set, cl->tag);
            writeback_to_l2(evict_base, cl->pub.data);
        }

        cl->pub.valid    = 1;
        cl->pub.modified = 0;
        cl->tag          = tag;
        cl->lru_stamp    = ++lru_clock;
        memcpy(cl->pub.data, l2_line->pub.data, LINE_SIZE);

        return cl->pub.data[off];
    }
}

/* -------------------------------------------------------------- */
void write_cache(uint64_t mem_addr, uint8_t value, mem_type_t type) {
    /* Writes always go to L1-D regardless of type.
     * Policy: write-back + write-allocate. */
    uint64_t off = offset_of(mem_addr);
    uint64_t set = l1d_set_of(mem_addr);
    uint64_t tag = l1d_tag_of(mem_addr);
    l1d_stats.accesses++;

    for (int w = 0; w < L1D_WAYS; w++) {
        internal_line_t *cl = &l1d_lines[set][w];
        if (cl->pub.valid && cl->tag == tag) {
            /* L1-D write hit */
            cl->pub.data[off] = value;
            cl->pub.modified  = 1;
            cl->lru_stamp     = ++lru_clock;
            return;
        }
    }

    /* L1-D write miss — write-allocate: fetch line, then write */
    l1d_stats.misses++;

    internal_line_t *l2_line = fill_from_l2(mem_addr);

    int victim = pick_victim(l1d_lines[set], L1D_WAYS);
    internal_line_t *cl = &l1d_lines[set][victim];

    /* writeback dirty evictee */
    if (cl->pub.valid && cl->pub.modified) {
        uint64_t evict_base = l1d_base_addr(set, cl->tag);
        writeback_to_l2(evict_base, cl->pub.data);
    }

    cl->pub.valid    = 1;
    cl->pub.modified = 1;
    cl->tag          = tag;
    cl->lru_stamp    = ++lru_clock;
    memcpy(cl->pub.data, l2_line->pub.data, LINE_SIZE);
    cl->pub.data[off] = value;
}

/* -------------------------------------------------------------- */
cache_stats_t get_l1_instr_stats() { return l1i_stats; }
cache_stats_t get_l1_data_stats()  { return l1d_stats; }
cache_stats_t get_l2_stats()       { return l2_stats_g; }

/* -------------------------------------------------------------- */
/* Return pointer to public cache_line_t if present, else NULL.   */

cache_line_t *get_l1_instr_cache_line(uint64_t mem_addr) {
    uint64_t set = l1i_set_of(mem_addr);
    uint64_t tag = l1i_tag_of(mem_addr);
    internal_line_t *cl = &l1i_lines[set][0];
    if (cl->pub.valid && cl->tag == tag) return &cl->pub;
    return NULL;
}

cache_line_t *get_l1_data_cache_line(uint64_t mem_addr) {
    uint64_t set = l1d_set_of(mem_addr);
    uint64_t tag = l1d_tag_of(mem_addr);
    for (int w = 0; w < L1D_WAYS; w++) {
        internal_line_t *cl = &l1d_lines[set][w];
        if (cl->pub.valid && cl->tag == tag) return &cl->pub;
    }
    return NULL;
}

cache_line_t *get_l2_cache_line(uint64_t mem_addr) {
    uint64_t set = l2_set_of(mem_addr);
    uint64_t tag = l2_tag_of(mem_addr);
    for (int w = 0; w < L2_WAYS; w++) {
        internal_line_t *cl = &l2_lines[set][w];
        if (cl->pub.valid && cl->tag == tag) return &cl->pub;
    }
    return NULL;
}