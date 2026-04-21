#include "tcache.h"
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ==============================================================
 * CACHE GEOMETRY
 *
 * Line size: 64 bytes → 6 offset bits
 *
 * L1-I:  32 KB, 1-way (direct mapped)
 *        sets = 32768 / (64 * 1) = 512   → 9 set-index bits
 *
 * L1-D:  32 KB, 2-way set-associative
 *        sets = 32768 / (64 * 2) = 256   → 8 set-index bits
 *
 * L2:    2 MB,  4-way set-associative
 *        sets = 2097152 / (64 * 4) = 8192 → 13 set-index bits
 * ============================================================== */

#define LINE_SIZE     64
#define LINE_BITS      6
#define LINE_MASK     (LINE_SIZE - 1)

#define L1I_WAYS      HW11_L1_INSTR_ASSOC
#define L1I_SETS      (HW11_L1_SIZE / (LINE_SIZE * L1I_WAYS))   /* 512  */
#define L1I_SET_BITS   9

#define L1D_WAYS      HW11_L1_DATA_ASSOC
#define L1D_SETS      (HW11_L1_SIZE / (LINE_SIZE * L1D_WAYS))   /* 256  */
#define L1D_SET_BITS   8

#define L2_WAYS       HW11_L2_ASSOC
#define L2_SETS       (HW11_L2_SIZE / (LINE_SIZE * L2_WAYS))    /* 8192 */
#define L2_SET_BITS   13

/* ==============================================================
 * CACHE STORAGE
 * cache_line_t now contains: valid, modified, tag, data[64]
 * Only a separate lru_stamp per way is needed beyond that.
 * ============================================================== */
static cache_line_t l1i[L1I_SETS][L1I_WAYS];
static cache_line_t l1d[L1D_SETS][L1D_WAYS];
static cache_line_t l2 [L2_SETS ][L2_WAYS ];

static uint32_t l1i_lru[L1I_SETS][L1I_WAYS];
static uint32_t l1d_lru[L1D_SETS][L1D_WAYS];
static uint32_t l2_lru [L2_SETS ][L2_WAYS ];

static cache_stats_t l1i_stats;
static cache_stats_t l1d_stats;
static cache_stats_t l2_stats_g;

static replacement_policy_e g_policy;
static uint32_t lru_clock = 0;

/* ==============================================================
 * ADDRESS DECOMPOSITION
 * ============================================================== */
static inline uint64_t offset_of(uint64_t a) { return a & LINE_MASK; }

static inline uint64_t l1i_set_of(uint64_t a) { return (a >> LINE_BITS) & (L1I_SETS - 1); }
static inline uint64_t l1i_tag_of(uint64_t a) { return a >> (LINE_BITS + L1I_SET_BITS); }
static inline uint64_t l1i_base_of(uint64_t set, uint64_t tag) {
    return (tag << (LINE_BITS + L1I_SET_BITS)) | (set << LINE_BITS);
}

static inline uint64_t l1d_set_of(uint64_t a) { return (a >> LINE_BITS) & (L1D_SETS - 1); }
static inline uint64_t l1d_tag_of(uint64_t a) { return a >> (LINE_BITS + L1D_SET_BITS); }
static inline uint64_t l1d_base_of(uint64_t set, uint64_t tag) {
    return (tag << (LINE_BITS + L1D_SET_BITS)) | (set << LINE_BITS);
}

static inline uint64_t l2_set_of(uint64_t a) { return (a >> LINE_BITS) & (L2_SETS - 1); }
static inline uint64_t l2_tag_of(uint64_t a) { return a >> (LINE_BITS + L2_SET_BITS); }
static inline uint64_t l2_base_of(uint64_t set, uint64_t tag) {
    return (tag << (LINE_BITS + L2_SET_BITS)) | (set << LINE_BITS);
}

/* ==============================================================
 * REPLACEMENT POLICY
 * ============================================================== */
static int pick_victim(cache_line_t *ways, uint32_t *stamps, int num_ways) {
    for (int w = 0; w < num_ways; w++)
        if (!ways[w].valid) return w;

    if (g_policy == RANDOM)
        return rand() % num_ways;

    int victim = 0;
    for (int w = 1; w < num_ways; w++)
        if (stamps[w] < stamps[victim]) victim = w;
    return victim;
}

/* ==============================================================
 * L2 HELPERS
 * ============================================================== */

static void writeback_to_l2(uint64_t base_addr, uint8_t *data) {
    uint64_t s  = l2_set_of(base_addr);
    uint64_t tg = l2_tag_of(base_addr);

    for (int w = 0; w < L2_WAYS; w++) {
        if (l2[s][w].valid && l2[s][w].tag == tg) {
            memcpy(l2[s][w].data, data, LINE_SIZE);
            l2[s][w].modified = 1;
            l2_lru[s][w] = ++lru_clock;
            return;
        }
    }

    int v = pick_victim(l2[s], l2_lru[s], L2_WAYS);

    if (l2[s][v].valid && l2[s][v].modified) {
        uint64_t eb = l2_base_of(s, l2[s][v].tag);
        for (int b = 0; b < LINE_SIZE; b++)
            write_memory(eb + b, l2[s][v].data[b]);
    }

    l2[s][v].valid    = 1;
    l2[s][v].modified = 1;
    l2[s][v].tag      = tg;
    l2_lru[s][v]      = ++lru_clock;
    memcpy(l2[s][v].data, data, LINE_SIZE);
}

static cache_line_t *fill_from_l2(uint64_t mem_addr) {
    uint64_t base = mem_addr & ~(uint64_t)LINE_MASK;
    uint64_t s    = l2_set_of(base);
    uint64_t tg   = l2_tag_of(base);

    for (int w = 0; w < L2_WAYS; w++) {
        if (l2[s][w].valid && l2[s][w].tag == tg) {
            l2_stats_g.accesses++;
            l2_lru[s][w] = ++lru_clock;
            return &l2[s][w];
        }
    }

    l2_stats_g.accesses++;
    l2_stats_g.misses++;

    int v = pick_victim(l2[s], l2_lru[s], L2_WAYS);

    if (l2[s][v].valid && l2[s][v].modified) {
        uint64_t eb = l2_base_of(s, l2[s][v].tag);
        for (int b = 0; b < LINE_SIZE; b++)
            write_memory(eb + b, l2[s][v].data[b]);
    }

    l2[s][v].valid    = 1;
    l2[s][v].modified = 0;
    l2[s][v].tag      = tg;
    l2_lru[s][v]      = ++lru_clock;
    for (int b = 0; b < LINE_SIZE; b++)
        l2[s][v].data[b] = read_memory(base + b);

    return &l2[s][v];
}

/* ==============================================================
 * PUBLIC API
 * ============================================================== */

void init_cache(replacement_policy_e policy) {
    g_policy  = policy;
    lru_clock = 0;
    srand((unsigned)time(NULL));

    memset(l1i,     0, sizeof(l1i));
    memset(l1d,     0, sizeof(l1d));
    memset(l2,      0, sizeof(l2));
    memset(l1i_lru, 0, sizeof(l1i_lru));
    memset(l1d_lru, 0, sizeof(l1d_lru));
    memset(l2_lru,  0, sizeof(l2_lru));
    memset(&l1i_stats,  0, sizeof(l1i_stats));
    memset(&l1d_stats,  0, sizeof(l1d_stats));
    memset(&l2_stats_g, 0, sizeof(l2_stats_g));
}

uint8_t read_cache(uint64_t mem_addr, mem_type_t type) {
    uint64_t off = offset_of(mem_addr);

    if (type == INSTR) {
        uint64_t s  = l1i_set_of(mem_addr);
        uint64_t tg = l1i_tag_of(mem_addr);
        l1i_stats.accesses++;

        if (l1i[s][0].valid && l1i[s][0].tag == tg) {
            l1i_lru[s][0] = ++lru_clock;
            return l1i[s][0].data[off];
        }

        l1i_stats.misses++;
        cache_line_t *l2line = fill_from_l2(mem_addr);

        l1i[s][0].valid    = 1;
        l1i[s][0].modified = 0;
        l1i[s][0].tag      = tg;
        l1i_lru[s][0]      = ++lru_clock;
        memcpy(l1i[s][0].data, l2line->data, LINE_SIZE);

        return l1i[s][0].data[off];

    } else {
        uint64_t s  = l1d_set_of(mem_addr);
        uint64_t tg = l1d_tag_of(mem_addr);
        l1d_stats.accesses++;

        for (int w = 0; w < L1D_WAYS; w++) {
            if (l1d[s][w].valid && l1d[s][w].tag == tg) {
                l1d_lru[s][w] = ++lru_clock;
                return l1d[s][w].data[off];
            }
        }

        l1d_stats.misses++;
        cache_line_t *l2line = fill_from_l2(mem_addr);

        int v = pick_victim(l1d[s], l1d_lru[s], L1D_WAYS);

        if (l1d[s][v].valid && l1d[s][v].modified)
            writeback_to_l2(l1d_base_of(s, l1d[s][v].tag), l1d[s][v].data);

        l1d[s][v].valid    = 1;
        l1d[s][v].modified = 0;
        l1d[s][v].tag      = tg;
        l1d_lru[s][v]      = ++lru_clock;
        memcpy(l1d[s][v].data, l2line->data, LINE_SIZE);

        return l1d[s][v].data[off];
    }
}

void write_cache(uint64_t mem_addr, uint8_t value, mem_type_t type) {
    uint64_t off = offset_of(mem_addr);
    uint64_t s   = l1d_set_of(mem_addr);
    uint64_t tg  = l1d_tag_of(mem_addr);
    l1d_stats.accesses++;

    for (int w = 0; w < L1D_WAYS; w++) {
        if (l1d[s][w].valid && l1d[s][w].tag == tg) {
            l1d[s][w].data[off] = value;
            l1d[s][w].modified  = 1;
            l1d_lru[s][w]       = ++lru_clock;
            return;
        }
    }

    l1d_stats.misses++;
    cache_line_t *l2line = fill_from_l2(mem_addr);

    int v = pick_victim(l1d[s], l1d_lru[s], L1D_WAYS);

    if (l1d[s][v].valid && l1d[s][v].modified)
        writeback_to_l2(l1d_base_of(s, l1d[s][v].tag), l1d[s][v].data);

    l1d[s][v].valid     = 1;
    l1d[s][v].modified  = 1;
    l1d[s][v].tag       = tg;
    l1d_lru[s][v]       = ++lru_clock;
    memcpy(l1d[s][v].data, l2line->data, LINE_SIZE);
    l1d[s][v].data[off] = value;
}

cache_stats_t get_l1_instr_stats() { return l1i_stats;  }
cache_stats_t get_l1_data_stats()  { return l1d_stats;  }
cache_stats_t get_l2_stats()       { return l2_stats_g; }

cache_line_t *get_l1_instr_cache_line(uint64_t mem_addr) {
    uint64_t s  = l1i_set_of(mem_addr);
    uint64_t tg = l1i_tag_of(mem_addr);
    if (l1i[s][0].valid && l1i[s][0].tag == tg) return &l1i[s][0];
    return NULL;
}

cache_line_t *get_l1_data_cache_line(uint64_t mem_addr) {
    uint64_t s  = l1d_set_of(mem_addr);
    uint64_t tg = l1d_tag_of(mem_addr);
    for (int w = 0; w < L1D_WAYS; w++)
        if (l1d[s][w].valid && l1d[s][w].tag == tg) return &l1d[s][w];
    return NULL;
}

cache_line_t *get_l2_cache_line(uint64_t mem_addr) {
    uint64_t s  = l2_set_of(mem_addr);
    uint64_t tg = l2_tag_of(mem_addr);
    for (int w = 0; w < L2_WAYS; w++)
        if (l2[s][w].valid && l2[s][w].tag == tg) return &l2[s][w];
    return NULL;
}