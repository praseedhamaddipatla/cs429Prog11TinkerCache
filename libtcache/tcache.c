#include "tcache.h"
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

/* ==============================================================
 * CACHE GEOMETRY
 *
 * Line size: 64 bytes → 6 offset bits
 *
 * L1-I:  32 KB, 1-way direct-mapped
 *        sets = 32768 / (64*1) = 512  → 9 index bits
 *
 * L1-D:  32 KB, 2-way set-associative
 *        sets = 32768 / (64*2) = 256  → 8 index bits
 *
 * L2:    2 MB,  4-way set-associative, INCLUSIVE of both L1s
 *        sets = 2097152 / (64*4) = 8192 → 13 index bits
 *
 * Address format (MSB → LSB):  tag | index | offset
 * ============================================================== */

#define LINE_SIZE    64
#define LINE_BITS     6
#define LINE_MASK    ((uint64_t)(LINE_SIZE - 1))

#define L1I_WAYS     HW11_L1_INSTR_ASSOC
#define L1I_SETS     (HW11_L1_SIZE / (LINE_SIZE * L1I_WAYS))
#define L1I_IDX_BITS  9

#define L1D_WAYS     HW11_L1_DATA_ASSOC
#define L1D_SETS     (HW11_L1_SIZE / (LINE_SIZE * L1D_WAYS))
#define L1D_IDX_BITS  8

#define L2_WAYS      HW11_L2_ASSOC
#define L2_SETS      (HW11_L2_SIZE / (LINE_SIZE * L2_WAYS))
#define L2_IDX_BITS  13

/* ==============================================================
 * CACHE ARRAYS  +  SEPARATE LRU STAMPS
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
 * ADDRESS DECOMPOSITION  (tag | index | offset)
 * ============================================================== */
static inline uint64_t line_offset(uint64_t a) { return a & LINE_MASK; }

static inline uint64_t l1i_idx(uint64_t a) { return (a >> LINE_BITS) & (L1I_SETS - 1); }
static inline uint64_t l1i_tag(uint64_t a) { return  a >> (LINE_BITS + L1I_IDX_BITS); }
static inline uint64_t l1i_base(uint64_t idx, uint64_t tag) {
    return (tag << (LINE_BITS + L1I_IDX_BITS)) | (idx << LINE_BITS);
}

static inline uint64_t l1d_idx(uint64_t a) { return (a >> LINE_BITS) & (L1D_SETS - 1); }
static inline uint64_t l1d_tag(uint64_t a) { return  a >> (LINE_BITS + L1D_IDX_BITS); }
static inline uint64_t l1d_base(uint64_t idx, uint64_t tag) {
    return (tag << (LINE_BITS + L1D_IDX_BITS)) | (idx << LINE_BITS);
}

static inline uint64_t l2_idx(uint64_t a) { return (a >> LINE_BITS) & (L2_SETS - 1); }
static inline uint64_t l2_tag(uint64_t a) { return  a >> (LINE_BITS + L2_IDX_BITS); }
static inline uint64_t l2_base(uint64_t idx, uint64_t tag) {
    return (tag << (LINE_BITS + L2_IDX_BITS)) | (idx << LINE_BITS);
}

/* ==============================================================
 * REPLACEMENT POLICY
 * ============================================================== */
static int pick_victim(cache_line_t *ways, uint32_t *stamps, int num_ways) {
    for (int w = 0; w < num_ways; w++) {
        if (!ways[w].valid) return w;
    }
    if (num_ways == 1) {
        return 0;
    }
    if (g_policy == RANDOM) {
        return rand() % num_ways;
    }
    int victim = 0;
    for (int w = 1; w < num_ways; w++) {
        if (stamps[w] < stamps[victim]) victim = w;
    }
    return victim;
}

/* ==============================================================
 * WRITEBACK: dirty line → L2  (counts as one L2 access)
 *
 * Used for:
 *   (a) L1-D eviction writeback
 *   (b) L1-I eviction writeback  (L1-I lines CAN be modified by INSTR writes)
 *   (c) L1-D coherency writeback before L1-I reads (per TA spec)
 * ============================================================== */
static void writeback_to_l2(uint64_t base_addr, uint8_t *data) {
    uint64_t s  = l2_idx(base_addr);
    uint64_t tg = l2_tag(base_addr);

    l2_stats_g.accesses++;

    for (int w = 0; w < L2_WAYS; w++) {
        if (l2[s][w].valid && l2[s][w].tag == tg) {
            memcpy(l2[s][w].data, data, LINE_SIZE);
            l2[s][w].modified = 1;
            l2_lru[s][w] = ++lru_clock;
            return;
        }
    }

    /* L2 miss on writeback — install the line */
    l2_stats_g.misses++;
    int victim = pick_victim(l2[s], l2_lru[s], L2_WAYS);

    /* evict L2 victim (inclusive: also invalidate from L1s) */
    if (l2[s][victim].valid) {
        uint64_t evict_base = l2_base(s, l2[s][victim].tag);
        if (l2[s][victim].modified) {
            for (int b = 0; b < LINE_SIZE; b++) {
                write_memory(evict_base + b, l2[s][victim].data[b]);
            }
        }
        /* invalidate L1-I copy */
        {
            uint64_t is  = l1i_idx(evict_base);
            uint64_t itg = l1i_tag(evict_base);
            for (int w = 0; w < L1I_WAYS; w++) {
                if (l1i[is][w].valid && l1i[is][w].tag == itg) {
                    if (l1i[is][w].modified) {
                        l2_stats_g.accesses++; /* writeback from L1-I counts as L2 access */
                        for (int b = 0; b < LINE_SIZE; b++) {
                            write_memory(evict_base + b, l1i[is][w].data[b]);
                        }
                    }
                    l1i[is][w].valid = 0;
                }
            }
        }
        /* invalidate L1-D copy */
        {
            uint64_t ds  = l1d_idx(evict_base);
            uint64_t dtg = l1d_tag(evict_base);
            for (int w = 0; w < L1D_WAYS; w++) {
                if (l1d[ds][w].valid && l1d[ds][w].tag == dtg) {
                    if (l1d[ds][w].modified) {
                        l2_stats_g.accesses++; /* writeback from L1-D counts as L2 access */
                        for (int b = 0; b < LINE_SIZE; b++) {
                            write_memory(evict_base + b, l1d[ds][w].data[b]);
                        }
                    }
                    l1d[ds][w].valid = 0;
                    break;
                }
            }
        }
        l2[s][victim].valid = 0;
    }

    l2[s][victim].valid    = 1;
    l2[s][victim].modified = 1;
    l2[s][victim].tag      = tg;
    l2_lru[s][victim]      = ++lru_clock;
    memcpy(l2[s][victim].data, data, LINE_SIZE);
}

/* ==============================================================
 * FILL FROM L2  (or main memory on L2 miss)
 *
 * Fetches the cache line into L2 if absent.  Counts as one L2 access.
 * On L2 eviction, invalidates L1 copies (inclusive).
 * ============================================================== */
static cache_line_t *fill_from_l2(uint64_t mem_addr) {
    uint64_t base = mem_addr & ~LINE_MASK;
    uint64_t s    = l2_idx(base);
    uint64_t tg   = l2_tag(base);

    l2_stats_g.accesses++;

    for (int w = 0; w < L2_WAYS; w++) {
        if (l2[s][w].valid && l2[s][w].tag == tg) {
            l2_lru[s][w] = ++lru_clock;
            return &l2[s][w];
        }
    }

    /* L2 miss: fetch from main memory */
    l2_stats_g.misses++;

    int victim = pick_victim(l2[s], l2_lru[s], L2_WAYS);

    /* evict L2 victim (inclusive: also invalidate from L1s) */
    if (l2[s][victim].valid) {
        uint64_t evict_base = l2_base(s, l2[s][victim].tag);
        if (l2[s][victim].modified) {
            for (int b = 0; b < LINE_SIZE; b++) {
                write_memory(evict_base + b, l2[s][victim].data[b]);
            }
        }
        /* invalidate / flush L1-I copy */
        {
            uint64_t is  = l1i_idx(evict_base);
            uint64_t itg = l1i_tag(evict_base);
            for (int w = 0; w < L1I_WAYS; w++) {
                if (l1i[is][w].valid && l1i[is][w].tag == itg) {
                    if (l1i[is][w].modified) {
                        l2_stats_g.accesses++; /* writeback from L1-I counts as L2 access */
                        for (int b = 0; b < LINE_SIZE; b++) {
                            write_memory(evict_base + b, l1i[is][w].data[b]);
                        }
                    }
                    l1i[is][w].valid = 0;
                }
            }
        }
        /* invalidate / flush L1-D copy */
        {
            uint64_t ds  = l1d_idx(evict_base);
            uint64_t dtg = l1d_tag(evict_base);
            for (int w = 0; w < L1D_WAYS; w++) {
                if (l1d[ds][w].valid && l1d[ds][w].tag == dtg) {
                    if (l1d[ds][w].modified) {
                        l2_stats_g.accesses++; /* writeback from L1-D counts as L2 access */
                        for (int b = 0; b < LINE_SIZE; b++) {
                            write_memory(evict_base + b, l1d[ds][w].data[b]);
                        }
                    }
                    l1d[ds][w].valid = 0;
                    break;
                }
            }
        }
        l2[s][victim].valid = 0;
    }

    l2[s][victim].valid    = 1;
    l2[s][victim].modified = 0;
    l2[s][victim].tag      = tg;
    l2_lru[s][victim]      = ++lru_clock;
    for (int b = 0; b < LINE_SIZE; b++) {
        l2[s][victim].data[b] = read_memory(base + b);
    }

    return &l2[s][victim];
}

/* ==============================================================
 * PUBLIC API
 * ============================================================== */

void init_cache(replacement_policy_e policy) {
    g_policy  = policy;
    lru_clock = 0;
    srand(1);

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

/* -------------------------------------------------------------- */
uint8_t read_cache(uint64_t mem_addr, mem_type_t type) {
    uint64_t off  = line_offset(mem_addr);
    uint64_t base = mem_addr & ~LINE_MASK;

    if (type == INSTR) {
        uint64_t s  = l1i_idx(mem_addr);
        uint64_t tg = l1i_tag(mem_addr);
        l1i_stats.accesses++;

        /* L1-I hit */
        for (int w = 0; w < L1I_WAYS; w++) {
            if (l1i[s][w].valid && l1i[s][w].tag == tg) {
                l1i_lru[s][w] = ++lru_clock;
                return l1i[s][w].data[off];
            }
        }

        /* L1-I miss */
        l1i_stats.misses++;

        /* Coherency: if L1-D has a dirty copy, write it back to L2
         * so that L1-I reads the latest data.
         * "Write backs from L1 due to other means" = L2 access. */
        {
            uint64_t ds  = l1d_idx(base);
            uint64_t dtg = l1d_tag(base);
            for (int w = 0; w < L1D_WAYS; w++) {
                if (l1d[ds][w].valid && l1d[ds][w].tag == dtg
                        && l1d[ds][w].modified) {
                    writeback_to_l2(base, l1d[ds][w].data);
                    l1d[ds][w].modified = 0;
                    break;
                }
            }
        }

        /* Fetch from L2 (one L2 access) */
        cache_line_t *l2line = fill_from_l2(mem_addr);

        /* Evict from L1-I if necessary, writing back dirty lines */
        int victim = pick_victim(l1i[s], l1i_lru[s], L1I_WAYS);
        if (l1i[s][victim].valid && l1i[s][victim].modified) {
            uint64_t evict_base = l1i_base(s, l1i[s][victim].tag);
            writeback_to_l2(evict_base, l1i[s][victim].data);
        }

        l1i[s][victim].valid    = 1;
        l1i[s][victim].modified = 0;
        l1i[s][victim].tag      = tg;
        l1i_lru[s][victim]      = ++lru_clock;
        memcpy(l1i[s][victim].data, l2line->data, LINE_SIZE);

        return l1i[s][victim].data[off];

    } else {
        uint64_t s  = l1d_idx(mem_addr);
        uint64_t tg = l1d_tag(mem_addr);
        l1d_stats.accesses++;

        /* L1-D hit */
        for (int w = 0; w < L1D_WAYS; w++) {
            if (l1d[s][w].valid && l1d[s][w].tag == tg) {
                l1d_lru[s][w] = ++lru_clock;
                return l1d[s][w].data[off];
            }
        }

        /* L1-D miss */
        l1d_stats.misses++;

        /* Coherency: if L1-I has a dirty copy of this line, write it back
         * to L2 first so that L1-D reads the latest data from L2.
         * "Write backs from L1 due to other means" = L2 access. */
        {
            uint64_t is  = l1i_idx(base);
            uint64_t itg = l1i_tag(base);
            for (int w = 0; w < L1I_WAYS; w++) {
                if (l1i[is][w].valid && l1i[is][w].tag == itg
                        && l1i[is][w].modified) {
                    writeback_to_l2(base, l1i[is][w].data);
                    l1i[is][w].modified = 0;
                    break;
                }
            }
        }

        cache_line_t *l2line = fill_from_l2(mem_addr);

        int victim = pick_victim(l1d[s], l1d_lru[s], L1D_WAYS);
        if (l1d[s][victim].valid && l1d[s][victim].modified) {
            writeback_to_l2(l1d_base(s, l1d[s][victim].tag),
                            l1d[s][victim].data);
        }

        l1d[s][victim].valid    = 1;
        l1d[s][victim].modified = 0;
        l1d[s][victim].tag      = tg;
        l1d_lru[s][victim]      = ++lru_clock;
        memcpy(l1d[s][victim].data, l2line->data, LINE_SIZE);

        return l1d[s][victim].data[off];
    }
}

/* -------------------------------------------------------------- */
void write_cache(uint64_t mem_addr, uint8_t value, mem_type_t type) {
    uint64_t off  = line_offset(mem_addr);
    uint64_t base = mem_addr & ~LINE_MASK;
 
    if (type == INSTR) {
        /* INSTR writes go to L1-I */
        uint64_t s  = l1i_idx(mem_addr);
        uint64_t tg = l1i_tag(mem_addr);
        l1i_stats.accesses++;
 
        /* Coherency: if L1-D has a dirty copy, write it back to L2 first,
         * then invalidate it.  A silent drop of dirty data violates
         * write-back semantics and corrupts the inclusive-cache invariant. */
        {
            uint64_t ds  = l1d_idx(base);
            uint64_t dtg = l1d_tag(base);
            for (int w = 0; w < L1D_WAYS; w++) {
                if (l1d[ds][w].valid && l1d[ds][w].tag == dtg) {
                    if (l1d[ds][w].modified) {          /* FIX: writeback dirty data */
                        writeback_to_l2(base, l1d[ds][w].data);
                    }
                    l1d[ds][w].valid = 0;
                    break;
                }
            }
        }
 
        /* L1-I hit */
        for (int w = 0; w < L1I_WAYS; w++) {
            if (l1i[s][w].valid && l1i[s][w].tag == tg) {
                l1i[s][w].data[off] = value;
                l1i[s][w].modified  = 1;
                l1i_lru[s][w]       = ++lru_clock;
                return;
            }
        }
 
        /* L1-I write miss — write-allocate: fetch from L2 first */
        l1i_stats.misses++;
 
        cache_line_t *l2line = fill_from_l2(mem_addr);
 
        int victim = pick_victim(l1i[s], l1i_lru[s], L1I_WAYS);
        if (l1i[s][victim].valid && l1i[s][victim].modified) {
            uint64_t evict_base = l1i_base(s, l1i[s][victim].tag);
            writeback_to_l2(evict_base, l1i[s][victim].data);
        }
 
        l1i[s][victim].valid     = 1;
        l1i[s][victim].modified  = 1;
        l1i[s][victim].tag       = tg;
        l1i_lru[s][victim]       = ++lru_clock;
        memcpy(l1i[s][victim].data, l2line->data, LINE_SIZE);
        l1i[s][victim].data[off] = value;
 
    } else {
        /* DATA writes go to L1-D */
        uint64_t s  = l1d_idx(mem_addr);
        uint64_t tg = l1d_tag(mem_addr);
        l1d_stats.accesses++;
 
        /* Coherency: if L1-I has a dirty copy, write it back to L2 first,
         * then invalidate it.  Same rationale as the INSTR case above. */
        {
            uint64_t is  = l1i_idx(base);
            uint64_t itg = l1i_tag(base);
            for (int w = 0; w < L1I_WAYS; w++) {
                if (l1i[is][w].valid && l1i[is][w].tag == itg) {
                    if (l1i[is][w].modified) {          /* FIX: writeback dirty data */
                        writeback_to_l2(base, l1i[is][w].data);
                    }
                    l1i[is][w].valid = 0;
                }
            }
        }
 
        /* L1-D hit */
        for (int w = 0; w < L1D_WAYS; w++) {
            if (l1d[s][w].valid && l1d[s][w].tag == tg) {
                l1d[s][w].data[off] = value;
                l1d[s][w].modified  = 1;
                l1d_lru[s][w]       = ++lru_clock;
                return;
            }
        }
 
        /* L1-D write miss — write-allocate: fetch from L2 first */
        l1d_stats.misses++;
 
        cache_line_t *l2line = fill_from_l2(mem_addr);
 
        int victim = pick_victim(l1d[s], l1d_lru[s], L1D_WAYS);
        if (l1d[s][victim].valid && l1d[s][victim].modified) {
            writeback_to_l2(l1d_base(s, l1d[s][victim].tag),
                            l1d[s][victim].data);
        }
 
        l1d[s][victim].valid     = 1;
        l1d[s][victim].modified  = 1;
        l1d[s][victim].tag       = tg;
        l1d_lru[s][victim]       = ++lru_clock;
        memcpy(l1d[s][victim].data, l2line->data, LINE_SIZE);
        l1d[s][victim].data[off] = value;
    }
}

/* -------------------------------------------------------------- */
cache_stats_t get_l1_instr_stats() { return l1i_stats;  }
cache_stats_t get_l1_data_stats()  { return l1d_stats;  }
cache_stats_t get_l2_stats()       { return l2_stats_g; }

/* -------------------------------------------------------------- */
cache_line_t *get_l1_instr_cache_line(uint64_t mem_addr) {
    uint64_t s  = l1i_idx(mem_addr);
    uint64_t tg = l1i_tag(mem_addr);
    for (int w = 0; w < L1I_WAYS; w++) {
        if (l1i[s][w].valid && l1i[s][w].tag == tg) {
            return &l1i[s][w];
        }
    }
    return NULL;
}

cache_line_t *get_l1_data_cache_line(uint64_t mem_addr) {
    uint64_t s  = l1d_idx(mem_addr);
    uint64_t tg = l1d_tag(mem_addr);
    for (int w = 0; w < L1D_WAYS; w++) {
        if (l1d[s][w].valid && l1d[s][w].tag == tg) {
            return &l1d[s][w];
        }
    }
    return NULL;
}

cache_line_t *get_l2_cache_line(uint64_t mem_addr) {
    uint64_t s  = l2_idx(mem_addr);
    uint64_t tg = l2_tag(mem_addr);
    for (int w = 0; w < L2_WAYS; w++) {
        if (l2[s][w].valid && l2[s][w].tag == tg) {
            return &l2[s][w];
        }
    }
    return NULL;
}
