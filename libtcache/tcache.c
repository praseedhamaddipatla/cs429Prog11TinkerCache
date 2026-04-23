#include "tcache.h"
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define LINE_SIZE 64
#define LINE_BITS 6
#define LINE_MASK ((uint64_t)(LINE_SIZE - 1))
#define L1I_WAYS HW11_L1_INSTR_ASSOC
#define L1I_SETS (HW11_L1_SIZE / (LINE_SIZE * L1I_WAYS))
#define L1I_IDX_BITS 9
#define L1D_WAYS HW11_L1_DATA_ASSOC
#define L1D_SETS (HW11_L1_SIZE / (LINE_SIZE * L1D_WAYS))
#define L1D_IDX_BITS 8
#define L2_WAYS HW11_L2_ASSOC
#define L2_SETS (HW11_L2_SIZE / (LINE_SIZE * L2_WAYS))
#define L2_IDX_BITS 13

static cache_line_t l1i[L1I_SETS][L1I_WAYS];
static cache_line_t l1d[L1D_SETS][L1D_WAYS];
static cache_line_t l2[L2_SETS][L2_WAYS];
static uint32_t l1i_lru[L1I_SETS][L1I_WAYS];
static uint32_t l1d_lru[L1D_SETS][L1D_WAYS];
static uint32_t l2_lru[L2_SETS][L2_WAYS];
static cache_stats_t l1i_stats, l1d_stats, l2_stats_g;
static replacement_policy_e g_policy;
static uint32_t lru_clock = 0;

static inline uint64_t line_offset(uint64_t a) { return a & LINE_MASK; }
static inline uint64_t l1i_idx(uint64_t a) {
    return (a >> LINE_BITS) & (L1I_SETS - 1);
}
static inline uint64_t l1i_tag(uint64_t a) {
    return a >> (LINE_BITS + L1I_IDX_BITS);
}
static inline uint64_t l1i_base(uint64_t s, uint64_t t) {
    return (t << (LINE_BITS + L1I_IDX_BITS)) | (s << LINE_BITS);
}
static inline uint64_t l1d_idx(uint64_t a) {
    return (a >> LINE_BITS) & (L1D_SETS - 1);
}
static inline uint64_t l1d_tag(uint64_t a) {
    return a >> (LINE_BITS + L1D_IDX_BITS);
}
static inline uint64_t l1d_base(uint64_t s, uint64_t t) {
    return (t << (LINE_BITS + L1D_IDX_BITS)) | (s << LINE_BITS);
}
static inline uint64_t l2_idx(uint64_t a) {
    return (a >> LINE_BITS) & (L2_SETS - 1);
}
static inline uint64_t l2_tag(uint64_t a) {
    return a >> (LINE_BITS + L2_IDX_BITS);
}
static inline uint64_t l2_base(uint64_t s, uint64_t t) {
    return (t << (LINE_BITS + L2_IDX_BITS)) | (s << LINE_BITS);
}

static int pick_victim(cache_line_t *ways, uint32_t *stamps, int n) {
    for (int w = 0; w < n; w++)
        if (!ways[w].valid)
            return w;
    if (g_policy == RANDOM)
        return rand() % n;
    int v = 0;
    for (int w = 1; w < n; w++)
        if (stamps[w] < stamps[v])
            v = w;
    return v;
}

static void l1i_invalidate(uint64_t base) {
    uint64_t s = l1i_idx(base), t = l1i_tag(base);
    if (l1i[s][0].valid && l1i[s][0].tag == t)
        l1i[s][0].valid = 0;
}

/* Silently update L2 with dirty L1-D data (coherency flush, NOT an L2 access)
 */
static void coherency_flush_to_l2(uint64_t base, uint8_t *data) {
    uint64_t s = l2_idx(base), t = l2_tag(base);
    for (int w = 0; w < L2_WAYS; w++) {
        if (l2[s][w].valid && l2[s][w].tag == t) {
            memcpy(l2[s][w].data, data, LINE_SIZE);
            l2[s][w].modified = 1;
            /* Do NOT update lru_stamp - this is not an access */
            return;
        }
    }
    /* L2 doesn't have it - just update main memory directly and clear dirty */
    for (int b = 0; b < LINE_SIZE; b++)
        write_memory(base + b, data[b]);
}

static void l2_evict_inclusive(uint64_t s, int v) {
    cache_line_t *lc = &l2[s][v];
    if (!lc->valid)
        return;
    uint64_t base = l2_base(s, lc->tag);
    if (lc->modified) {
        for (int b = 0; b < LINE_SIZE; b++)
            write_memory(base + b, lc->data[b]);
        lc->modified = 0;
    }
    l1i_invalidate(base);
    {
        uint64_t ds = l1d_idx(base), dt = l1d_tag(base);
        for (int w = 0; w < L1D_WAYS; w++) {
            if (l1d[ds][w].valid && l1d[ds][w].tag == dt) {
                if (l1d[ds][w].modified) {
                    for (int b = 0; b < LINE_SIZE; b++)
                        write_memory(base + b, l1d[ds][w].data[b]);
                    l1d[ds][w].modified = 0;
                }
                l1d[ds][w].valid = 0;
                break;
            }
        }
    }
    lc->valid = 0;
}

/* Counted L2 access for eviction writebacks */
static void writeback_l1d_to_l2(uint64_t base, uint8_t *data) {
    uint64_t s = l2_idx(base), t = l2_tag(base);
    l2_stats_g.accesses++;
    for (int w = 0; w < L2_WAYS; w++) {
        if (l2[s][w].valid && l2[s][w].tag == t) {
            memcpy(l2[s][w].data, data, LINE_SIZE);
            l2[s][w].modified = 1;
            l2_lru[s][w] = ++lru_clock;
            return;
        }
    }
    l2_stats_g.misses++;
    int v = pick_victim(l2[s], l2_lru[s], L2_WAYS);
    l2_evict_inclusive(s, v);
    l2[s][v].valid = 1;
    l2[s][v].modified = 1;
    l2[s][v].tag = t;
    l2_lru[s][v] = ++lru_clock;
    memcpy(l2[s][v].data, data, LINE_SIZE);
}

static cache_line_t *fill_from_l2(uint64_t mem_addr) {
    uint64_t base = mem_addr & ~LINE_MASK, s = l2_idx(base), t = l2_tag(base);
    l2_stats_g.accesses++;
    for (int w = 0; w < L2_WAYS; w++) {
        if (l2[s][w].valid && l2[s][w].tag == t) {
            l2_lru[s][w] = ++lru_clock;
            return &l2[s][w];
        }
    }
    l2_stats_g.misses++;
    int v = pick_victim(l2[s], l2_lru[s], L2_WAYS);
    l2_evict_inclusive(s, v);
    l2[s][v].valid = 1;
    l2[s][v].modified = 0;
    l2[s][v].tag = t;
    l2_lru[s][v] = ++lru_clock;
    for (int b = 0; b < LINE_SIZE; b++)
        l2[s][v].data[b] = read_memory(base + b);
    return &l2[s][v];
}

void init_cache(replacement_policy_e policy) {
    g_policy = policy;
    lru_clock = 0;
    srand(0);
    memset(l1i, 0, sizeof(l1i));
    memset(l1d, 0, sizeof(l1d));
    memset(l2, 0, sizeof(l2));
    memset(l1i_lru, 0, sizeof(l1i_lru));
    memset(l1d_lru, 0, sizeof(l1d_lru));
    memset(l2_lru, 0, sizeof(l2_lru));
    memset(&l1i_stats, 0, sizeof(l1i_stats));
    memset(&l1d_stats, 0, sizeof(l1d_stats));
    memset(&l2_stats_g, 0, sizeof(l2_stats_g));
}

uint8_t read_cache(uint64_t mem_addr, mem_type_t type) {
    uint64_t off = line_offset(mem_addr), base = mem_addr & ~LINE_MASK;
    if (type == INSTR) {
        uint64_t s = l1i_idx(mem_addr), t = l1i_tag(mem_addr);
        l1i_stats.accesses++;
        if (l1i[s][0].valid && l1i[s][0].tag == t) {
            l1i_lru[s][0] = ++lru_clock;
            return l1i[s][0].data[off];
        }
        l1i_stats.misses++;
        /* Coherency: silently flush dirty L1-D to L2 (NOT an L2 access) */
        {
            uint64_t ds = l1d_idx(base), dt = l1d_tag(base);
            for (int w = 0; w < L1D_WAYS; w++) {
                if (l1d[ds][w].valid && l1d[ds][w].tag == dt &&
                    l1d[ds][w].modified) {
                    coherency_flush_to_l2(base, l1d[ds][w].data);
                    l1d[ds][w].modified = 0;
                    break;
                }
            }
        }
        cache_line_t *l2line = fill_from_l2(mem_addr);
        l1i[s][0].valid = 1;
        l1i[s][0].modified = 0;
        l1i[s][0].tag = t;
        l1i_lru[s][0] = ++lru_clock;
        memcpy(l1i[s][0].data, l2line->data, LINE_SIZE);
        return l1i[s][0].data[off];
    } else {
        uint64_t s = l1d_idx(mem_addr), t = l1d_tag(mem_addr);
        l1d_stats.accesses++;
        for (int w = 0; w < L1D_WAYS; w++) {
            if (l1d[s][w].valid && l1d[s][w].tag == t) {
                l1d_lru[s][w] = ++lru_clock;
                return l1d[s][w].data[off];
            }
        }
        l1d_stats.misses++;
        cache_line_t *l2line = fill_from_l2(mem_addr);
        int v = pick_victim(l1d[s], l1d_lru[s], L1D_WAYS);
        if (l1d[s][v].valid && l1d[s][v].modified) {
            writeback_l1d_to_l2(l1d_base(s, l1d[s][v].tag), l1d[s][v].data);
        }
        l1d[s][v].valid = 1;
        l1d[s][v].modified = 0;
        l1d[s][v].tag = t;
        l1d_lru[s][v] = ++lru_clock;
        memcpy(l1d[s][v].data, l2line->data, LINE_SIZE);
        return l1d[s][v].data[off];
    }
}

void write_cache(uint64_t mem_addr, uint8_t value, mem_type_t type) {
    uint64_t off = line_offset(mem_addr), base = mem_addr & ~LINE_MASK;
    uint64_t s = l1d_idx(mem_addr), t = l1d_tag(mem_addr);
    l1d_stats.accesses++;
    l1i_invalidate(base);
    for (int w = 0; w < L1D_WAYS; w++) {
        if (l1d[s][w].valid && l1d[s][w].tag == t) {
            l1d[s][w].data[off] = value;
            l1d[s][w].modified = 1;
            l1d_lru[s][w] = ++lru_clock;
            return;
        }
    }
    l1d_stats.misses++;
    cache_line_t *l2line = fill_from_l2(mem_addr);
    int v = pick_victim(l1d[s], l1d_lru[s], L1D_WAYS);
    if (l1d[s][v].valid && l1d[s][v].modified) {
        writeback_l1d_to_l2(l1d_base(s, l1d[s][v].tag), l1d[s][v].data);
    }
    l1d[s][v].valid = 1;
    l1d[s][v].modified = 1;
    l1d[s][v].tag = t;
    l1d_lru[s][v] = ++lru_clock;
    memcpy(l1d[s][v].data, l2line->data, LINE_SIZE);
    l1d[s][v].data[off] = value;
}

cache_stats_t get_l1_instr_stats() { return l1i_stats; }
cache_stats_t get_l1_data_stats() { return l1d_stats; }
cache_stats_t get_l2_stats() { return l2_stats_g; }

cache_line_t *get_l1_instr_cache_line(uint64_t a) {
    uint64_t s = l1i_idx(a), t = l1i_tag(a);
    if (l1i[s][0].valid && l1i[s][0].tag == t)
        return &l1i[s][0];
    return NULL;
}
cache_line_t *get_l1_data_cache_line(uint64_t a) {
    uint64_t s = l1d_idx(a), t = l1d_tag(a);
    for (int w = 0; w < L1D_WAYS; w++)
        if (l1d[s][w].valid && l1d[s][w].tag == t)
            return &l1d[s][w];
    return NULL;
}
cache_line_t *get_l2_cache_line(uint64_t a) {
    uint64_t s = l2_idx(a), t = l2_tag(a);
    for (int w = 0; w < L2_WAYS; w++)
        if (l2[s][w].valid && l2[s][w].tag == t)
            return &l2[s][w];
    return NULL;
}