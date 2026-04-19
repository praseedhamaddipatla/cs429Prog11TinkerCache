#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>

#define START 0x1000
#define MEM_SIZE (1 << 19)
#define REG 32
#define INC 4
#define DEBUG 1

/* =========================================================
 * CACHE CONFIGURATION
 * =========================================================
 *
 * L1 Instruction: 32KB, direct-mapped, 64B lines
 *   sets  = 32768 / 64 = 512,  ways = 1
 *
 * L1 Data:        32KB, 2-way set-associative, 64B lines
 *   sets  = 32768 / (64*2) = 256,  ways = 2
 *
 * L2 Combined:    2MB,  4-way set-associative, 64B lines
 *   sets  = 2097152 / (64*4) = 8192, ways = 4
 *
 * Replacement:
 *   Phase 1 – LRU  (REPLACEMENT_POLICY == 0)
 *   Phase 2 – Random (REPLACEMENT_POLICY == 1)
 * ========================================================= */

#define CACHE_LINE_SIZE   64        /* bytes per cache line              */
#define L1I_SETS          512       /* L1-I: direct mapped               */
#define L1I_WAYS          1
#define L1D_SETS          256       /* L1-D: 2-way SA                    */
#define L1D_WAYS          2
#define L2_SETS           8192      /* L2:   4-way SA                    */
#define L2_WAYS           4

/* Set to 0 for LRU, 1 for Random */
#ifndef REPLACEMENT_POLICY
#define REPLACEMENT_POLICY 0
#endif

/* ---- cache line ---- */
typedef struct {
    uint8_t  valid;
    uint8_t  modified;        /* dirty bit (write-back) */
    uint64_t tag;
    uint8_t  data[CACHE_LINE_SIZE];
    uint32_t lru_counter;     /* for LRU: lower == older */
} CacheLine;

/* ---- cache ---- */
typedef struct {
    int       num_sets;
    int       num_ways;
    CacheLine *lines;         /* [num_sets * num_ways] */
    /* statistics */
    uint64_t  hits;
    uint64_t  misses;
    const char *name;
} Cache;

/* global caches */
static Cache l1i, l1d, l2;
static uint32_t lru_clock = 0;   /* global counter for LRU timestamps */

/* ---- machine state ---- */
static uint64_t pc;
static int running;
static uint64_t regs[REG];
static uint8_t mem[MEM_SIZE];

/* =========================================================
 * CACHE INIT / HELPERS
 * ========================================================= */

static void cache_init(Cache *c, int sets, int ways, const char *name) {
    c->num_sets = sets;
    c->num_ways = ways;
    c->hits     = 0;
    c->misses   = 0;
    c->name     = name;
    c->lines    = (CacheLine *)calloc(sets * ways, sizeof(CacheLine));
    if (!c->lines) {
        fprintf(stderr, "cache_init: out of memory\n");
        exit(1);
    }
}

static inline CacheLine *cache_line(Cache *c, int set, int way) {
    return &c->lines[set * c->num_ways + way];
}

/* Returns offset-within-line bits (log2 of line size = 6) */
#define LINE_OFFSET_BITS  6
#define LINE_OFFSET_MASK  (CACHE_LINE_SIZE - 1)

static inline int  addr_set(uint64_t addr, int num_sets) {
    return (int)((addr >> LINE_OFFSET_BITS) & (num_sets - 1));
}
static inline uint64_t addr_tag(uint64_t addr, int num_sets) {
    /* bits above the set-index field */
    int set_bits = 0;
    int s = num_sets;
    while (s > 1) { set_bits++; s >>= 1; }
    return addr >> (LINE_OFFSET_BITS + set_bits);
}

/* pick victim way for replacement */
static int pick_victim(Cache *c, int set) {
#if REPLACEMENT_POLICY == 1
    /* Random */
    return rand() % c->num_ways;
#else
    /* LRU: find way with smallest lru_counter */
    int victim = 0;
    uint32_t oldest = cache_line(c, set, 0)->lru_counter;
    for (int w = 1; w < c->num_ways; w++) {
        uint32_t cnt = cache_line(c, set, w)->lru_counter;
        if (cnt < oldest) { oldest = cnt; victim = w; }
    }
    return victim;
#endif
}

/* Write a dirty L1 line back to L2 (not to main memory yet) */
static void writeback_l1_to_l2(Cache *l2c, uint64_t tag, int l1_set,
                                int l1_set_bits, CacheLine *cl) {
    /* Reconstruct the base address of this cache line */
    uint64_t base = (tag << (LINE_OFFSET_BITS + l1_set_bits)) |
                    ((uint64_t)l1_set << LINE_OFFSET_BITS);

    int set2 = addr_set(base, l2c->num_sets);
    uint64_t tag2 = addr_tag(base, l2c->num_sets);

    /* find in L2 */
    for (int w = 0; w < l2c->num_ways; w++) {
        CacheLine *lc = cache_line(l2c, set2, w);
        if (lc->valid && lc->tag == tag2) {
            memcpy(lc->data, cl->data, CACHE_LINE_SIZE);
            lc->modified = 1;
            return;
        }
    }
    /* not in L2 – install it */
    int victim = pick_victim(l2c, set2);
    CacheLine *lc = cache_line(l2c, set2, victim);
    if (lc->valid && lc->modified) {
        /* write L2 dirty line back to main memory */
        uint64_t l2_base = (lc->tag << (LINE_OFFSET_BITS + 13)) |
                           ((uint64_t)set2 << LINE_OFFSET_BITS);
        if (l2_base + CACHE_LINE_SIZE <= MEM_SIZE)
            memcpy(&mem[l2_base], lc->data, CACHE_LINE_SIZE);
    }
    lc->valid    = 1;
    lc->modified = 1;
    lc->tag      = tag2;
    memcpy(lc->data, cl->data, CACHE_LINE_SIZE);
    lc->lru_counter = ++lru_clock;
}

/* Fill a cache line from L2 (or main memory if not in L2) */
static void fill_from_l2(Cache *l2c, uint64_t base_addr,
                          uint8_t *out_data) {
    int set2 = addr_set(base_addr, l2c->num_sets);
    uint64_t tag2 = addr_tag(base_addr, l2c->num_sets);

    /* search L2 */
    for (int w = 0; w < l2c->num_ways; w++) {
        CacheLine *lc = cache_line(l2c, set2, w);
        if (lc->valid && lc->tag == tag2) {
            l2c->hits++;
            memcpy(out_data, lc->data, CACHE_LINE_SIZE);
            lc->lru_counter = ++lru_clock;
            return;
        }
    }

    /* L2 miss – fetch from main memory */
    l2c->misses++;
    uint64_t aligned = base_addr & ~(uint64_t)LINE_OFFSET_MASK;
    if (aligned + CACHE_LINE_SIZE <= MEM_SIZE)
        memcpy(out_data, &mem[aligned], CACHE_LINE_SIZE);
    else
        memset(out_data, 0, CACHE_LINE_SIZE);

    /* install in L2 */
    int victim = pick_victim(l2c, set2);
    CacheLine *lc = cache_line(l2c, set2, victim);
    if (lc->valid && lc->modified) {
        /* evict dirty L2 line to main memory */
        int set2_bits = 13; /* log2(8192) */
        uint64_t l2_base = (lc->tag << (LINE_OFFSET_BITS + set2_bits)) |
                           ((uint64_t)set2 << LINE_OFFSET_BITS);
        if (l2_base + CACHE_LINE_SIZE <= MEM_SIZE)
            memcpy(&mem[l2_base], lc->data, CACHE_LINE_SIZE);
    }
    lc->valid       = 1;
    lc->modified    = 0;
    lc->tag         = tag2;
    lc->lru_counter = ++lru_clock;
    memcpy(lc->data, out_data, CACHE_LINE_SIZE);
}

/* =========================================================
 * CACHE READ/WRITE  (called by simulator instead of direct
 * memory access)
 * ========================================================= */

/* Read 8 bytes through the cache hierarchy */
static uint64_t cache_load64(uint64_t addr) {
    if (addr + 8 > MEM_SIZE) {
        fprintf(stderr, "Simulation error\n");
        exit(1);
    }

    uint64_t aligned = addr & ~(uint64_t)LINE_OFFSET_MASK;
    int      offset  = (int)(addr & LINE_OFFSET_MASK);
    int      set     = addr_set(addr, l1d.num_sets);
    uint64_t tag     = addr_tag(addr, l1d.num_sets);

    /* Search L1-D */
    for (int w = 0; w < l1d.num_ways; w++) {
        CacheLine *cl = cache_line(&l1d, set, w);
        if (cl->valid && cl->tag == tag) {
            l1d.hits++;
            cl->lru_counter = ++lru_clock;
            uint64_t v = 0;
            for (int i = 0; i < 8; i++)
                v |= (uint64_t)cl->data[offset + i] << (8 * i);
            return v;
        }
    }

    /* L1-D miss */
    l1d.misses++;

    /* fetch line from L2/mem */
    uint8_t line_data[CACHE_LINE_SIZE];
    fill_from_l2(&l2, aligned, line_data);

    /* install in L1-D (evict victim if needed) */
    int victim = pick_victim(&l1d, set);
    CacheLine *cl = cache_line(&l1d, set, victim);
    if (cl->valid && cl->modified) {
        /* write-back dirty L1 line to L2 */
        int l1d_set_bits = 8; /* log2(256) */
        writeback_l1_to_l2(&l2, cl->tag, set, l1d_set_bits, cl);
    }
    cl->valid       = 1;
    cl->modified    = 0;
    cl->tag         = tag;
    cl->lru_counter = ++lru_clock;
    memcpy(cl->data, line_data, CACHE_LINE_SIZE);

    uint64_t v = 0;
    for (int i = 0; i < 8; i++)
        v |= (uint64_t)cl->data[offset + i] << (8 * i);
    return v;
}

/* Write 8 bytes through the cache hierarchy (write-back, write-allocate) */
static void cache_store64(uint64_t addr, uint64_t val) {
    if (addr % 8 != 0 || addr + 7 >= MEM_SIZE) {
        fprintf(stderr, "Simulation error\n");
        exit(1);
    }

    uint64_t aligned = addr & ~(uint64_t)LINE_OFFSET_MASK;
    int      offset  = (int)(addr & LINE_OFFSET_MASK);
    int      set     = addr_set(addr, l1d.num_sets);
    uint64_t tag     = addr_tag(addr, l1d.num_sets);

    /* Search L1-D */
    for (int w = 0; w < l1d.num_ways; w++) {
        CacheLine *cl = cache_line(&l1d, set, w);
        if (cl->valid && cl->tag == tag) {
            l1d.hits++;
            cl->lru_counter = ++lru_clock;
            cl->modified = 1;
            for (int i = 0; i < 8; i++)
                cl->data[offset + i] = (val >> (8 * i)) & 0xFF;
            /* also update main memory directly for safety */
            for (int i = 0; i < 8; i++)
                mem[addr + i] = (val >> (8 * i)) & 0xFF;
            return;
        }
    }

    /* L1-D miss – write-allocate: fetch line, then write */
    l1d.misses++;

    uint8_t line_data[CACHE_LINE_SIZE];
    fill_from_l2(&l2, aligned, line_data);

    int victim = pick_victim(&l1d, set);
    CacheLine *cl = cache_line(&l1d, set, victim);
    if (cl->valid && cl->modified) {
        int l1d_set_bits = 8;
        writeback_l1_to_l2(&l2, cl->tag, set, l1d_set_bits, cl);
    }
    cl->valid       = 1;
    cl->modified    = 1;
    cl->tag         = tag;
    cl->lru_counter = ++lru_clock;
    memcpy(cl->data, line_data, CACHE_LINE_SIZE);

    for (int i = 0; i < 8; i++)
        cl->data[offset + i] = (val >> (8 * i)) & 0xFF;
    /* write through to main memory for correctness */
    for (int i = 0; i < 8; i++)
        mem[addr + i] = (val >> (8 * i)) & 0xFF;
}

/* Fetch a 4-byte instruction through L1-I */
static uint32_t cache_fetch_instr(uint64_t ipc) {
    if (ipc + 3 >= MEM_SIZE) {
        fprintf(stderr, "Simulation error\n");
        exit(1);
    }

    int      offset = (int)(ipc & LINE_OFFSET_MASK);
    int      set    = addr_set(ipc, l1i.num_sets);
    uint64_t tag    = addr_tag(ipc, l1i.num_sets);

    /* L1-I is direct-mapped (ways == 1) */
    CacheLine *cl = cache_line(&l1i, set, 0);

    if (cl->valid && cl->tag == tag) {
        l1i.hits++;
        cl->lru_counter = ++lru_clock;
    } else {
        /* miss */
        l1i.misses++;

        uint64_t aligned = ipc & ~(uint64_t)LINE_OFFSET_MASK;
        uint8_t line_data[CACHE_LINE_SIZE];
        fill_from_l2(&l2, aligned, line_data);

        /* instructions are read-only: no write-back needed */
        cl->valid       = 1;
        cl->modified    = 0;
        cl->tag         = tag;
        cl->lru_counter = ++lru_clock;
        memcpy(cl->data, line_data, CACHE_LINE_SIZE);
    }

    return (uint32_t)cl->data[offset]
        | ((uint32_t)cl->data[offset + 1] << 8)
        | ((uint32_t)cl->data[offset + 2] << 16)
        | ((uint32_t)cl->data[offset + 3] << 24);
}

/* Print cache statistics */
static void print_cache_stats(void) {
    printf("\n=== Cache Statistics (%s) ===\n",
           REPLACEMENT_POLICY ? "Random" : "LRU");

    printf("L1-I  hits: %llu  misses: %llu  total: %llu  miss-rate: %.2f%%\n",
           (unsigned long long)l1i.hits,
           (unsigned long long)l1i.misses,
           (unsigned long long)(l1i.hits + l1i.misses),
           (l1i.hits + l1i.misses) ?
               100.0 * l1i.misses / (l1i.hits + l1i.misses) : 0.0);

    printf("L1-D  hits: %llu  misses: %llu  total: %llu  miss-rate: %.2f%%\n",
           (unsigned long long)l1d.hits,
           (unsigned long long)l1d.misses,
           (unsigned long long)(l1d.hits + l1d.misses),
           (l1d.hits + l1d.misses) ?
               100.0 * l1d.misses / (l1d.hits + l1d.misses) : 0.0);

    printf("L2    hits: %llu  misses: %llu  total: %llu  miss-rate: %.2f%%\n",
           (unsigned long long)l2.hits,
           (unsigned long long)l2.misses,
           (unsigned long long)(l2.hits + l2.misses),
           (l2.hits + l2.misses) ?
               100.0 * l2.misses / (l2.hits + l2.misses) : 0.0);
}

/* =========================================================
 * ORIGINAL SIMULATOR CODE  (memory accesses now go through
 * cache_load64 / cache_store64 / cache_fetch_instr)
 * ========================================================= */

void initMachine(void) {
    memset(mem, 0, sizeof(mem));
    memset(regs, 0, sizeof(regs));
    regs[31] = MEM_SIZE;
    pc = START;
    running = 1;

    /* initialise caches */
    srand((unsigned)time(NULL));
    cache_init(&l1i, L1I_SETS, L1I_WAYS, "L1-I");
    cache_init(&l1d, L1D_SETS, L1D_WAYS, "L1-D");
    cache_init(&l2,  L2_SETS,  L2_WAYS,  "L2");
}

/* helpers */
static uint32_t getOpcode(uint32_t i) { return (i >> 27) & 0x1F; }
static uint32_t getrd(uint32_t i) { return (i >> 22) & 0x1F; }
static uint32_t getrs(uint32_t i) { return (i >> 17) & 0x1F; }
static uint32_t getrt(uint32_t i) { return (i >> 12) & 0x1F; }
static inline uint64_t getImm(uint32_t instr) { return instr & 0xFFF; }

static int32_t getL(uint32_t i) {
    int32_t imm = i & 0xFFF;
    if (imm & 0x800)
        imm |= ~0xFFF;
    return imm;
}

/* memory helpers – now backed by cache */
uint64_t load64(uint64_t addr) {
    if (addr > MEM_SIZE - 8) {
        fprintf(stderr, "Simulation error\n");
        exit(1);
    }
    return cache_load64(addr);
}

void store64(uint64_t addr, uint64_t val) {
    if (addr % 8 != 0 || addr + 7 >= MEM_SIZE) {
        fprintf(stderr, "Simulation error\n");
        exit(1);
    }
    cache_store64(addr, val);
}

/* fetch – now through L1-I */
uint32_t fetchInstr(void) {
    return cache_fetch_instr(pc);
}

/* execution helpers */
#define NEXT pc += INC

/* logic */
void execAnd(uint32_t i) {
    regs[getrd(i)] = regs[getrs(i)] & regs[getrt(i)];
    NEXT;
}
void execOr(uint32_t i) {
    regs[getrd(i)] = regs[getrs(i)] | regs[getrt(i)];
    NEXT;
}
void execXor(uint32_t i) {
    regs[getrd(i)] = regs[getrs(i)] ^ regs[getrt(i)];
    NEXT;
}
void execNot(uint32_t i) {
    regs[getrd(i)] = ~regs[getrs(i)];
    NEXT;
}

/* shifts */
void execShftr(uint32_t i) {
    regs[getrd(i)] = regs[getrs(i)] >> regs[getrt(i)];
    NEXT;
}
void execShftri(uint32_t i) {
    regs[getrd(i)] >>= getL(i);
    NEXT;
}
void execShftl(uint32_t i) {
    regs[getrd(i)] = regs[getrs(i)] << regs[getrt(i)];
    NEXT;
}
void execShftli(uint32_t i) {
    regs[getrd(i)] <<= getL(i);
    NEXT;
}

/* arithmetic */
void execAdd(uint32_t i) {
    regs[getrd(i)] = regs[getrs(i)] + regs[getrt(i)];
    NEXT;
}
void execAddi(uint32_t i) {
    regs[getrd(i)] += getL(i);
    NEXT;
}
void execSub(uint32_t i) {
    regs[getrd(i)] = regs[getrs(i)] - regs[getrt(i)];
    NEXT;
}
void execSubi(uint32_t i) {
    regs[getrd(i)] -= getL(i);
    NEXT;
}
void execMul(uint32_t i) {
    regs[getrd(i)] = regs[getrs(i)] * regs[getrt(i)];
    NEXT;
}
void execDiv(uint32_t i) {
    if (!regs[getrt(i)]) {
        fprintf(stderr, "Simulation error\n");
        exit(1);
    }
    regs[getrd(i)] = (int64_t)regs[getrs(i)] / (int64_t)regs[getrt(i)];
    NEXT;
}

/* mov */
void execMovLoad(uint32_t i) {
    uint64_t addr = regs[getrs(i)] + getL(i);
    regs[getrd(i)] = load64(addr);
    NEXT;
}
void execMovStore(uint32_t i) {
    uint64_t addr = regs[getrd(i)] + getL(i);
    store64(addr, regs[getrs(i)]);
    NEXT;
}
void execMovReg(uint32_t i) {
    regs[getrd(i)] = regs[getrs(i)];
    NEXT;
}
void execMovImm(uint32_t i) {
    uint32_t rd = getrd(i);
    uint64_t L  = getImm(i);
    regs[rd] = (regs[rd] & ~0xFFFULL) | L;
    NEXT;
}

/* control */
void execBrgt(uint32_t instr) {
    uint32_t rd = getrd(instr);
    uint32_t rs = getrs(instr);
    uint32_t rt = getrt(instr);
    int64_t v1 = (int64_t)regs[rs];
    int64_t v2 = (int64_t)regs[rt];
    if (v1 > v2)
        pc = regs[rd];
    else
        pc = pc + INC;
}

/* priv */
void execPriv(uint32_t i) {
    uint32_t L = getImm(i);
    switch (L) {
    case 0x0:
        print_cache_stats();
        exit(0);

    case 0x3: {
        uint32_t rd = getrd(i);
        uint32_t rs = getrs(i);
        uint64_t p  = regs[rs];
        if (p == 0) {
            char buf[256];
            if (!fgets(buf, sizeof(buf), stdin)) {
                fprintf(stderr, "Simulation error\n");
                exit(1);
            }
            char *end;
            errno = 0;
            unsigned long long val = strtoull(buf, &end, 10);
            if (errno == ERANGE || end == buf) {
                fprintf(stderr, "Simulation error\n");
                exit(1);
            }
            if (buf[0] == '-') {
                fprintf(stderr, "Simulation error\n");
                exit(1);
            }
            while (*end == ' ' || *end == '\t') end++;
            if (*end != '\n' && *end != '\0') {
                fprintf(stderr, "Simulation error\n");
                exit(1);
            }
            regs[rd] = val;
        }
        pc += INC;
        return;
    }

    case 0x4: {
        uint32_t rd = getrd(i);
        uint32_t rs = getrs(i);
        uint64_t p  = regs[rd];
        if (p == 1)
            printf("%lu\n", (long unsigned int)regs[rs]);
        pc = pc + INC;
        return;
    }

    default:
        fprintf(stderr, "Simulation error\n");
        exit(1);
    }
}

/* floating point */
void execAddf(uint32_t i) {
    double a, b, c;
    memcpy(&a, &regs[getrs(i)], 8);
    memcpy(&b, &regs[getrt(i)], 8);
    c = a + b;
    memcpy(&regs[getrd(i)], &c, 8);
    pc += INC;
}
void execSubf(uint32_t i) {
    double a, b, c;
    memcpy(&a, &regs[getrs(i)], 8);
    memcpy(&b, &regs[getrt(i)], 8);
    c = a - b;
    memcpy(&regs[getrd(i)], &c, 8);
    pc += INC;
}
void execMulf(uint32_t i) {
    double a, b, c;
    memcpy(&a, &regs[getrs(i)], 8);
    memcpy(&b, &regs[getrt(i)], 8);
    c = a * b;
    memcpy(&regs[getrd(i)], &c, 8);
    pc += INC;
}
void execDivf(uint32_t i) {
    double a, b, c;
    memcpy(&a, &regs[getrs(i)], 8);
    memcpy(&b, &regs[getrt(i)], 8);
    if (b == 0.0) {
        fprintf(stderr, "Simulation error\n");
        exit(1);
    }
    c = a / b;
    memcpy(&regs[getrd(i)], &c, 8);
    pc += INC;
}

/* branch */
void execBr(uint32_t i)    { pc = regs[getrd(i)]; }
void execBrrReg(uint32_t i){ pc += regs[getrd(i)]; }
void execBrrImm(uint32_t i){ pc += getL(i); }
void execBrnz(uint32_t i) {
    if (regs[getrs(i)] == 0) NEXT;
    else pc = regs[getrd(i)];
}
void execCall(uint32_t i) {
    uint64_t retAddr = pc + INC;
    store64(regs[31] - 8, retAddr);
    pc = regs[getrd(i)];
}
void execReturn(void) {
    uint64_t retAddr = load64(regs[31] - 8);
    pc = retAddr;
}

/* main loop */
void runSim(void) {
    while (running) {
        uint32_t i  = fetchInstr();
        uint32_t op = getOpcode(i);

        switch (op) {
        case 0x00: execAnd(i);     break;
        case 0x01: execOr(i);      break;
        case 0x02: execXor(i);     break;
        case 0x03: execNot(i);     break;
        case 0x04: execShftr(i);   break;
        case 0x05: execShftri(i);  break;
        case 0x06: execShftl(i);   break;
        case 0x07: execShftli(i);  break;
        case 0x18: execAdd(i);     break;
        case 0x19: execAddi(i);    break;
        case 0x1A: execSub(i);     break;
        case 0x1B: execSubi(i);    break;
        case 0x1C: execMul(i);     break;
        case 0x1D: execDiv(i);     break;
        case 0x10: execMovLoad(i); break;
        case 0x11: execMovReg(i);  break;
        case 0x12: execMovImm(i);  break;
        case 0x13: execMovStore(i);break;
        case 0x0E: execBrgt(i);    break;
        case 0x0F: execPriv(i);    break;
        case 0x14: execAddf(i);    break;
        case 0x15: execSubf(i);    break;
        case 0x16: execMulf(i);    break;
        case 0x17: execDivf(i);    break;
        case 0x08: execBr(i);      break;
        case 0x09: execBrrReg(i);  break;
        case 0x0A: execBrrImm(i);  break;
        case 0x0B: execBrnz(i);    break;
        case 0x0C: execCall(i);    break;
        case 0x0D: execReturn();   break;
        default:
            fprintf(stderr, "Simulation error\n");
            exit(1);
        }
    }
}

/* load file */
int procFile(const char *file) {
    FILE *f = fopen(file, "rb");
    if (!f) return 1;

    uint64_t a = START;
    int c;
    while ((c = fgetc(f)) != EOF)
        mem[a++] = c;

    fclose(f);
    return 0;
}

/* main */
int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <file>\n", argv[0]);
        return 1;
    }

    initMachine();

    if (procFile(argv[1]) != 0) {
        fprintf(stderr, "Invalid tinker filepath\n");
        return 1;
    }

    runSim();
    return 0;
}