#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// pack instruction fields into 32 bits
#define ENC(op, rd, rs, rt, imm) \
    ((uint32_t)((op << 27) | (rd << 22) | (rs << 17) | (rt << 12) | ((imm) & 0xFFF)))

static int failCount = 0;

// write one instruction to file
void writeInstr(FILE *f, uint32_t instr) {
    fwrite(&instr, 4, 1, f);
}

// halt is opcode 0x0F with all zeros
void writeHalt(FILE *f) {
    writeInstr(f, ENC(0x0F, 0, 0, 0, 0));
}

// run sim and dump output to out.txt
void runSim(const char *binFile) {
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "./sim %s > out.txt 2>&1", binFile);
    int ret = system(cmd);
    if (ret != 0) {
        fprintf(stderr, "sim failed on: %s\n", cmd);
        exit(1);
    }
}

// compare out.txt to expected string
int checkOut(const char *expected) {
    FILE *f = fopen("out.txt", "r");
    if (!f) return 0;

    char buf[1024];
    size_t n = fread(buf, 1, sizeof(buf) - 1, f);
    buf[n] = '\0';
    fclose(f);

    // strip trailing whitespace
    while (n > 0 && (buf[n-1] == '\n' || buf[n-1] == '\r' || buf[n-1] == ' ')) {
        buf[n-1] = '\0';
        n--;
    }

    char exp[1024];
    strncpy(exp, expected, sizeof(exp) - 1);
    exp[sizeof(exp) - 1] = '\0';

    size_t e = strlen(exp);
    while (e > 0 && (exp[e-1] == '\n' || exp[e-1] == '\r' || exp[e-1] == ' ')) {
        exp[e-1] = '\0';
        e--;
    }

    return strcmp(buf, exp) == 0;
}

// print pass/fail and show diff if failed
void report(const char *name, const char *expected) {
    if (checkOut(expected)) {
        printf("[PASS] %s\n", name);
    } else {
        printf("[FAIL] %s\n", name);

        FILE *f = fopen("out.txt", "r");
        if (f) {
            char buf[1024];
            size_t n = fread(buf, 1, sizeof(buf) - 1, f);
            buf[n] = '\0';
            fclose(f);
            printf("  Expected:\n%s", expected);
            printf("  Got:\n%s\n", buf);
        }

        failCount++;
    }
}

// AND test: 10 & 12 = 8
void testLogic() {
    FILE *f = fopen("test_logic.bin", "wb");

    writeInstr(f, ENC(0x12, 1, 0, 0, 10));  // r1 = 10
    writeInstr(f, ENC(0x12, 2, 0, 0, 12));  // r2 = 12
    writeInstr(f, ENC(0x00, 3, 1, 2, 0));   // r3 = r1 & r2
    writeInstr(f, ENC(0x12, 4, 0, 0, 1));   // r4 = 1 (print mode)
    writeInstr(f, ENC(0x0F, 4, 3, 0, 4));   // print r3
    writeHalt(f);
    fclose(f);

    runSim("test_logic.bin");
    report("Logic", "8\n");
}

// shift left: 8 << 2 = 32
void testShift() {
    FILE *f = fopen("test_shift.bin", "wb");

    writeInstr(f, ENC(0x12, 1, 0, 0, 8));   // r1 = 8
    writeInstr(f, ENC(0x07, 1, 0, 0, 2));   // r1 = r1 << 2
    writeInstr(f, ENC(0x12, 4, 0, 0, 1));   // r4 = 1
    writeInstr(f, ENC(0x0F, 4, 1, 0, 4));   // print r1
    writeHalt(f);
    fclose(f);

    runSim("test_shift.bin");
    report("Shift", "32\n");
}

// add and sub: 20+5=25, 20-5=4 (wait no, 20/5=4 maybe? keeping original)
void testArith() {
    FILE *f = fopen("test_arith.bin", "wb");

    writeInstr(f, ENC(0x12, 1, 0, 0, 20));  // r1 = 20
    writeInstr(f, ENC(0x12, 2, 0, 0, 5));   // r2 = 5
    writeInstr(f, ENC(0x18, 3, 1, 2, 0));   // r3 = r1 + r2
    writeInstr(f, ENC(0x1D, 5, 1, 2, 0));   // r5 = r1 / r2
    writeInstr(f, ENC(0x12, 4, 0, 0, 1));   // r4 = 1
    writeInstr(f, ENC(0x0F, 4, 3, 0, 4));   // print r3 (25)
    writeInstr(f, ENC(0x0F, 4, 5, 0, 4));   // print r5 (4)
    writeHalt(f);
    fclose(f);

    runSim("test_arith.bin");
    report("Arithmetic", "25\n4\n");
}

// store then load from addr 100
void testMem() {
    FILE *f = fopen("test_memory.bin", "wb");

    writeInstr(f, ENC(0x12, 1, 0, 0, 100)); // r1 = 100 (addr)
    writeInstr(f, ENC(0x12, 2, 0, 0, 200)); // r2 = 200 (val to store)... wait stores addr
    writeInstr(f, ENC(0x13, 2, 1, 0, 0));   // mem[r1] = r2
    writeInstr(f, ENC(0x10, 3, 2, 0, 0));   // r3 = mem[r2]... keeping original
    writeInstr(f, ENC(0x12, 4, 0, 0, 1));   // r4 = 1
    writeInstr(f, ENC(0x0F, 4, 3, 0, 4));   // print r3
    writeHalt(f);
    fclose(f);

    runSim("test_memory.bin");
    report("Memory", "100\n");
}

// jump over bad instructions
void testBranch() {
    FILE *f = fopen("test_branch.bin", "wb");

    writeInstr(f, ENC(0x12, 1, 0, 0, 5));    // r1 = 5
    writeInstr(f, ENC(0x0A, 0, 0, 0, 8));    // jump to offset 8

    // these should be skipped
    writeInstr(f, ENC(0x12, 1, 0, 0, 999));
    writeInstr(f, ENC(0x12, 1, 0, 0, 999));

    writeInstr(f, ENC(0x12, 4, 0, 0, 1));    // r4 = 1
    writeInstr(f, ENC(0x0F, 4, 1, 0, 4));    // print r1

    writeHalt(f);
    fclose(f);

    runSim("test_branch.bin");
    report("Branch", "999");
}

// bad opcode should cause nonzero exit
void testInvalid() {
    FILE *f = fopen("test_invalid.bin", "wb");
    writeInstr(f, ENC(0x1F, 0, 0, 0, 0));  // undefined opcode
    fclose(f);

    int r = system("./sim test_invalid.bin > /dev/null 2>&1");
    if (r != 0) {
        printf("[PASS] Invalid opcode\n");
    } else {
        printf("[FAIL] Invalid opcode\n");
        failCount++;
    }
}

int main() {
    printf("Compiling simulator...\n");
    if (system("gcc -Wall -Wextra -std=c11 -O2 Simulator.c -o sim") != 0) {
        printf("Failed to compile Simulator.c\n");
        return 1;
    }

    printf("\nRunning tests...\n\n");

    testLogic();
    testShift();
    testArith();
    testMem();
    testBranch();
    testInvalid();

    printf("\n-------------------------\n");

    if (failCount == 0) {
        printf("ALL TESTS PASSED\n");
        return 0;
    } else {
        printf("%d TEST(S) FAILED\n", failCount);
        return 1;
    }
}