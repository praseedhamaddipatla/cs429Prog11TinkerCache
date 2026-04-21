# cs429Prog11TinkerCache
Tinker simulator connected to cache implementation between simulator and memory.

Name: Praseedha Maddipatla

EID: psm2357

To compile:./build.sh

To run:./build/hw11

To compare the LRU and Random replacement policies, I designed two targeted access patterns that stress the L1-D cache in different ways. The first test (Test 5) performs four sequential passes over a 64 KB array using both policies, with Random averaged over 10 runs to reduce noise from its nondeterminism. The second test (Test 6) constructs an explicit thrashing pattern by repeatedly cycling through 3 addresses that all map to the same 2-way L1-D set — one more address than there are ways, which is the classic worst case for any deterministic policy. Both tests measure L1-D miss counts as the primary metric, since that is where the two policies diverge most visibly at this cache size.

Both tests showed Random outperforming LRU. On the 64 KB sequential scan, LRU produced 4096 L1-D misses across all four passes while Random averaged approximately 3651 — because the 64 KB working set maps exactly 4 lines to every 2-way set, creating a thrashing condition where LRU deterministically evicts the line that will be needed soonest on the next pass. On the explicit thrash pattern, LRU missed on every single one of the 6000 accesses (100% miss rate) while Random averaged around 4002 misses (67%), since Random occasionally keeps a useful line by chance. These results confirm the well-known weakness of LRU: it performs optimally when there is genuine temporal locality, but degrades badly under adversarial or capacity-exceeding access patterns, where Random's unpredictability becomes an advantage.