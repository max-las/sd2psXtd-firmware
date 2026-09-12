#include "ps1_dirty.h"
#include "ps1_cardman.h"
#include "ps1_mc_data_interface.h"
#include "debug.h"
#ifdef WITH_PSRAM
#include <psram/psram.h>
#endif

#include "bigmem.h"
#define dirty_heap bigmem.ps1.dirty_heap
#define dirty_map bigmem.ps1.dirty_map

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#define PS1_BLOCK_SIZE 8192

spin_lock_t *ps1_dirty_spin_lock;
volatile uint32_t ps1_dirty_lockout;
int ps1_dirty_activity;

static int num_dirty;

static uint8_t flushbuf[PS1_BLOCK_SIZE];
static int flushbuf_sectors = 0;
static int flushbuf_first_sector = -1;
static int flushbuf_last_sector = -1;

#define SWAP(a, b) do { \
    uint16_t tmp = a; \
    a = b; \
    b = tmp; \
} while (0);

void ps1_dirty_init(void) {
    if (!ps1_dirty_spin_lock)
        ps1_dirty_spin_lock = spin_lock_init(spin_lock_claim_unused(1));
}

void __time_critical_func(ps1_dirty_mark)(uint32_t sector) {
    if (sector < sizeof(dirty_map)) {
        /* already marked? */
        if (dirty_map[sector])
            return;

        /* update map */
        dirty_map[sector] = 1;

        /* update heap */
        int cur = num_dirty++;
        dirty_heap[cur] = sector;
        while (dirty_heap[cur] < dirty_heap[(cur-1)/2]) {
            SWAP(dirty_heap[cur], dirty_heap[(cur-1)/2]);
            cur = (cur-1)/2;
        }
    }
}

static void heapify(int i) {
    int l = i * 2 + 1;
    int r = i * 2 + 2;
    int best = i;
    if (l < num_dirty && dirty_heap[l] < dirty_heap[best])
        best = l;
    if (r < num_dirty && dirty_heap[r] < dirty_heap[best])
        best = r;
    if (best != i) {
        SWAP(dirty_heap[i], dirty_heap[best]);
        heapify(best);
    }
}

int ps1_dirty_get_marked(void) {
    if (num_dirty == 0)
        return -1;

    uint16_t ret = dirty_heap[0];

    /* update heap */
    dirty_heap[0] = dirty_heap[--num_dirty];
    heapify(0);

    /* update map */
    dirty_map[ret] = 0;

    return ret;
}

void ps1_dirty_task(void) {
    int num_after = 0;
    int hit = 0;
    int writes = 0;
    bool contiguity_broken = false;
    uint64_t start = time_us_64();
    while (1) {
        if (!ps1_dirty_lockout_expired())
            break;
        /* do up to 100ms of work per call to dirty_task */
        if ((time_us_64() - start) > 100 * 1000)
            break;

        ps1_dirty_lock();
        int sector = ps1_dirty_get_marked();
        if (sector == -1) {
            ps1_dirty_unlock();
            break;
        }
        num_after = num_dirty;
        uint8_t *sector_data = flushbuf + (flushbuf_sectors * PS1_PAGE_SIZE);
#if WITH_PSRAM
        psram_read_dma(sector * PS1_PAGE_SIZE, sector_data, PS1_PAGE_SIZE, NULL);
        psram_wait_for_dma();
#else
        uint8_t* page = ps1_mc_data_interface_get_page(sector);
        memcpy(sector_data, page, PS1_PAGE_SIZE);
#endif
        ps1_dirty_unlock();

        ++hit;

        if (flushbuf_sectors && sector != flushbuf_last_sector + 1) {
            contiguity_broken = true;
        } else {
            ++flushbuf_sectors;
            if (flushbuf_first_sector < 0) flushbuf_first_sector = sector;
            flushbuf_last_sector = sector;
        }

        if (contiguity_broken || num_after == 0 || flushbuf_sectors == PS1_BLOCK_SIZE / PS1_PAGE_SIZE) {
            QPRINTF("ps1 - write sectors %d to %d\n", flushbuf_first_sector, flushbuf_last_sector);
            if (ps1_cardman_write_sectors(flushbuf, flushbuf_sectors, flushbuf_first_sector) == 0) {
                ++writes;
            } else {
                // TODO: do something if we get too many errors?
                // for now lets push it back into the heap and try again later
                QPRINTF("!! writing sectors 0x%x to 0x%x failed\n", flushbuf_first_sector, flushbuf_last_sector);
                ps1_dirty_lock();
                for (int sector_to_retry = flushbuf_first_sector; sector_to_retry <= flushbuf_last_sector; sector_to_retry++) {
                    ps1_dirty_mark(sector_to_retry);
                }
                ps1_dirty_unlock();
            }
            flushbuf_sectors = 0;
            flushbuf_first_sector = -1;
            flushbuf_last_sector = -1;
        }

        if (contiguity_broken) {
            contiguity_broken = false;
            if (num_after == 0) {
                QPRINTF("ps1 - write sector %d\n", sector);
                if (ps1_cardman_write_sectors(sector_data, 1, sector) == 0) {
                    ++writes;
                } else {
                    // TODO: do something if we get too many errors?
                    // for now lets push it back into the heap and try again later
                    QPRINTF("!! writing sector 0x%x failed\n", sector);
                    ps1_dirty_lock();
                    ps1_dirty_mark(sector);
                    ps1_dirty_unlock();
                }
            } else {
                memcpy(flushbuf, sector_data, PS1_PAGE_SIZE);
                flushbuf_sectors = 1;
                flushbuf_first_sector = sector;
                flushbuf_last_sector = sector;
            }
        }
    }

    if (hit) {
        if (writes) ps1_cardman_flush();

        uint64_t end = time_us_64();
        QPRINTF("remain to flush - %d - this one flushed %d and took %d ms\n", num_after, hit, (uint32_t)((end - start) / 1000));
    }

    if (num_after || !ps1_dirty_lockout_expired())
        ps1_dirty_activity = 1;
    else
        ps1_dirty_activity = 0;
}
