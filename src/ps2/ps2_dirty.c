#include "ps2_dirty.h"
#include "history_tracker/ps2_history_tracker.h"
#include "psram.h"
#include "ps2_cardman.h"
#include "debug.h"

#include "bigmem.h"
#define dirty_heap bigmem.ps2.dirty_heap
#define dirty_map bigmem.ps2.dirty_map

#include <hardware/sync.h>
#include <pico/platform.h>
#include <stdio.h>

#define FLUSHBUF_SIZE 8192

spin_lock_t *ps2_dirty_spin_lock;
volatile uint32_t ps2_dirty_lockout;
int ps2_dirty_activity = 0;

static int num_dirty;

static uint8_t flushbuf[FLUSHBUF_SIZE]; // MTODO: share with ps1 side
static int flushbuf_sectors = 0;
static int flushbuf_first_sector = -1;
static int flushbuf_last_sector = -1;

#define SWAP(a, b) do { \
    uint16_t tmp = a; \
    a = b; \
    b = tmp; \
} while (0);

static inline bool dirty_map_is_marked(uint32_t sector) {
    return dirty_map[sector / 8] & (1 << (sector % 8));
}

static inline void dirty_map_mark_sector(uint32_t sector) {
    dirty_map[sector / 8] |= (1 << (sector % 8));
}

static inline void dirty_map_unmark_sector(uint32_t sector) {
    dirty_map[sector / 8] &= ~(1 << (sector % 8));
}

void ps2_dirty_init(void) {
    if (!ps2_dirty_spin_lock)
        ps2_dirty_spin_lock = spin_lock_init(spin_lock_claim_unused(1));
}

void __time_critical_func(ps2_dirty_mark)(uint32_t sector) {
    if (sector < (sizeof(dirty_map) * 8)) {
        /* already marked? */
        if (dirty_map_is_marked(sector))
            return;

        /* update map */
        dirty_map_mark_sector(sector);

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

int ps2_dirty_get_marked(void) {
    if (num_dirty == 0)
        return -1;

    uint16_t ret = dirty_heap[0];

    /* update heap */
    dirty_heap[0] = dirty_heap[--num_dirty];
    heapify(0);

    /* update map */
    dirty_map_unmark_sector(ret);

    return ret;
}

/* this goes through blocks in psram marked as dirty and flushes them to sd */
void ps2_dirty_task(void) {
    int num_after = 0;
    int hit = 0;
    int writes = 0;
    bool contiguity_broken = false;
    uint64_t start = time_us_64();
    while (1) {
        if (!ps2_dirty_lockout_expired())
            break;
        /* do up to 100ms of work per call to dirty_taks */
        if ((time_us_64() - start) > 100 * 1000)
            break;

        ps2_dirty_lock();
        int sector = ps2_dirty_get_marked();
        if (sector == -1) {
            ps2_dirty_unlock();
            break;
        }
        num_after = num_dirty;
        uint8_t *sector_data = flushbuf + (flushbuf_sectors * PS2_PAGE_SIZE);
        psram_read_dma(sector * PS2_PAGE_SIZE, sector_data, PS2_PAGE_SIZE, NULL);
        psram_wait_for_dma();
        ps2_dirty_unlock();

        ++hit;

        if (flushbuf_sectors && sector != flushbuf_last_sector + 1) {
            contiguity_broken = true;
        } else {
            ++flushbuf_sectors;
            if (flushbuf_first_sector < 0) flushbuf_first_sector = sector;
            flushbuf_last_sector = sector;
        }

        if (contiguity_broken || num_after == 0 || flushbuf_sectors == FLUSHBUF_SIZE / PS2_PAGE_SIZE) {
            // DPRINTF("ps2 - write sectors %d to %d\n", flushbuf_first_sector, flushbuf_last_sector);
            if (ps2_cardman_write_sectors(flushbuf, flushbuf_sectors, flushbuf_first_sector) == 0) {
                ++writes;
                for (int sector_to_track = flushbuf_first_sector; sector_to_track <= flushbuf_last_sector; sector_to_track++) {
                    ps2_history_tracker_registerPageWrite(sector_to_track);
                }
            } else {
                // TODO: do something if we get too many errors?
                // for now lets push it back into the heap and try again later
                DPRINTF("!! writing sectors 0x%x to 0x%x failed\n", flushbuf_first_sector, flushbuf_last_sector);
                ps2_dirty_lock();
                for (int sector_to_retry = flushbuf_first_sector; sector_to_retry <= flushbuf_last_sector; sector_to_retry++) {
                    ps2_dirty_mark(sector_to_retry);
                }
                ps2_dirty_unlock();
            }
            flushbuf_sectors = 0;
            flushbuf_first_sector = -1;
            flushbuf_last_sector = -1;
        }

        if (contiguity_broken) {
            contiguity_broken = false;
            if (num_after == 0) {
                // DPRINTF("ps2 - write sector %d\n", sector);
                if (ps2_cardman_write_sectors(sector_data, 1, sector) == 0) {
                    ++writes;
                    ps2_history_tracker_registerPageWrite(sector);
                } else {
                    // TODO: do something if we get too many errors?
                    // for now lets push it back into the heap and try again later
                    DPRINTF("!! writing sector 0x%x failed\n", sector);
                    ps2_dirty_lock();
                    ps2_dirty_mark(sector);
                    ps2_dirty_unlock();
                }
            } else {
                memcpy(flushbuf, sector_data, PS2_PAGE_SIZE);
                flushbuf_sectors = 1;
                flushbuf_first_sector = sector;
                flushbuf_last_sector = sector;
            }
        }
    }

    if (hit) {
        if (writes) ps2_cardman_flush();

        uint64_t end = time_us_64();
        DPRINTF("remain to flush - %d - this one flushed %d and took %d ms\n", num_after, hit, (uint32_t)((end - start) / 1000));
    }

    if (num_after || !ps2_dirty_lockout_expired())
        ps2_dirty_activity = 1;
    else
        ps2_dirty_activity = 0;
}
