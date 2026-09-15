#include "ps2_dirty.h"
#include "history_tracker/ps2_history_tracker.h"
#include "psram.h"
#include "ps2_cardman.h"
#include "debug.h"

#include "bigmem.h"
#define dirty_heap bigmem.ps2.dirty_heap
#define dirty_map bigmem.ps2.dirty_map
#define flushbuf dirty_flushbuf
#define FLUSHBUF_SIZE DIRTY_FLUSHBUF_SIZE

#include <hardware/sync.h>
#include <pico/platform.h>
#include <stdio.h>
#include <string.h>

spin_lock_t *ps2_dirty_spin_lock;
volatile uint32_t ps2_dirty_lockout;
int ps2_dirty_activity = 0;

static int num_dirty;

static int flushbuf_sd_blocks_count = 0;
static int flushbuf_first_sd_block = -1;
static int flushbuf_last_sd_block = -1;
static int flushbuf_ps2_sectors[FLUSHBUF_SIZE / PS2_PAGE_SIZE];
static int flushbuf_ps2_sectors_count = 0;
static int unmarked_ps2_sectors[SD_BLOCK_SIZE / PS2_PAGE_SIZE];
static int unmarked_ps2_sectors_count = 0;

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

static void write_flushbuf(void) {
    if (ps2_cardman_write_sd_blocks(flushbuf, flushbuf_sd_blocks_count, flushbuf_first_sd_block) == 0) {
        for (int i = 0; i < flushbuf_ps2_sectors_count; i++) {
            ps2_history_tracker_registerPageWrite(flushbuf_ps2_sectors[i]);
        }
    } else {
        // TODO: do something if we get too many errors?
        // for now lets push it back into the heap and try again later
        DPRINTF("!! writing sd blocks 0x%x to 0x%x failed\n", flushbuf_first_sd_block, flushbuf_last_sd_block);
        ps2_dirty_lock();
        for (int i = 0; i < flushbuf_ps2_sectors_count; i++) {
            ps2_dirty_mark(flushbuf_ps2_sectors[i]);
        }
        ps2_dirty_unlock();
    }
    flushbuf_ps2_sectors_count = 0;
    flushbuf_sd_blocks_count = 0;
    flushbuf_first_sd_block = -1;
    flushbuf_last_sd_block = -1;
}

static void register_flushbuf_sd_block(int sd_block) {
    ++flushbuf_sd_blocks_count;
    if (flushbuf_first_sd_block < 0) flushbuf_first_sd_block = sd_block;
    flushbuf_last_sd_block = sd_block;
    memcpy(flushbuf_ps2_sectors + flushbuf_ps2_sectors_count,
           unmarked_ps2_sectors,
           unmarked_ps2_sectors_count * sizeof(unmarked_ps2_sectors[0]));
    flushbuf_ps2_sectors_count += unmarked_ps2_sectors_count;
    unmarked_ps2_sectors_count = 0;
}

static int sector_sd_block(int sector) {
    int sector_offset = sector * PS2_PAGE_SIZE;
    int sd_block_offset = sector_offset - (sector_offset % SD_BLOCK_SIZE);
    return sd_block_offset / SD_BLOCK_SIZE;
}

/* this goes through blocks in psram marked as dirty and flushes them to sd */
void ps2_dirty_task(void) {
    int num_after = 0;
    int hit = 0;
    bool contiguity_broken = false;
    uint64_t start = time_us_64();

    while (1) {
        if (!ps2_dirty_lockout_expired())
            break;
        /* do up to 100ms of work per call to dirty_taks */
        if ((time_us_64() - start) > 100 * 1000)
            break;

        ps2_dirty_lock();

        int sector, sd_block, next_sd_block;
        do {
            sector = ps2_dirty_get_marked();
            num_after = num_dirty;
            if (sector == -1) break;

            unmarked_ps2_sectors[unmarked_ps2_sectors_count] = sector;
            ++unmarked_ps2_sectors_count;
            sd_block = sector_sd_block(sector);
            next_sd_block = num_after == 0 ? -1 : sector_sd_block(dirty_heap[0]);
        } while (next_sd_block == sd_block);

        if (sector == -1) {
            ps2_dirty_unlock();
            break;
        }

        uint8_t *sd_block_slot = flushbuf + (flushbuf_sd_blocks_count * SD_BLOCK_SIZE);
        psram_read_dma(sd_block * SD_BLOCK_SIZE, sd_block_slot, SD_BLOCK_SIZE, NULL);
        psram_wait_for_dma();
        ps2_dirty_unlock();

        ++hit;

        if (flushbuf_sd_blocks_count > 0 && sd_block != flushbuf_last_sd_block + 1) {
            contiguity_broken = true;
        } else {
            register_flushbuf_sd_block(sd_block);
        }

        if (contiguity_broken || flushbuf_sd_blocks_count == FLUSHBUF_SIZE / SD_BLOCK_SIZE) {
            write_flushbuf();
        }

        if (contiguity_broken) {
            memcpy(flushbuf, sd_block_slot, SD_BLOCK_SIZE);
            register_flushbuf_sd_block(sd_block);
            contiguity_broken = false;
        }
    }

    if (flushbuf_sectors_count > 0) write_flushbuf();

    if (hit) {
        ps2_cardman_flush();

        uint64_t end = time_us_64();
        DPRINTF("remain to flush - %d - this one flushed %d and took %u ms\n", num_after, hit, (uint32_t)((end - start) / 1000));
    }

    if (num_after || !ps2_dirty_lockout_expired())
        ps2_dirty_activity = 1;
    else
        ps2_dirty_activity = 0;
}
