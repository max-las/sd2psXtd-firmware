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

#define FLUSHBUF_SIZE 8192
#define SD_SECTOR_SIZE 512

spin_lock_t *ps1_dirty_spin_lock;
volatile uint32_t ps1_dirty_lockout;
int ps1_dirty_activity;

static int num_dirty;

static uint8_t flushbuf[FLUSHBUF_SIZE];
static int flushbuf_sd_sectors_count = 0;
static int flushbuf_first_sd_sector_addr = -1;
static int flushbuf_last_sd_sector_addr = -1;
static int flushbuf_ps1_sectors[FLUSHBUF_SIZE / PS1_PAGE_SIZE];
static int flushbuf_ps1_sectors_count = 0;

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

static int write_flushbuf(void) {
    int writes = 0;

    if (ps1_cardman_write_sd_sectors(flushbuf, flushbuf_sd_sectors_count, flushbuf_first_sd_sector_addr) == 0) {
        ++writes;
    } else {
        // TODO: do something if we get too many errors?
        // for now lets push it back into the heap and try again later
        QPRINTF("!! writing sectors 0x%x to 0x%x failed\n",
                flushbuf_first_sd_sector_addr / PS1_PAGE_SIZE,
                flushbuf_last_sd_sector_addr / PS1_PAGE_SIZE);
        ps1_dirty_lock();
        for (int i = 0; i < flushbuf_ps1_sectors_count; i++) {
            ps1_dirty_mark(flushbuf_ps1_sectors[i]);
        }
        ps1_dirty_unlock();
    }
    flushbuf_ps1_sectors_count = 0;
    flushbuf_sd_sectors_count = 0;
    flushbuf_first_sd_sector_addr = -1;
    flushbuf_last_sd_sector_addr = -1;

    return writes;
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
        int memcard_sector_addr = sector * PS1_PAGE_SIZE;
        int sd_sector_addr = memcard_sector_addr - (memcard_sector_addr % SD_SECTOR_SIZE);
        uint8_t *sd_sector_data = flushbuf + (flushbuf_sd_sectors_count * SD_SECTOR_SIZE);
        if (sd_sector_addr > flushbuf_last_sd_sector_addr) {
#if WITH_PSRAM
            psram_read_dma(sd_sector_addr, sd_sector_data, SD_SECTOR_SIZE, NULL);
            psram_wait_for_dma();
#else
            for (int cursor = 0; cursor < SD_SECTOR_SIZE; cursor += PS1_PAGE_SIZE) {
                uint32_t target = (sd_sector_addr + cursor) / PS1_PAGE_SIZE;
                uint8_t *page = ps1_mc_data_interface_get_page(target);
                memcpy(sd_sector_data + cursor, page, PS1_PAGE_SIZE);
            }
#endif
        }

        ps1_dirty_unlock();

        ++hit;

        if (flushbuf_sd_sectors_count > 0 && sd_sector_addr > flushbuf_last_sd_sector_addr + SD_SECTOR_SIZE) {
            contiguity_broken = true;
        } else {
            if (sd_sector_addr > flushbuf_last_sd_sector_addr) {
                ++flushbuf_sd_sectors_count;
                if (flushbuf_first_sd_sector_addr < 0) flushbuf_first_sd_sector_addr = sd_sector_addr;
                flushbuf_last_sd_sector_addr = sd_sector_addr;
            }
            flushbuf_ps1_sectors[flushbuf_ps1_sectors_count] = sector;
            ++flushbuf_ps1_sectors_count;
        }

        if (contiguity_broken || num_after == 0 || flushbuf_sd_sectors_count == FLUSHBUF_SIZE / SD_SECTOR_SIZE) {
            writes += write_flushbuf();
        }

        if (contiguity_broken) {
            memcpy(flushbuf, sd_sector_data, SD_SECTOR_SIZE);
            flushbuf_sd_sectors_count = 1;
            flushbuf_first_sd_sector_addr = sd_sector_addr;
            flushbuf_last_sd_sector_addr = sd_sector_addr;
            flushbuf_ps1_sectors[0] = sector;
            flushbuf_ps1_sectors_count = 1;
            contiguity_broken = false;
            if (num_after == 0) writes += write_flushbuf();
        }
    }

    if (hit) {
        if (writes) ps1_cardman_flush();

        uint64_t end = time_us_64();
        QPRINTF("remain to flush - %d - this one flushed %d and took %u ms\n", num_after, hit, (uint32_t)((end - start) / 1000));
    }

    if (num_after || !ps1_dirty_lockout_expired())
        ps1_dirty_activity = 1;
    else
        ps1_dirty_activity = 0;
}
