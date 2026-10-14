/*
 * Hisense A9 (HLTE556N) E Ink backend for minuitwrp.
 *
 * WHY THIS EXISTS
 * ---------------
 * This panel is NOT driven by DRM. Stock recovery's own DRM atomic commits
 * fail exactly as TWRP's do; it then renders through a Hisense software TCON
 * that streams E Ink waveform frames. A plain DRM client writes RGB pixels
 * into what is really a waveform transport buffer, which is why TWRP's output
 * appeared as stripes over a fraction of the panel.
 *
 * The entire E Ink implementation is exported from stock's librecovery_ui.so,
 * so we load that prebuilt and call it rather than reimplementing waveform
 * generation. Verified on-device 2026-08-09 (four equal grey bands, edge to
 * edge, at the full 824x1648).
 *
 * PIPELINE
 *   gr_init()      - stock minui; reports 824x1648 and owns the DRM device
 *   InitEpd()      - mutexes/conds + EinkSwTconInit(), starts the TCON thread
 *   EpdUnblank()   - powers the panel
 *   drawImage(s)   - THE draw call; converts our RGBA frame to waveform frames
 *
 * TRAPS (each cost a debugging cycle -- see the a9-twrp skill)
 *   - drawImage() must be given an 824x1648 surface. gr_get_drv_buffer()
 *     returns the 448x829 *output transport*; drawing into that squashes the
 *     image into the top ~28% of the panel.
 *   - Stock's GRSurface is a C++ object: [vptr][size_t w][size_t h][size_t
 *     row_bytes], and drawImage fetches pixels through vtable slot +0x10.
 *     TWRP's GRSurface is a plain struct, so we keep both views of one buffer.
 *   - The SW TCON runs in a thread of THIS process. It must stay alive for the
 *     update to reach the panel, so never tear this down mid-draw.
 *   - Nothing else may hold DRM master, or the commit fails with -EACCES.
 *     That is why this backend is tried BEFORE open_drm() and, when it
 *     succeeds, the DRM backend is never opened.
 *
 * Selection is by runtime probe: if the prebuilt is absent, init() returns
 * NULL and minui falls through to DRM. No build flag, so no dependency on
 * vendor/twrp's Soong export allowlist.
 *
 * FULL REFRESH AND POWER-OFF STILL (2026-10-14)
 * ---------------------------------------------
 * Read from the library's disassembly (sha256 2b187fca...9cdb; addresses are
 * vaddrs; docs/twrp-eink-ux.md in the A9 repo has the details):
 *   - The TCON thread (0x3b164) calls WaitForNextImage(), which calls
 *     ModeDecision_MirrorMode(img, handle, 18, 18, force_clear_flag, 3).
 *     Mode 3 and the temperatures are hard-coded. ModeDecision only stores
 *     mode and (force != 0); Update_lookUp_table() then picks the table:
 *     force -> mode 2 -> the 256x38 table at 0x1a89c that drives every
 *     pixel (full GC16); otherwise mode 3 -> the 256x38 table at 0x1ce9c that
 *     skips unchanged pixels.
 *   - ForceClearGhosting(int) only writes force_clear_flag. With it set,
 *     WaitForNextImage also skips its "image is repeat" memcmp, and clears
 *     the flag again after that one update (epd_clear_revert). So
 *     ForceClearGhosting(1) + drawImage(current frame) = one full redraw.
 *   - Producer idle: frame_is_end == 1 (set after the last drive frame),
 *     image_update == 0 (reset when the thread waits for the next image) and
 *     frame_lose == 0 (no queued image). Consumer idle: Que read == write.
 *     All four are exported data symbols, so we can wait for "on the glass".
 *   - There is no usable fast (DU/A2) mode: modes 1/6/8/9 index a 4x10 table
 *     at 0x1a874 with the full 8-bit old<<4|new value and read into the GC16
 *     table. Not used.
 */

#include <dlfcn.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "graphics.h"
#include <pixelflinger/pixelflinger.h>

#define HMCT_W 824
#define HMCT_H 1648
#define HMCT_LIB "/system/lib64/libhmct_epd.so"

static GRSurface* hmct_init(minui_backend*);
static GRSurface* hmct_flip(minui_backend*);
static void hmct_blank(minui_backend*, bool);
static void hmct_exit(minui_backend*);

static minui_backend my_backend = {
    .init = hmct_init,
    .flip = hmct_flip,
    .blank = hmct_blank,
    .exit = hmct_exit,
};

minui_backend* open_hmct_epd() {
    return &my_backend;
}

static void* lib = NULL;
static void (*p_drawImage)(void*) = NULL;
static void (*p_EpdBlank)(void) = NULL;
static void (*p_EpdUnblank)(void) = NULL;
static void (*p_DeInitEpd)(void) = NULL;
static int (*p_ForceClearGhosting)(int) = NULL;

/* TCON state, exported by the library (see the header comment). */
static volatile unsigned char* p_frame_is_end = NULL;
static volatile unsigned char* p_image_update = NULL;
static volatile unsigned char* p_frame_lose = NULL;
static volatile int* p_que = NULL;  /* [0] = read index, [1] = write index */

static unsigned char* pixels = NULL;
static GRSurface gr_surf;

/* true once init has fully succeeded; false again after exit. */
static bool ready = false;
/* Set by gr_epd_show_image(): no more UI frames reach the panel. */
static bool frozen = false;
/* Set by gr_epd_force_next_flip(): the next flip is a full refresh. */
static bool force_next = false;
/* Serialises drawImage() between the GUI thread and callers of the
 * gr_epd_* API (power-off can come from the ORS thread). */
static pthread_mutex_t draw_lock = PTHREAD_MUTEX_INITIALIZER;

/* Stock's GRSurface, reconstructed. Field offsets measured on-device:
 * width=448 height=829 row_bytes=1792 at +8/+16/+24 (8-byte stride).
 * drawImage reads only vt, height and row_bytes; `data` is ours. */
struct StockSurface {
    void** vt;
    size_t width;
    size_t height;
    size_t row_bytes;
    unsigned char* data;
};
static StockSurface stock_surf;
static void* stock_vt[8];

/* vtable slot 2 (byte offset 0x10) -- drawImage calls this for the pixels. */
static unsigned char* stock_get_data(void* self) {
    return ((StockSurface*)self)->data;
}

static GRSurface* hmct_init(minui_backend* backend __unused) {
    lib = dlopen(HMCT_LIB, RTLD_NOW | RTLD_GLOBAL);
    if (!lib) {
        printf("hmct_epd: %s not available (%s); falling back\n", HMCT_LIB, dlerror());
        return NULL;
    }

    int  (*p_gr_init)(void)  = (int (*)(void))dlsym(lib, "_Z7gr_initv");
    void (*p_InitEpd)(void)  = (void (*)(void))dlsym(lib, "_Z7InitEpdv");
    p_EpdUnblank = (void (*)(void))dlsym(lib, "_Z10EpdUnblankv");
    p_EpdBlank   = (void (*)(void))dlsym(lib, "_Z8EpdBlankv");
    p_DeInitEpd  = (void (*)(void))dlsym(lib, "_Z9DeInitEpdv");
    p_drawImage  = (void (*)(void*))dlsym(lib, "_Z9drawImageP9GRSurface");

    if (!p_gr_init || !p_InitEpd || !p_EpdUnblank || !p_drawImage) {
        printf("hmct_epd: required symbols missing; falling back\n");
        dlclose(lib);
        lib = NULL;
        return NULL;
    }

    /* Optional: full refresh and the power-off wait. Missing symbols only
     * disable those features, never the display. */
    p_ForceClearGhosting = (int (*)(int))dlsym(lib, "_Z18ForceClearGhostingi");
    p_frame_is_end = (volatile unsigned char*)dlsym(lib, "frame_is_end");
    p_image_update = (volatile unsigned char*)dlsym(lib, "image_update");
    p_frame_lose   = (volatile unsigned char*)dlsym(lib, "frame_lose");
    p_que          = (volatile int*)dlsym(lib, "Que");

    /* Stock minui takes the DRM device here. Nothing else may hold master. */
    int rc = p_gr_init();
    printf("hmct_epd: stock gr_init() = %d\n", rc);

    p_InitEpd();
    p_EpdUnblank();

    const size_t row_bytes = (size_t)HMCT_W * 4;
    pixels = (unsigned char*)calloc(1, row_bytes * HMCT_H);
    if (!pixels) {
        printf("hmct_epd: cannot allocate %zu bytes\n", row_bytes * HMCT_H);
        return NULL;
    }

    stock_vt[2] = (void*)stock_get_data;
    stock_surf.vt = stock_vt;
    stock_surf.width = HMCT_W;
    stock_surf.height = HMCT_H;
    stock_surf.row_bytes = row_bytes;
    stock_surf.data = pixels;

    gr_surf.width = HMCT_W;
    gr_surf.height = HMCT_H;
    gr_surf.row_bytes = (int)row_bytes;
    gr_surf.pixel_bytes = 4;
    gr_surf.data = pixels;
    gr_surf.format = GGL_PIXEL_FORMAT_RGBA_8888;

    ready = true;
    printf("hmct_epd: using Hisense E Ink backend, %dx%d (full refresh %s, idle wait %s)\n",
           HMCT_W, HMCT_H, p_ForceClearGhosting ? "yes" : "no",
           (p_frame_is_end && p_image_update && p_frame_lose && p_que) ? "yes" : "no");
    return &gr_surf;
}

/* Single-buffered on purpose: drawImage consumes the frame synchronously and
 * the TCON thread drives it out, so there is nothing to page-flip between. */
static GRSurface* hmct_flip(minui_backend* backend __unused) {
    pthread_mutex_lock(&draw_lock);
    if (p_drawImage && !frozen) {
        /* One-shot, like gr_epd_full_refresh(): the TCON reads the flag when
         * it takes the next image, which is this frame (or a newer one that
         * replaced it in the lost-image slot while an update was running). */
        if (force_next && p_ForceClearGhosting) p_ForceClearGhosting(1);
        p_drawImage(&stock_surf);
    }
    force_next = false;
    pthread_mutex_unlock(&draw_lock);
    return &gr_surf;
}

static void hmct_blank(minui_backend* backend __unused, bool blank) {
    /* E Ink holds its image with the panel powered down, so blanking buys
     * nothing and makes a dead UI indistinguishable from a live one. Only
     * honour an explicit unblank. */
    if (!blank && p_EpdUnblank) p_EpdUnblank();
}

static void hmct_exit(minui_backend* backend __unused) {
    ready = false;
    if (p_DeInitEpd) p_DeInitEpd();
    free(pixels);
    pixels = NULL;
    if (lib) { dlclose(lib); lib = NULL; }
}

/* ---- gr_epd_* API (declared in minuitwrp/minui.h) ---------------------- */

bool gr_epd_active(void) {
    return ready;
}

int gr_epd_full_refresh(void) {
    if (!ready || !p_ForceClearGhosting) return -1;
    pthread_mutex_lock(&draw_lock);
    if (!frozen) {
        /* One-shot: the TCON clears the flag after the forced update. If an
         * update is running, drawImage queues this frame and the force
         * applies to it when the thread picks it up. */
        p_ForceClearGhosting(1);
        p_drawImage(&stock_surf);
    }
    pthread_mutex_unlock(&draw_lock);
    printf("hmct_epd: full refresh\n");
    return 0;
}

int gr_epd_force_next_flip(void) {
    if (!ready || !p_ForceClearGhosting) return -1;
    pthread_mutex_lock(&draw_lock);
    force_next = true;
    pthread_mutex_unlock(&draw_lock);
    return 0;
}

static bool tcon_idle(void) {
    return *p_frame_is_end == 1 && *p_image_update == 0 && *p_frame_lose == 0 &&
           p_que[0] == p_que[1];
}

/* Wait until the TCON has driven out everything it was given. Polls every
 * 20 ms and wants three idle readings in a row, so a momentary state between
 * two updates is not mistaken for the end. */
static int wait_tcon_idle(int timeout_ms) {
    if (!p_frame_is_end || !p_image_update || !p_frame_lose || !p_que) {
        usleep(timeout_ms > 0 ? (useconds_t)timeout_ms * 1000 : 0);
        return -1;
    }
    int stable = 0;
    for (int waited = 0; waited < timeout_ms; waited += 20) {
        if (tcon_idle()) {
            if (++stable >= 3) return waited;
        } else {
            stable = 0;
        }
        usleep(20000);
    }
    return -1;
}

int gr_epd_show_image(const char* png_path, int timeout_ms) {
    if (!ready || !png_path) return -1;

    gr_surface s = NULL;
    int rc = res_create_surface(png_path, &s);
    if (rc < 0 || !s) {
        printf("hmct_epd: cannot load %s (%d); no still image\n", png_path, rc);
        return -1;
    }
    GGLSurface* img = (GGLSurface*)s;

    /* White canvas, image centred and clipped to the panel. */
    const size_t row_bytes = (size_t)HMCT_W * 4;
    unsigned char* buf = (unsigned char*)malloc(row_bytes * HMCT_H);
    if (!buf) {
        res_free_surface(s);
        return -1;
    }
    memset(buf, 0xff, row_bytes * HMCT_H);
    const int cw = (int)img->width < HMCT_W ? (int)img->width : HMCT_W;
    const int ch = (int)img->height < HMCT_H ? (int)img->height : HMCT_H;
    const int dx = (HMCT_W - cw) / 2, dy = (HMCT_H - ch) / 2;
    const int sx = ((int)img->width - cw) / 2, sy = ((int)img->height - ch) / 2;
    for (int y = 0; y < ch; y++) {
        memcpy(buf + (size_t)(dy + y) * row_bytes + (size_t)dx * 4,
               img->data + ((size_t)(sy + y) * img->stride + sx) * 4, (size_t)cw * 4);
    }
    res_free_surface(s);

    StockSurface still = stock_surf;
    still.data = buf;

    pthread_mutex_lock(&draw_lock);
    frozen = true;  /* no UI frame may follow the still */
    if (p_ForceClearGhosting) p_ForceClearGhosting(1);  /* full drive: no ghost of the UI */
    p_drawImage(&still);  /* copies the pixels before it returns */
    pthread_mutex_unlock(&draw_lock);
    free(buf);

    int waited = wait_tcon_idle(timeout_ms);
    /* The consumer thread may still be flipping the last drive frame. */
    usleep(200000);
    if (waited >= 0)
        printf("hmct_epd: still image %s on the panel after ~%d ms\n", png_path, waited);
    else
        printf("hmct_epd: still image %s: no idle confirmation within %d ms\n", png_path, timeout_ms);
    return waited >= 0 ? 0 : -1;
}
