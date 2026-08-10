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
 */

#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>

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

static unsigned char* pixels = NULL;
static GRSurface gr_surf;

/* Stock's GRSurface, reconstructed. Field offsets measured on-device:
 * width=448 height=829 row_bytes=1792 at +8/+16/+24 (8-byte stride). */
struct StockSurface {
    void** vt;
    size_t width;
    size_t height;
    size_t row_bytes;
};
static StockSurface stock_surf;
static void* stock_vt[8];

/* vtable slot 2 (byte offset 0x10) -- drawImage calls this for the pixels. */
static unsigned char* stock_get_data(void* self __unused) {
    return pixels;
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

    gr_surf.width = HMCT_W;
    gr_surf.height = HMCT_H;
    gr_surf.row_bytes = (int)row_bytes;
    gr_surf.pixel_bytes = 4;
    gr_surf.data = pixels;
    gr_surf.format = GGL_PIXEL_FORMAT_RGBA_8888;

    printf("hmct_epd: using Hisense E Ink backend, %dx%d\n", HMCT_W, HMCT_H);
    return &gr_surf;
}

/* Single-buffered on purpose: drawImage consumes the frame synchronously and
 * the TCON thread drives it out, so there is nothing to page-flip between. */
static GRSurface* hmct_flip(minui_backend* backend __unused) {
    if (p_drawImage) p_drawImage(&stock_surf);
    return &gr_surf;
}

static void hmct_blank(minui_backend* backend __unused, bool blank) {
    /* E Ink holds its image with the panel powered down, so blanking buys
     * nothing and makes a dead UI indistinguishable from a live one. Only
     * honour an explicit unblank. */
    if (!blank && p_EpdUnblank) p_EpdUnblank();
}

static void hmct_exit(minui_backend* backend __unused) {
    if (p_DeInitEpd) p_DeInitEpd();
    free(pixels);
    pixels = NULL;
    if (lib) { dlclose(lib); lib = NULL; }
}
