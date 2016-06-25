//
// os/x11/gl_display.h: Apple display definitions.
//
// CEN64: Cycle-Accurate Nintendo 64 Emulator.
// Copyright (C) 2015, Tyler J. Stachecki.
//
// This file is subject to the terms and conditions defined in
// 'LICENSE', which is part of this source code package.
//

#ifndef CEN64_OS_X11_GL_DISPLAY
#define CEN64_OS_X11_GL_DISPLAY
#include "gl_common.h"
#include <stddef.h>

#define CEN64_GL_DISPLAY_BAD (NULL)
typedef int cen64_gl_display;

//
// Creates a cen64_gl_display (where 'source' selects the display to use).
//
// If source is NULL, it is treated as a don't care.
//
static inline cen64_gl_display cen64_gl_display_create(const char *source) {
  return 1;
}

// Releases resources allocated by cen64_gl_display_create.
static inline void cen64_gl_display_destroy(cen64_gl_display display) {
}

// Returns the number of screens present on a given cen64_gl_display.
static inline int cen64_gl_display_get_num_screens(cen64_gl_display display) {
  return 1;
}

#endif

