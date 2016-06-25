//
// os/x11/gl_window.c: Apple/OpenGL window definitions.
//
// CEN64: Cycle-Accurate Nintendo 64 Emulator.
// Copyright (C) 2015, Tyler J. Stachecki.
//
// This file is subject to the terms and conditions defined in
// 'LICENSE', which is part of this source code package.
//

#include "common.h"
#include "device/device.h"
#include "gl_common.h"
#include "gl_config.h"
#include "gl_display.h"
#include "gl_screen.h"
#include "gl_window.h"
#include "input.h"
#include "timer.h"
#include "vi/controller.h"
#include "vi/render.h"
#include <unistd.h>

enum _event_type {
  EVENT_INPUT,
  EVENT_QUIT,
  EVENT_RENDER
};

static int cen64_gl_window_create_objects(cen64_gl_window window);
static enum _event_type cen64_gl_window_pump_events(struct vi_controller *vi);

// Creates an (initially hidden) cen64_gl_window.
cen64_gl_window cen64_gl_window_create(
  cen64_gl_display display, cen64_gl_screen screen,
  const cen64_gl_config *config, const char *title) {
  cen64_gl_window window;

  if (SDL_Init(SDL_INIT_VIDEO)) {
    return CEN64_GL_WINDOW_BAD;
  }

  if ((window = calloc(sizeof(*window), 1)) == NULL)
    return CEN64_GL_WINDOW_BAD;

  window->window = SDL_CreateWindow(title, SDL_WINDOWPOS_CENTERED,
    SDL_WINDOWPOS_CENTERED, 640, 474, SDL_WINDOW_OPENGL |
    SDL_WINDOW_HIDDEN | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);

  if (window->window == NULL) {
    free(window);
    return CEN64_GL_WINDOW_BAD;
  }

  // Create synchronization primitives for the window.
  if (cen64_gl_window_create_objects(window)) {
    SDL_DestroyWindow(window->window);
    free(window);
    return CEN64_GL_WINDOW_BAD;
  }

  window->exit_requested = false;

  return window;
}

// Handles events that come from X11.
 enum _event_type cen64_gl_window_pump_events(struct vi_controller *vi) {
  bool released, exit_requested = false;
  SDL_Event event;
  enum _event_type event_type = EVENT_INPUT;

  cen64_mutex_lock(&vi->window->event_mutex);

  while (SDL_PollEvent(&event)) {
    switch (event.type) {
      case SDL_QUIT:
        vi->window->exit_requested = true;
        event_type = EVENT_QUIT;
        break;
      case SDL_WINDOWEVENT:
        if (event.window.event == SDL_WINDOWEVENT_RESIZED) {
          gl_window_resize_cb(event.window.data1, event.window.data2);
        }
        break;
      case SDL_KEYDOWN:
        keyboard_press_callback(vi->bus, event.key.keysym.sym);
        break;
      case SDL_KEYUP:
        keyboard_release_callback(vi->bus, event.key.keysym.sym);
        break;
      case SDL_USEREVENT:
        event_type = EVENT_RENDER;
        break;
    }

    if (vi->window->exit_requested) {
      break;
    }
  }

  cen64_mutex_unlock(&vi->window->event_mutex);

  return event_type;
}

// Allocate mutexes, pipes, etc. for the UI/window.
int cen64_gl_window_create_objects(cen64_gl_window window) {
  if (cen64_mutex_create(&window->event_mutex)) {
    return 1;
  }

  if (cen64_mutex_create(&window->render_mutex)) {
    cen64_mutex_destroy(&window->event_mutex);
    return 1;
  }

  uint32_t render_event_type = SDL_RegisterEvents(1);
  if (render_event_type == (uint32_t)-1) {
    cen64_mutex_destroy(&window->event_mutex);
    cen64_mutex_destroy(&window->render_mutex);
    return 1;
  }

  window->render_event.type = render_event_type;

  return 0;
}

// Thread that controls the user interface, etc.
int cen64_gl_window_thread(struct cen64_device *device) {
  struct vi_controller *vi = &device->vi;
  cen64_time last_update_time;
  cen64_gl_window window = vi->window;
  unsigned frame_count;

  // Okay, main UI thread loop starts here.
  for (frame_count = 0, get_time(&last_update_time) ; ;) {
    int event_type = cen64_gl_window_pump_events(&device->vi);
    if(event_type == 1) {
      break;
    }

    if (event_type == 2) {
      cen64_mutex_lock(&window->render_mutex);

      gl_window_render_frame(vi, window->frame_buffer,
        window->frame_hres, window->frame_vres,
        window->frame_hskip, window->frame_type);

      cen64_mutex_unlock(&window->render_mutex);

      if (++frame_count == 60) {
        char title[128];
        cen64_time current_time;
        float ns;

        // Compute time spent rendering last 60 frames, reset timer/counter.
        get_time(&current_time);
        ns = compute_time_difference(&current_time, &last_update_time);
        last_update_time = current_time;
        frame_count = 0;

        sprintf(title,
          "CEN64 ["CEN64_COMPILER" - "CEN64_ARCH_DIR"/"CEN64_ARCH_SUPPORT"]"
          " - %.1f VI/s", (60 / (ns / NS_PER_SEC)));

        cen64_gl_window_set_title(window, title);
      }
    }
  }

  return 0;
}

