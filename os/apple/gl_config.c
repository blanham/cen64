//
// os/apple/gl_config.c: Apple framebuffer configuration.
//
// CEN64: Cycle-Accurate Nintendo 64 Emulator.
// Copyright (C) 2015, Tyler J. Stachecki.
//
// This file is subject to the terms and conditions defined in
// 'LICENSE', which is part of this source code package.
//

#include "common.h"
#include "gl_common.h"
#include "gl_config.h"
#include "gl_hints.h"

//
// Creates a matching cen64_gl_config from a cen64_gl_hints struct.
//
// On error, CEN64_GL_CONFIG_BAD is returned.
//
// XXX: This needs to be improved
cen64_gl_config *cen64_gl_config_create(cen64_gl_display display,
  cen64_gl_screen screen, const cen64_gl_hints *hints, int *matching) {
  int idx = 0;

  int *attribute_list = malloc(64 * sizeof(int));

  // Build the attributes list using the provided hints.
  //attribute_list[idx++] = SDL_GL_X_RENDERABLE;
  //attribute_list[idx++] = True;

  //attribute_list[idx++] = SDL_GL_X_VISUAL_TYPE;
  //attribute_list[idx++] = SDL_GL_TRUE_COLOR;

  //attribute_list[idx++] = SDL_GL_RENDER_TYPE;
  //attribute_list[idx++] = hints->context_type;

  //attribute_list[idx++] = SDL_GL_DRAWABLE_TYPE;
  //attribute_list[idx++] = hints->drawable_type;

  if (hints->double_buffered != -1) {
    attribute_list[idx++] = SDL_GL_DOUBLEBUFFER;
    attribute_list[idx++] = hints->double_buffered;
  }

  if (hints->stereoscopic != -1) {
    attribute_list[idx++] = SDL_GL_STEREO;
    attribute_list[idx++] = hints->stereoscopic;
  }

  if (hints->rgb_color_depth != -1) {
    int component_depth = hints->rgb_color_depth > 0
      ? hints->rgb_color_depth / 3
      : 0;

    attribute_list[idx++] = SDL_GL_RED_SIZE;
    attribute_list[idx++] = component_depth;

    attribute_list[idx++] = SDL_GL_GREEN_SIZE;
    attribute_list[idx++] = component_depth;

    attribute_list[idx++] = SDL_GL_BLUE_SIZE;
    attribute_list[idx++] = component_depth;
  }

  if (hints->alpha_color_depth != -1) {
    attribute_list[idx++] = SDL_GL_ALPHA_SIZE;
    attribute_list[idx++] = hints->alpha_color_depth;
  }

  if (hints->depth_buffer_size != -1) {
    attribute_list[idx++] = SDL_GL_DEPTH_SIZE;
    attribute_list[idx++] = hints->depth_buffer_size;
  }

  if (hints->num_aux_buffers != -1) {
    //attribute_list[idx++] = SDL_GL_AUX_BUFFERS;
    //attribute_list[idx++] = hints->num_aux_buffers;
  }

  if (hints->stencil_buffer_size != -1) {
    attribute_list[idx++] = SDL_GL_STENCIL_SIZE;
    attribute_list[idx++] = hints->stencil_buffer_size;
  }

  if (hints->accum_buffer_red_bits != -1) {
    attribute_list[idx++] = SDL_GL_ACCUM_RED_SIZE;
    attribute_list[idx++] = hints->accum_buffer_red_bits;
  }

  if (hints->accum_buffer_green_bits != -1) {
    attribute_list[idx++] = SDL_GL_ACCUM_GREEN_SIZE;
    attribute_list[idx++] = hints->accum_buffer_green_bits;
  }

  if (hints->accum_buffer_blue_bits != -1) {
    attribute_list[idx++] = SDL_GL_ACCUM_BLUE_SIZE;
    attribute_list[idx++] = hints->accum_buffer_blue_bits;
  }

  if (hints->accum_buffer_alpha_bits != -1) {
    attribute_list[idx++] = SDL_GL_ACCUM_ALPHA_SIZE;
    attribute_list[idx++] = hints->accum_buffer_alpha_bits;
  }

  for (int i = 0; i < idx; i += 2) {
    SDL_GL_SetAttribute(attribute_list[i], attribute_list[i + 1]);
  }

  return attribute_list;
}

//
// Fetches an attribute from the cen64_gl_config object.
//
int cen64_gl_config_fetch_attribute(cen64_gl_display display,
  cen64_gl_config *config, int what) {
  int value, status;

  status = SDL_GL_GetAttribute(what, &value);
  if (status < 0)
    return -1;

  return value;
}

