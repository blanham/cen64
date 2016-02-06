#include "common.h"
#include "bus/controller.h"
#include "device/device.h"
#include "ri/controller.h"
#include "tctables.h"
#include "vr4300/interface.h"
#include <stdarg.h>
#include <stdint.h>
#include <string.h>




#include <assert.h>
#include <math.h>
#include <GL/gl.h>
#include <GL/glext.h>



#define SP_INTERRUPT  0x1
#define SI_INTERRUPT  0x2
#define AI_INTERRUPT  0x4
#define VI_INTERRUPT  0x8
#define PI_INTERRUPT  0x10
#define DP_INTERRUPT  0x20

#define SP_STATUS_HALT      0x0001
#define SP_STATUS_BROKE     0x0002
#define SP_STATUS_DMABUSY   0x0004
#define SP_STATUS_DMAFULL   0x0008
#define SP_STATUS_IOFULL    0x0010
#define SP_STATUS_SSTEP     0x0020
#define SP_STATUS_INTR_BREAK  0x0040
#define SP_STATUS_SIGNAL0   0x0080
#define SP_STATUS_SIGNAL1   0x0100
#define SP_STATUS_SIGNAL2   0x0200
#define SP_STATUS_SIGNAL3   0x0400
#define SP_STATUS_SIGNAL4   0x0800
#define SP_STATUS_SIGNAL5   0x1000
#define SP_STATUS_SIGNAL6   0x2000
#define SP_STATUS_SIGNAL7   0x4000

#define DP_STATUS_XBUS_DMA    0x01
#define DP_STATUS_FREEZE    0x02
#define DP_STATUS_FLUSH     0x04
#define DP_STATUS_START_GCLK    0x008
#define DP_STATUS_TMEM_BUSY   0x010
#define DP_STATUS_PIPE_BUSY   0x020
#define DP_STATUS_CMD_BUSY    0x040
#define DP_STATUS_CBUF_READY    0x080
#define DP_STATUS_DMA_BUSY    0x100
#define DP_STATUS_END_VALID   0x200
#define DP_STATUS_START_VALID   0x400

#define R4300i_SP_Intr 1



#define LOG_RDP_EXECUTION 0
#define DETAILED_LOGGING 0



typedef struct
{
  int32_t r, g, b, a;
} COLOR;








uint8_t R = 0;
uint8_t G = 0;
uint8_t B = 0;
float viewport[4]; // Used For GL View Portion
uint8_t texconv[0x800000]; // Texture Conversion Storage
uint8_t texmask[0x800000]; // Texture Mask Storage

float v1x = 0.0; // Vertex For Tiangles & Quads
float v1y = 0.0;
float v2x = 0.0;
float v2y = 0.0;
float v3x = 0.0;
float v3y = 0.0;

FILE *rdp_exec;

uint32_t rdp_cmd_data[0x10000];
uint32_t rdp_cmd_ptr = 0;
uint32_t rdp_cmd_cur = 0;
uint32_t ptr_onstart = 0;

uint8_t* rdram_8;
uint16_t* rdram_16;

typedef struct
{
  int format;
  int size;
  int line;
  int tmem;
  int palette;
  int clamp_t, mirror_t, clamp_s, mirror_s;
  int mask_t, shift_t, mask_s, shift_s;

  uint16_t sl, tl, sh, th, dxt;

} TILE;

typedef struct
{
  int sub_a_rgb0;
  int sub_b_rgb0;
  int mul_rgb0;
  int add_rgb0;
  int sub_a_a0;
  int sub_b_a0;
  int mul_a0;
  int add_a0;

  int sub_a_rgb1;
  int sub_b_rgb1;
  int mul_rgb1;
  int add_rgb1;
  int sub_a_a1;
  int sub_b_a1;
  int mul_a1;
  int add_a1;
} COMBINE_MODES;

typedef struct 
{
  int stalederivs;
  int dolod;
  int partialreject_1cycle; 
  int partialreject_2cycle;
  int special_bsel0; 
  int special_bsel1;
  int rgb_alpha_dither;
} MODEDERIVS;

typedef struct
{
  int cycle_type;
  int persp_tex_en;
  int detail_tex_en;
  int sharpen_tex_en;
  int tex_lod_en;
  int en_tlut;
  int tlut_type;
  int sample_type;
  int mid_texel;
  int bi_lerp0;
  int bi_lerp1;
  int convert_one;
  int key_en;
  int rgb_dither_sel;
  int alpha_dither_sel;
  int blend_m1a_0;
  int blend_m1a_1;
  int blend_m1b_0;
  int blend_m1b_1;
  int blend_m2a_0;
  int blend_m2a_1;
  int blend_m2b_0;
  int blend_m2b_1;
  int force_blend;
  int alpha_cvg_select;
  int cvg_times_alpha;
  int z_mode;
  int cvg_dest;
  int color_on_cvg;
  int image_read_en;
  int z_update_en;
  int z_compare_en;
  int antialias_en;
  int z_source_sel;
  int dither_alpha_en;
  int alpha_compare_en;
  MODEDERIVS f;
} OTHER_MODES;

#define PIXEL_SIZE_4BIT  0
#define PIXEL_SIZE_8BIT  1
#define PIXEL_SIZE_16BIT 2
#define PIXEL_SIZE_32BIT 3

#define CYCLE_TYPE_1    0
#define CYCLE_TYPE_2    1
#define CYCLE_TYPE_COPY 2
#define CYCLE_TYPE_FILL 3

#define FORMAT_RGBA 0
#define FORMAT_YUV  1
#define FORMAT_CI   2
#define FORMAT_IA   3
#define FORMAT_I    4

#define ZMODE_OPAQUE            0
#define ZMODE_INTERPENETRATING  1
#define ZMODE_TRANSPARENT       2
#define ZMODE_DECAL             3

static int fb_format = FORMAT_RGBA;
static uint32_t fb_address = 0;

COMBINE_MODES combine;
OTHER_MODES other_modes;

COLOR blend_color;
COLOR prim_color;
COLOR env_color;
COLOR fog_color;
COLOR key_scale;
COLOR key_center;
COLOR key_width;
static int32_t primitive_lod_frac = 0;

uint32_t fill_color;

uint32_t primitive_z;
uint16_t primitive_delta_z;

static int fb_size = 0;
static int fb_width = 0;

static int ti_format = FORMAT_RGBA;
static int ti_size = PIXEL_SIZE_4BIT;
static int ti_width = 0;
static uint32_t ti_address = 0;
static uint32_t tlut_address = 0;

static uint32_t zb_address = 0;

static TILE tile[8];

uint8_t TMEM[0x1000]; 

#define tlut ((uint16_t*)(&TMEM[0x800]))

static int32_t k0 = 0, k1 = 0, k2 = 0, k3 = 0, k4 = 0, k5 = 0;

uint32_t *rdram;
static uint32_t* rsp_dmem;

static void fatalerror(const char * err, ...)
{
  char VsprintfBuffer[200];
  va_list arg;
  va_start(arg, err);
  vsprintf(VsprintfBuffer, err, arg);
#ifdef WIN32
  MessageBoxA(0,VsprintfBuffer,"RDP: fatal error",MB_OK);
#endif
#ifndef WIN32
  printf(VsprintfBuffer);
#endif
  va_end(arg);
  exit(0);
}

void RDPSetRDRAMPointer(uint8_t *rdram_ptr) {
  rdram = (uint32_t *) rdram_ptr;
  rdram_8 = (uint8_t*)rdram;
  rdram_16 = (uint16_t*)rdram;
}

void RDPSetRSPDMEMPointer(uint8_t *rsp_dmem_ptr) {
  rsp_dmem = (uint32_t *) rsp_dmem_ptr;
}

//static uint16_t bswap16(uint16_t x) { return ((x << 8) & 0xFF00) | ((x >> 8) & 0x00FF); }
static uint32_t bswap32(uint32_t x) { return __builtin_bswap32(x); }

#define RDRAM_MASK 0x007fffff


#define RREADADDR8(rdst, in) {(in) &= RDRAM_MASK; (rdst) = ((in) <= plim) ? (rdram_8[(in)]) : 0;}
#define RREADIDX16(rdst, in) {(in) &= (RDRAM_MASK >> 1); (rdst) = ((in) <= idxlim16) ? (byteswap_16(rdram_16[(in)])) : 0;}
#define RREADIDX32(rdst, in) {(in) &= (RDRAM_MASK >> 2); (rdst) = ((in) <= idxlim32) ? (byteswap_32(rdram[(in)])) : 0;}

#define RWRITEADDR8(in, val)  {(in) &= RDRAM_MASK; if ((in) <= plim) rdram_8[(in)] = (val);}
#define RWRITEIDX16(in, val)  {(in) &= (RDRAM_MASK >> 1); if ((in) <= idxlim16) rdram_16[(in)] = byteswap_16(val);}
#define RWRITEIDX32(in, val)  {(in) &= (RDRAM_MASK >> 2); if ((in) <= idxlim32) rdram[(in)] = byteswap_32(val);}

uint32_t z64gl_command = 0;
uint32_t command_counter = 0;
int SaveLoaded = 0;
uint32_t max_level = 0;
int32_t min_level = 0;
int32_t* PreScale;
uint32_t tvfadeoutstate[625];
int rdp_pipeline_crashed = 0;

uint8_t hidden_bits[0x400000];

static struct cen64_device *cen64;


#define rsp_imem ((uint32_t*)(cen64->rsp.mem+0x1000))
#define rsp_dmem ((uint32_t*)cen64->rsp.mem)

#define rdram ((uint32_t*)cen64->ri.ram)
#define rdram16 ((uint16_t*)cen64->ri.ram)
#define rdram8 (cen64->ri.ram)

#define vi_width (cen64->vi.regs[VI_WIDTH_REG])

#define dp_start (cen64->rdp.regs[DPC_START_REG])
#define dp_end (cen64->rdp.regs[DPC_END_REG])
#define dp_current (cen64->rdp.regs[DPC_CURRENT_REG])
#define dp_status (cen64->rdp.regs[DPC_STATUS_REG])

uint32_t idxlim16 = 0x1fffff;
uint32_t idxlim32 = 0xfffff;

cen64_cold int angrylion_rdp_init(struct cen64_device *device)
{
  cen64 = device;
  memset(TMEM, 0, 0x1000);
  memset(hidden_bits, 3, sizeof(hidden_bits));
  memset(tile, 0, sizeof(tile));

  memset(&prim_color, 0, sizeof(COLOR));
  memset(&env_color, 0, sizeof(COLOR));
  memset(&key_scale, 0, sizeof(COLOR));
  memset(&key_center, 0, sizeof(COLOR));

  rdram_8 = (uint8_t*)rdram;
  rdram_16 = (uint16_t*)rdram;
  return 0;
}

static const uint32_t rdp_command_length[64] = {
  8,      
  8,      
  8,      
  8,      
  8,      
  8,      
  8,      
  8,      
  32,     
  32+16,    
  32+64,    
  32+64+16, 
  32+64,    
  32+64+16, 
  32+64+64, 
  32+64+64+16,
  8,      
  8,      
  8,      
  8,      
  8,      
  8,      
  8,      
  8,      
  8,      
  8,      
  8,      
  8,      
  8,      
  8,      
  8,      
  8,      
  8,      
  8,      
  8,      
  8,      
  16,     
  16,     
  8,      
  8,      
  8,      
  8,      
  8,      
  8,      
  8,      
  8,      
  8,      
  8,      
  8,      
  8,      
  8,      
  8,      
  8,      
  8,      
  8,      
  8,      
  8,      
  8,      
  8,      
  8,      
  8,      
  8,      
  8,      
  8     
};

static void rdp_invalid(uint32_t w1, uint32_t w2)
{
}

static void rdp_noop(uint32_t w1, uint32_t w2)
{
}

static void rdp_tri_noshade(uint32_t w1, uint32_t w2)
{
  float width = fb_width;           // Framebuffer Width In Pixels
  float height = ((width / 4) * 3); // Framebuffer Height In Pixels

  uint32_t w3 = rdp_cmd_data[rdp_cmd_cur + 2]; // Load RDP Command Word 3
  uint32_t w4 = rdp_cmd_data[rdp_cmd_cur + 3]; // Load RDP Command Word 4
  uint32_t w5 = rdp_cmd_data[rdp_cmd_cur + 4]; // Load RDP Command Word 5
  uint32_t w6 = rdp_cmd_data[rdp_cmd_cur + 5]; // Load RDP Command Word 6
  uint32_t w7 = rdp_cmd_data[rdp_cmd_cur + 6]; // Load RDP Command Word 7
  uint32_t w8 = rdp_cmd_data[rdp_cmd_cur + 7]; // Load RDP Command Word 8

  if (fb_size == PIXEL_SIZE_16BIT) { // Frame Buffer Size Of Pixels = 16 bits
    R = ((fill_color >> 11) & 0x1F) << 3;
    G = ((fill_color >>  6) & 0x1F) << 3;
    B = ((fill_color >>  1) & 0x1F) << 3;
  }
  if (fb_size == PIXEL_SIZE_32BIT) { // Frame Buffer Size Of Pixels = 32 bits
    R = fill_color >> 24;
    G = fill_color >> 16;
    B = fill_color >>  8;
  }

  uint8_t col[9] = { // 3 Byte RGB Colour * 3 Triangle Points
    R, G, B,
    R, G, B,
    R, G, B
  };

  uint8_t dir = (w1 >> 23) & 1; // Load Triangle Direction Flag (Left = 0, Right = 1)

  uint8_t yls = (w1 >> 13) & 1;  // Load YL Sign Portion
  float yli = (w1 >> 2) & 0x7FF; // Load YL Integer Portion
  float ylf =  w1 & 3;           // Load YL Fraction Portion
  float yl = yli + (ylf * 0.25); // Load YL Float
  if (yls) yl = 0.0 - yl;

  uint8_t yms = (w2 >> 29) & 1;   // Load YM Sign Portion
  float ymi = (w2 >> 18) & 0x7FF; // Load YM Integer Portion
  float ymf = (w2 >> 16) & 3;     // Load YM Fraction Portion
  float ym = ymi + (ymf * 0.25);  // Load YM Float
  if (yms) ym = 0.0 - ym;

  uint8_t yhs = (w2 >> 13) & 1;  // Load YH Sign Portion
  float yhi = (w2 >> 2) & 0x7FF; // Load YH Integer Portion
  float yhf =  w2 & 3;           // Load YH Fraction Portion
  float yh = yhi + (yhf * 0.25); // Load YH Float
  if (yhs) yh = 0.0 - yh;

  float xli = (int16_t)((w3 >> 16) & 0xFFFF); // Load XL Integer Portion
  float xlf = (uint16_t)(w3 & 0xFFFF);        // Load XL Fraction Portion
  float xl = xli + (xlf * (1.0 / 65536));     // Load XL Float

  float dxldyi = (int16_t)((w4 >> 16) & 0xFFFF);   // Load DxLDy Integer Portion
  float dxldyf = (uint16_t)(w4 & 0xFFFF);          // Load DxLDy Fraction Portion
  float dxldy = dxldyi + (dxldyf * (1.0 / 65536)); // Load DxLDy Float

  float xhi = (int16_t)((w5 >> 16) & 0xFFFF); // Load XH Integer Portion
  float xhf = (uint16_t)(w5 & 0xFFFF);        // Load XH Fraction Portion
  float xh = xhi + (xhf * (1.0 / 65536));     // Load XH Float

  float dxhdyi = (int16_t)((w6 >> 16) & 0xFFFF);   // Load DxHDy Integer Portion
  float dxhdyf = (uint16_t)(w6 & 0xFFFF);          // Load DxHDy Fraction Portion
  float dxhdy = dxhdyi + (dxhdyf * (1.0 / 65536)); // Load DxHDy Float

  float xmi = (int16_t)((w7 >> 16) & 0xFFFF); // Load XM Integer Portion
  float xmf = (uint16_t)(w7 & 0xFFFF);        // Load XM Fraction Portion
  float xm = xmi + (xmf * (1.0 / 65536));     // Load XM Float

  float dxmdyi = (int16_t)((w8 >> 16) & 0xFFFF);   // Load DxMDy Integer Portion
  float dxmdyf = (uint16_t)(w8 & 0xFFFF);          // Load DxMDy Fraction Portion
  float dxmdy = dxmdyi + (dxmdyf * (1.0 / 65536)); // Load DxMDy Float

////  if (dir) { // Right Direction Triangle
  v1x = xh; // Load Vertex 1 X
  v1x = -1.0 + (v1x / width)  * 2.0;
  v1y = yh; // Load Vertex 1 Y
  v1y =  1.0 - (v1y / height) * 2.0;

  v2x = xl; // Load Vertex 2 X
  v2x = -1.0 + (v2x / width)  * 2.0;
  v2y = ym; // Load Vertex 2 Y
  v2y =  1.0 - (v2y / height) * 2.0;

  v3x = xh + (dxhdy * (yl - yh)); // Load Vertex 3 X
  if ((v3x == v2x) && (v3y == v2y)) v3x = xh + (dxmdy * (ym - yh));
  v3x = -1.0 + (v3x / width)  * 2.0;
  v3y = yl; // Load Vertex 3 Y
  v3y =  1.0 - (v3y / height) * 2.0;
////  }

  ////  else { // Left Direction Triangle
////    v1x = xh + ((xm - xh) / (1.0 - (dxmdy / dxhdy))); // Load Vertex 1 X
//    v1x = xh; // Load Vertex 1 X
//    if (dxmdy && dxhdy) v1x += (xm - xh) / (1.0 - (dxmdy / dxhdy));
//    else v1x += (xm - xh);
////    v1x = -1.0 + (v1x / width) * 2.0; ** TESTED PERFECT **

////    v1y = floor(yh) + ((xm - xh) / (dxhdy - dxmdy));  // Load Vertex 1 Y
//    v1y = floor(yh); // Load Vertex 1 Y
//    if (dxhdy - dxmdy) v1y += ((xm - xh) / (dxhdy - dxmdy));
//    else v1y += (xm - xh);
////    v1y = 1.0 - (v1y / height) * 2.0; ** TESTED PERFECT **

//    v2x = xm + (((xl - xm) + (dxmdy * floor(yh)) - (0.25 * dxldy * ceil(4.0 * ym))) / (1.0 - (dxldy / dxmdy))) - (dxmdy * floor(yh)); // Load Vertex 2 X
////    v2x = -1.0 + (v2x / width) * 2.0;
//    v2y = ((xl - xm) + (dxmdy * floor(yh)) - (0.25 * dxldy * ceil(4.0 * ym)) / (dxmdy - dxldy)); // Load Vertex 2 Y
////    v2y = 1.0 - (v2y / height) * 2.0;

//    v3x = xh + (((xl - xm) + (dxhdy * floor(yh)) - (0.25 * dxldy * ceil(4.0 * ym))) / (1.0 - (dxldy / dxhdy))) - (dxhdy * floor(yh)); // Load Vertex 3 X
////    v3x = -1.0 + (v3x / width) * 2.0;
//    v3y = ((xl - xm) + (dxhdy * floor(yh)) - (0.25 * dxldy * ceil(4.0 * ym))) / (dxhdy - dxldy); // Load Vertex 3 Y
////    v3y = 1.0 - (v3y / height) * 2.0;
////  }
  printf("V1=%f,%f V2=%f,%f V3=%f,%f\n", v1x,v1y, v2x,v2y, v3x,v3y);

  float tri[6] = { // Triangle Vertex Space (X,Y * 3 Triangle Points)
    v1x, v1y,
    v2x, v2y,
    v3x, v3y
  };

  glColorPointer(3, GL_UNSIGNED_BYTE, 0, col); // Set Color Array Pointer
  glVertexPointer(2, GL_FLOAT, 0, tri);        // Set Vertex Array Pointer
  glDrawArrays(GL_TRIANGLES, 0, 3); // Output Triangle
}

static void rdp_tri_noshade_z(uint32_t w1, uint32_t w2)
{
  float width = fb_width;           // Framebuffer Width In Pixels
  float height = ((width / 4) * 3); // Framebuffer Height In Pixels

  uint32_t w3 = rdp_cmd_data[rdp_cmd_cur + 2]; // Load RDP Command Word 3
  uint32_t w4 = rdp_cmd_data[rdp_cmd_cur + 3]; // Load RDP Command Word 4
  uint32_t w5 = rdp_cmd_data[rdp_cmd_cur + 4]; // Load RDP Command Word 5
  uint32_t w6 = rdp_cmd_data[rdp_cmd_cur + 5]; // Load RDP Command Word 6
  uint32_t w7 = rdp_cmd_data[rdp_cmd_cur + 6]; // Load RDP Command Word 7
  uint32_t w8 = rdp_cmd_data[rdp_cmd_cur + 7]; // Load RDP Command Word 8

  if (fb_size == PIXEL_SIZE_16BIT) { // Frame Buffer Size Of Pixels = 16 bits
    R = ((fill_color >> 11) & 0x1F) << 3;
    G = ((fill_color >>  6) & 0x1F) << 3;
    B = ((fill_color >>  1) & 0x1F) << 3;
  }
  if (fb_size == PIXEL_SIZE_32BIT) { // Frame Buffer Size Of Pixels = 32 bits
    R = fill_color >> 24;
    G = fill_color >> 16;
    B = fill_color >>  8;
  }

  uint8_t col[9] = { // 3 Byte RGB Colour * 3 Triangle Points
    R, G, B,
    R, G, B,
    R, G, B
  };

  uint8_t dir = (w1 >> 23) & 1; // Load Triangle Direction Flag (Left = 0, Right = 1)

  uint8_t yls = (w1 >> 13) & 1;  // Load YL Sign Portion
  float yli = (w1 >> 2) & 0x7FF; // Load YL Integer Portion
  float ylf =  w1 & 3;           // Load YL Fraction Portion
  float yl = yli + (ylf * 0.25); // Load YL Float
  if (yls) yl = 0.0 - yl;

  uint8_t yms = (w2 >> 29) & 1;   // Load YM Sign Portion
  float ymi = (w2 >> 18) & 0x7FF; // Load YM Integer Portion
  float ymf = (w2 >> 16) & 3;     // Load YM Fraction Portion
  float ym = ymi + (ymf * 0.25);  // Load YM Float
  if (yms) ym = 0.0 - ym;

  uint8_t yhs = (w2 >> 13) & 1;  // Load YH Sign Portion
  float yhi = (w2 >> 2) & 0x7FF; // Load YH Integer Portion
  float yhf =  w2 & 3;           // Load YH Fraction Portion
  float yh = yhi + (yhf * 0.25); // Load YH Float
  if (yhs) yh = 0.0 - yh;

  float xli = (int16_t)((w3 >> 16) & 0xFFFF); // Load XL Integer Portion
  float xlf = (uint16_t)(w3 & 0xFFFF);        // Load XL Fraction Portion
  float xl = xli + (xlf * (1.0 / 65536));     // Load XL Float

  float dxldyi = (int16_t)((w4 >> 16) & 0xFFFF);   // Load DxLDy Integer Portion
  float dxldyf = (uint16_t)(w4 & 0xFFFF);          // Load DxLDy Fraction Portion
  float dxldy = dxldyi + (dxldyf * (1.0 / 65536)); // Load DxLDy Float

  float xhi = (int16_t)((w5 >> 16) & 0xFFFF); // Load XH Integer Portion
  float xhf = (uint16_t)(w5 & 0xFFFF);        // Load XH Fraction Portion
  float xh = xhi + (xhf * (1.0 / 65536));     // Load XH Float

  float dxhdyi = (int16_t)((w6 >> 16) & 0xFFFF);   // Load DxHDy Integer Portion
  float dxhdyf = (uint16_t)(w6 & 0xFFFF);          // Load DxHDy Fraction Portion
  float dxhdy = dxhdyi + (dxhdyf * (1.0 / 65536)); // Load DxHDy Float

  float xmi = (int16_t)((w7 >> 16) & 0xFFFF); // Load XM Integer Portion
  float xmf = (uint16_t)(w7 & 0xFFFF);        // Load XM Fraction Portion
  float xm = xmi + (xmf * (1.0 / 65536));     // Load XM Float

  float dxmdyi = (int16_t)((w8 >> 16) & 0xFFFF);   // Load DxMDy Integer Portion
  float dxmdyf = (uint16_t)(w8 & 0xFFFF);          // Load DxMDy Fraction Portion
  float dxmdy = dxmdyi + (dxmdyf * (1.0 / 65536)); // Load DxMDy Float

  v1x = xh; // Load Vertex 1 X
  v1x = -1.0 + (v1x / width)  * 2.0;
  v1y = yh; // Load Vertex 1 Y
  v1y =  1.0 - (v1y / height) * 2.0;

  v2x = xl; // Load Vertex 2 X
  v2x = -1.0 + (v2x / width)  * 2.0;
  v2y = ym; // Load Vertex 2 Y
  v2y =  1.0 - (v2y / height) * 2.0;

  v3x = xh + (dxhdy * (yl - yh)); // Load Vertex 3 X
  if ((v3x == v2x) && (v3y == v2y)) v3x = xh + (dxmdy * (ym - yh));
  v3x = -1.0 + (v3x / width)  * 2.0;
  v3y = yl; // Load Vertex 3 Y
  v3y =  1.0 - (v3y / height) * 2.0;

  float tri[6] = { // Triangle Vertex Space (X,Y * 3 Triangle Points)
    v1x, v1y,
    v2x, v2y,
    v3x, v3y
  };

  glColorPointer(3, GL_UNSIGNED_BYTE, 0, col); // Set Color Array Pointer
  glVertexPointer(2, GL_FLOAT, 0, tri);        // Set Vertex Array Pointer
  glDrawArrays(GL_TRIANGLES, 0, 3); // Output Triangle
}

static void rdp_tri_tex(uint32_t w1, uint32_t w2)
{
  float width = fb_width;           // Framebuffer Width In Pixels
  float height = ((width / 4) * 3); // Framebuffer Height In Pixels

  uint32_t w3 = rdp_cmd_data[rdp_cmd_cur + 2]; // Load RDP Command Word 3
  uint32_t w4 = rdp_cmd_data[rdp_cmd_cur + 3]; // Load RDP Command Word 4
  uint32_t w5 = rdp_cmd_data[rdp_cmd_cur + 4]; // Load RDP Command Word 5
  uint32_t w6 = rdp_cmd_data[rdp_cmd_cur + 5]; // Load RDP Command Word 6
  uint32_t w7 = rdp_cmd_data[rdp_cmd_cur + 6]; // Load RDP Command Word 7
  uint32_t w8 = rdp_cmd_data[rdp_cmd_cur + 7]; // Load RDP Command Word 8

  uint8_t col[12] = { // 4 Byte RGBA Colour * 3 Triangle Points
    prim_color.r, prim_color.g, prim_color.b, prim_color.a,
    prim_color.r, prim_color.g, prim_color.b, prim_color.a,
    prim_color.r, prim_color.g, prim_color.b, prim_color.a
  };

  uint8_t dir = (w1 >> 23) & 1; // Load Triangle Direction Flag (Left = 0, Right = 1)

  uint8_t tilenum = (w1 >> 16) & 7; // Load Tile Number

  uint8_t yls = (w1 >> 13) & 1;  // Load YL Sign Portion
  float yli = (w1 >> 2) & 0x7FF; // Load YL Integer Portion
  float ylf =  w1 & 3;           // Load YL Fraction Portion
  float yl = yli + (ylf * 0.25); // Load YL Float
  if (yls) yl = 0.0 - yl;

  uint8_t yms = (w2 >> 29) & 1;   // Load YM Sign Portion
  float ymi = (w2 >> 18) & 0x7FF; // Load YM Integer Portion
  float ymf = (w2 >> 16) & 3;     // Load YM Fraction Portion
  float ym = ymi + (ymf * 0.25);  // Load YM Float
  if (yms) ym = 0.0 - ym;

  uint8_t yhs = (w2 >> 13) & 1;  // Load YH Sign Portion
  float yhi = (w2 >> 2) & 0x7FF; // Load YH Integer Portion
  float yhf =  w2 & 3;           // Load YH Fraction Portion
  float yh = yhi + (yhf * 0.25); // Load YH Float
  if (yhs) yh = 0.0 - yh;

  float xli = (int16_t)((w3 >> 16) & 0xFFFF); // Load XL Integer Portion
  float xlf = (uint16_t)(w3 & 0xFFFF);        // Load XL Fraction Portion
  float xl = xli + (xlf * (1.0 / 65536));     // Load XL Float

  float dxldyi = (int16_t)((w4 >> 16) & 0xFFFF);   // Load DxLDy Integer Portion
  float dxldyf = (uint16_t)(w4 & 0xFFFF);          // Load DxLDy Fraction Portion
  float dxldy = dxldyi + (dxldyf * (1.0 / 65536)); // Load DxLDy Float

  float xhi = (int16_t)((w5 >> 16) & 0xFFFF); // Load XH Integer Portion
  float xhf = (uint16_t)(w5 & 0xFFFF);        // Load XH Fraction Portion
  float xh = xhi + (xhf * (1.0 / 65536));     // Load XH Float

  float dxhdyi = (int16_t)((w6 >> 16) & 0xFFFF);   // Load DxHDy Integer Portion
  float dxhdyf = (uint16_t)(w6 & 0xFFFF);          // Load DxHDy Fraction Portion
  float dxhdy = dxhdyi + (dxhdyf * (1.0 / 65536)); // Load DxHDy Float

  float xmi = (int16_t)((w7 >> 16) & 0xFFFF); // Load XM Integer Portion
  float xmf = (uint16_t)(w7 & 0xFFFF);        // Load XM Fraction Portion
  float xm = xmi + (xmf * (1.0 / 65536));     // Load XM Float

  float dxmdyi = (int16_t)((w8 >> 16) & 0xFFFF);   // Load DxMDy Integer Portion
  float dxmdyf = (uint16_t)(w8 & 0xFFFF);          // Load DxMDy Fraction Portion
  float dxmdy = dxmdyi + (dxmdyf * (1.0 / 65536)); // Load DxMDy Float

  v1x = xh; // Load Vertex 1 X
  v1x = -1.0 + (v1x / width)  * 2.0;
  v1y = yh; // Load Vertex 1 Y
  v1y =  1.0 - (v1y / height) * 2.0;

  v2x = xl; // Load Vertex 2 X
  v2x = -1.0 + (v2x / width)  * 2.0;
  v2y = ym; // Load Vertex 2 Y
  v2y =  1.0 - (v2y / height) * 2.0;

  v3x = xh + (dxhdy * (yl - yh)); // Load Vertex 3 X
  if ((v3x == v2x) && (v3y == v2y)) v3x = xh + (dxmdy * (ym - yh));
  v3x = -1.0 + (v3x / width)  * 2.0;
  v3y = yl; // Load Vertex 3 Y
  v3y =  1.0 - (v3y / height) * 2.0;

  float tri[6] = { // Triangle Vertex Space (X,Y * 3 Triangle Points)
    v1x, v1y,
    v2x, v2y,
    v3x, v3y
  };

  uint16_t tile_width  = (tile[tilenum].sh >> 2) + 1;
  uint16_t tile_height = (tile[tilenum].th >> 2) + 1;
  uint8_t* texture = rdram_8 + ti_address;
  
  glBindTexture(GL_TEXTURE_2D, tilenum);

  if (tile[tilenum].format == FORMAT_RGBA) {
    if (tile[tilenum].size == PIXEL_SIZE_16BIT) glTexImage2D(GL_TEXTURE_2D, tilenum, GL_RGBA, tile_width, tile_height, 0, GL_RGBA, GL_UNSIGNED_SHORT_5_5_5_1, texture);
    if (tile[tilenum].size == PIXEL_SIZE_32BIT) glTexImage2D(GL_TEXTURE_2D, tilenum, GL_RGBA, tile_width, tile_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, texture);
  }

  if (tile[tilenum].format == FORMAT_CI) {
    int i = 0;
    if (tile[tilenum].size == PIXEL_SIZE_4BIT) {
      for (i = 0; i != tile_width * tile_height << 2; i += 4) {
        texconv[i]   = rdram_8[(tlut_address + (tile[tilenum].palette << 5)) + ((texture[i >> 2] << 1) >> 4)];
        texconv[i+1] = rdram_8[(tlut_address + (tile[tilenum].palette << 5)) + ((texture[i >> 2] << 1) >> 4) + 1];
        texconv[i+2] = rdram_8[(tlut_address + (tile[tilenum].palette << 5)) + ((texture[i >> 2] << 1) & 0xF)];
        texconv[i+3] = rdram_8[(tlut_address + (tile[tilenum].palette << 5)) + ((texture[i >> 2] << 1) & 0xF) + 1];
      }
    }

    if (tile[tilenum].size == PIXEL_SIZE_8BIT) {
      for (i = 0; i != tile_width * tile_height << 1; i += 2) {
        texconv[i]   = rdram_8[tlut_address + (texture[i >> 1] << 1)];
        texconv[i+1] = rdram_8[tlut_address + (texture[i >> 1] << 1) + 1];
      }
    }

    if (other_modes.tlut_type == 0) glTexImage2D(GL_TEXTURE_2D, tilenum, GL_RGBA, tile_width, tile_height, 0, GL_RGBA, GL_UNSIGNED_SHORT_5_5_5_1, texconv);
    if (other_modes.tlut_type == 1) glTexImage2D(GL_TEXTURE_2D, tilenum, GL_LUMINANCE_ALPHA, tile_width, tile_height, 0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, texconv);
  }

  if (tile[tilenum].format == FORMAT_IA) {
    if (tile[tilenum].size == PIXEL_SIZE_4BIT) {
      int i = 0;
      for (i = 0; i != tile_width * tile_height << 2; i += 4) {
         texconv[i]   =  (texture[i >> 2] >> 6) * 85;
         texconv[i+1] = ((texture[i >> 2] >> 4) & 3) * 85;
         texconv[i+2] = ((texture[i >> 2] >> 2) & 3) * 85;
         texconv[i+3] =  (texture[i >> 2] & 3)  * 85;
      }
      glTexImage2D(GL_TEXTURE_2D, tilenum, GL_LUMINANCE_ALPHA, tile_width, tile_height, 0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, texconv);
    }
    if (tile[tilenum].size == PIXEL_SIZE_8BIT) {
      int i = 0;
      for (i = 0; i != tile_width * tile_height << 1; i += 2) {
         texconv[i]   = (texture[i >> 1] >> 4)  * 17;
         texconv[i+1] = (texture[i >> 1] & 0xF) * 17;
      }
      glTexImage2D(GL_TEXTURE_2D, tilenum, GL_LUMINANCE_ALPHA, tile_width, tile_height, 0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, texconv);
    }
    if (tile[tilenum].size == PIXEL_SIZE_16BIT) glTexImage2D(GL_TEXTURE_2D, tilenum, GL_LUMINANCE_ALPHA, tile_width, tile_height, 0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, texture);
  }

  if (tile[tilenum].format == FORMAT_I) {
    if (tile[tilenum].size == PIXEL_SIZE_4BIT) {
      int i = 0;
      for (i = 0; i != tile_width * tile_height; i += 2) {
         texconv[i]   = (texture[i >> 1] >> 4)  * 17;
         texconv[i+1] = (texture[i >> 1] & 0xF) * 17;
      }
      glTexImage2D(GL_TEXTURE_2D, tilenum, GL_INTENSITY, tile_width, tile_height, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, texconv);
    }
    if (tile[tilenum].size == PIXEL_SIZE_8BIT) glTexImage2D(GL_TEXTURE_2D, tilenum, GL_INTENSITY, tile_width, tile_height, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, texture);
  }

  if (tile[tilenum].clamp_s) glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  if (tile[tilenum].clamp_t) glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  if (tile[tilenum].mirror_s) glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT);
  if (tile[tilenum].mirror_t) glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT);
  if ((! tile[tilenum].clamp_s) && (! tile[tilenum].mirror_s)) glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
  if ((! tile[tilenum].clamp_t) && (! tile[tilenum].mirror_t)) glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

  glColorPointer(4, GL_UNSIGNED_BYTE, 0, col); // Set Color Array Pointer
  glVertexPointer(2, GL_FLOAT, 0, tri);        // Set Vertex Array Pointer

  glEnable(GL_BLEND); // Enable Color Blending
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // Alpha blending function
  glDrawArrays(GL_TRIANGLES, 0, 3); // Output Triangle
  glDisable(GL_BLEND); // Disable Color Blending
}

static void rdp_tri_tex_z(uint32_t w1, uint32_t w2)
{
  float width = fb_width;           // Framebuffer Width In Pixels
  float height = ((width / 4) * 3); // Framebuffer Height In Pixels

  uint32_t w3 = rdp_cmd_data[rdp_cmd_cur + 2]; // Load RDP Command Word 3
  uint32_t w4 = rdp_cmd_data[rdp_cmd_cur + 3]; // Load RDP Command Word 4
  uint32_t w5 = rdp_cmd_data[rdp_cmd_cur + 4]; // Load RDP Command Word 5
  uint32_t w6 = rdp_cmd_data[rdp_cmd_cur + 5]; // Load RDP Command Word 6
  uint32_t w7 = rdp_cmd_data[rdp_cmd_cur + 6]; // Load RDP Command Word 7
  uint32_t w8 = rdp_cmd_data[rdp_cmd_cur + 7]; // Load RDP Command Word 8

  uint8_t col[12] = { // 4 Byte RGBA Colour * 3 Triangle Points
    prim_color.r, prim_color.g, prim_color.b, prim_color.a,
    prim_color.r, prim_color.g, prim_color.b, prim_color.a,
    prim_color.r, prim_color.g, prim_color.b, prim_color.a
  };

  uint8_t dir = (w1 >> 23) & 1; // Load Triangle Direction Flag (Left = 0, Right = 1)

  uint8_t tilenum = (w1 >> 16) & 7; // Load Tile Number

  uint8_t yls = (w1 >> 13) & 1;  // Load YL Sign Portion
  float yli = (w1 >> 2) & 0x7FF; // Load YL Integer Portion
  float ylf =  w1 & 3;           // Load YL Fraction Portion
  float yl = yli + (ylf * 0.25); // Load YL Float
  if (yls) yl = 0.0 - yl;

  uint8_t yms = (w2 >> 29) & 1;   // Load YM Sign Portion
  float ymi = (w2 >> 18) & 0x7FF; // Load YM Integer Portion
  float ymf = (w2 >> 16) & 3;     // Load YM Fraction Portion
  float ym = ymi + (ymf * 0.25);  // Load YM Float
  if (yms) ym = 0.0 - ym;

  uint8_t yhs = (w2 >> 13) & 1;  // Load YH Sign Portion
  float yhi = (w2 >> 2) & 0x7FF; // Load YH Integer Portion
  float yhf =  w2 & 3;           // Load YH Fraction Portion
  float yh = yhi + (yhf * 0.25); // Load YH Float
  if (yhs) yh = 0.0 - yh;

  float xli = (int16_t)((w3 >> 16) & 0xFFFF); // Load XL Integer Portion
  float xlf = (uint16_t)(w3 & 0xFFFF);        // Load XL Fraction Portion
  float xl = xli + (xlf * (1.0 / 65536));     // Load XL Float

  float dxldyi = (int16_t)((w4 >> 16) & 0xFFFF);   // Load DxLDy Integer Portion
  float dxldyf = (uint16_t)(w4 & 0xFFFF);          // Load DxLDy Fraction Portion
  float dxldy = dxldyi + (dxldyf * (1.0 / 65536)); // Load DxLDy Float

  float xhi = (int16_t)((w5 >> 16) & 0xFFFF); // Load XH Integer Portion
  float xhf = (uint16_t)(w5 & 0xFFFF);        // Load XH Fraction Portion
  float xh = xhi + (xhf * (1.0 / 65536));     // Load XH Float

  float dxhdyi = (int16_t)((w6 >> 16) & 0xFFFF);   // Load DxHDy Integer Portion
  float dxhdyf = (uint16_t)(w6 & 0xFFFF);          // Load DxHDy Fraction Portion
  float dxhdy = dxhdyi + (dxhdyf * (1.0 / 65536)); // Load DxHDy Float

  float xmi = (int16_t)((w7 >> 16) & 0xFFFF); // Load XM Integer Portion
  float xmf = (uint16_t)(w7 & 0xFFFF);        // Load XM Fraction Portion
  float xm = xmi + (xmf * (1.0 / 65536));     // Load XM Float

  float dxmdyi = (int16_t)((w8 >> 16) & 0xFFFF);   // Load DxMDy Integer Portion
  float dxmdyf = (uint16_t)(w8 & 0xFFFF);          // Load DxMDy Fraction Portion
  float dxmdy = dxmdyi + (dxmdyf * (1.0 / 65536)); // Load DxMDy Float

  v1x = xh; // Load Vertex 1 X
  v1x = -1.0 + (v1x / width)  * 2.0;
  v1y = yh; // Load Vertex 1 Y
  v1y =  1.0 - (v1y / height) * 2.0;

  v2x = xl; // Load Vertex 2 X
  v2x = -1.0 + (v2x / width)  * 2.0;
  v2y = ym; // Load Vertex 2 Y
  v2y =  1.0 - (v2y / height) * 2.0;

  v3x = xh + (dxhdy * (yl - yh)); // Load Vertex 3 X
  if ((v3x == v2x) && (v3y == v2y)) v3x = xh + (dxmdy * (ym - yh));
  v3x = -1.0 + (v3x / width)  * 2.0;
  v3y = yl; // Load Vertex 3 Y
  v3y =  1.0 - (v3y / height) * 2.0;

  float tri[6] = { // Triangle Vertex Space (X,Y * 3 Triangle Points)
    v1x, v1y,
    v2x, v2y,
    v3x, v3y
  };

  uint16_t tile_width  = (tile[tilenum].sh >> 2) + 1;
  uint16_t tile_height = (tile[tilenum].th >> 2) + 1;
  uint8_t* texture = rdram_8 + ti_address;
  
  glBindTexture(GL_TEXTURE_2D, tilenum);

  if (tile[tilenum].format == FORMAT_RGBA) {
    if (tile[tilenum].size == PIXEL_SIZE_16BIT) glTexImage2D(GL_TEXTURE_2D, tilenum, GL_RGBA, tile_width, tile_height, 0, GL_RGBA, GL_UNSIGNED_SHORT_5_5_5_1, texture);
    if (tile[tilenum].size == PIXEL_SIZE_32BIT) glTexImage2D(GL_TEXTURE_2D, tilenum, GL_RGBA, tile_width, tile_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, texture);
  }

  if (tile[tilenum].format == FORMAT_CI) {
    int i = 0;
    if (tile[tilenum].size == PIXEL_SIZE_4BIT) {
      for (i = 0; i != tile_width * tile_height << 2; i += 4) {
        texconv[i]   = rdram_8[(tlut_address + (tile[tilenum].palette << 5)) + ((texture[i >> 2] << 1) >> 4)];
        texconv[i+1] = rdram_8[(tlut_address + (tile[tilenum].palette << 5)) + ((texture[i >> 2] << 1) >> 4) + 1];
        texconv[i+2] = rdram_8[(tlut_address + (tile[tilenum].palette << 5)) + ((texture[i >> 2] << 1) & 0xF)];
        texconv[i+3] = rdram_8[(tlut_address + (tile[tilenum].palette << 5)) + ((texture[i >> 2] << 1) & 0xF) + 1];
      }
    }

    if (tile[tilenum].size == PIXEL_SIZE_8BIT) {
      for (i = 0; i != tile_width * tile_height << 1; i += 2) {
        texconv[i]   = rdram_8[tlut_address + (texture[i >> 1] << 1)];
        texconv[i+1] = rdram_8[tlut_address + (texture[i >> 1] << 1) + 1];
      }
    }

    if (other_modes.tlut_type == 0) glTexImage2D(GL_TEXTURE_2D, tilenum, GL_RGBA, tile_width, tile_height, 0, GL_RGBA, GL_UNSIGNED_SHORT_5_5_5_1, texconv);
    if (other_modes.tlut_type == 1) glTexImage2D(GL_TEXTURE_2D, tilenum, GL_LUMINANCE_ALPHA, tile_width, tile_height, 0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, texconv);
  }

  if (tile[tilenum].format == FORMAT_IA) {
    if (tile[tilenum].size == PIXEL_SIZE_4BIT) {
      int i = 0;
      for (i = 0; i != tile_width * tile_height << 2; i += 4) {
         texconv[i]   =  (texture[i >> 2] >> 6) * 85;
         texconv[i+1] = ((texture[i >> 2] >> 4) & 3) * 85;
         texconv[i+2] = ((texture[i >> 2] >> 2) & 3) * 85;
         texconv[i+3] =  (texture[i >> 2] & 3)  * 85;
      }
      glTexImage2D(GL_TEXTURE_2D, tilenum, GL_LUMINANCE_ALPHA, tile_width, tile_height, 0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, texconv);
    }
    if (tile[tilenum].size == PIXEL_SIZE_8BIT) {
      int i = 0;
      for (i = 0; i != tile_width * tile_height << 1; i += 2) {
         texconv[i]   = (texture[i >> 1] >> 4)  * 17;
         texconv[i+1] = (texture[i >> 1] & 0xF) * 17;
      }
      glTexImage2D(GL_TEXTURE_2D, tilenum, GL_LUMINANCE_ALPHA, tile_width, tile_height, 0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, texconv);
    }
    if (tile[tilenum].size == PIXEL_SIZE_16BIT) glTexImage2D(GL_TEXTURE_2D, tilenum, GL_LUMINANCE_ALPHA, tile_width, tile_height, 0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, texture);
  }

  if (tile[tilenum].format == FORMAT_I) {
    if (tile[tilenum].size == PIXEL_SIZE_4BIT) {
      int i = 0;
      for (i = 0; i != tile_width * tile_height; i += 2) {
         texconv[i]   = (texture[i >> 1] >> 4)  * 17;
         texconv[i+1] = (texture[i >> 1] & 0xF) * 17;
      }
      glTexImage2D(GL_TEXTURE_2D, tilenum, GL_INTENSITY, tile_width, tile_height, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, texconv);
    }
    if (tile[tilenum].size == PIXEL_SIZE_8BIT) glTexImage2D(GL_TEXTURE_2D, tilenum, GL_INTENSITY, tile_width, tile_height, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, texture);
  }

  if (tile[tilenum].clamp_s) glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  if (tile[tilenum].clamp_t) glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  if (tile[tilenum].mirror_s) glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT);
  if (tile[tilenum].mirror_t) glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT);
  if ((! tile[tilenum].clamp_s) && (! tile[tilenum].mirror_s)) glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
  if ((! tile[tilenum].clamp_t) && (! tile[tilenum].mirror_t)) glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

  glColorPointer(4, GL_UNSIGNED_BYTE, 0, col); // Set Color Array Pointer
  glVertexPointer(2, GL_FLOAT, 0, tri);        // Set Vertex Array Pointer

  glEnable(GL_BLEND); // Enable Color Blending
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // Alpha blending function
  glDrawArrays(GL_TRIANGLES, 0, 3); // Output Triangle
  glDisable(GL_BLEND); // Disable Color Blending
}

static void rdp_tri_shade(uint32_t w1, uint32_t w2)
{
  float width = fb_width;           // Framebuffer Width In Pixels
  float height = ((width / 4) * 3); // Framebuffer Height In Pixels

  uint32_t w3 = rdp_cmd_data[rdp_cmd_cur + 2]; // Load RDP Command Word 3
  uint32_t w4 = rdp_cmd_data[rdp_cmd_cur + 3]; // Load RDP Command Word 4
  uint32_t w5 = rdp_cmd_data[rdp_cmd_cur + 4]; // Load RDP Command Word 5
  uint32_t w6 = rdp_cmd_data[rdp_cmd_cur + 5]; // Load RDP Command Word 6
  uint32_t w7 = rdp_cmd_data[rdp_cmd_cur + 6]; // Load RDP Command Word 7
  uint32_t w8 = rdp_cmd_data[rdp_cmd_cur + 7]; // Load RDP Command Word 8

  uint8_t col[12] = { // 4 Byte RGBA Colour * 3 Triangle Points
    prim_color.r, prim_color.g, prim_color.b, prim_color.a,
    prim_color.r, prim_color.g, prim_color.b, prim_color.a,
    prim_color.r, prim_color.g, prim_color.b, prim_color.a
  };

  uint8_t dir = (w1 >> 23) & 1; // Load Triangle Direction Flag (Left = 0, Right = 1)

  uint8_t yls = (w1 >> 13) & 1;  // Load YL Sign Portion
  float yli = (w1 >> 2) & 0x7FF; // Load YL Integer Portion
  float ylf =  w1 & 3;           // Load YL Fraction Portion
  float yl = yli + (ylf * 0.25); // Load YL Float
  if (yls) yl = 0.0 - yl;

  uint8_t yms = (w2 >> 29) & 1;   // Load YM Sign Portion
  float ymi = (w2 >> 18) & 0x7FF; // Load YM Integer Portion
  float ymf = (w2 >> 16) & 3;     // Load YM Fraction Portion
  float ym = ymi + (ymf * 0.25);  // Load YM Float
  if (yms) ym = 0.0 - ym;

  uint8_t yhs = (w2 >> 13) & 1;  // Load YH Sign Portion
  float yhi = (w2 >> 2) & 0x7FF; // Load YH Integer Portion
  float yhf =  w2 & 3;           // Load YH Fraction Portion
  float yh = yhi + (yhf * 0.25); // Load YH Float
  if (yhs) yh = 0.0 - yh;

  float xli = (int16_t)((w3 >> 16) & 0xFFFF); // Load XL Integer Portion
  float xlf = (uint16_t)(w3 & 0xFFFF);        // Load XL Fraction Portion
  float xl = xli + (xlf * (1.0 / 65536));     // Load XL Float

  float dxldyi = (int16_t)((w4 >> 16) & 0xFFFF);   // Load DxLDy Integer Portion
  float dxldyf = (uint16_t)(w4 & 0xFFFF);          // Load DxLDy Fraction Portion
  float dxldy = dxldyi + (dxldyf * (1.0 / 65536)); // Load DxLDy Float

  float xhi = (int16_t)((w5 >> 16) & 0xFFFF); // Load XH Integer Portion
  float xhf = (uint16_t)(w5 & 0xFFFF);        // Load XH Fraction Portion
  float xh = xhi + (xhf * (1.0 / 65536));     // Load XH Float

  float dxhdyi = (int16_t)((w6 >> 16) & 0xFFFF);   // Load DxHDy Integer Portion
  float dxhdyf = (uint16_t)(w6 & 0xFFFF);          // Load DxHDy Fraction Portion
  float dxhdy = dxhdyi + (dxhdyf * (1.0 / 65536)); // Load DxHDy Float

  float xmi = (int16_t)((w7 >> 16) & 0xFFFF); // Load XM Integer Portion
  float xmf = (uint16_t)(w7 & 0xFFFF);        // Load XM Fraction Portion
  float xm = xmi + (xmf * (1.0 / 65536));     // Load XM Float

  float dxmdyi = (int16_t)((w8 >> 16) & 0xFFFF);   // Load DxMDy Integer Portion
  float dxmdyf = (uint16_t)(w8 & 0xFFFF);          // Load DxMDy Fraction Portion
  float dxmdy = dxmdyi + (dxmdyf * (1.0 / 65536)); // Load DxMDy Float

  v1x = xh; // Load Vertex 1 X
  v1x = -1.0 + (v1x / width)  * 2.0;
  v1y = yh; // Load Vertex 1 Y
  v1y =  1.0 - (v1y / height) * 2.0;

  v2x = xl; // Load Vertex 2 X
  v2x = -1.0 + (v2x / width)  * 2.0;
  v2y = ym; // Load Vertex 2 Y
  v2y =  1.0 - (v2y / height) * 2.0;

  v3x = xh + (dxhdy * (yl - yh)); // Load Vertex 3 X
  if ((v3x == v2x) && (v3y == v2y)) v3x = xh + (dxmdy * (ym - yh));
  v3x = -1.0 + (v3x / width)  * 2.0;
  v3y = yl; // Load Vertex 3 Y
  v3y =  1.0 - (v3y / height) * 2.0;

  float tri[6] = { // Triangle Vertex Space (X,Y * 3 Triangle Points)
    v1x, v1y,
    v2x, v2y,
    v3x, v3y
  };

  glColorPointer(4, GL_UNSIGNED_BYTE, 0, col); // Set Color Array Pointer
  glVertexPointer(2, GL_FLOAT, 0, tri);        // Set Vertex Array Pointer
  glDrawArrays(GL_TRIANGLES, 0, 3); // Output Triangle
}

static void rdp_tri_shade_z(uint32_t w1, uint32_t w2)
{
  float width = fb_width;           // Framebuffer Width In Pixels
  float height = ((width / 4) * 3); // Framebuffer Height In Pixels

  uint32_t w3 = rdp_cmd_data[rdp_cmd_cur + 2]; // Load RDP Command Word 3
  uint32_t w4 = rdp_cmd_data[rdp_cmd_cur + 3]; // Load RDP Command Word 4
  uint32_t w5 = rdp_cmd_data[rdp_cmd_cur + 4]; // Load RDP Command Word 5
  uint32_t w6 = rdp_cmd_data[rdp_cmd_cur + 5]; // Load RDP Command Word 6
  uint32_t w7 = rdp_cmd_data[rdp_cmd_cur + 6]; // Load RDP Command Word 7
  uint32_t w8 = rdp_cmd_data[rdp_cmd_cur + 7]; // Load RDP Command Word 8

  uint8_t col[12] = { // 4 Byte RGBA Colour * 3 Triangle Points
    prim_color.r, prim_color.g, prim_color.b, prim_color.a,
    prim_color.r, prim_color.g, prim_color.b, prim_color.a,
    prim_color.r, prim_color.g, prim_color.b, prim_color.a
  };

  uint8_t dir = (w1 >> 23) & 1; // Load Triangle Direction Flag (Left = 0, Right = 1)

  uint8_t yls = (w1 >> 13) & 1;  // Load YL Sign Portion
  float yli = (w1 >> 2) & 0x7FF; // Load YL Integer Portion
  float ylf =  w1 & 3;           // Load YL Fraction Portion
  float yl = yli + (ylf * 0.25); // Load YL Float
  if (yls) yl = 0.0 - yl;

  uint8_t yms = (w2 >> 29) & 1;   // Load YM Sign Portion
  float ymi = (w2 >> 18) & 0x7FF; // Load YM Integer Portion
  float ymf = (w2 >> 16) & 3;     // Load YM Fraction Portion
  float ym = ymi + (ymf * 0.25);  // Load YM Float
  if (yms) ym = 0.0 - ym;

  uint8_t yhs = (w2 >> 13) & 1;  // Load YH Sign Portion
  float yhi = (w2 >> 2) & 0x7FF; // Load YH Integer Portion
  float yhf =  w2 & 3;           // Load YH Fraction Portion
  float yh = yhi + (yhf * 0.25); // Load YH Float
  if (yhs) yh = 0.0 - yh;

  float xli = (int16_t)((w3 >> 16) & 0xFFFF); // Load XL Integer Portion
  float xlf = (uint16_t)(w3 & 0xFFFF);        // Load XL Fraction Portion
  float xl = xli + (xlf * (1.0 / 65536));     // Load XL Float

  float dxldyi = (int16_t)((w4 >> 16) & 0xFFFF);   // Load DxLDy Integer Portion
  float dxldyf = (uint16_t)(w4 & 0xFFFF);          // Load DxLDy Fraction Portion
  float dxldy = dxldyi + (dxldyf * (1.0 / 65536)); // Load DxLDy Float

  float xhi = (int16_t)((w5 >> 16) & 0xFFFF); // Load XH Integer Portion
  float xhf = (uint16_t)(w5 & 0xFFFF);        // Load XH Fraction Portion
  float xh = xhi + (xhf * (1.0 / 65536));     // Load XH Float

  float dxhdyi = (int16_t)((w6 >> 16) & 0xFFFF);   // Load DxHDy Integer Portion
  float dxhdyf = (uint16_t)(w6 & 0xFFFF);          // Load DxHDy Fraction Portion
  float dxhdy = dxhdyi + (dxhdyf * (1.0 / 65536)); // Load DxHDy Float

  float xmi = (int16_t)((w7 >> 16) & 0xFFFF); // Load XM Integer Portion
  float xmf = (uint16_t)(w7 & 0xFFFF);        // Load XM Fraction Portion
  float xm = xmi + (xmf * (1.0 / 65536));     // Load XM Float

  float dxmdyi = (int16_t)((w8 >> 16) & 0xFFFF);   // Load DxMDy Integer Portion
  float dxmdyf = (uint16_t)(w8 & 0xFFFF);          // Load DxMDy Fraction Portion
  float dxmdy = dxmdyi + (dxmdyf * (1.0 / 65536)); // Load DxMDy Float

  v1x = xh; // Load Vertex 1 X
  v1x = -1.0 + (v1x / width)  * 2.0;
  v1y = yh; // Load Vertex 1 Y
  v1y =  1.0 - (v1y / height) * 2.0;

  v2x = xl; // Load Vertex 2 X
  v2x = -1.0 + (v2x / width)  * 2.0;
  v2y = ym; // Load Vertex 2 Y
  v2y =  1.0 - (v2y / height) * 2.0;

  v3x = xh + (dxhdy * (yl - yh)); // Load Vertex 3 X
  if ((v3x == v2x) && (v3y == v2y)) v3x = xh + (dxmdy * (ym - yh));
  v3x = -1.0 + (v3x / width)  * 2.0;
  v3y = yl; // Load Vertex 3 Y
  v3y =  1.0 - (v3y / height) * 2.0;

  float tri[6] = { // Triangle Vertex Space (X,Y * 3 Triangle Points)
    v1x, v1y,
    v2x, v2y,
    v3x, v3y
  };

  glColorPointer(4, GL_UNSIGNED_BYTE, 0, col); // Set Color Array Pointer
  glVertexPointer(2, GL_FLOAT, 0, tri);        // Set Vertex Array Pointer
  glDrawArrays(GL_TRIANGLES, 0, 3); // Output Triangle
}

static void rdp_tri_texshade(uint32_t w1, uint32_t w2)
{
  float width = fb_width;           // Framebuffer Width In Pixels
  float height = ((width / 4) * 3); // Framebuffer Height In Pixels

  uint32_t w3 = rdp_cmd_data[rdp_cmd_cur + 2]; // Load RDP Command Word 3
  uint32_t w4 = rdp_cmd_data[rdp_cmd_cur + 3]; // Load RDP Command Word 4
  uint32_t w5 = rdp_cmd_data[rdp_cmd_cur + 4]; // Load RDP Command Word 5
  uint32_t w6 = rdp_cmd_data[rdp_cmd_cur + 5]; // Load RDP Command Word 6
  uint32_t w7 = rdp_cmd_data[rdp_cmd_cur + 6]; // Load RDP Command Word 7
  uint32_t w8 = rdp_cmd_data[rdp_cmd_cur + 7]; // Load RDP Command Word 8

  uint8_t col[12] = { // 4 Byte RGBA Colour * 3 Triangle Points
    prim_color.r, prim_color.g, prim_color.b, prim_color.a,
    prim_color.r, prim_color.g, prim_color.b, prim_color.a,
    prim_color.r, prim_color.g, prim_color.b, prim_color.a
  };

  uint8_t dir = (w1 >> 23) & 1; // Load Triangle Direction Flag (Left = 0, Right = 1)

  uint8_t tilenum = (w1 >> 16) & 7; // Load Tile Number

  uint8_t yls = (w1 >> 13) & 1;  // Load YL Sign Portion
  float yli = (w1 >> 2) & 0x7FF; // Load YL Integer Portion
  float ylf =  w1 & 3;           // Load YL Fraction Portion
  float yl = yli + (ylf * 0.25); // Load YL Float
  if (yls) yl = 0.0 - yl;

  uint8_t yms = (w2 >> 29) & 1;   // Load YM Sign Portion
  float ymi = (w2 >> 18) & 0x7FF; // Load YM Integer Portion
  float ymf = (w2 >> 16) & 3;     // Load YM Fraction Portion
  float ym = ymi + (ymf * 0.25);  // Load YM Float
  if (yms) ym = 0.0 - ym;

  uint8_t yhs = (w2 >> 13) & 1;  // Load YH Sign Portion
  float yhi = (w2 >> 2) & 0x7FF; // Load YH Integer Portion
  float yhf =  w2 & 3;           // Load YH Fraction Portion
  float yh = yhi + (yhf * 0.25); // Load YH Float
  if (yhs) yh = 0.0 - yh;

  float xli = (int16_t)((w3 >> 16) & 0xFFFF); // Load XL Integer Portion
  float xlf = (uint16_t)(w3 & 0xFFFF);        // Load XL Fraction Portion
  float xl = xli + (xlf * (1.0 / 65536));     // Load XL Float

  float dxldyi = (int16_t)((w4 >> 16) & 0xFFFF);   // Load DxLDy Integer Portion
  float dxldyf = (uint16_t)(w4 & 0xFFFF);          // Load DxLDy Fraction Portion
  float dxldy = dxldyi + (dxldyf * (1.0 / 65536)); // Load DxLDy Float

  float xhi = (int16_t)((w5 >> 16) & 0xFFFF); // Load XH Integer Portion
  float xhf = (uint16_t)(w5 & 0xFFFF);        // Load XH Fraction Portion
  float xh = xhi + (xhf * (1.0 / 65536));     // Load XH Float

  float dxhdyi = (int16_t)((w6 >> 16) & 0xFFFF);   // Load DxHDy Integer Portion
  float dxhdyf = (uint16_t)(w6 & 0xFFFF);          // Load DxHDy Fraction Portion
  float dxhdy = dxhdyi + (dxhdyf * (1.0 / 65536)); // Load DxHDy Float

  float xmi = (int16_t)((w7 >> 16) & 0xFFFF); // Load XM Integer Portion
  float xmf = (uint16_t)(w7 & 0xFFFF);        // Load XM Fraction Portion
  float xm = xmi + (xmf * (1.0 / 65536));     // Load XM Float

  float dxmdyi = (int16_t)((w8 >> 16) & 0xFFFF);   // Load DxMDy Integer Portion
  float dxmdyf = (uint16_t)(w8 & 0xFFFF);          // Load DxMDy Fraction Portion
  float dxmdy = dxmdyi + (dxmdyf * (1.0 / 65536)); // Load DxMDy Float

  v1x = xh; // Load Vertex 1 X
  v1x = -1.0 + (v1x / width)  * 2.0;
  v1y = yh; // Load Vertex 1 Y
  v1y =  1.0 - (v1y / height) * 2.0;

  v2x = xl; // Load Vertex 2 X
  v2x = -1.0 + (v2x / width)  * 2.0;
  v2y = ym; // Load Vertex 2 Y
  v2y =  1.0 - (v2y / height) * 2.0;

  v3x = xh + (dxhdy * (yl - yh)); // Load Vertex 3 X
  if ((v3x == v2x) && (v3y == v2y)) v3x = xh + (dxmdy * (ym - yh));
  v3x = -1.0 + (v3x / width)  * 2.0;
  v3y = yl; // Load Vertex 3 Y
  v3y =  1.0 - (v3y / height) * 2.0;

  float tri[6] = { // Triangle Vertex Space (X,Y * 3 Triangle Points)
    v1x, v1y,
    v2x, v2y,
    v3x, v3y
  };

  uint16_t tile_width  = (tile[tilenum].sh >> 2) + 1;
  uint16_t tile_height = (tile[tilenum].th >> 2) + 1;
  uint8_t* texture = rdram_8 + ti_address;
  
  glBindTexture(GL_TEXTURE_2D, tilenum);

  if (tile[tilenum].format == FORMAT_RGBA) {
    if (tile[tilenum].size == PIXEL_SIZE_16BIT) glTexImage2D(GL_TEXTURE_2D, tilenum, GL_RGBA, tile_width, tile_height, 0, GL_RGBA, GL_UNSIGNED_SHORT_5_5_5_1, texture);
    if (tile[tilenum].size == PIXEL_SIZE_32BIT) glTexImage2D(GL_TEXTURE_2D, tilenum, GL_RGBA, tile_width, tile_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, texture);
  }

  if (tile[tilenum].format == FORMAT_CI) {
    int i = 0;
    if (tile[tilenum].size == PIXEL_SIZE_4BIT) {
      for (i = 0; i != tile_width * tile_height << 2; i += 4) {
        texconv[i]   = rdram_8[(tlut_address + (tile[tilenum].palette << 5)) + ((texture[i >> 2] << 1) >> 4)];
        texconv[i+1] = rdram_8[(tlut_address + (tile[tilenum].palette << 5)) + ((texture[i >> 2] << 1) >> 4) + 1];
        texconv[i+2] = rdram_8[(tlut_address + (tile[tilenum].palette << 5)) + ((texture[i >> 2] << 1) & 0xF)];
        texconv[i+3] = rdram_8[(tlut_address + (tile[tilenum].palette << 5)) + ((texture[i >> 2] << 1) & 0xF) + 1];
      }
    }

    if (tile[tilenum].size == PIXEL_SIZE_8BIT) {
      for (i = 0; i != tile_width * tile_height << 1; i += 2) {
        texconv[i]   = rdram_8[tlut_address + (texture[i >> 1] << 1)];
        texconv[i+1] = rdram_8[tlut_address + (texture[i >> 1] << 1) + 1];
      }
    }

    if (other_modes.tlut_type == 0) glTexImage2D(GL_TEXTURE_2D, tilenum, GL_RGBA, tile_width, tile_height, 0, GL_RGBA, GL_UNSIGNED_SHORT_5_5_5_1, texconv);
    if (other_modes.tlut_type == 1) glTexImage2D(GL_TEXTURE_2D, tilenum, GL_LUMINANCE_ALPHA, tile_width, tile_height, 0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, texconv);
  }

  if (tile[tilenum].format == FORMAT_IA) {
    if (tile[tilenum].size == PIXEL_SIZE_4BIT) {
      int i = 0;
      for (i = 0; i != tile_width * tile_height << 2; i += 4) {
         texconv[i]   =  (texture[i >> 2] >> 6) * 85;
         texconv[i+1] = ((texture[i >> 2] >> 4) & 3) * 85;
         texconv[i+2] = ((texture[i >> 2] >> 2) & 3) * 85;
         texconv[i+3] =  (texture[i >> 2] & 3)  * 85;
      }
      glTexImage2D(GL_TEXTURE_2D, tilenum, GL_LUMINANCE_ALPHA, tile_width, tile_height, 0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, texconv);
    }
    if (tile[tilenum].size == PIXEL_SIZE_8BIT) {
      int i = 0;
      for (i = 0; i != tile_width * tile_height << 1; i += 2) {
         texconv[i]   = (texture[i >> 1] >> 4)  * 17;
         texconv[i+1] = (texture[i >> 1] & 0xF) * 17;
      }
      glTexImage2D(GL_TEXTURE_2D, tilenum, GL_LUMINANCE_ALPHA, tile_width, tile_height, 0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, texconv);
    }
    if (tile[tilenum].size == PIXEL_SIZE_16BIT) glTexImage2D(GL_TEXTURE_2D, tilenum, GL_LUMINANCE_ALPHA, tile_width, tile_height, 0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, texture);
  }

  if (tile[tilenum].format == FORMAT_I) {
    if (tile[tilenum].size == PIXEL_SIZE_4BIT) {
      int i = 0;
      for (i = 0; i != tile_width * tile_height; i += 2) {
         texconv[i]   = (texture[i >> 1] >> 4)  * 17;
         texconv[i+1] = (texture[i >> 1] & 0xF) * 17;
      }
      glTexImage2D(GL_TEXTURE_2D, tilenum, GL_INTENSITY, tile_width, tile_height, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, texconv);
    }
    if (tile[tilenum].size == PIXEL_SIZE_8BIT) glTexImage2D(GL_TEXTURE_2D, tilenum, GL_INTENSITY, tile_width, tile_height, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, texture);
  }

  if (tile[tilenum].clamp_s) glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  if (tile[tilenum].clamp_t) glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  if (tile[tilenum].mirror_s) glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT);
  if (tile[tilenum].mirror_t) glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT);
  if ((! tile[tilenum].clamp_s) && (! tile[tilenum].mirror_s)) glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
  if ((! tile[tilenum].clamp_t) && (! tile[tilenum].mirror_t)) glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

  glColorPointer(4, GL_UNSIGNED_BYTE, 0, col); // Set Color Array Pointer
  glVertexPointer(2, GL_FLOAT, 0, tri);        // Set Vertex Array Pointer

  glEnable(GL_BLEND); // Enable Color Blending
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // Alpha blending function
  glDrawArrays(GL_TRIANGLES, 0, 3); // Output Triangle
  glDisable(GL_BLEND); // Disable Color Blending
}

static void rdp_tri_texshade_z(uint32_t w1, uint32_t w2)
{
  float width = fb_width;           // Framebuffer Width In Pixels
  float height = ((width / 4) * 3); // Framebuffer Height In Pixels

  uint32_t w3 = rdp_cmd_data[rdp_cmd_cur + 2]; // Load RDP Command Word 3
  uint32_t w4 = rdp_cmd_data[rdp_cmd_cur + 3]; // Load RDP Command Word 4
  uint32_t w5 = rdp_cmd_data[rdp_cmd_cur + 4]; // Load RDP Command Word 5
  uint32_t w6 = rdp_cmd_data[rdp_cmd_cur + 5]; // Load RDP Command Word 6
  uint32_t w7 = rdp_cmd_data[rdp_cmd_cur + 6]; // Load RDP Command Word 7
  uint32_t w8 = rdp_cmd_data[rdp_cmd_cur + 7]; // Load RDP Command Word 8

  uint8_t col[12] = { // 4 Byte RGBA Colour * 3 Triangle Points
    prim_color.r, prim_color.g, prim_color.b, prim_color.a,
    prim_color.r, prim_color.g, prim_color.b, prim_color.a,
    prim_color.r, prim_color.g, prim_color.b, prim_color.a
  };

  uint8_t dir = (w1 >> 23) & 1; // Load Triangle Direction Flag (Left = 0, Right = 1)

  uint8_t tilenum = (w1 >> 16) & 7; // Load Tile Number

  uint8_t yls = (w1 >> 13) & 1;  // Load YL Sign Portion
  float yli = (w1 >> 2) & 0x7FF; // Load YL Integer Portion
  float ylf =  w1 & 3;           // Load YL Fraction Portion
  float yl = yli + (ylf * 0.25); // Load YL Float
  if (yls) yl = 0.0 - yl;

  uint8_t yms = (w2 >> 29) & 1;   // Load YM Sign Portion
  float ymi = (w2 >> 18) & 0x7FF; // Load YM Integer Portion
  float ymf = (w2 >> 16) & 3;     // Load YM Fraction Portion
  float ym = ymi + (ymf * 0.25);  // Load YM Float
  if (yms) ym = 0.0 - ym;

  uint8_t yhs = (w2 >> 13) & 1;  // Load YH Sign Portion
  float yhi = (w2 >> 2) & 0x7FF; // Load YH Integer Portion
  float yhf =  w2 & 3;           // Load YH Fraction Portion
  float yh = yhi + (yhf * 0.25); // Load YH Float
  if (yhs) yh = 0.0 - yh;

  float xli = (int16_t)((w3 >> 16) & 0xFFFF); // Load XL Integer Portion
  float xlf = (uint16_t)(w3 & 0xFFFF);        // Load XL Fraction Portion
  float xl = xli + (xlf * (1.0 / 65536));     // Load XL Float

  float dxldyi = (int16_t)((w4 >> 16) & 0xFFFF);   // Load DxLDy Integer Portion
  float dxldyf = (uint16_t)(w4 & 0xFFFF);          // Load DxLDy Fraction Portion
  float dxldy = dxldyi + (dxldyf * (1.0 / 65536)); // Load DxLDy Float

  float xhi = (int16_t)((w5 >> 16) & 0xFFFF); // Load XH Integer Portion
  float xhf = (uint16_t)(w5 & 0xFFFF);        // Load XH Fraction Portion
  float xh = xhi + (xhf * (1.0 / 65536));     // Load XH Float

  float dxhdyi = (int16_t)((w6 >> 16) & 0xFFFF);   // Load DxHDy Integer Portion
  float dxhdyf = (uint16_t)(w6 & 0xFFFF);          // Load DxHDy Fraction Portion
  float dxhdy = dxhdyi + (dxhdyf * (1.0 / 65536)); // Load DxHDy Float

  float xmi = (int16_t)((w7 >> 16) & 0xFFFF); // Load XM Integer Portion
  float xmf = (uint16_t)(w7 & 0xFFFF);        // Load XM Fraction Portion
  float xm = xmi + (xmf * (1.0 / 65536));     // Load XM Float

  float dxmdyi = (int16_t)((w8 >> 16) & 0xFFFF);   // Load DxMDy Integer Portion
  float dxmdyf = (uint16_t)(w8 & 0xFFFF);          // Load DxMDy Fraction Portion
  float dxmdy = dxmdyi + (dxmdyf * (1.0 / 65536)); // Load DxMDy Float

  v1x = xh; // Load Vertex 1 X
  v1x = -1.0 + (v1x / width)  * 2.0;
  v1y = yh; // Load Vertex 1 Y
  v1y =  1.0 - (v1y / height) * 2.0;

  v2x = xl; // Load Vertex 2 X
  v2x = -1.0 + (v2x / width)  * 2.0;
  v2y = ym; // Load Vertex 2 Y
  v2y =  1.0 - (v2y / height) * 2.0;

  v3x = xh + (dxhdy * (yl - yh)); // Load Vertex 3 X
  if ((v3x == v2x) && (v3y == v2y)) v3x = xh + (dxmdy * (ym - yh));
  v3x = -1.0 + (v3x / width)  * 2.0;
  v3y = yl; // Load Vertex 3 Y
  v3y =  1.0 - (v3y / height) * 2.0;

  float tri[6] = { // Triangle Vertex Space (X,Y * 3 Triangle Points)
    v1x, v1y,
    v2x, v2y,
    v3x, v3y
  };

  uint16_t tile_width  = (tile[tilenum].sh >> 2) + 1;
  uint16_t tile_height = (tile[tilenum].th >> 2) + 1;
  uint8_t* texture = rdram_8 + ti_address;
  
  glBindTexture(GL_TEXTURE_2D, tilenum);

  if (tile[tilenum].format == FORMAT_RGBA) {
    if (tile[tilenum].size == PIXEL_SIZE_16BIT) glTexImage2D(GL_TEXTURE_2D, tilenum, GL_RGBA, tile_width, tile_height, 0, GL_RGBA, GL_UNSIGNED_SHORT_5_5_5_1, texture);
    if (tile[tilenum].size == PIXEL_SIZE_32BIT) glTexImage2D(GL_TEXTURE_2D, tilenum, GL_RGBA, tile_width, tile_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, texture);
  }

  if (tile[tilenum].format == FORMAT_CI) {
    int i = 0;
    if (tile[tilenum].size == PIXEL_SIZE_4BIT) {
      for (i = 0; i != tile_width * tile_height << 2; i += 4) {
        texconv[i]   = rdram_8[(tlut_address + (tile[tilenum].palette << 5)) + ((texture[i >> 2] << 1) >> 4)];
        texconv[i+1] = rdram_8[(tlut_address + (tile[tilenum].palette << 5)) + ((texture[i >> 2] << 1) >> 4) + 1];
        texconv[i+2] = rdram_8[(tlut_address + (tile[tilenum].palette << 5)) + ((texture[i >> 2] << 1) & 0xF)];
        texconv[i+3] = rdram_8[(tlut_address + (tile[tilenum].palette << 5)) + ((texture[i >> 2] << 1) & 0xF) + 1];
      }
    }

    if (tile[tilenum].size == PIXEL_SIZE_8BIT) {
      for (i = 0; i != tile_width * tile_height << 1; i += 2) {
        texconv[i]   = rdram_8[tlut_address + (texture[i >> 1] << 1)];
        texconv[i+1] = rdram_8[tlut_address + (texture[i >> 1] << 1) + 1];
      }
    }

    if (other_modes.tlut_type == 0) glTexImage2D(GL_TEXTURE_2D, tilenum, GL_RGBA, tile_width, tile_height, 0, GL_RGBA, GL_UNSIGNED_SHORT_5_5_5_1, texconv);
    if (other_modes.tlut_type == 1) glTexImage2D(GL_TEXTURE_2D, tilenum, GL_LUMINANCE_ALPHA, tile_width, tile_height, 0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, texconv);
  }

  if (tile[tilenum].format == FORMAT_IA) {
    if (tile[tilenum].size == PIXEL_SIZE_4BIT) {
      int i = 0;
      for (i = 0; i != tile_width * tile_height << 2; i += 4) {
         texconv[i]   =  (texture[i >> 2] >> 6) * 85;
         texconv[i+1] = ((texture[i >> 2] >> 4) & 3) * 85;
         texconv[i+2] = ((texture[i >> 2] >> 2) & 3) * 85;
         texconv[i+3] =  (texture[i >> 2] & 3)  * 85;
      }
      glTexImage2D(GL_TEXTURE_2D, tilenum, GL_LUMINANCE_ALPHA, tile_width, tile_height, 0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, texconv);
    }
    if (tile[tilenum].size == PIXEL_SIZE_8BIT) {
      int i = 0;
      for (i = 0; i != tile_width * tile_height << 1; i += 2) {
         texconv[i]   = (texture[i >> 1] >> 4)  * 17;
         texconv[i+1] = (texture[i >> 1] & 0xF) * 17;
      }
      glTexImage2D(GL_TEXTURE_2D, tilenum, GL_LUMINANCE_ALPHA, tile_width, tile_height, 0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, texconv);
    }
    if (tile[tilenum].size == PIXEL_SIZE_16BIT) glTexImage2D(GL_TEXTURE_2D, tilenum, GL_LUMINANCE_ALPHA, tile_width, tile_height, 0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, texture);
  }

  if (tile[tilenum].format == FORMAT_I) {
    if (tile[tilenum].size == PIXEL_SIZE_4BIT) {
      int i = 0;
      for (i = 0; i != tile_width * tile_height; i += 2) {
         texconv[i]   = (texture[i >> 1] >> 4)  * 17;
         texconv[i+1] = (texture[i >> 1] & 0xF) * 17;
      }
      glTexImage2D(GL_TEXTURE_2D, tilenum, GL_INTENSITY, tile_width, tile_height, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, texconv);
    }
    if (tile[tilenum].size == PIXEL_SIZE_8BIT) glTexImage2D(GL_TEXTURE_2D, tilenum, GL_INTENSITY, tile_width, tile_height, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, texture);
  }

  if (tile[tilenum].clamp_s) glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  if (tile[tilenum].clamp_t) glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  if (tile[tilenum].mirror_s) glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT);
  if (tile[tilenum].mirror_t) glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT);
  if ((! tile[tilenum].clamp_s) && (! tile[tilenum].mirror_s)) glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
  if ((! tile[tilenum].clamp_t) && (! tile[tilenum].mirror_t)) glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

  glColorPointer(4, GL_UNSIGNED_BYTE, 0, col); // Set Color Array Pointer
  glVertexPointer(2, GL_FLOAT, 0, tri);        // Set Vertex Array Pointer

  glEnable(GL_BLEND); // Enable Color Blending
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // Alpha blending function
  glDrawArrays(GL_TRIANGLES, 0, 3); // Output Triangle
  glDisable(GL_BLEND); // Disable Color Blending
}

static void rdp_tex_rect(uint32_t w1, uint32_t w2)
{
  float width = fb_width;           // Framebuffer Width In Pixels
  float height = ((width / 4) * 3); // Framebuffer Height In Pixels

  uint32_t w3 = rdp_cmd_data[rdp_cmd_cur + 2]; // Load RDP Command Word 3
  uint32_t w4 = rdp_cmd_data[rdp_cmd_cur + 3]; // Load RDP Command Word 4

  uint8_t col[16] = { // 4 Byte RGBA Colour * 4 Quad Points
    prim_color.r, prim_color.g, prim_color.b, prim_color.a,
    prim_color.r, prim_color.g, prim_color.b, prim_color.a,
    prim_color.r, prim_color.g, prim_color.b, prim_color.a,
    prim_color.r, prim_color.g, prim_color.b, prim_color.a
  };

  if (other_modes.cycle_type == 2) { // Cycle Type Copy
    col[0]  = 255; col[1]  = 255; col[2]  = 255; col[3]  = 255; // 4 Byte RGBA Colour * 4 Quad Points
    col[4]  = 255; col[5]  = 255; col[6]  = 255; col[7]  = 255;
    col[8]  = 255; col[9]  = 255; col[10] = 255; col[11] = 255;
    col[12] = 255; col[13] = 255; col[14] = 255; col[15] = 255;
  }

  float xli = (w1 >> 14) & 0x3FF; // Load XL Integer Portion
  float xlf = (w1 >> 12) & 3;     // Load XL Fraction Portion
  float xl = xli + (xlf * 0.25);  // Load XL Float

  float yli = (w1 >> 2) & 0x3FF; // Load YL Integer Portion
  float ylf =  w1 & 3;           // Load YL Fraction Portion
  float yl = yli + (ylf * 0.25); // Load YL Float

  float xhi = (w2 >> 14) & 0x3FF; // Load XH Integer Portion
  float xhf = (w2 >> 12) & 3;     // Load XH Fraction Portion
  float xh = xhi + (xhf * 0.25);  // Load XH Float

  float yhi = (w2 >> 2) & 0x3FF; // Load YH Integer Portion
  float yhf =  w2 & 3;           // Load YH Fraction Portion
  float yh = yhi + (yhf * 0.25); // Load YH Float

  uint8_t tilenum = (w2 >> 24) & 7; // Load Tile Number
    
  uint8_t ss = (w3 >> 31) & 1;      // Load S Sign Portion
  float   si = (w3 >> 21) & 0x3FF;  // Load S Integer Portion
  float   sf = (w3 >> 16) & 0x1F;   // Load S Fraction Portion
  float s = si + (sf * (1.0 / 32)); // Load S Float
  if (ss) s = 0.0 - s;

  uint8_t ts = (w3 >> 15) & 1;      // Load T Sign Portion
  float   ti = (w3 >>  5) & 0x3FF;  // Load T Integer Portion
  float   tf =  w3 & 0x1F;          // Load T Fraction Portion
  float t = ti + (tf * (1.0 / 32)); // Load T Float
  if (ts) t = 0.0 - t;

  uint8_t dsdxs = (w4 >> 31) & 1;              // Load DsDx Sign Portion
  float dsdxi = (w4 >> 26) & 0x1F;             // Load DsDx Integer Portion
  float dsdxf =  w4 & 0x3FF;                   // Load DsDx Fraction Portion
  float dsdx = dsdxi + (dsdxf * (1.0 / 1024)); // Load DsDx Float
  if (dsdxs) dsdx = 0.0 - dsdx;

  uint8_t dtdys = (w4 >> 15) & 1;              // Load DtDy Sign Portion
  float   dtdyi = (w4 >> 10) & 0x1F;           // Load DtDy Integer Portion
  float   dtdyf =  w4 & 0x3FF;                 // Load DtDy Fraction Portion
  float dtdy = dtdyi + (dtdyf * (1.0 / 1024)); // Load DtDy Float
  if (dtdys) dtdy = 0.0 - dtdy;

  v1x = -1.0 + ((xl /  width) * 2.0); // Load Vertex 1 X
  v1y =  1.0 - ((yl / height) * 2.0); // Load Vertex 1 Y
  v2x = -1.0 + ((xh /  width) * 2.0); // Load Vertex 2 X
  v2y =  1.0 - ((yh / height) * 2.0); // Load Vertex 2 Y

  float quad[8] = { // Quad Vertex Space (X,Y * 4 Quad Points)
    v1x, v1y,
    v2x, v1y,
    v2x, v2y,
    v1x, v2y
  };

  uint16_t tile_width  = (tile[tilenum].sh >> 2) + 1;
  uint16_t tile_height = (tile[tilenum].th >> 2) + 1;
  uint8_t* texture = rdram_8 + ti_address;

  float u1 = s / tile_width;  // Load Texture Coord 1 U
  float v1 = t / tile_height; // Load Texture Coord 1 V
  float u2 = dsdx * (1.0 / dsdx); // Load Texture Coord 2 U
  float v2 = dtdy * (1.0 / dtdy); // Load Texture Coord 2 V

  if (tile[tilenum].mask_s || tile[tilenum].mask_t) {
    uint8_t dxti = (tile[tilenum].dxt >> 11) & 1;
    float   dxtf = tile[tilenum].dxt & 0x7FF;
    float dxt = ceil(dxti + (dxtf * (1.0 / 2048)));

    uint32_t oldtile_width = tile_width;
    uint32_t oldtile_height = tile_height;
    if (tile[tilenum].mask_s) tile_width  = 1 << tile[tilenum].mask_s;
    if (tile[tilenum].mask_t) tile_height = 1 << tile[tilenum].mask_t;

    u1 *= (xl - xh) / tile_width;
    v1 *= (yl - yh) / tile_height;
    u2 *= (xl - xh) / tile_width;
    v2 *= (yl - yh) / tile_height;

    uint32_t i = 0;
    uint32_t address = 0;
    uint32_t xstep = 0;
    float pixelsize = 0;
    if (tile[tilenum].size == PIXEL_SIZE_4BIT)  pixelsize = 0.5;
    if (tile[tilenum].size == PIXEL_SIZE_8BIT)  pixelsize = 1.0;
    if (tile[tilenum].size == PIXEL_SIZE_16BIT) pixelsize = 2.0;
    if (tile[tilenum].size == PIXEL_SIZE_32BIT) pixelsize = 4.0;
    for (i=0; i != pixelsize * tile_width * tile_height; i++) {
      if (xstep == (pixelsize * tile_width)) {
        address += ((pixelsize * oldtile_width) - (pixelsize * tile_width)) + ((dxt * oldtile_height) * (pixelsize * oldtile_width));
        xstep = 0;
      }
      texmask[i] = texture[address];
      address++;
      xstep++;
    }

    texture = texmask;
  }

  float quaduv[8] = { // Quad UV Space (U,V * 4 Quad Points)
    u2, v2,
    u1, v2,
    u1, v1,
    u2, v1
  };

  glBindTexture(GL_TEXTURE_2D, tilenum);

  if (tile[tilenum].format == FORMAT_RGBA) {
    if (tile[tilenum].size == PIXEL_SIZE_16BIT) glTexImage2D(GL_TEXTURE_2D, tilenum, GL_RGBA, tile_width, tile_height, 0, GL_RGBA, GL_UNSIGNED_SHORT_5_5_5_1, texture);
    if (tile[tilenum].size == PIXEL_SIZE_32BIT) glTexImage2D(GL_TEXTURE_2D, tilenum, GL_RGBA, tile_width, tile_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, texture);
  }

  if (tile[tilenum].format == FORMAT_CI) {
    int i = 0;
    if (tile[tilenum].size == PIXEL_SIZE_4BIT) {
      for (i = 0; i != tile_width * tile_height << 2; i += 4) {
        texconv[i]   = rdram_8[(tlut_address + (tile[tilenum].palette << 5)) + ((texture[i >> 2] << 1) >> 4)];
        texconv[i+1] = rdram_8[(tlut_address + (tile[tilenum].palette << 5)) + ((texture[i >> 2] << 1) >> 4) + 1];
        texconv[i+2] = rdram_8[(tlut_address + (tile[tilenum].palette << 5)) + ((texture[i >> 2] << 1) & 0xF)];
        texconv[i+3] = rdram_8[(tlut_address + (tile[tilenum].palette << 5)) + ((texture[i >> 2] << 1) & 0xF) + 1];
      }
    }

    if (tile[tilenum].size == PIXEL_SIZE_8BIT) {
      for (i = 0; i != tile_width * tile_height << 1; i += 2) {
        texconv[i]   = rdram_8[tlut_address + (texture[i >> 1] << 1)];
        texconv[i+1] = rdram_8[tlut_address + (texture[i >> 1] << 1) + 1];
      }
    }

    if (other_modes.tlut_type == 0) glTexImage2D(GL_TEXTURE_2D, tilenum, GL_RGBA, tile_width, tile_height, 0, GL_RGBA, GL_UNSIGNED_SHORT_5_5_5_1, texconv);
    if (other_modes.tlut_type == 1) glTexImage2D(GL_TEXTURE_2D, tilenum, GL_LUMINANCE_ALPHA, tile_width, tile_height, 0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, texconv);
  }

  if (tile[tilenum].format == FORMAT_IA) {
    if (tile[tilenum].size == PIXEL_SIZE_4BIT) {
      int i = 0;
      for (i = 0; i != tile_width * tile_height << 2; i += 4) {
         texconv[i]   =  (texture[i >> 2] >> 6) * 85;
         texconv[i+1] = ((texture[i >> 2] >> 4) & 3) * 85;
         texconv[i+2] = ((texture[i >> 2] >> 2) & 3) * 85;
         texconv[i+3] =  (texture[i >> 2] & 3)  * 85;
      }
      glTexImage2D(GL_TEXTURE_2D, tilenum, GL_LUMINANCE_ALPHA, tile_width, tile_height, 0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, texconv);
    }
    if (tile[tilenum].size == PIXEL_SIZE_8BIT) {
      int i = 0;
      for (i = 0; i != tile_width * tile_height << 1; i += 2) {
         texconv[i]   = (texture[i >> 1] >> 4)  * 17;
         texconv[i+1] = (texture[i >> 1] & 0xF) * 17;
      }
      glTexImage2D(GL_TEXTURE_2D, tilenum, GL_LUMINANCE_ALPHA, tile_width, tile_height, 0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, texconv);
    }
    if (tile[tilenum].size == PIXEL_SIZE_16BIT) glTexImage2D(GL_TEXTURE_2D, tilenum, GL_LUMINANCE_ALPHA, tile_width, tile_height, 0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, texture);
  }

  if (tile[tilenum].format == FORMAT_I) {
    if (tile[tilenum].size == PIXEL_SIZE_4BIT) {
      int i = 0;
      for (i = 0; i != tile_width * tile_height; i += 2) {
         texconv[i]   = (texture[i >> 1] >> 4)  * 17;
         texconv[i+1] = (texture[i >> 1] & 0xF) * 17;
      }
      glTexImage2D(GL_TEXTURE_2D, tilenum, GL_INTENSITY, tile_width, tile_height, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, texconv);
    }
    if (tile[tilenum].size == PIXEL_SIZE_8BIT) glTexImage2D(GL_TEXTURE_2D, tilenum, GL_INTENSITY, tile_width, tile_height, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, texture);
  }

  if (tile[tilenum].clamp_s) glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  if (tile[tilenum].clamp_t) glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  if (tile[tilenum].mirror_s) glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT);
  if (tile[tilenum].mirror_t) glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT);
  if ((! tile[tilenum].clamp_s) && (! tile[tilenum].mirror_s)) glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
  if ((! tile[tilenum].clamp_t) && (! tile[tilenum].mirror_t)) glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

  glColorPointer(4, GL_UNSIGNED_BYTE, 0, col); // Set Color Array Pointer
  glVertexPointer(2, GL_FLOAT, 0, quad);       // Set Vertex Array Pointer
  glTexCoordPointer(2, GL_FLOAT, 0, quaduv);   // Set Tex Coord Array Pointer

  glEnable(GL_BLEND); // Enable Color Blending
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // Alpha blending function
  glDrawArrays(GL_QUADS, 0, 4); // Output Quad
  glDisable(GL_BLEND); // Disable Color Blending
}

static void rdp_tex_rect_flip(uint32_t w1, uint32_t w2)
{
  float width = fb_width;           // Framebuffer Width In Pixels
  float height = ((width / 4) * 3); // Framebuffer Height In Pixels

  uint32_t w3 = rdp_cmd_data[rdp_cmd_cur + 2]; // Load RDP Command Word 3
  uint32_t w4 = rdp_cmd_data[rdp_cmd_cur + 3]; // Load RDP Command Word 4

  uint8_t col[16] = { // 4 Byte RGBA Colour * 4 Quad Points
    prim_color.r, prim_color.g, prim_color.b, prim_color.a,
    prim_color.r, prim_color.g, prim_color.b, prim_color.a,
    prim_color.r, prim_color.g, prim_color.b, prim_color.a,
    prim_color.r, prim_color.g, prim_color.b, prim_color.a
  };
  
  float xli = (w1 >> 14) & 0x3FF; // Load XL Integer Portion
  float xlf = (w1 >> 12) & 3;     // Load XL Fraction Portion
  float xl = xli + (xlf * 0.25);  // Load XL Float

  float yli = (w1 >> 2) & 0x3FF; // Load YL Integer Portion
  float ylf =  w1 & 3;           // Load YL Fraction Portion
  float yl = yli + (ylf * 0.25); // Load YL Float

  float xhi = (w2 >> 14) & 0x3FF; // Load XH Integer Portion
  float xhf = (w2 >> 12) & 3;     // Load XH Fraction Portion
  float xh = xhi + (xhf * 0.25);  // Load XH Float

  float yhi = (w2 >> 2) & 0x3FF; // Load YH Integer Portion
  float yhf =  w2 & 3;           // Load YH Fraction Portion
  float yh = yhi + (yhf * 0.25); // Load YH Float

  uint8_t tilenum = (w2 >> 24) & 7; // Load Tile Number
    
  uint8_t ss = (w3 >> 31) & 1;      // Load S Sign Portion
  float   si = (w3 >> 21) & 0x3FF;  // Load S Integer Portion
  float   sf = (w3 >> 16) & 0x1F;   // Load S Fraction Portion
  float s = si + (sf * (1.0 / 32)); // Load S Float
  if (ss) s = 0.0 - s;

  uint8_t ts = (w3 >> 15) & 1;      // Load T Sign Portion
  float   ti = (w3 >>  5) & 0x3FF;  // Load T Integer Portion
  float   tf =  w3 & 0x1F;          // Load T Fraction Portion
  float t = ti + (tf * (1.0 / 32)); // Load T Float
  if (ts) t = 0.0 - t;

  uint8_t dsdxs = (w4 >> 31) & 1;              // Load DsDx Sign Portion
  float dsdxi = (w4 >> 26) & 0x1F;             // Load DsDx Integer Portion
  float dsdxf =  w4 & 0x3FF;                   // Load DsDx Fraction Portion
  float dsdx = dsdxi + (dsdxf * (1.0 / 1024)); // Load DsDx Float
  if (dsdxs) dsdx = 0.0 - dsdx;

  uint8_t dtdys = (w4 >> 15) & 1;              // Load DtDy Sign Portion
  float   dtdyi = (w4 >> 10) & 0x1F;           // Load DtDy Integer Portion
  float   dtdyf =  w4 & 0x3FF;                 // Load DtDy Fraction Portion
  float dtdy = dtdyi + (dtdyf * (1.0 / 1024)); // Load DtDy Float
  if (dtdys) dtdy = 0.0 - dtdy;

  v1x = -1.0 + ((xl /  width) * 2.0); // Load Vertex 1 X
  v1y =  1.0 - ((yl / height) * 2.0); // Load Vertex 1 Y
  v2x = -1.0 + ((xh /  width) * 2.0); // Load Vertex 2 X
  v2y =  1.0 - ((yh / height) * 2.0); // Load Vertex 2 Y

  float quad[8] = { // Quad Vertex Space (X,Y * 4 Quad Points)
    v1x, v1y,
    v2x, v1y,
    v2x, v2y,
    v1x, v2y
  };

  uint16_t tile_width  = (tile[tilenum].sh >> 2) + 1;
  uint16_t tile_height = (tile[tilenum].th >> 2) + 1;
  uint8_t* texture = rdram_8 + ti_address;

  float u1 = s / tile_width;  // Load Texture Coord 1 U
  float v1 = t / tile_height; // Load Texture Coord 1 V
  float u2 = dsdx * (1.0 / dsdx); // Load Texture Coord 2 U
  float v2 = dtdy * (1.0 / dtdy); // Load Texture Coord 2 V

//  if (tile[tilenum].mask_s || tile[tilenum].mask_t) {
//    uint32_t oldtile_width = tile_width;
//    uint32_t oldtile_height = tile_height;
//    tile_width  = 1 << tile[tilenum].mask_s;
//    tile_height = 1 << tile[tilenum].mask_t;

//    u1 *= (xl - xh) / tile_width;
//    v1 *= (yl - yh) / tile_height;
//    u2 *= (xl - xh) / tile_width;
//    v2 *= (yl - yh) / tile_height;

//    uint32_t i = 0;
//    uint32_t address = 0;
//    uint32_t xstep = 0;
//    float pixelsize = 0;
//    if (tile[tilenum].size == PIXEL_SIZE_4BIT)  pixelsize = 0.5;
//    if (tile[tilenum].size == PIXEL_SIZE_8BIT)  pixelsize = 1.0;
//    if (tile[tilenum].size == PIXEL_SIZE_16BIT) pixelsize = 2.0;
//    if (tile[tilenum].size == PIXEL_SIZE_32BIT) pixelsize = 4.0;
//    for (i=0; i != pixelsize * tile_width * tile_height; i++) {
//      if (xstep == (pixelsize * tile_width)) {
//        address += ((pixelsize * oldtile_width) - (pixelsize * tile_width));
//        xstep = 0;
//      }
//      texmask[i] = texture[address];
//      address++;
//      xstep++;
//    }

//    texture = texmask;
//  }

  float quaduv[8] = { // Quad UV Space (U,V * 4 Quad Points)
    v2, u2,
    v2, u1,
    v1, u1,
    v1, u2
  };

  glBindTexture(GL_TEXTURE_2D, tilenum);

  if (tile[tilenum].format == FORMAT_RGBA) {
    if (tile[tilenum].size == PIXEL_SIZE_16BIT) glTexImage2D(GL_TEXTURE_2D, tilenum, GL_RGBA, tile_width, tile_height, 0, GL_RGBA, GL_UNSIGNED_SHORT_5_5_5_1, texture);
    if (tile[tilenum].size == PIXEL_SIZE_32BIT) glTexImage2D(GL_TEXTURE_2D, tilenum, GL_RGBA, tile_width, tile_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, texture);
  }

  if (tile[tilenum].format == FORMAT_CI) {
    int i = 0;
    if (tile[tilenum].size == PIXEL_SIZE_4BIT) {
      for (i = 0; i != tile_width * tile_height << 2; i += 4) {
        texconv[i]   = rdram_8[(tlut_address + (tile[tilenum].palette << 5)) + ((texture[i >> 2] << 1) >> 4)];
        texconv[i+1] = rdram_8[(tlut_address + (tile[tilenum].palette << 5)) + ((texture[i >> 2] << 1) >> 4) + 1];
        texconv[i+2] = rdram_8[(tlut_address + (tile[tilenum].palette << 5)) + ((texture[i >> 2] << 1) & 0xF)];
        texconv[i+3] = rdram_8[(tlut_address + (tile[tilenum].palette << 5)) + ((texture[i >> 2] << 1) & 0xF) + 1];
      }
    }

    if (tile[tilenum].size == PIXEL_SIZE_8BIT) {
      for (i = 0; i != tile_width * tile_height << 1; i += 2) {
        texconv[i]   = rdram_8[tlut_address + (texture[i >> 1] << 1)];
        texconv[i+1] = rdram_8[tlut_address + (texture[i >> 1] << 1) + 1];
      }
    }

    if (other_modes.tlut_type == 0) glTexImage2D(GL_TEXTURE_2D, tilenum, GL_RGBA, tile_width, tile_height, 0, GL_RGBA, GL_UNSIGNED_SHORT_5_5_5_1, texconv);
    if (other_modes.tlut_type == 1) glTexImage2D(GL_TEXTURE_2D, tilenum, GL_LUMINANCE_ALPHA, tile_width, tile_height, 0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, texconv);
  }

  if (tile[tilenum].format == FORMAT_IA) {
    if (tile[tilenum].size == PIXEL_SIZE_4BIT) {
      int i = 0;
      for (i = 0; i != tile_width * tile_height << 2; i += 4) {
         texconv[i]   =  (texture[i >> 2] >> 6) * 85;
         texconv[i+1] = ((texture[i >> 2] >> 4) & 3) * 85;
         texconv[i+2] = ((texture[i >> 2] >> 2) & 3) * 85;
         texconv[i+3] =  (texture[i >> 2] & 3)  * 85;
      }
      glTexImage2D(GL_TEXTURE_2D, tilenum, GL_LUMINANCE_ALPHA, tile_width, tile_height, 0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, texconv);
    }
    if (tile[tilenum].size == PIXEL_SIZE_8BIT) {
      int i = 0;
      for (i = 0; i != tile_width * tile_height << 1; i += 2) {
         texconv[i]   = (texture[i >> 1] >> 4)  * 17;
         texconv[i+1] = (texture[i >> 1] & 0xF) * 17;
      }
      glTexImage2D(GL_TEXTURE_2D, tilenum, GL_LUMINANCE_ALPHA, tile_width, tile_height, 0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, texconv);
    }
    if (tile[tilenum].size == PIXEL_SIZE_16BIT) glTexImage2D(GL_TEXTURE_2D, tilenum, GL_LUMINANCE_ALPHA, tile_width, tile_height, 0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, texture);
  }

  if (tile[tilenum].format == FORMAT_I) {
    if (tile[tilenum].size == PIXEL_SIZE_4BIT) {
      int i = 0;
      for (i = 0; i != tile_width * tile_height; i += 2) {
         texconv[i]   = (texture[i >> 1] >> 4)  * 17;
         texconv[i+1] = (texture[i >> 1] & 0xF) * 17;
      }
      glTexImage2D(GL_TEXTURE_2D, tilenum, GL_INTENSITY, tile_width, tile_height, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, texconv);
    }
    if (tile[tilenum].size == PIXEL_SIZE_8BIT) glTexImage2D(GL_TEXTURE_2D, tilenum, GL_INTENSITY, tile_width, tile_height, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, texture);
  }

  if (tile[tilenum].clamp_s) glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  if (tile[tilenum].clamp_t) glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  if (tile[tilenum].mirror_s) glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT);
  if (tile[tilenum].mirror_t) glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT);
  if ((! tile[tilenum].clamp_s) && (! tile[tilenum].mirror_s)) glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
  if ((! tile[tilenum].clamp_t) && (! tile[tilenum].mirror_t)) glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

  glColorPointer(4, GL_UNSIGNED_BYTE, 0, col); // Set Color Array Pointer
  glVertexPointer(2, GL_FLOAT, 0, quad);       // Set Vertex Array Pointer
  glTexCoordPointer(2, GL_FLOAT, 0, quaduv);   // Set Tex Coord Array Pointer

  glEnable(GL_BLEND); // Enable Color Blending
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // Alpha blending function
  glDrawArrays(GL_QUADS, 0, 4); // Output Quad
  glDisable(GL_BLEND); // Disable Color Blending
}

static void rdp_sync_load(uint32_t w1, uint32_t w2)
{
}

static void rdp_sync_pipe(uint32_t w1, uint32_t w2)
{
}

static void rdp_sync_tile(uint32_t w1, uint32_t w2)
{
}

static void rdp_sync_full(uint32_t w1, uint32_t w2)
{
  signal_rcp_interrupt(cen64->bus.vr4300, MI_INTR_DP);
}

static void rdp_set_key_gb(uint32_t w1, uint32_t w2)
{
  key_width.g = (w1 >> 12) & 0xfff;
  key_width.b = w1 & 0xfff;
  key_center.g = (w2 >> 24) & 0xff;
  key_scale.g = (w2 >> 16) & 0xff;
  key_center.b = (w2 >> 8) & 0xff;
  key_scale.b = w2 & 0xff;
}

static void rdp_set_key_r(uint32_t w1, uint32_t w2)
{
  key_width.r = (w2 >> 16) & 0xfff;
  key_center.r = (w2 >> 8) & 0xff;
  key_scale.r = w2 & 0xff;
}

static void rdp_set_convert(uint32_t w1, uint32_t w2)
{
  k0 = (w1 >> 13) & 0x1ff;
  k1 = (w1 >> 4) & 0x1ff;
  k2 = ((w1 & 0xf) << 5) | ((w2 >> 27) & 0x1f);
  k3 = (w2 >> 18) & 0x1ff;
  k4 = (w2 >> 9) & 0x1ff;
  k5 = w2 & 0x1ff;
}

static void rdp_set_scissor(uint32_t w1, uint32_t w2)
{
  uint32_t width = fb_width + 1; // Framebuffer Width In Pixels
  uint32_t height = ((width >> 2) * 3); // Framebuffer Height In Pixels

  float xhi = (w1 >> 14) & 0x3FF; // Load XH Integer Portion
  float xhf = (w1 >> 12) & 3;     // Load XH Fraction Portion
  float xh = xhi + (xhf * 0.25);  // Load XH Float

  float yhi = (w1 >> 2) & 0x3FF; // Load YH Integer Portion
  float yhf = w1 & 3;            // Load YH Fraction Portion
  float yh = yhi + (yhf * 0.25); // Load YH Float

  float xli = (w2 >> 14) & 0x3FF; // Load XL Integer Portion
  float xlf = (w2 >> 12) & 3;     // Load XL Fraction Portion
  float xl = xli + (xlf * 0.25);  // Load XL Float

  float yli = (w2 >> 2) & 0x3FF; // Load YL Integer Portion
  float ylf = w2 & 3;            // Load YL Fraction Portion
  float yl = yli + (ylf * 0.25); // Load YL Float

  glGetFloatv(GL_VIEWPORT, viewport); // Get Current GL Window View Portion

  uint32_t v1xi = xh * (viewport[0] /  width); // Load Vertex 1 X
  uint32_t v1yi = yh * (viewport[1] / height); // Load Vertex 1 Y
  uint32_t v2xi = xl * (viewport[2] /  width); // Load Vertex 2 X
  uint32_t v2yi = yl * (viewport[3] / height); // Load Vertex 2 Y

//  glScissor(v1xi, v1yi, v2xi, v2yi); // Run Scissor Test
}

static void rdp_set_prim_depth(uint32_t w1, uint32_t w2)
{
  primitive_z = w2 & (0x7fff << 16);
  primitive_delta_z = (uint16_t)(w2);
}

static void rdp_set_other_modes(uint32_t w1, uint32_t w2)
{
  other_modes.cycle_type      = (w1 >> 20) & 0x3;
  other_modes.persp_tex_en    = (w1 & 0x80000) ? 1 : 0;
  other_modes.detail_tex_en   = (w1 & 0x40000) ? 1 : 0;
  other_modes.sharpen_tex_en    = (w1 & 0x20000) ? 1 : 0;
  other_modes.tex_lod_en      = (w1 & 0x10000) ? 1 : 0;
  other_modes.en_tlut       = (w1 & 0x08000) ? 1 : 0;
  other_modes.tlut_type     = (w1 & 0x04000) ? 1 : 0;
  other_modes.sample_type     = (w1 & 0x02000) ? 1 : 0;
  other_modes.mid_texel     = (w1 & 0x01000) ? 1 : 0;
  other_modes.bi_lerp0      = (w1 & 0x00800) ? 1 : 0;
  other_modes.bi_lerp1      = (w1 & 0x00400) ? 1 : 0;
  other_modes.convert_one     = (w1 & 0x00200) ? 1 : 0;
  other_modes.key_en        = (w1 & 0x00100) ? 1 : 0;
  other_modes.rgb_dither_sel    = (w1 >> 6) & 0x3;
  other_modes.alpha_dither_sel  = (w1 >> 4) & 0x3;
  other_modes.blend_m1a_0     = (w2 >> 30) & 0x3;
  other_modes.blend_m1a_1     = (w2 >> 28) & 0x3;
  other_modes.blend_m1b_0     = (w2 >> 26) & 0x3;
  other_modes.blend_m1b_1     = (w2 >> 24) & 0x3;
  other_modes.blend_m2a_0     = (w2 >> 22) & 0x3;
  other_modes.blend_m2a_1     = (w2 >> 20) & 0x3;
  other_modes.blend_m2b_0     = (w2 >> 18) & 0x3;
  other_modes.blend_m2b_1     = (w2 >> 16) & 0x3;
  other_modes.force_blend     = (w2 >> 14) & 1;
  other_modes.alpha_cvg_select  = (w2 >> 13) & 1;
  other_modes.cvg_times_alpha   = (w2 >> 12) & 1;
  other_modes.z_mode        = (w2 >> 10) & 0x3;
  other_modes.cvg_dest      = (w2 >> 8) & 0x3;
  other_modes.color_on_cvg    = (w2 >> 7) & 1;
  other_modes.image_read_en   = (w2 >> 6) & 1;
  other_modes.z_update_en     = (w2 >> 5) & 1;
  other_modes.z_compare_en    = (w2 >> 4) & 1;
  other_modes.antialias_en    = (w2 >> 3) & 1;
  other_modes.z_source_sel    = (w2 >> 2) & 1;
  other_modes.dither_alpha_en   = (w2 >> 1) & 1;
  other_modes.alpha_compare_en  = (w2) & 1;

  other_modes.f.stalederivs = 1;
}

static void rdp_set_tile_size(uint32_t w1, uint32_t w2)
{
  uint8_t tilenum = (w2 >> 24) & 7;
  tile[tilenum].sl = (w1 >> 12) & 0xFFF;
  tile[tilenum].tl =  w1 & 0xFFF;
  tile[tilenum].sh = (w2 >> 12) & 0xFFF;
  tile[tilenum].th =  w2 & 0xFFF;
}
  
static void rdp_load_block(uint32_t w1, uint32_t w2)
{
  uint8_t tilenum = (w2 >> 24) & 7;
  tile[tilenum].sl = (w1 >> 12) & 0xFFF;
  tile[tilenum].tl =  w1 & 0xFFF;
  tile[tilenum].sh = (w2 >> 12) & 0xFFF;
  tile[tilenum].dxt = w2 & 0xFFF;
}

static void rdp_load_tlut(uint32_t w1, uint32_t w2)
{
  tlut_address = ti_address;
}

static void rdp_load_tile(uint32_t w1, uint32_t w2)
{
  uint8_t tilenum = (w2 >> 24) & 7;
  tile[tilenum].sl = (w1 >> 12) & 0xFFF;
  tile[tilenum].tl =  w1 & 0xFFF;
  tile[tilenum].sh = (w2 >> 12) & 0xFFF;
  tile[tilenum].th =  w2 & 0xFFF;
}

static void rdp_set_tile(uint32_t w1, uint32_t w2)
{
  int tilenum = (w2 >> 24) & 7;

  tile[tilenum].format   = (w1 >> 21) & 7;
  tile[tilenum].size     = (w1 >> 19) & 3;
  tile[tilenum].line     = (w1 >>  9) & 0x1FF;
  tile[tilenum].tmem     =  w1 & 0x1FF;
  tile[tilenum].palette  = (w2 >> 20) & 0xF;
  tile[tilenum].clamp_t  = (w2 >> 19) & 1;
  tile[tilenum].mirror_t = (w2 >> 18) & 1;
  tile[tilenum].mask_t   = (w2 >> 14) & 0xF;
  tile[tilenum].shift_t  = (w2 >> 10) & 0xF;
  tile[tilenum].clamp_s  = (w2 >>  9) & 1;
  tile[tilenum].mirror_s = (w2 >>  8) & 1;
  tile[tilenum].mask_s   = (w2 >>  4) & 0xF;
  tile[tilenum].shift_s  =  w2 & 0xF;
}

static void rdp_fill_rect(uint32_t w1, uint32_t w2)
{
  float width = fb_width;           // Framebuffer Width In Pixels
  float height = ((width / 4) * 3); // Framebuffer Height In Pixels

  if (fb_size == PIXEL_SIZE_16BIT) { // Frame Buffer Size Of Pixels = 16 bits
    R = ((fill_color >> 11) & 0x1F) << 3;
    G = ((fill_color >>  6) & 0x1F) << 3;
    B = ((fill_color >>  1) & 0x1F) << 3;
  }
  if (fb_size == PIXEL_SIZE_32BIT) { // Frame Buffer Size Of Pixels = 32 bits
    R = fill_color >> 24;
    G = fill_color >> 16;
    B = fill_color >>  8;
  }

  uint8_t col[12] = { // 3 Byte RGB Colour * 4 Quad Points
    R, G, B,
    R, G, B,
    R, G, B,
    R, G, B
  };

  float xli = (w1 >> 14) & 0x3FF; // Load XL Integer Portion
  float xlf = (w1 >> 12) & 3;     // Load XL Fraction Portion
  float xl = xli + (xlf * 0.25);  // Load XL Float

  float yli = (w1 >>  2) & 0x3FF; // Load YL Integer Portion
  float ylf =  w1 & 3;            // Load YL Fraction Portion
  float yl = yli + (ylf * 0.25);  // Load YL Float

  float xhi = (w2 >> 14) & 0x3FF; // Load XH Integer Portion
  float xhf = (w2 >> 12) & 3;     // Load XH Fraction Portion
  float xh = xhi + (xhf * 0.25);  // Load XH Float

  float yhi = (w2 >>  2) & 0x3FF; // Load YH Integer Portion
  float yhf =  w2 & 3;            // Load YH Fraction Portion
  float yh = yhi + (yhf * 0.25);  // Load YH Float

  v1x = -1.0 + ((xl /  width) * 2.0); // Load Vertex 1 X
  v1y =  1.0 - ((yl / height) * 2.0); // Load Vertex 1 Y
  v2x = -1.0 + ((xh /  width) * 2.0); // Load Vertex 2 X
  v2y =  1.0 - ((yh / height) * 2.0); // Load Vertex 2 Y

  float quad[8] = { // Quad Vertex Space (X,Y * 4 Quad Points)
    v1x, v1y,
    v2x, v1y,
    v2x, v2y,
    v1x, v2y
  };

  glColorPointer(3, GL_UNSIGNED_BYTE, 0, col); // Set Color Array Pointer
  glVertexPointer(2, GL_FLOAT, 0, quad);       // Set Vertex Array Pointer
  glDrawArrays(GL_QUADS, 0, 4); // Output Quad
}

static void rdp_set_fill_color(uint32_t w1, uint32_t w2)
{
  fill_color = w2;
}

static void rdp_set_fog_color(uint32_t w1, uint32_t w2)
{
  fog_color.r = (w2 >> 24) & 0xff;
  fog_color.g = (w2 >> 16) & 0xff;
  fog_color.b = (w2 >>  8) & 0xff;
  fog_color.a = (w2 >>  0) & 0xff;
}

static void rdp_set_blend_color(uint32_t w1, uint32_t w2)
{
  blend_color.r = (w2 >> 24) & 0xff;
  blend_color.g = (w2 >> 16) & 0xff;
  blend_color.b = (w2 >>  8) & 0xff;
  blend_color.a = (w2 >>  0) & 0xff;
}

static void rdp_set_prim_color(uint32_t w1, uint32_t w2)
{
  min_level = (w1 >> 8) & 0x1f;
  primitive_lod_frac = w1 & 0xff;
  prim_color.r = (w2 >> 24) & 0xff;
  prim_color.g = (w2 >> 16) & 0xff;
  prim_color.b = (w2 >>  8) & 0xff;
  prim_color.a = (w2 >>  0) & 0xff;
}

static void rdp_set_env_color(uint32_t w1, uint32_t w2)
{
  env_color.r = (w2 >> 24) & 0xff;
  env_color.g = (w2 >> 16) & 0xff;
  env_color.b = (w2 >>  8) & 0xff;
  env_color.a = (w2 >>  0) & 0xff;
}

static void rdp_set_combine(uint32_t w1, uint32_t w2)
{
  combine.sub_a_rgb0  = (w1 >> 20) & 0xf;
  combine.mul_rgb0  = (w1 >> 15) & 0x1f;
  combine.sub_a_a0  = (w1 >> 12) & 0x7;
  combine.mul_a0    = (w1 >>  9) & 0x7;
  combine.sub_a_rgb1  = (w1 >>  5) & 0xf;
  combine.mul_rgb1  = (w1 >>  0) & 0x1f;

  combine.sub_b_rgb0  = (w2 >> 28) & 0xf;
  combine.sub_b_rgb1  = (w2 >> 24) & 0xf;
  combine.sub_a_a1  = (w2 >> 21) & 0x7;
  combine.mul_a1    = (w2 >> 18) & 0x7;
  combine.add_rgb0  = (w2 >> 15) & 0x7;
  combine.sub_b_a0  = (w2 >> 12) & 0x7;
  combine.add_a0    = (w2 >>  9) & 0x7;
  combine.add_rgb1  = (w2 >>  6) & 0x7;
  combine.sub_b_a1  = (w2 >>  3) & 0x7;
  combine.add_a1    = (w2 >>  0) & 0x7;

  other_modes.f.stalederivs = 1;
}

static void rdp_set_texture_image(uint32_t w1, uint32_t w2)
{
  ti_format  = (w1 >> 21) & 7;
  ti_size    = (w1 >> 19) & 3;
  ti_width   = (w1 & 0x3FF) + 1;
  ti_address = w2 & 0xFFFFFF;
}

static void rdp_set_mask_image(uint32_t w1, uint32_t w2)
{
  zb_address = w2 & 0xFFFFFF;
}

static void rdp_set_color_image(uint32_t w1, uint32_t w2)
{
  fb_format  = (w1 >> 21) & 7;
  fb_size    = (w1 >> 19) & 3;
  fb_width   = w1 & 0x3FF;
  fb_address = w2 & 0xFFFFFF;
}

static void (*const rdp_command_table[64])(uint32_t w1, uint32_t w2) = {
  rdp_noop,          rdp_invalid,           rdp_invalid,        rdp_invalid,
  rdp_invalid,       rdp_invalid,           rdp_invalid,        rdp_invalid,
  rdp_tri_noshade,   rdp_tri_noshade_z,     rdp_tri_tex,        rdp_tri_tex_z,
  rdp_tri_shade,     rdp_tri_shade_z,       rdp_tri_texshade,   rdp_tri_texshade_z,
  
  rdp_invalid,       rdp_invalid,           rdp_invalid,        rdp_invalid,
  rdp_invalid,       rdp_invalid,           rdp_invalid,        rdp_invalid,
  rdp_invalid,       rdp_invalid,           rdp_invalid,        rdp_invalid,
  rdp_invalid,       rdp_invalid,           rdp_invalid,        rdp_invalid,
  
  rdp_invalid,       rdp_invalid,           rdp_invalid,        rdp_invalid,
  rdp_tex_rect,      rdp_tex_rect_flip,     rdp_sync_load,      rdp_sync_pipe,
  rdp_sync_tile,     rdp_sync_full,         rdp_set_key_gb,     rdp_set_key_r,
  rdp_set_convert,   rdp_set_scissor,       rdp_set_prim_depth, rdp_set_other_modes,
  
  rdp_load_tlut,     rdp_invalid,           rdp_set_tile_size,  rdp_load_block,
  rdp_load_tile,     rdp_set_tile,          rdp_fill_rect,      rdp_set_fill_color,
  rdp_set_fog_color, rdp_set_blend_color,   rdp_set_prim_color, rdp_set_env_color,
  rdp_set_combine,   rdp_set_texture_image, rdp_set_mask_image, rdp_set_color_image
};

void rdp_process_list(void)
{
  int i, length;
  uint32_t cmd, cmd_length;
  uint32_t dp_current_al = dp_current & ~7, dp_end_al = dp_end & ~7; 

  dp_status &= ~DP_STATUS_FREEZE;
  
  
  
  

  

  if (dp_end_al <= dp_current_al)
  {
    
    
    
    
    
    
    return;
  }

  length = (dp_end_al - dp_current_al) >> 2;

  ptr_onstart = rdp_cmd_ptr;

  

  
  
  
  
  

  
  if ((rdp_cmd_ptr + length) & ~0xffff)
    fatalerror("rdp_process_list: rdp_cmd_ptr overflow: length 0x%x ptr_onstart 0x%x", length, ptr_onstart);

  
  dp_current_al >>= 2;

  if (dp_status & DP_STATUS_XBUS_DMA)
  {
    for (i = 0; i < length; i ++)
    {
      rdp_cmd_data[rdp_cmd_ptr] = byteswap_32(rsp_dmem[dp_current_al & 0x3ff]);
      rdp_cmd_ptr++;
      dp_current_al++;
    }
  }
  else
  {
    for (i = 0; i < length; i ++)
    {
      RREADIDX32(rdp_cmd_data[rdp_cmd_ptr], dp_current_al);
      rdp_cmd_ptr++;
      dp_current_al++;
    }
  }
  

  while (rdp_cmd_cur < rdp_cmd_ptr && !rdp_pipeline_crashed)
  {
    cmd = (rdp_cmd_data[rdp_cmd_cur] >> 24) & 0x3f;
    cmd_length = rdp_command_length[cmd] >> 2;

    
    
    if ((rdp_cmd_ptr - rdp_cmd_cur) < cmd_length)
    {
      
      dp_start = dp_current = dp_end;
      return;
    }
    
    if (LOG_RDP_EXECUTION)
    {
      char string[4000];
      if (1)
      {
      z64gl_command += cmd_length;
      
      
      rdp_dasm(string);
      fprintf(rdp_exec, "%08X: %08X %08X   %s\n", command_counter, rdp_cmd_data[rdp_cmd_cur+0], rdp_cmd_data[rdp_cmd_cur+1], string);
      }
      command_counter++;
    }

    
    
    

    
    rdp_command_table[cmd](rdp_cmd_data[rdp_cmd_cur+0], rdp_cmd_data[rdp_cmd_cur + 1]);
    
    rdp_cmd_cur += cmd_length;
  };
  rdp_cmd_ptr = 0;
  rdp_cmd_cur = 0;
  dp_start = dp_current = dp_end;
  
  
  
}

