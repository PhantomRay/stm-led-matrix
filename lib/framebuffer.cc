// -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
// Copyright (C) 2013 Henner Zeller <h.zeller@acm.org>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation version 2.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://gnu.org/licenses/gpl-2.0.txt>

// The framebuffer is the workhorse: it represents the frame in some internal
// format that is friendly to be dumped to the matrix quickly. Provides methods
// to manipulate the content.

#include "framebuffer-internal.h"

#include <assert.h>
#include <ctype.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <algorithm>
#include <sys/mman.h>
#include <time.h>

#include "gpio.h"

namespace rgb_matrix {
namespace internal {
enum
{
  KBITPlanes = 16 // maximum usable bitplanes.
};

// We need one global instance of a timing correct pulser. There are different
// implementations depending on the context.
static PinPulser *sOutputEnablePulser = NULL;

#ifdef ONLY_SINGLE_SUB_PANEL
#define SUB_PANELS_ 1
#else
#define SUB_PANELS_ 2
#endif

PixelDesignator *PixelDesignatorMap::get(int x, int y) {
  if (x < 0 || y < 0 || x >= width_ || y >= height_)
    return NULL;
  return buffer_ + (y*width_) + x;
}

PixelDesignatorMap::PixelDesignatorMap(int width, int height,
                                       const PixelDesignator &fill_bits)
  : width_(width), height_(height), fill_bits_(fill_bits),
    buffer_(new PixelDesignator[width * height]) {
}

PixelDesignatorMap::~PixelDesignatorMap() {
  delete [] buffer_;
}

// Different panel types use different techniques to set the row address.
// We abstract that away with different implementations of RowAddressSetter
class RowAddressSetter {
public:
  virtual ~RowAddressSetter() {}
  virtual gpio_bits_t need_bits() const = 0;
  virtual void SetRowAddress(GPIO *io, int row) = 0;
};

namespace {

// The default DirectRowAddressSetter just sets the address in parallel
// output lines ABCDE with A the LSB and E the MSB.
class DirectRowAddressSetter : public RowAddressSetter {
public:
  DirectRowAddressSetter(int double_rows, const HardwareMapping &h)
    : row_mask_(0), last_row_(-1) {
    assert(double_rows <= 32);  // need to resize row_lookup_
    if (double_rows > 16) row_mask_ |= h.e;
    if (double_rows > 8)  row_mask_ |= h.d;
    if (double_rows > 4)  row_mask_ |= h.c;
    if (double_rows > 2)  row_mask_ |= h.b;
    row_mask_ |= h.a;
    for (int i = 0; i < double_rows; ++i) {
      // To avoid the bit-fiddle in the critical path, utilize
      // a lookup-table for all possible rows.
      gpio_bits_t row_address = (i & 0x01) ? h.a : 0;
      row_address |= (i & 0x02) ? h.b : 0;
      row_address |= (i & 0x04) ? h.c : 0;
      row_address |= (i & 0x08) ? h.d : 0;
      row_address |= (i & 0x10) ? h.e : 0;
      row_lookup_[i] = row_address;
    }
  }

  virtual gpio_bits_t need_bits() const { return row_mask_; }

  virtual void SetRowAddress(GPIO *io, int row) {
    if (row == last_row_) return;
    io->WriteMaskedBits(row_lookup_[row], row_mask_);
    last_row_ = row;
  }

private:
  gpio_bits_t row_mask_;
  gpio_bits_t row_lookup_[32];
  int last_row_;
};

// The SM5266RowAddressSetter (ABC Shifter + DE direct) sets bits ABC using
// a 8 bit shifter and DE directly. The panel this works with has 8 SM5266
// shifters (4 for the top 32 rows and 4 for the bottom 32 rows).
// DE is used to select the active shifter
// (rows 1-8/33-40, 9-16/41-48, 17-24/49-56, 25-32/57-64).
// Rows are enabled by shifting in 8 bits (high bit first) with a high bit
// enabling that row. This allows up to 8 rows per group to be active at the
// same time (if they have the same content), but that isn't implemented here.
// BK, DIN and DCK are the designations on the SM5266P datasheet.
// BK = Enable Input, DIN = Serial In, DCK = Clock
class SM5266RowAddressSetter : public RowAddressSetter {
public:
  SM5266RowAddressSetter(int double_rows, const HardwareMapping &h)
    : row_mask_(h.a | h.b | h.c),
      last_row_(-1),
      bk_(h.c),
      din_(h.b),
      dck_(h.a) {
    assert(double_rows <= 32); // designed for up to 1/32 panel
    if (double_rows > 8)  row_mask_ |= h.d;
    if (double_rows > 16) row_mask_ |= h.e;
    for (int i = 0; i < double_rows; ++i) {
      gpio_bits_t row_address = 0;
      row_address |= (i & 0x08) ? h.d : 0;
      row_address |= (i & 0x10) ? h.e : 0;
      row_lookup_[i] = row_address;
    }
  }

  virtual gpio_bits_t need_bits() const { return row_mask_; }

  virtual void SetRowAddress(GPIO *io, int row) {
    if (row == last_row_) return;
    io->SetBits(bk_);  // Enable serial input for the shifter
    for (int r = 7; r >= 0; r--) {
      if (row % 8 == r) {
        io->SetBits(din_);
      } else {
        io->ClearBits(din_);
      }
      io->SetBits(dck_);
      io->SetBits(dck_);  // Longer clock time; tested with Pi3
      io->ClearBits(dck_);
    }
    io->ClearBits(bk_);  // Disable serial input to keep unwanted bits out of the shifters
    last_row_ = row;
    // Set bits D and E to enable the proper shifter to display the selected
    // row.
    io->WriteMaskedBits(row_lookup_[row], row_mask_);
  }

private:
  gpio_bits_t row_mask_;
  int last_row_;
  const gpio_bits_t bk_;
  const gpio_bits_t din_;
  const gpio_bits_t dck_;
  gpio_bits_t row_lookup_[32];
};
// This is mostly experimental at this point. It works with the one panel I have
// seen that does AB, but might need smallish tweaks to work with all panels
// that do this.
class ShiftRegisterRowAddressSetter : public RowAddressSetter {
public:
  ShiftRegisterRowAddressSetter(int double_rows, const HardwareMapping &h)
    : double_rows_(double_rows),
      row_mask_(h.a | h.b), clock_(h.a), data_(h.b),
      last_row_(-1) {
  }
  virtual gpio_bits_t need_bits() const { return row_mask_; }

  virtual void SetRowAddress(GPIO *io, int row) {
    if (row == last_row_) return;
    for (int activate = 0; activate < double_rows_; ++activate) {
      io->ClearBits(clock_);
      if (activate == double_rows_ - 1 - row) {
        io->ClearBits(data_);
      } else {
        io->SetBits(data_);
      }
      io->SetBits(clock_);
    }
    io->ClearBits(clock_);
    io->SetBits(clock_);
    last_row_ = row;
  }

private:
  const int double_rows_;
  const gpio_bits_t row_mask_;
  const gpio_bits_t clock_;
  const gpio_bits_t data_;
  int last_row_;
};

// Issue #823
// An shift register row address setter that does not use B but C for the
// data. Clock is inverted.
class ABCShiftRegisterRowAddressSetter : public RowAddressSetter {
public:
  ABCShiftRegisterRowAddressSetter(int double_rows, const HardwareMapping &h)
    : double_rows_(double_rows),
      row_mask_(h.a | h.c),
      clock_(h.a),
      data_(h.c),
      last_row_(-1) {
  }
  virtual gpio_bits_t need_bits() const { return row_mask_; }

  virtual void SetRowAddress(GPIO *io, int row) {
    for (int activate = 0; activate < double_rows_; ++activate) {
      io->ClearBits(clock_);
      if (activate == double_rows_ - 1 - row) {
        io->SetBits(data_);
      } else {
        io->ClearBits(data_);
      }
      io->SetBits(clock_);
    }
    io->SetBits(clock_);
    io->ClearBits(clock_);
    last_row_ = row;
  }

private:
  const int double_rows_;
  const gpio_bits_t row_mask_;
  const gpio_bits_t clock_;
  const gpio_bits_t data_;
  int last_row_;
};

// The DirectABCDRowAddressSetter sets the address by one of
// row pin ABCD for 32х16 matrix 1:4 multiplexing. The matrix has
// 4 addressable rows. Row is selected by a low level on the
// corresponding row address pin. Other row address pins must be in high level.
//
// Row addr| 0 | 1 | 2 | 3
// --------+---+---+---+---
// Line A  | 0 | 1 | 1 | 1
// Line B  | 1 | 0 | 1 | 1
// Line C  | 1 | 1 | 0 | 1
// Line D  | 1 | 1 | 1 | 0
class DirectABCDLineRowAddressSetter : public RowAddressSetter {
public:
  DirectABCDLineRowAddressSetter(int double_rows, const HardwareMapping &h)
    : last_row_(-1) {
	row_mask_ = h.a | h.b | h.c | h.d;

	row_lines_[0] = /*h.a |*/ h.b | h.c | h.d;
	row_lines_[1] = h.a /*| h.b*/ | h.c | h.d;
	row_lines_[2] = h.a | h.b /*| h.c */| h.d;
	row_lines_[3] = h.a | h.b | h.c /*| h.d*/;
  }

  virtual gpio_bits_t need_bits() const { return row_mask_; }

  virtual void SetRowAddress(GPIO *io, int row) {
    if (row == last_row_) return;

    gpio_bits_t row_address = row_lines_[row % 4];

    io->WriteMaskedBits(row_address, row_mask_);
    last_row_ = row;
  }

private:
  gpio_bits_t row_lines_[4];
  gpio_bits_t row_mask_;
  int last_row_;
};

} // namespace

const struct HardwareMapping *Framebuffer::hardware_mapping_ = NULL;
RowAddressSetter *Framebuffer::row_setter_ = NULL;

Framebuffer::Framebuffer(int rows, int columns, int parallel,
                         int scan_mode,
                         const char *led_sequence, bool inverse_color,
                         PixelDesignatorMap **mapper)
  : rows_(rows),
    parallel_(parallel),
    height_(rows * parallel),
    columns_(columns),
    scan_mode_(scan_mode),
    inverse_color_(inverse_color),
    pwm_bits_(16), do_luminance_correct_(true), brightness_(100),
    double_rows_(rows / SUB_PANELS_),
    buffer_size_(double_rows_ * columns_ * KBITPlanes * sizeof(gpio_bits_t)),
    shared_mapper_(mapper) {
  assert(hardware_mapping_ != NULL);   // Called InitHardwareMapping() ?
  assert(shared_mapper_ != NULL);  // Storage should be provided by RGBMatrix.
  assert(rows_ >=4 && rows_ <= 64 && rows_ % 2 == 0);
  if (parallel > hardware_mapping_->max_parallel_chains) {
    fprintf(stderr, "The %s GPIO mapping only supports %d parallel chain%s, "
            "but %d was requested.\n", hardware_mapping_->name,
            hardware_mapping_->max_parallel_chains,
            hardware_mapping_->max_parallel_chains > 1 ? "s" : "", parallel);
    abort();
  }
  assert(parallel >= 1 && parallel <= 3);

  bitplane_buffer_ = new gpio_bits_t[double_rows_ * columns_ * KBITPlanes];

  // If we're the first Framebuffer created, the shared PixelMapper is
  // still NULL, so create one.
  // The first PixelMapper represents the physical layout of a standard matrix
  // with the specific knowledge of the framebuffer, setting up PixelDesignators
  // in a way that they are useful for this Framebuffer.
  //
  // Newly created PixelMappers then can just re-arrange PixelDesignators
  // from the parent PixelMapper opaquely without having to know the details.
  if (*shared_mapper_ == NULL) {
    // Gather all the bits for given color for fast Fill()s and use the right
    // bits according to the led sequence
    const struct HardwareMapping &h = *hardware_mapping_;
    gpio_bits_t r = h.p0_r1 | h.p0_r2 | h.p1_r1 | h.p1_r2 | h.p2_r1 | h.p2_r2;
    gpio_bits_t g = h.p0_g1 | h.p0_g2 | h.p1_g1 | h.p1_g2 | h.p2_g1 | h.p2_g2;
    gpio_bits_t b = h.p0_b1 | h.p0_b2 | h.p1_b1 | h.p1_b2 | h.p2_b1 | h.p2_b2;
    PixelDesignator fill_bits;
    fill_bits.r_bit = GetGpioFromLedSequence('R', led_sequence, r, g, b);
    fill_bits.g_bit = GetGpioFromLedSequence('G', led_sequence, r, g, b);
    fill_bits.b_bit = GetGpioFromLedSequence('B', led_sequence, r, g, b);

    *shared_mapper_ = new PixelDesignatorMap(columns_, height_, fill_bits);
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < columns_; ++x) {
        InitDefaultDesignator(x, y, led_sequence, (*shared_mapper_)->get(x, y));
      }
    }
  }

  Clear();
}

Framebuffer::~Framebuffer() {
  delete [] bitplane_buffer_;
}

// TODO: this should also be parsed from some special formatted string, e.g.
// {addr={22,23,24,25,15},oe=18,clk=17,strobe=4, p0={11,27,7,8,9,10},...}
/* static */ void Framebuffer::InitHardwareMapping(const char *named_hardware) {
  if (named_hardware == NULL || *named_hardware == '\0') {
    named_hardware = "regular";
  }

  struct HardwareMapping *mapping = NULL;
  for (HardwareMapping *it = matrix_hardware_mappings; it->name; ++it) {
    if (strcasecmp(it->name, named_hardware) == 0) {
      mapping = it;
      break;
    }
  }

  if (!mapping) {
    fprintf(stderr, "There is no hardware mapping named '%s'.\nAvailable: ",
            named_hardware);
    for (HardwareMapping *it = matrix_hardware_mappings; it->name; ++it) {
      if (it != matrix_hardware_mappings) fprintf(stderr, ", ");
      fprintf(stderr, "'%s'", it->name);
    }
    fprintf(stderr, "\n");
    abort();
  }

  if (mapping->max_parallel_chains == 0) {
    // Auto determine.
    struct HardwareMapping *h = mapping;
    if ((h->p0_r1 | h->p0_g1 | h->p0_g1 | h->p0_r2 | h->p0_g2 | h->p0_g2) > 0)
      ++mapping->max_parallel_chains;
    if ((h->p1_r1 | h->p1_g1 | h->p1_g1 | h->p1_r2 | h->p1_g2 | h->p1_g2) > 0)
      ++mapping->max_parallel_chains;
    if ((h->p2_r1 | h->p2_g1 | h->p2_g1 | h->p2_r2 | h->p2_g2 | h->p2_g2) > 0)
      ++mapping->max_parallel_chains;
  }
  hardware_mapping_ = mapping;
}

/* static */ void Framebuffer::InitGPIO(GPIO *io, int rows, int parallel,
                                        bool allow_hardware_pulsing,
                                        int pwm_lsb_nanoseconds,
                                        int dither_bits,
                                        int row_address_type) {
  // if (sOutputEnablePulser != NULL)
  //   return; // already initialized.

  const struct HardwareMapping &h = *hardware_mapping_;
  // Tell GPIO about all bits we intend to use.
  gpio_bits_t all_used_bits = 0;

  all_used_bits |= h.output_enable | h.clock | h.strobe;

  all_used_bits |= h.p0_r1 | h.p0_g1 | h.p0_b1 | h.p0_r2 | h.p0_g2 | h.p0_b2;
  if (parallel >= 2) {
    all_used_bits |= h.p1_r1 | h.p1_g1 | h.p1_b1 | h.p1_r2 | h.p1_g2 | h.p1_b2;
  }
  if (parallel >= 3) {
    all_used_bits |= h.p2_r1 | h.p2_g1 | h.p2_b1 | h.p2_r2 | h.p2_g2 | h.p2_b2;
  }

  const int double_rows = rows / SUB_PANELS_;
  switch (row_address_type) {
  case 0:
    row_setter_ = new DirectRowAddressSetter(double_rows, h);
    break;
  case 1:
    row_setter_ = new ShiftRegisterRowAddressSetter(double_rows, h);
    break;
  case 2:
    row_setter_ = new DirectABCDLineRowAddressSetter(double_rows, h);
    break;
  default:
    assert(0); // unexpected type.
  }

  all_used_bits |= row_setter_->need_bits();

  // Adafruit HAT identified by the same prefix.
  const bool is_some_adafruit_hat = (0 == strncmp(h.name, "adafruit-hat",
                                                  strlen("adafruit-hat")));
  // Initialize outputs, make sure that all of these are supported bits.
  const gpio_bits_t result = io->InitOutputs(all_used_bits,
                                             is_some_adafruit_hat);
  assert(result == all_used_bits); // Impl: all bits declared in gpio.cc ?

  std::vector<int> bitplane_timings;
  uint32_t timing_ns = pwm_lsb_nanoseconds;
  for (int b = 0; b < KBITPlanes; ++b) {
    bitplane_timings.push_back(timing_ns);
    if (b >= dither_bits) timing_ns *= 2;
  }
  sOutputEnablePulser = PinPulser::Create(io, 0/*h.output_enable*/,
                                          false/*allow_hardware_pulsing*/,
                                          bitplane_timings);
}

// NOTE: first version for panel initialization sequence, need to refine
// until it is more clear how different panel types are initialized to be
// able to abstract this more.

static void InitFM6126(GPIO *io, const struct HardwareMapping &h, int columns)
{
  const uint32_t bits_on = h.p0_r1 | h.p0_g1 | h.p0_b1 | h.p0_r2 | h.p0_g2 | h.p0_b2 | h.p1_r1 | h.p1_g1 | h.p1_b1 | h.p1_r2 | h.p1_g2 | h.p1_b2 | h.p2_r1 | h.p2_g1 | h.p2_b1 | h.p2_r2 | h.p2_g2 | h.p2_b2 | h.a; // Address bit 'A' is always on.
  const uint32_t bits_off = h.a;
  const gpio_bits_t mask = bits_on | h.strobe;

  // Init bits. TODO: customize, as we can do things such as brightness here,
  // which would allow more higher quality output.
  static const char* init_b12 = "0111111111111111";  // full bright
  static const char *init_b13 = "0000000001000000"; // panel on.

  io->ClearBits(h.clock | h.strobe);

  for (int i = 0; i < columns; ++i) {
    gpio_bits_t value = init_b12[i % 16] == '0' ? bits_off : bits_on;
    if (i > columns - 12) value |= h.strobe;
    io->WriteMaskedBits(value, mask);
    io->SetBits(h.clock);
    io->ClearBits(h.clock);
  }
  io->ClearBits(h.strobe);

  for (int i = 0; i < columns; ++i) {
    gpio_bits_t value = init_b13[i % 16] == '0' ? bits_off : bits_on;
    if (i > columns - 13) value |= h.strobe;
    io->WriteMaskedBits(value, mask);
    io->SetBits(h.clock);
    io->ClearBits(h.clock);
  }
  io->ClearBits(h.strobe);
}

// The FM6217 is very similar to the FM6216.
// FM6217 adds Register 3 to allow for automatic bad pixel supression.
static void InitFM6127(GPIO *io, const struct HardwareMapping &h, int columns) {
  const gpio_bits_t bits_r_on= h.p0_r1 | h.p0_r2;
  const gpio_bits_t bits_g_on= h.p0_g1 | h.p0_g2;
  const gpio_bits_t bits_b_on= h.p0_b1 | h.p0_b2;
  const gpio_bits_t bits_on= bits_r_on | bits_g_on | bits_b_on;
  const gpio_bits_t bits_off = 0;

  const gpio_bits_t mask = bits_on | h.strobe;

  static const char* init_b12 = "1111111111001110";  // register 1
  static const char* init_b13 = "1110000001100010";  // register 2.
  static const char* init_b11 = "0101111100000000";  // register 3.
  io->ClearBits(h.clock | h.strobe);
  for (int i = 0; i < columns; ++i) {
    gpio_bits_t value = init_b12[i % 16] == '0' ? bits_off : bits_on;
    if (i > columns - 12) value |= h.strobe;
    io->WriteMaskedBits(value, mask);
    io->SetBits(h.clock);
    io->ClearBits(h.clock);
  }
  io->ClearBits(h.strobe);

  for (int i = 0; i < columns; ++i) {
    gpio_bits_t value = init_b13[i % 16] == '0' ? bits_off : bits_on;
    if (i > columns - 13) value |= h.strobe;
    io->WriteMaskedBits(value, mask);
    io->SetBits(h.clock);
    io->ClearBits(h.clock);
  }
  io->ClearBits(h.strobe);

  for (int i = 0; i < columns; ++i) {
    gpio_bits_t value = init_b11[i % 16] == '0' ? bits_off : bits_on;
    if (i > columns - 11) value |= h.strobe;
    io->WriteMaskedBits(value, mask);
    io->SetBits(h.clock);
    io->ClearBits(h.clock);
  }
  io->ClearBits(h.strobe);
}


/*static*/ void Framebuffer::InitializePanels(GPIO *io,
                                              const char *panel_type,
                                              int columns) {
  if (!panel_type || panel_type[0] == '\0') return;
  if (strncasecmp(panel_type, "fm6126", 6) == 0) {
    InitFM6126(io, *hardware_mapping_, columns);
  }
  else if (strncasecmp(panel_type, "fm6127", 6) == 0) {
    InitFM6127(io, *hardware_mapping_, columns);
  }
  // else if (strncasecmp(...))  // more init types
  else {
    fprintf(stderr, "Unknown panel type '%s'; typo ?\n", panel_type);
  }
}

bool Framebuffer::SetPWMBits(uint8_t value) {
  if (value < 1 || value > KBITPlanes)
    return false;
  pwm_bits_ = value;
  return true;
}

/*b[0..15] - b0 is MSB(2^15) and b15 is LSB(2^0).*/
inline gpio_bits_t *Framebuffer::ValueAt(int double_row, int column, int bit) {
  // return &bitplane_buffer_[ double_row * (columns_ * KBITPlanes)
  //                           + bit * columns_
  //                           + column ];

  uint8_t row_id, row_rem, col_id, col_rem;
  uint8_t panel_num = columns_ / 64; //the number of chain
  uint8_t panel_id = column / 64;
  column = column % 64;

  row_id = double_row % 8;
  row_rem = (double_row < 8)? 1:0;
  col_id = (row_rem * 4) + column / 16;
  col_rem = column % 16;
  return &bitplane_buffer_[ (row_id * 16 * 8 * panel_num + col_rem * 8 * panel_num +
                              panel_id * 8 + col_id)* KBITPlanes + bit];

}

void Framebuffer::Clear() {
  if (inverse_color_) {
    Fill(0, 0, 0);
  } else  {
    // Cheaper.
    memset(bitplane_buffer_, 0,
           sizeof(*bitplane_buffer_) * double_rows_ * columns_ * KBITPlanes);
  }
}

// Do CIE1931 luminance correction and scale to output bitplanes
static uint16_t luminance_cie1931(uint8_t c, uint8_t brightness)
{
  float out_factor = ((1 << KBITPlanes) - 1);
  float v = (float)c * brightness / 255.0;
  return out_factor * ((v <= 8) ? v / 902.3 : pow((v + 16) / 116.0, 3));
}

struct ColorLookup {
  uint16_t color[256];
};
static ColorLookup *CreateLuminanceCIE1931LookupTable() {
  ColorLookup *for_brightness = new ColorLookup[100];
  for (int c = 0; c < 256; ++c)
    for (int b = 0; b < 100; ++b)
      for_brightness[b].color[c] = luminance_cie1931(c, b + 1);

  return for_brightness;
}

static inline uint16_t CIEMapColor(uint8_t brightness, uint8_t c) {
  static ColorLookup *luminance_lookup = CreateLuminanceCIE1931LookupTable();
  return luminance_lookup[brightness - 1].color[c];
}

// Non luminance correction. TODO: consider getting rid of this.
static inline uint16_t DirectMapColor(uint8_t brightness, uint8_t c) {
  // simple scale down the color value
  c = c * brightness / 100;

  enum
  {
    shift = KBITPlanes - 8
  }; //constexpr; shift to be left aligned.
  return (shift > 0) ? (c << shift) : (c >> -shift);
}

// static inline uint16_t MyMapColor(uint8_t brightness, uint8_t c) {
//   uint16_t  Translate8To16Bit[256] = {0,46,92,139,186,233,280,327,375,422,470,519,567,615,664,713,762,812,861,911,961,1011,1061,1112,1163,1214,1265,1317,1368,1420,1473,1525,1578,1631,1684,1737,1791,1844,1899,1953,2007,2062,2117,2173,2228,2284,2340,2397,2453,2510,2568,2625,2683,2741,2799,2858,2917,2976,3036,3096,3156,3216,3277,3338,3399,3461,3523,3586,3648,3711,3775,3838,3902,3967,4032,4097,4162,4228,4294,4361,4428,4495,4563,4631,4699,4768,4838,4907,4978,5048,5119,5191,5262,5335,5407,5481,5554,5628,5703,5778,5853,5929,6006,6083,6160,6238,6317,6396,6476,6556,6636,6718,6799,6882,6965,7048,7132,7217,7302,7388,7475,7562,7650,7739,7828,7918,8008,8099,8191,8284,8377,8472,8567,8662,8759,8856,8954,9053,9153,9253,9355,9457,9560,9664,9769,9875,9982,10090,10199,10309,10420,10532,10645,10760,10875,10991,11109,11228,11348,11469,11591,11715,11840,11967,12094,12223,12354,12486,12620,12755,12891,13030,13169,13311,13454,13599,13746,13895,14045,14198,14352,14509,14667,14828,14991,15157,15324,15494,15667,15842,16020,16200,16383,16569,16758,16951,17146,17345,17547,17752,17961,18174,18391,18612,18837,19067,19301,19539,19783,20032,20286,20546,20812,21083,21361,21646,21938,22237,22544,22859,23183,23516,23859,24211,24575,24950,25338,25739,26153,26583,27029,27493,27975,28478,29003,29553,30130,30736,31375,32051,32767,33530,34345,35221,36167,37195,38322,39567,40959,42537,44359,46514,49151,52551,57343,65535};
//   return Translate8To16Bit[c];
// }

inline void Framebuffer::MapColors(
  uint8_t r, uint8_t g, uint8_t b,
  uint16_t *red, uint16_t *green, uint16_t *blue) {

  if (do_luminance_correct_) {
    *red   = CIEMapColor(brightness_, r);
    *green = CIEMapColor(brightness_, g);
    *blue  = CIEMapColor(brightness_, b);
  } else {
    *red   = DirectMapColor(brightness_, r);
    *green = DirectMapColor(brightness_, g);
    *blue  = DirectMapColor(brightness_, b);
  }
    // *red = MyMapColor(brightness_, r);
    // *green = MyMapColor(brightness_, g);
    // *blue = MyMapColor(brightness_, b);

  if (inverse_color_) {
    *red = ~(*red);
    *green = ~(*green);
    *blue = ~(*blue);
  }
}

void Framebuffer::Fill(uint8_t r, uint8_t g, uint8_t b) {
  uint16_t red, green, blue;
  MapColors(r, g, b, &red, &green, &blue);
  const PixelDesignator &fill = (*shared_mapper_)->GetFillColorBits();
  gpio_bits_t color_mask =  ~(fill.r_bit | fill.g_bit | fill.b_bit);

  for (int b = 0; b < KBITPlanes; ++b) {
    uint16_t mask = 0x8000 >> b;
    gpio_bits_t plane_bits = 0;
    plane_bits |= ((red & mask) == mask) ? fill.r_bit : 0;
    plane_bits |= ((green & mask) == mask) ? fill.g_bit : 0;
    plane_bits |= ((blue & mask) == mask) ? fill.b_bit : 0;

    for (int row = 0; row < double_rows_; ++row) {
      for (int col = 0; col < columns_; ++col) {
        gpio_bits_t *row_data = ValueAt(row, col, b);
        *row_data = (*row_data & color_mask) | plane_bits;
      }
    }
  }
}

int Framebuffer::width() const { return (*shared_mapper_)->width(); }
int Framebuffer::height() const { return (*shared_mapper_)->height(); }

void Framebuffer::SetPixel(int x, int y, uint8_t r, uint8_t g, uint8_t b) {
  const PixelDesignator *designator = (*shared_mapper_)->get(x, y);
  if (designator == NULL) return;
  const int pos = designator->gpio_word;
  if (pos < 0) return;  // non-used pixel marker.

  uint16_t red, green, blue;
  MapColors(r, g, b, &red, &green, &blue);
    
  // red = 0xffff;
  // green = 0;
  // blue = 0;

  uint32_t *bits = bitplane_buffer_ + pos;
  const int min_bit_plane = 0;//KBITPlanes - pwm_bits_;
  // bits += (columns_ * min_bit_plane);
  const uint32_t r_bits = designator->r_bit;
  const uint32_t g_bits = designator->g_bit;
  const uint32_t b_bits = designator->b_bit;
  const uint32_t designator_mask = designator->mask;
  for (uint16_t i = min_bit_plane; i < KBITPlanes; i++)
  {
    // uint16_t mask = 1 << i;
    uint16_t mask = 0x8000 >> i;
    // for (uint16_t mask = 1<<min_bit_plane; mask != 1<<KBITPlanes; mask <<=1 ) {
    uint32_t color_bits = 0;
    if (red & mask)   color_bits |= r_bits;
    if (green & mask) color_bits |= g_bits;
    if (blue & mask)  color_bits |= b_bits;
    *bits = (*bits & designator_mask) | color_bits;
    bits ++;//bits += columns_; //--21/08/2022 due to ValueAT update
  }
}

// Strange LED-mappings such as RBG or so are handled here.
gpio_bits_t Framebuffer::GetGpioFromLedSequence(char col,
                                                const char *led_sequence,
                                                gpio_bits_t default_r,
                                                gpio_bits_t default_g,
                                                gpio_bits_t default_b) {
  const char *pos = strchr(led_sequence, col);
  if (pos == NULL) pos = strchr(led_sequence, tolower(col));
  if (pos == NULL) {
    fprintf(stderr, "LED sequence '%s' does not contain any '%c'.\n",
            led_sequence, col);
    abort();
  }
  switch (pos - led_sequence) {
  case 0: return default_r;
  case 1: return default_g;
  case 2: return default_b;
  }
  return default_r; // String too long, should've been caught earlier.
}

void Framebuffer::InitDefaultDesignator(int x, int y, const char *seq,
                                        PixelDesignator *d) {
  const struct HardwareMapping &h = *hardware_mapping_;
  gpio_bits_t *bits = ValueAt(y % double_rows_, x, 0);
  d->gpio_word = bits - bitplane_buffer_;
  d->r_bit = d->g_bit = d->b_bit = 0;
  if (y < rows_) {
    if (y < double_rows_) {
      d->r_bit = GetGpioFromLedSequence('R', seq, h.p0_r1, h.p0_g1, h.p0_b1);
      d->g_bit = GetGpioFromLedSequence('G', seq, h.p0_r1, h.p0_g1, h.p0_b1);
      d->b_bit = GetGpioFromLedSequence('B', seq, h.p0_r1, h.p0_g1, h.p0_b1);
    } else {
      d->r_bit = GetGpioFromLedSequence('R', seq, h.p0_r2, h.p0_g2, h.p0_b2);
      d->g_bit = GetGpioFromLedSequence('G', seq, h.p0_r2, h.p0_g2, h.p0_b2);
      d->b_bit = GetGpioFromLedSequence('B', seq, h.p0_r2, h.p0_g2, h.p0_b2);
    }
  }
  else if (y >= rows_ && y < 2 * rows_) {
    if (y - rows_ < double_rows_) {
      d->r_bit = GetGpioFromLedSequence('R', seq, h.p1_r1, h.p1_g1, h.p1_b1);
      d->g_bit = GetGpioFromLedSequence('G', seq, h.p1_r1, h.p1_g1, h.p1_b1);
      d->b_bit = GetGpioFromLedSequence('B', seq, h.p1_r1, h.p1_g1, h.p1_b1);
    } else {
      d->r_bit = GetGpioFromLedSequence('R', seq, h.p1_r2, h.p1_g2, h.p1_b2);
      d->g_bit = GetGpioFromLedSequence('G', seq, h.p1_r2, h.p1_g2, h.p1_b2);
      d->b_bit = GetGpioFromLedSequence('B', seq, h.p1_r2, h.p1_g2, h.p1_b2);
    }
  }
  else  {
    if (y - 2 * rows_ < double_rows_) {
      d->r_bit = GetGpioFromLedSequence('R', seq, h.p2_r1, h.p2_g1, h.p2_b1);
      d->g_bit = GetGpioFromLedSequence('G', seq, h.p2_r1, h.p2_g1, h.p2_b1);
      d->b_bit = GetGpioFromLedSequence('B', seq, h.p2_r1, h.p2_g1, h.p2_b1);
    } else {
      d->r_bit = GetGpioFromLedSequence('R', seq, h.p2_r2, h.p2_g2, h.p2_b2);
      d->g_bit = GetGpioFromLedSequence('G', seq, h.p2_r2, h.p2_g2, h.p2_b2);
      d->b_bit = GetGpioFromLedSequence('B', seq, h.p2_r2, h.p2_g2, h.p2_b2);
    }
  }

  d->mask = ~(d->r_bit | d->g_bit | d->b_bit);
}

void Framebuffer::Serialize(const char **data, size_t *len) const {
  *data = reinterpret_cast<const char*>(bitplane_buffer_);
  *len = buffer_size_;
}

bool Framebuffer::Deserialize(const char *data, size_t len) {
  if (len != buffer_size_) return false;
  memcpy(bitplane_buffer_, data, len);
  return true;
}

void Framebuffer::CopyFrom(const Framebuffer *other) {
  if (other == this) return;
  memcpy(bitplane_buffer_, other->bitplane_buffer_, buffer_size_);
}

static void setScanLine(GPIO *io, const struct HardwareMapping &h, uint8_t* p_row)
{
  uint8_t row = *p_row;
  gpio_bits_t row_mask_ = h.a | h.b | h.c;
  gpio_bits_t row_address = (row & 0x01) ? h.a : 0;
              row_address |= (row & 0x02) ? h.b : 0;
              row_address |= (row & 0x04) ? h.c : 0;

  io->WriteMaskedBits(row_address, row_mask_);
  *p_row = (row + 1) & 0x07;
}
  
static void sendLAT(GPIO *io, const struct HardwareMapping &h, int clocks, int additionalClock = 0)
{
  additionalClock = 16 - clocks;
  for (int i = 0; i < additionalClock; i++)
  {
    io->SetBits(h.clock);
    io->ClearBits(h.clock);
  }
  io->SetBits(h.strobe);
  for (int i = 0; i < clocks; i++)
  {
    io->SetBits(h.clock);
    io->ClearBits(h.clock);
  }
  io->ClearBits(h.strobe);
}

#define REG_NUM 5
uint16_t reg_val[REG_NUM][3];
uint8_t cmd_latchs[REG_NUM] = {4,6,8,10,2};

static void sendConfData(GPIO *io, const struct HardwareMapping &h, uint16_t reg_dat[], uint8_t reg_id, uint8_t chip_num)
{  
  sendLAT(io, h, 14);           // Pre-active command
  
  gpio_bits_t config_dat[16];  
  for (int b = 0; b < 16; b++) {
    uint16_t data_mask = 0x8000 >> b;      
    uint32_t value = 0;
    if (reg_dat[0] & data_mask){
      value |= h.p0_r1 | h.p0_r2;
    }
    if (reg_dat[1] & data_mask){
      value |= h.p0_g1 | h.p0_g2;
    }
    if (reg_dat[2] & data_mask){
      value |= h.p0_b1 | h.p0_b2;
    }
    config_dat[b] = value;
  }

  const uint32_t mask = h.p0_r1 | h.p0_g1 | h.p0_b1 | h.p0_r2 | h.p0_g2 | h.p0_b2 | h.clock;
  for (uint8_t i = 0; i < chip_num; i++) { 
    for (uint8_t b = 0; b < 16; b++) {
      if((i == (chip_num-1)) && (b == (16 - cmd_latchs[reg_id]))){
        io->SetBits(h.strobe);
      }
      io->WriteMaskedBits(config_dat[b], mask); // color + reset clock
      io->SetBits(h.clock); // Rising edge: clock color in.
    }
    io->ClearBits(mask); //io->ClearBits(h.clock); 
    io->ClearBits(h.strobe);
  }
}
/* about 167us when CHIP_NUM = 8 */
static void InitICN2153(GPIO *io, const struct HardwareMapping &h, uint8_t chip_num)
{
  /*reg1*/
  reg_val[0][0] = (0x07 << 8) | (1 << 6) | (3 << 4)  | (0 << 3) | (0 << 0);
  reg_val[0][1] = reg_val[0][0];
  reg_val[0][2] = reg_val[0][0];
  /*reg2*/
  reg_val[1][0] = (31 << 10) | (0 << 9) | (64 << 1)  | (0 << 0);
  reg_val[1][1] = (reg_val[1][0] & 0x83FF) | (28 << 10);
  reg_val[1][2] = (reg_val[1][0] & 0x83FF) | (23 << 10);
  /*reg3*/
  reg_val[2][0] = (4 << 12) | (0 << 10) | (1 << 9)  | (0 << 8) | (0 << 4) | (0 << 3)  | (1 << 2) | (3 << 0);
  reg_val[2][1] = reg_val[2][0];
  reg_val[2][2] = reg_val[2][0];
  /*reg4*/
  reg_val[3][0] = 0x0040;
  reg_val[3][1] = reg_val[3][0];
  reg_val[3][2] = reg_val[3][0];
  /*reg5 or Debug*/
  reg_val[4][0] = 0x0008;         //Reg5:0x003C
  reg_val[4][1] = reg_val[4][0];
  reg_val[4][2] = reg_val[4][0];

  // io->ClearBits(h.clock | h.strobe);
  /*following code running time - 4.5us*/
  sendLAT(io, h, 14);     // Pre-active command
  sendLAT(io, h, 12);     // Enable all output channels
  
  for(int id = 0; id < REG_NUM; id++){
    /* 34us when CHIP_NUM = 8 */
    sendConfData(io, h, reg_val[id], id, chip_num);
  }
}

void Framebuffer::DumpToMatrix(GPIO *io, int pwm_low_bit)
{
  gpio_bits_t out;
const struct HardwareMapping &h = *hardware_mapping_;
  static bool flag = false;
  int chip_num = (columns_/64)*8;
  if(flag == false){
      flag = true;
      printf("New Start %d\n", h.p0_r1);
      // fprintf(stdout, "panel_type %s \n", panel_type_);
      // fprintf(stdout, "columns %d, double_rows_ %d\n", columns_, double_rows_);
      InitICN2153(io, *hardware_mapping_, chip_num);
      return;
  }

// /*test for point */
// for(int j=0;j<62;j++){
//   for(int i=0;i<24;i++){
//     SetPixel(4*i, j, 128,0,0);
//   }
// }
// for(int i=0;i<24;i++){
//   SetPixel(4*i, 63, 0,128,0);
// }
  sendLAT(io, h, 14);  // Pre-active command
  sendLAT(io, h, 12);
  // sendConfData(io, h, 0, 0, 0, 2); 
  sendLAT(io, h, 3);  // Vertical sync
  
  // struct timespec oe_sleep_time = { 0, 1000 };
  
  gpio_bits_t color_clk_mask = 0; // Mask of bits while clocking in.
  color_clk_mask = h.p0_r1 | h.p0_g1 | h.p0_b1 | h.p0_r2 | h.p0_g2 | h.p0_b2
                   | h.clock | h.strobe;
  gpio_bits_t row_mask_ = h.a | h.b | h.c | h.d;
  gpio_bits_t row_lookup_[32];
  for (int i = 0; i < double_rows_; ++i) {
    // To avoid the bit-fiddle in the critical path, utilize
    // a lookup-table for all possible rows.
    gpio_bits_t row_address = (i & 0x01) ? h.a : 0;
    row_address |= (i & 0x02) ? h.b : 0;
    row_address |= (i & 0x04) ? h.c : 0;
    row_address |= 0;//(i & 0x08) ? h.d : 0;
    row_address |= 0;//(i & 0x10) ? h.e : 0;
    row_lookup_[i] = row_address;
  }
  uint8_t scan_line = 0;
  int id;
  int refresh_cnt = 0;  
uint32_t start_time = GetMicrosecondCounter();  
  
    // gpio_bits_t *pixel_color = bitplane_buffer_;
  //   for (uint16_t pixel_cnt = 0; pixel_cnt < 16*8; pixel_cnt++) {
  //       for (id = 0; id < 4; id++) {
  //         io->SetBits(h.output_enable);
  //         io->ClearBits(h.output_enable);
  //       }
  //       //setScanLine(io, *hardware_mapping_, &scan_line);
  //       io->WriteMaskedBits(row_lookup_[scan_line], row_mask_);
  //       scan_line = (scan_line + 1) & 0x07;
  //       /*36us(128bit=16bitx8) when CHIP_NUM = 8*/
  //       //out = 0;//h_p0_r1
  //       for (int j = 0; j < chip_num; j++)
  //       {
  //         for (id = 0; id < (134/chip_num); id++) {
  //           io->SetBits(h.output_enable);   // nanosleep(&oe_sleep_time, NULL);
  //           io->ClearBits(h.output_enable); // nanosleep(&oe_sleep_time, NULL);
  //         }
  //         for (int b = 0; b < 16; b++)
  //         {
  //           io->WriteMaskedBits(*pixel_color, color_clk_mask); // col + reset clock
  //           pixel_color++;
  //           if((j == (chip_num - 1)) && (b == 15)){
  //             io->SetBits(h.strobe);
  //           }
  //           io->SetBits(h.clock); // Rising edge: clock color in.
  //         }
  //         io->ClearBits(color_clk_mask); // clock back to normal.
  //       }
  //       for (id = 0; id < (134%chip_num); id++) {
  //         io->SetBits(h.output_enable);
  //         io->ClearBits(h.output_enable);
  //       }
  //   }
  //   // refresh_cnt += 4; //=16*8/SCAN_NUM
  // // }
  // refresh_cnt = 16 * 8;
  // for (; refresh_cnt < 64*8; refresh_cnt++) {
  //   for (id = 0; id < 4; id++) {
  //     io->SetBits(h.output_enable);
  //     io->SetBits(h.output_enable);
  //     io->ClearBits(h.output_enable);
  //   }
  //   io->WriteMaskedBits(row_lookup_[scan_line], row_mask_);
  //   scan_line = (scan_line + 1) & 0x07;
  //   for (id = 0; id < 134; id++) {
  //     io->SetBits(h.output_enable);
  //     io->SetBits(h.output_enable);
  //     io->ClearBits(h.output_enable);
  //   }
  // }
// printf("passtime=%d\n", GetMicrosecondCounter() - start_time);

  uint16_t gclk_cnt = 0;
  uint32_t scan_cnt = 0;
  uint32_t chip_bits = chip_num * 16U;
  gpio_bits_t *pixel_color = bitplane_buffer_;
  for (uint16_t pixel_cnt = 0; pixel_cnt < 16 * 8; pixel_cnt++) {
    for (uint16_t double_pixel_cnt = 0; double_pixel_cnt < chip_bits; double_pixel_cnt++) {          
      io->SetBits(h.output_enable);
      gclk_cnt++;
      io->ClearBits(h.output_enable);
      if(gclk_cnt == 4){
        // setScanLine(io, *hardware_mapping_, &scan_line);
        // SetRowAddress(io, scan_line);
        io->WriteMaskedBits(row_lookup_[scan_line] | *pixel_color, row_mask_ | color_clk_mask);
        scan_line = (scan_line + 1) & 0x07;
      }
      else{
        if(gclk_cnt == 138){
          gclk_cnt = 0;
          scan_cnt++;
        }
        io->WriteMaskedBits(*pixel_color, color_clk_mask); // col + reset clock
      } 
      if(double_pixel_cnt == (chip_bits - 1)){
        io->SetBits(h.strobe);
      }
      pixel_color++;
      io->SetBits(h.clock); // Rising edge: clock color in.            
    }
    io->ClearBits(color_clk_mask);
  }
  // printf("%d\n", scan_cnt);
  //scan_cnt = chip_num*16U*128
  while (scan_cnt < 8 * 64) {
    io->SetBits(h.output_enable);
    gclk_cnt++;
    io->ClearBits(h.output_enable);
    if(gclk_cnt == 4){
      io->WriteMaskedBits(row_lookup_[scan_line], row_mask_);
      scan_line = (scan_line + 1) & 0x07;
    }
    else{
      if(gclk_cnt == 138){
        gclk_cnt = 0;
        scan_cnt++;
      }
    }
  }
}


} // namespace internal
} // namespace rgb_matrix
