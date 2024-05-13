    /***************************************************************\
    *                                                               *
    *   CRC-16 library including 4-bit burst error correction.      *
    *   This version specific to the CCITT-16 CRC type.             *
    *                                                               *
    *   Thanks Lammert Bies for the original CRC library,           *
    *   and Nick Bonniere for the error-correction code             *
    *   (configured for a 29-byte packet)                           *
    *   based on research paper by Travis Mandel and Jens Mache.    *
    *   Dense table lookup algorithm by Moshe Braner, 2024          *
    *                                                               *
    \***************************************************************/

#if defined(ARDUINO)
#include <Arduino.h>
#endif /* ARDUINO */

#include <stdlib.h>  // - for qsort

/*
#include <stdio.h>

#define int32_t int
#define uint32_t unsigned int
#define int16_t short int
#define uint16_t unsigned short int
#define bool unsigned short int
#define byte unsigned char
#define uint8_t unsigned char
#define int64_t long long int
#define float64_t double
*/

#define FALSE      0
#define TRUE       1

#include "lib_crc_bec.h"

// just a stub, to enable OGNbase RF.cpp to compile using this library:
void update_crc8(unsigned char *crc, unsigned char m){}


    /*******************************************************************\
    *                                                                   *
    *   #define P_xxxx                                                  *
    *                                                                   *
    *   The CRC's are computed using polynomials. The  coefficients     *
    *   for the algorithms are defined by the following constants.      *
    *                                                                   *
    \*******************************************************************/

#define                 P_CCITT     0x1021

   /*******************************************************************\
    *                                                                   *
    *   static int crc_tab...init                                       *
    *   static unsigned ... crc_tab...[]                                *
    *                                                                   *
    *   The algorithms use tables with precalculated  values.  This     *
    *   speeds  up  the calculation dramaticaly. The first time the     *
    *   CRC function is called, the table for that specific  calcu-     *
    *   lation  is set up. The ...init variables are used to deter-     *
    *   mine if the initialization has taken place. The  calculated     *
    *   values are stored in the crc_tab... arrays.                     *
    *                                                                   *
    *   The variables are declared static. This makes them  invisi-     *
    *   ble for other modules of the program.                           *
    *                                                                   *
    \*******************************************************************/

static unsigned short   crc_tab_ccitt[256]; // static -> init to 0

//static unsigned short   bec_tab[65536];     // static -> init to 0
// - this large table is 97% zeros
// - instead use a smaller table with the zeros removed:

typedef struct dense_bec_entry_struct {
  uint16_t crc_value;
  uint16_t bit_index;
} dense_bec_entry_t;

dense_bec_entry_t dense_bec_table[1841];

int cmp_dense_bec_entries(const void *a, const void *b)   // for qsort
{
  dense_bec_entry_t *aa = (dense_bec_entry_t *) a;
  dense_bec_entry_t *bb = (dense_bec_entry_t *) b;
  if ( aa->crc_value < bb->crc_value ) return -1;
  if ( aa->crc_value > bb->crc_value ) return 1;
  /* if == */ return 0;
}

void prepare_dense()
{
    dense_bec_table[0].crc_value = 0;
    dense_bec_table[0].bit_index = 0;
    qsort ((void *) &dense_bec_table[1], 1839, sizeof(dense_bec_entry_t), cmp_dense_bec_entries);
    dense_bec_table[1840].crc_value = 0xFFFF;
    dense_bec_table[1840].bit_index = 0;
    //Serial.print("BEC table [   1]: "); Serial.println(dense_bec_table[1].crc_value);
    //Serial.print("BEC table [1839]: "); Serial.println(dense_bec_table[1839].crc_value);
}


// Function to look up a value in the dense table.
// Does a recursive search, aided by the approximately
// uniform distribution of the CRC values through the table.
// This lookup never uses more than 10 iterations of the while(),
// and on the average about 3 if the given CRC value is found,
// and about 4 iterations if not found.
uint16_t lookup_dense(uint16_t crc)
{
    int bottom = 1;
    int top = 1839;
    uint32_t low_crc, high_crc;
    while (1) {
        low_crc = dense_bec_table[bottom].crc_value;
        if (low_crc == crc)
            return dense_bec_table[bottom].bit_index;     // found
        if (crc < low_crc)
            return 0;                     // not in table
        if (top <= bottom)
            return 0;                     // not in table
        high_crc = dense_bec_table[top].crc_value;
        if (high_crc == crc)
            return dense_bec_table[top].bit_index;        // found
        if (crc > high_crc)
            return 0;                     // not in table
        // make a calculated guess as to where the entry is
        int32_t low_guess = bottom + (((crc-low_crc)*1839) >> 16) - 1;
        int32_t high_guess = low_guess + 2;
        if (high_guess >= top) {
            high_guess = top - 1;
            if (low_guess > high_guess)
                low_guess = high_guess;
        }
        if (low_guess <= bottom) {
            low_guess = bottom + 1;
            if (high_guess < low_guess)
                high_guess = low_guess;
        }
        uint32_t low_crc = dense_bec_table[low_guess].crc_value;
        if (low_crc == crc)
            return dense_bec_table[low_guess].bit_index;     // found
        uint32_t high_crc = dense_bec_table[high_guess].crc_value;
        if (high_crc == crc)
            return dense_bec_table[high_guess].bit_index;     // found
        if (high_crc < crc) {          // can only be higher in the table
            bottom = high_guess+1;
            int i = (top+high_guess)>>1;   // bisection
            if (dense_bec_table[i].crc_value > crc)
                top = i;
        } else if (low_crc < crc) {    // && high_crc > crc - straddle
            bottom = low_guess+1;
            top = high_guess-1;
        } else {                       // can only be lower in the table
            top = low_guess-1;
            int i = (bottom+low_guess)>>1;   // bisection
            if (dense_bec_table[i].crc_value < crc)
                bottom = i;
        }
    }
    return 0;     // not reachable
}


    /*******************************************************************\
    *                                                                   *
    * unsigned short update_crc_ccitt_16( unsigned long crc, char c );  *
    *                                                                   *
    *   The function update_crc_ccitt_16 calculates a new CRC-CCITT     *
    *   value  based  on the previous value of the CRC and the next     *
    *   byte of the data to be checked.                                 *
    *                                                                   *
    \*******************************************************************/

unsigned short update_crc_ccitt_16( unsigned short crc, unsigned char c ) {

    unsigned short tmp, short_c;

    short_c  = 0x00ff & (unsigned short) c;

    tmp = (crc >> 8) ^ short_c;
    crc = (crc << 8) ^ crc_tab_ccitt[tmp];

    return crc;

}  /* update_crc_ccitt_16 */

    /*******************************************************************\
    *                                                                   *
    *   void init_crc_ccitt_tab( void );                          *
    *                                                                   *
    *   The function init_crc_ccitt_tab() is used to fill the  array     *
    *   for calculation of the CRC-CCITT with values.                   *
    *                                                                   *
    \*******************************************************************/

void init_crc_ccitt_tab( void ) {

    int i, j;
    unsigned short crc, c;

    for (i=0; i<256; i++) {

        crc = 0;
        c   = ((unsigned short) i) << 8;

        for (j=0; j<8; j++) {

            if ( (crc ^ c) & 0x8000 ) crc = ( crc << 1 ) ^ P_CCITT;
            else                      crc =   crc << 1;

            c = c << 1;
        }

        crc_tab_ccitt[i] = crc;
    }

}  /* init_crc_ccitt_tab */

/*----------------------------------------------------------------------------
// With the CRC-CCITT polynomial, it is possible to correct up to 4 bits in error in a burst
// for packets up to 1500 bits
 ----------------------------------------------------------------------------*/

#define  NumPatterns  8

  //create an error pattern table that describes all patterns from 1 to 4 bit error bursts
  const unsigned short ErrorPattern[NumPatterns] = {
    0b1000000000000000,                                          //1 bit error in 4 bits
    0b1100000000000000, 0b1010000000000000, 0b1001000000000000,  //2 bit errors in 4 bits
    0b1110000000000000, 0b1101000000000000, 0b1011000000000000,  //3 bit errors in 4 bits
    0b1111000000000000                                           //4 bit errors in 4 bits
  };

  const unsigned short NumberOfBits[NumPatterns] = {
    1,
    2, 2, 2,
    3, 3, 3,
    4
  };

    /*******************************************************************\
    *                                                                   *
    *   void init_crc_bec_tab( void );                                  *
    *                                                                   *
    *   The function init_crc_bec_tab() is used to fill the array       *
    *   of the CRC bit-error-correction table with values.              *
    *                                                                   *
    \*******************************************************************/

void init_crc_bec_tab( unsigned short PacketSize ) {

  short Pattern, ShiftedPattern;
  short PktByte_I, PktByte_J;
  unsigned short CRC_0, P;
  int bec_entries = 0;

  for (Pattern = 0;  Pattern < NumPatterns; Pattern++) {            // do all patterns
    for (PktByte_I = 0; PktByte_I < PacketSize; PktByte_I++) {      // do pattern shifted through all packet bytes
      P = ErrorPattern[Pattern];
//      for (ShiftedPattern = 0; ShiftedPattern < NumPatterns; ShiftedPattern++) {
      for (ShiftedPattern = 0; ShiftedPattern < 8; ShiftedPattern++) {  // shift one whole byte
        CRC_0 = 0;
        PktByte_J = 0;
        if (! ((PktByte_I == PacketSize-1) && ((P & 0xff) != 0))) {     // skip if pattern overflows last byte
          while (PktByte_J <= PacketSize-1) {
            if (PktByte_J == PktByte_I) {
              CRC_0 = update_crc_ccitt_16(CRC_0, P >> 8);
              if (! (PktByte_J == PacketSize-1)) {                  // skip if end of packet
                PktByte_J++;
                CRC_0 = update_crc_ccitt_16(CRC_0,P & 0xff);
              }
            } else {
              CRC_0 = update_crc_ccitt_16(CRC_0,0);
            }
            PktByte_J++;
          }
          // index table contains error position indexes
          //bec_tab[CRC_0] = (Pattern << 8) + (PktByte_I << 3) + ShiftedPattern + 1;
          if (CRC_0 != 0) {
            ++bec_entries;  // start at dense_bec_table[1]
            if (bec_entries <= 1839) {
              dense_bec_table[bec_entries].crc_value = CRC_0;
              dense_bec_table[bec_entries].bit_index = (Pattern << 8) + (PktByte_I << 3) + ShiftedPattern + 1;
            }
          }
          P >>= 1;
        }
      }
    }
  }
  //Serial.print("BEC table entries: "); Serial.println(bec_entries);
}  /* init_crc_bec_tab */


    /*******************************************************************\
    *                                                                   *
    * unsigned short bec_correct_errors(unsigned short crc, char *pkt); *
    *                                                                   *
    *   The function bec_correct_errors() is used to correct            *
    *   CRC bit error burst up to 4 bits long.                          *
    *                                                                   *
    \*******************************************************************/
unsigned short bec_correct_errors( unsigned short crc, unsigned char *pkt ) {

//function CRC_CorrectErrors(CRC : word; Data: Pointer) : boolean;

  short Pattern, ShiftedPattern;
  short PktByte_I;
  unsigned short P;
  short ErrorIndex;

  //uses CRC as index to table
  //ErrorIndex = bec_tab[crc];
  ErrorIndex = lookup_dense(crc);
  if (ErrorIndex == 0) {
    return 0;     // no correction possible
  } else {
    ErrorIndex--;     // remove error flag offset
    // get pattern index, shift offset and byte position
    ShiftedPattern = ErrorIndex & ((1<<3) -1);    // bits 0 to 2
    PktByte_I = (ErrorIndex  & ((1<<8)-1)) >> 3;  // bits 3 to 7
    // changed packet to have data only, so must offset by 3 for sync
    PktByte_I = PktByte_I - 3;
    if (PktByte_I < 0) { // should never happen as SYNC doesn't have errors
      return 0;   // no correction possible
    } else {
      Pattern = ErrorIndex >> 8;            // bits 8 to 15
      P = ErrorPattern[Pattern] >> ShiftedPattern;
      // do correction
      pkt[PktByte_I] = pkt[PktByte_I] ^ (P >> 8);
      if ((P & 0xff) != 0) {
        pkt[PktByte_I+1] = pkt[PktByte_I+1] ^ (P & 0xff);
      }
    }
    return NumberOfBits[Pattern];   // correction is possible
  }
}  /* bec_correct_errors */



void crc_setup( void )
{
    init_crc_ccitt_tab();
    init_crc_bec_tab(29);
    prepare_dense();
}

