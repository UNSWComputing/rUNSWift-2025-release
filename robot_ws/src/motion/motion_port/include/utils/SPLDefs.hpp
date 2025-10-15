#pragma once

/** The field coordinate system in mm and radians (rad)
 *  X -- is along the length of the field, +ve towards opponent's goal
 *  Y -- is along the width of the field, +ve towards the left hand side
 *  0 rad -- facing straight towards opponent's goal at origin
 *  radians are calculated counter clock-wise
 *  NOTE: we use -PI, not PI for 180 degrees
 *  NOTE: unless explicitly specified all dimensions includes line width
 */

//#define USING_SMALL_FIELD

// Will need to re-measure field when it is built, OR find a way to use Vision definitions / drop AbsCoord class
#if !defined(USING_SMALL_FIELD)
   #define FIELD_LENGTH 9010
   #define FIELD_WIDTH 6020

   #define FIELD_LENGTH_OFFSET 700
   #define FIELD_WIDTH_OFFSET 700

#else
   #define FIELD_LENGTH 4240
   #define FIELD_WIDTH 2350

   #define FIELD_LENGTH_OFFSET 130
   #define FIELD_WIDTH_OFFSET 130

#endif

/** Field dimensions including edge offsets */
#define FULL_FIELD_LENGTH (FIELD_LENGTH + (FIELD_LENGTH_OFFSET * 2))
#define FULL_FIELD_WIDTH (FIELD_WIDTH + (FIELD_WIDTH_OFFSET * 2))
