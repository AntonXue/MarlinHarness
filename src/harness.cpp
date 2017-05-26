/*
 * Everything in here is dangerous.
 */
#include "harness.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

// Please don't break :(
#define _BV(PIN) (1 << PIN)
#define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })
#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })



unsigned long millis() {
    return (unsigned long) time(NULL) * 1000;  // Technically wrong.
}

#include "types.h"
#include "macros.h"
#include "enum.h"
#include "MarlinConfig.h"
#include "Conditionals_post.h"
#include "Marlin.h"

// Define prototypes and constants needed
#if ENABLED(DISTINCT_E_FACTORS) && E_STEPPERS > 1
  #define XYZE_N (XYZ + E_STEPPERS)
  #define E_AXIS_N (E_AXIS + extruder)
#else
  #undef DISTINCT_E_FACTORS
  #define XYZE_N XYZE
  #define E_AXIS_N E_AXIS
#endif

#if ENABLED(AUTO_BED_LEVELING_BILINEAR)
  #if ENABLED(DELTA)
    #define ADJUST_DELTA(V) \
      if (planner.abl_enabled) { \
        const float zadj = bilinear_z_offset(V); \
        delta[A_AXIS] += zadj; \
        delta[B_AXIS] += zadj; \
        delta[C_AXIS] += zadj; \
      }
  #else
    #define ADJUST_DELTA(V) if (planner.abl_enabled) { delta[Z_AXIS] += bilinear_z_offset(V); }
  #endif
#else
  #if IS_KINEMATIC
    #define ADJUST_DELTA(V) NOOP
  #else
    #define ADJUST_DELTA(V) NOOP // Hack I, Anton, added
  #endif
#endif

#define MMM_TO_MMS(MM_M) ((MM_M)/60.0)
#define MMS_SCALED(MM_S) ((MM_S)*feedrate_percentage*0.01)
#define sq(a) (a * a)

#if PLANNER_LEVELING
    #define ARG_X float lx
    #define ARG_Y float ly
    #define ARG_Z float lz
    /**
     * Apply leveling to transform a cartesian position
     * as it will be given to the planner and steppers.
     */
    static void apply_leveling(float &lx, float &ly, float &lz);
    static void apply_leveling(float logical[XYZ]) { apply_leveling(logical[X_AXIS], logical[Y_AXIS], logical[Z_AXIS]); }
    static void unapply_leveling(float logical[XYZ]);
#else
    #define ARG_X const float &lx
    #define ARG_Y const float &ly
    #define ARG_Z const float &lz
#endif

float workspace_offset[XYZ] = { 0 };
volatile long count_position[NUM_AXIS] = { 0 };

static float feedrate_mm_s = MMM_TO_MMS(1500.0);
int feedrate_percentage = 100, saved_feedrate_percentage,
    flow_percentage[EXTRUDERS] = ARRAY_BY_EXTRUDERS1(100);
uint8_t active_extruder = 0;

float current_position[XYZE] = { 0.0 };
float destination[XYZE] = { 0.0 };

float delta[ABC],
      endstop_adj[ABC] = { 0 };

// planner.steps_to_mm
float steps_to_mm[XYZE_N];

// These values are loaded or reset at boot time when setup() calls
// settings.load(), which calls recalc_delta_settings().
float delta_radius,
      delta_tower_angle_trim[2],
      delta_tower[ABC][2],
      delta_diagonal_rod,
      delta_calibration_radius,
      delta_diagonal_rod_2_tower[ABC],
      // delta_segments_per_second,
      delta_clip_start_height = Z_MAX_POS;
float delta_safe_distance_from_top();

#if ENABLED(DELTA)
    float delta_segments_per_second = Z_MAX_POS;
#else
  #if IS_SCARA
    float delta_segments_per_second = SCARA_SEGMENTS_PER_SECOND;
  #else
    float delta_segments_per_second = 0;
  #endif
#endif

void line_to_destination();
void line_to_destination(float);
bool prepare_kinematic_move_to(float ltarget[XYZE]);
bool prepare_move_to_destination_dualx();
bool prepare_move_to_cartesian();

float abs(float f) { return fabs(f); }

// Float constants for SCARA calculations
const float L1 = SCARA_LINKAGE_1, L2 = SCARA_LINKAGE_2,
            L1_2 = sq(float(L1)), L1_2_2 = 2.0 * L1_2,
            L2_2 = sq(float(L2));

#if ENABLED(DELTA)
  #define DELTA_Z(T) raw[Z_AXIS] + _SQRT(     \
    delta_diagonal_rod_2_tower[T] - HYPOT2(   \
        delta_tower[T][X_AXIS] - raw[X_AXIS], \
        delta_tower[T][Y_AXIS] - raw[Y_AXIS]  \
      )                                       \
    )

  #define DELTA_RAW_IK() do {        \
    delta[A_AXIS] = DELTA_Z(A_AXIS); \
    delta[B_AXIS] = DELTA_Z(B_AXIS); \
    delta[C_AXIS] = DELTA_Z(C_AXIS); \
  } while(0)

  #define DELTA_LOGICAL_IK() do {      \
    const float raw[XYZ] = {           \
      RAW_X_POSITION(logical[X_AXIS]), \
      RAW_Y_POSITION(logical[Y_AXIS]), \
      RAW_Z_POSITION(logical[Z_AXIS])  \
    };                                 \
    DELTA_RAW_IK();                    \
  } while(0)

  #define DELTA_DEBUG() do { \
      SERIAL_ECHOPAIR("cartesian X:", raw[X_AXIS]); \
      SERIAL_ECHOPAIR(" Y:", raw[Y_AXIS]);          \
      SERIAL_ECHOLNPAIR(" Z:", raw[Z_AXIS]);        \
      SERIAL_ECHOPAIR("delta A:", delta[A_AXIS]);   \
      SERIAL_ECHOPAIR(" B:", delta[B_AXIS]);        \
      SERIAL_ECHOLNPAIR(" C:", delta[C_AXIS]);      \
    } while(0)

  void inverse_kinematics(const float logical[XYZ]) {
    DELTA_LOGICAL_IK();
    // DELTA_DEBUG();
  }
#else
  #if ENABLED(MORGAN_SCARA)
    void inverse_kinematics(const float logical[XYZ]) {

      static float C2, S2, SK1, SK2, THETA, PSI;

      float sx = RAW_X_POSITION(logical[X_AXIS]) - SCARA_OFFSET_X,  // Translate SCARA to standard X Y
            sy = RAW_Y_POSITION(logical[Y_AXIS]) - SCARA_OFFSET_Y;  // With scaling factor.

      if (L1 == L2)
        C2 = HYPOT2(sx, sy) / L1_2_2 - 1;
      else
        C2 = (HYPOT2(sx, sy) - (L1_2 + L2_2)) / (2.0 * L1 * L2);

      S2 = sqrt(sq(C2) - 1);

      // Unrotated Arm1 plus rotated Arm2 gives the distance from Center to End
      SK1 = L1 + L2 * C2;

      // Rotated Arm2 gives the distance from Arm1 to Arm2
      SK2 = L2 * S2;

      // Angle of Arm1 is the difference between Center-to-End angle and the Center-to-Elbow
      THETA = atan2(SK1, SK2) - atan2(sx, sy);

      // Angle of Arm2
      PSI = atan2(S2, C2);

      delta[A_AXIS] = DEGREES(THETA);        // theta is support arm angle
      delta[B_AXIS] = DEGREES(THETA + PSI);  // equal to sub arm angle (inverted motor)
      delta[C_AXIS] = logical[Z_AXIS];

      printf("------------------------------------------------------\n");
      printf("SCARA_OFF_X: %d, SCARA_OFF_Y: %d\n",
             SCARA_OFFSET_X, SCARA_OFFSET_Y);
      printf("logical[XYZ] = [%f, %f, %f]\n",
             logical[X_AXIS], logical[Y_AXIS], logical[Z_AXIS]);
      printf("C2: %f, S2: %f, SK1: %f, SK2: %f, THETA: %f, PSI: %f, sx: %f, sy: %f\n",
              C2, S2, SK1, SK2, THETA, PSI, sx, sy);
      printf("L1: %f, L1_2: %f, L1_2_2: %f, L2: %f, L2_2: %f\n",
             L1, L1_2, L1_2_2, L2, L2_2);
              
      /*
        DEBUG_POS("SCARA IK", logical);
        DEBUG_POS("SCARA IK", delta);
        SERIAL_ECHOPAIR("  SCARA (x,y) ", sx);
        SERIAL_ECHOPAIR(",", sy);
        SERIAL_ECHOPAIR(" C2=", C2);
        SERIAL_ECHOPAIR(" S2=", S2);
        SERIAL_ECHOPAIR(" Theta=", THETA);
        SERIAL_ECHOLNPAIR(" Phi=", PHI);
      //*/
    }
  #else
    void inverse_kinematics(const float logical[XYZ]) {
      printf("inverse_kinematics called when it should not have been\n");
    }
  #endif
#endif

// Axis position functions
// stepper.position
long position(AxisEnum axis) {
  // CRITICAL_SECTION_START;
  const long count_pos = count_position[axis];
  // CRITICAL_SECTION_END;
  return count_pos;
}

// stepper.get_axis_position_mm
float get_axis_position_mm(AxisEnum axis) {
  float axis_steps;
  #if IS_CORE
    // Requesting one of the "core" axes?
    if (axis == CORE_AXIS_1 || axis == CORE_AXIS_2) {
      CRITICAL_SECTION_START;
      // ((a1+a2)+(a1-a2))/2 -> (a1+a2+a1-a2)/2 -> (a1+a1)/2 -> a1
      // ((a1+a2)-(a1-a2))/2 -> (a1+a2-a1+a2)/2 -> (a2+a2)/2 -> a2
      axis_steps = 0.5f * (
        axis == CORE_AXIS_2 ? CORESIGN(count_position[CORE_AXIS_1] - count_position[CORE_AXIS_2])
                            : count_position[CORE_AXIS_1] + count_position[CORE_AXIS_2]
      );
      CRITICAL_SECTION_END;
    }
    else
      axis_steps = position(axis);
  #else
    axis_steps = position(axis);
  #endif
  return axis_steps * steps_to_mm[axis];
}

// stepper.get_axis_position_degrees
float get_axis_position_degrees(AxisEnum axis) { return get_axis_position_mm(axis); }

// BUFFER LINE FUNCTIONS
static void _buffer_line(const float &a, const float &b, const float &c, const float &e, float fr_mm_s, const uint8_t extruder);

static FORCE_INLINE void buffer_line(ARG_X, ARG_Y, ARG_Z, const float &e, const float &fr_mm_s, const uint8_t extruder) {
  #if PLANNER_LEVELING && IS_CARTESIAN
    apply_leveling(lx, ly, lz);
  #endif
  _buffer_line(lx, ly, lz, e, fr_mm_s, extruder);
}

static FORCE_INLINE void buffer_line_kinematic(const float ltarget[XYZE], const float &fr_mm_s, const uint8_t extruder) {
  #if PLANNER_LEVELING
    float lpos[XYZ] = { ltarget[X_AXIS], ltarget[Y_AXIS], ltarget[Z_AXIS] };
    apply_leveling(lpos);
  #else
    const float * const lpos = ltarget;
  #endif
  #if IS_KINEMATIC
    inverse_kinematics(lpos);
    _buffer_line(delta[A_AXIS], delta[B_AXIS], delta[C_AXIS], ltarget[E_AXIS], fr_mm_s, extruder);
  #else
    _buffer_line(lpos[X_AXIS], lpos[Y_AXIS], lpos[Z_AXIS], ltarget[E_AXIS], fr_mm_s, extruder);
  #endif
}

// planner._buffer_line
void _buffer_line(const float &a, const float &b, const float &c, const float &e, float fr_mm_s, const uint8_t extruder) {
    char* p;
    #if IS_KINEMATIC
      #if ENABLED(DELTA)
        p = (char*)"DELTA";
      #endif
      #if IS_SCARA
        p = (char*)"SCARA";
      #endif
    #else
      p = (char*)"Regular";
    #endif
    printf("[%s] bufln(%f, %f, %f, %f, %f, %d)\n", p,a,b,c,e,fr_mm_s,extruder);
}

  // The target position of the tool in absolute steps

// LINE TO DESTINATION
inline void line_to_destination(float fr_mm_s) {
    // planner.buffer_line
    buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], fr_mm_s, active_extruder);
}

inline void line_to_destination() { line_to_destination(feedrate_mm_s); }


// PREPARE MOVE TO FUNCTIONS
bool prepare_kinematic_move_to(float ltarget[XYZE]) {
    // Get the top feedrate of the move in the XY plane
    float _feedrate_mm_s = MMS_SCALED(feedrate_mm_s);

    // If the move is only in Z/E don't split up the move
    if (ltarget[X_AXIS] == current_position[X_AXIS] && ltarget[Y_AXIS] == current_position[Y_AXIS]) {
      // planner.buffer_line
      printf("BUFFERING HERE FOR SOME REASON\n");
      buffer_line_kinematic(ltarget, _feedrate_mm_s, active_extruder);
      return false;
    }

    // Get the cartesian distances moved in XYZE
    float difference[XYZE];
    LOOP_XYZE(i) difference[i] = ltarget[i] - current_position[i];

    // Get the linear distance in XYZ
    float cartesian_mm = sqrt(sq(difference[X_AXIS]) + sq(difference[Y_AXIS]) + sq(difference[Z_AXIS]));

    // If the move is very short, check the E move distance
    if (UNEAR_ZERO(cartesian_mm)) cartesian_mm = abs(difference[E_AXIS]);

    // No E move either? Game over.
    if (UNEAR_ZERO(cartesian_mm)) return true;

    // Minimum number of seconds to move the given distance
    float seconds = cartesian_mm / _feedrate_mm_s;
    printf("Seconds: %f\n", seconds);

    // The number of segments-per-second times the duration
    // gives the number of segments
    uint16_t segments = delta_segments_per_second * seconds;
    printf("Segments: %d\n", segments);

    // For SCARA minimum segment size is 0.25mm
    #if IS_SCARA
      NOMORE(segments, cartesian_mm * 4);
    #endif

    // At least one segment is required
    NOLESS(segments, 1);

    // The approximate length of each segment
    const float inv_segments = 1.0 / float(segments),
                segment_distance[XYZE] = {
                  difference[X_AXIS] * inv_segments,
                  difference[Y_AXIS] * inv_segments,
                  difference[Z_AXIS] * inv_segments,
                  difference[E_AXIS] * inv_segments
                };

    // SERIAL_ECHOPAIR("mm=", cartesian_mm);
    // SERIAL_ECHOPAIR(" seconds=", seconds);
    // SERIAL_ECHOLNPAIR(" segments=", segments);

    #if IS_SCARA
      // SCARA needs to scale the feed rate from mm/s to degrees/s
      const float inv_segment_length = min(10.0, float(segments) / cartesian_mm), // 1/mm/segs
                  feed_factor = inv_segment_length * _feedrate_mm_s;
      float oldA = get_axis_position_degrees(A_AXIS);
      float oldB = get_axis_position_degrees(B_AXIS);
    #endif

    // Get the logical current position as starting point
    float logical[XYZE];
    COPY(logical, current_position);

    // Drop one segment so the last move is to the exact target.
    // If there's only 1 segment, loops will be skipped entirely.
    --segments;

    // Calculate and execute the segments
    for (uint16_t s = segments + 1; --s;) {
      LOOP_XYZE(i) logical[i] += segment_distance[i];
      #if ENABLED(DELTA)
        DELTA_LOGICAL_IK(); // Delta can inline its kinematics
      #else
        inverse_kinematics(logical);
      #endif

      ADJUST_DELTA(logical); // Adjust Z if bed leveling is enabled

      // printf("D: [%f, %f, %f]\n", delta[A_AXIS],delta[B_AXIS],delta[C_AXIS]);

      #if IS_SCARA
        // For SCARA scale the feed rate from mm/s to degrees/s
        // Use ratio between the length of the move and the larger angle change
        const float adiff = abs(delta[A_AXIS] - oldA),
                    bdiff = abs(delta[B_AXIS] - oldB);
        // planner.buffer_line
        buffer_line(delta[A_AXIS], delta[B_AXIS], delta[C_AXIS], logical[E_AXIS], max(adiff, bdiff) * feed_factor, active_extruder);
        oldA = delta[A_AXIS];
        oldB = delta[B_AXIS];
      #else
        // planner.buffer_line
        buffer_line(delta[A_AXIS], delta[B_AXIS], delta[C_AXIS], logical[E_AXIS], _feedrate_mm_s, active_extruder);
      #endif
    }

    // Since segment_distance is only approximate,
    // the final move must be to the exact destination.

    #if IS_SCARA
      // For SCARA scale the feed rate from mm/s to degrees/s
      // With segments > 1 length is 1 segment, otherwise total length
      inverse_kinematics(ltarget);
      ADJUST_DELTA(logical);
      const float adiff = abs(delta[A_AXIS] - oldA),
                  bdiff = abs(delta[B_AXIS] - oldB);
      // planner.buffer_line
      buffer_line(delta[A_AXIS], delta[B_AXIS], delta[C_AXIS], logical[E_AXIS], max(adiff, bdiff) * feed_factor, active_extruder);
    #else
      // planner.buffer_line
      buffer_line_kinematic(ltarget, _feedrate_mm_s, active_extruder);
    #endif

    return false;
  }

bool prepare_move_to_destination_cartesian() {
    // Do not use feedrate_percentage for E or Z only moves
    if (current_position[X_AXIS] == destination[X_AXIS] && current_position[Y_AXIS] == destination[Y_AXIS]) {
      line_to_destination();
    }
    else {
      #if ENABLED(MESH_BED_LEVELING)
        if (mbl.active()) {
          mesh_line_to_destination(MMS_SCALED(feedrate_mm_s));
          return true;
        }
        else
      #elif ENABLED(AUTO_BED_LEVELING_UBL)
        if (ubl.state.active) {
          ubl_line_to_destination(MMS_SCALED(feedrate_mm_s), active_extruder);
          return true;
        }
        else
      #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
        // planner.abl_enabled
        if (planner.abl_enabled) {
          bilinear_line_to_destination(MMS_SCALED(feedrate_mm_s));
          return true;
        }
        else
      #endif
          line_to_destination(MMS_SCALED(feedrate_mm_s));
    }
    return false;
}

void calc_move(float cur[XYZE], float tgt[XYZE]) {
    for (int i = 0; i < XYZE; i++) {
        current_position[i] = cur[i];
        destination[i] = tgt[i];
    }

    #if IS_KINEMATIC
        prepare_kinematic_move_to(tgt);
    #else
        #if DUAL_X_CARRIAGE // Most likely need to do the ENABLED thing.
            prepare_move_to_destination_dualx()
        #else
            prepare_move_to_destination_cartesian();
        #endif
    #endif
}
