/*
 * Everything in here is dangerous.
 */
#include "harness.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

int acc_steps[4] = { 0 };

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

#define F_CPU 16000000

// Define prototypes and constants needed
#ifndef SCARA_LINKAGE_1
  #define SCARA_LINKAGE_1 0
#endif
#ifndef SCARA_LINKAGE_2
  #define SCARA_LINKAGE_2 0
#endif

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
#define MINIMAL_STEP_RATE 120

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
float axis_steps_per_mm[XYZE_N] = DEFAULT_AXIS_STEPS_PER_UNIT;
float max_feedrate_mm_s[XYZE] = DEFAULT_MAX_FEEDRATE;
uint32_t max_acceleration_steps_per_s2[XYZE_N] = DEFAULT_MAX_ACCELERATION;

float max_allowable_speed(const float &accel, const float &target_velocity, const float &distance) {
    return sqrt(sq(target_velocity) - 2 * accel * distance);
}

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
uint32_t highest_rate = 1;
uint32_t cutoff_long = 4294967295UL / highest_rate; // Anton: What the fuck

long pl_position[NUM_AXIS] = { 0 };
float volumetric_multiplier[EXTRUDERS] = ARRAY_BY_EXTRUDERS1(1.0);
uint8_t g_uc_extruder_last_move[EXTRUDERS] = { 0 };
float min_feedrate_mm_s = DEFAULT_MINIMUMFEEDRATE;
float min_travel_feedrate_mm_s = DEFAULT_MINTRAVELFEEDRATE;
float retract_acceleration = DEFAULT_RETRACT_ACCELERATION;
float travel_acceleration = DEFAULT_TRAVEL_ACCELERATION;
float acceleration = DEFAULT_ACCELERATION;
float min_segment_time = DEFAULT_MINSEGMENTTIME;

float max_jerk[XYZE]={DEFAULT_XJERK,DEFAULT_YJERK,DEFAULT_ZJERK,DEFAULT_EJERK};

// Anton: previous speed information is sketchy if we plan only one movement.
float previous_speed[NUM_AXIS] = {0.0};
float previous_nominal_speed = 0.0;


#if ENABLED(DELTA)
    float delta_segments_per_second = Z_MAX_POS;
#else
  #if IS_SCARA
    float delta_segments_per_second = SCARA_SEGMENTS_PER_SECOND;
  #else
    float delta_segments_per_second = 0;
  #endif
#endif
// Block information
enum BlockFlagBit {
  // Recalculate trapezoids on entry junction. For optimization.
  BLOCK_BIT_RECALCULATE,

  // Nominal speed always reached.
  // i.e., The segment is long enough, so the nominal speed is reachable if accelerating
  // from a safe speed (in consideration of jerking from zero speed).
  BLOCK_BIT_NOMINAL_LENGTH,

  // Start from a halt at the start of this block, respecting the maximum allowed jerk.
  BLOCK_BIT_START_FROM_FULL_HALT,

  // The block is busy
  BLOCK_BIT_BUSY
};

enum BlockFlag {
  BLOCK_FLAG_RECALCULATE          = _BV(BLOCK_BIT_RECALCULATE),
  BLOCK_FLAG_NOMINAL_LENGTH       = _BV(BLOCK_BIT_NOMINAL_LENGTH),
  BLOCK_FLAG_START_FROM_FULL_HALT = _BV(BLOCK_BIT_START_FROM_FULL_HALT),
  BLOCK_FLAG_BUSY                 = _BV(BLOCK_BIT_BUSY)
};

/**
 * struct block_t
 *
 * A single entry in the planner buffer.
 * Tracks linear movement over multiple axes.
 *
 * The "nominal" values are as-specified by gcode, and
 * may never actually be reached due to acceleration limits.
 */
typedef struct {

  uint8_t flag;                             // Block flags (See BlockFlag enum above)

  unsigned char active_extruder;            // The extruder to move (if E move)

  // Fields used by the Bresenham algorithm for tracing the line
  int32_t steps[NUM_AXIS];                  // Step count along each axis
  uint32_t step_event_count;                // The number of step events required to complete this block

  #if ENABLED(MIXING_EXTRUDER)
    uint32_t mix_event_count[MIXING_STEPPERS]; // Scaled step_event_count for the mixing steppers
  #endif

  int32_t accelerate_until,                 // The index of the step event on which to stop acceleration
          decelerate_after,                 // The index of the step event on which to start decelerating
          acceleration_rate;                // The acceleration rate used for acceleration calculation

  uint8_t direction_bits;                   // The direction bit set for this block (refers to *_DIRECTION_BIT in config.h)

  // Advance extrusion
  #if ENABLED(LIN_ADVANCE)
    bool use_advance_lead;
    uint32_t abs_adv_steps_multiplier8; // Factorised by 2^8 to avoid float
  #elif ENABLED(ADVANCE)
    int32_t advance_rate;
    volatile int32_t initial_advance;
    volatile int32_t final_advance;
    float advance;
  #endif

  // Fields used by the motion planner to manage acceleration
  float nominal_speed,                      // The nominal speed for this block in mm/sec
        entry_speed,                        // Entry speed at previous-current junction in mm/sec
        max_entry_speed,                    // Maximum allowable junction entry speed in mm/sec
        millimeters,                        // The total travel of this block in mm
        acceleration;                       // acceleration mm/sec^2

  // Settings for the trapezoid generator
  uint32_t nominal_rate,                    // The nominal step rate for this block in step_events/sec
           initial_rate,                    // The jerk-adjusted step rate at start of block
           final_rate,                      // The minimal rate at exit
           acceleration_steps_per_s2;       // acceleration steps/sec^2

  #if FAN_COUNT > 0
    uint16_t fan_speed[FAN_COUNT];
  #endif

  #if ENABLED(BARICUDA)
    uint32_t valve_pressure, e_to_p_pressure;
  #endif

  uint32_t segment_time;

} block_t;

void line_to_destination();
void line_to_destination(float);
bool prepare_kinematic_move_to(float ltarget[XYZE]);
bool prepare_move_to_destination_dualx();
bool prepare_move_to_cartesian();
void calculate_trapezoid_for_block(block_t* const block, const float &entry_factor, const float &exit_factor);

void recalculate();
void recalculate_trapezoids();
void reverse_pass();
void reverse_pass_kernel(block_t* const current, const block_t* next);
void forward_pass();
void forward_pass_kernel(const block_t* previous, block_t* const current);


float estimate_acceleration_distance(const float &initial_rate,const float &target_rate,const float &accel) {
  if (accel == 0) return 0; // accel was 0, set acceleration distance to 0
  return (sq(target_rate) - sq(initial_rate)) / (accel * 2);
}

float intersection_distance(const float &initial_rate,const float &final_rate, const float &accel,const float &distance) {
  if (accel == 0) return 0; // accel was 0, set intersection distance to 0
  return (accel * 2 * distance - sq(initial_rate) + sq(final_rate)) / (accel * 4);
}

float abs(float f) { return fabs(f); }

// Float constants for SCARA calculations
const float L1 = SCARA_LINKAGE_1, L2 = SCARA_LINKAGE_2,
            L1_2 = sq(float(L1)), L1_2_2 = 2.0 * L1_2,
            L2_2 = sq(float(L2));

#if ENABLED(DELTA_FAST_SQRT)
  /**
   * Fast inverse sqrt from Quake III Arena
   * See: https://en.wikipedia.org/wiki/Fast_inverse_square_root
   */
  float Q_rsqrt(float number) {
    long i;
    float x2, y;
    const float threehalfs = 1.5f;
    x2 = number * 0.5f;
    y  = number;
    i  = * ( long * ) &y;     // evil floating point bit level hacking
    i  = 0x5F3759DF - ( i >> 1 );   // what the f***?
    y  = * ( float * ) &i;
    y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration
    // y  = y * ( threehalfs - ( x2 * y * y ) );   // 2nd iteration, this can be removed
    return y;
  }
  #define _SQRT(n) (1.0f / Q_rsqrt(n))
#else
  #define _SQRT(n) sqrt(n)
#endif

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

      // S2 = sqrt(sq(C2) - 1);
      S2 = sqrt(1 - sq(C2));  // Anton: beekm's fix

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

      // printf("SCARA_OFF_X: %d, SCARA_OFF_Y: %d\n",
      //        SCARA_OFFSET_X, SCARA_OFFSET_Y);
      printf("logical[XYZ] = [%f, %f, %f]\n",
             logical[X_AXIS], logical[Y_AXIS], logical[Z_AXIS]);
      // printf("C2: %f, S2: %f, SK1: %f, SK2: %f, THETA: %f, PSI: %f, sx: %f, sy: %f\n",
      //         C2, S2, SK1, SK2, THETA, PSI, sx, sy);
      // printf("L1: %f, L1_2: %f, L1_2_2: %f, L2: %f, L2_2: %f\n",
      //        L1, L1_2, L1_2_2, L2, L2_2);
              
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
      // CRITICAL_SECTION_START;
      // ((a1+a2)+(a1-a2))/2 -> (a1+a2+a1-a2)/2 -> (a1+a1)/2 -> a1
      // ((a1+a2)-(a1-a2))/2 -> (a1+a2-a1+a2)/2 -> (a2+a2)/2 -> a2
      axis_steps = 0.5f * (
        axis == CORE_AXIS_2 ? CORESIGN(count_position[CORE_AXIS_1] - count_position[CORE_AXIS_2])
                            : count_position[CORE_AXIS_1] + count_position[CORE_AXIS_2]
      );
      // CRITICAL_SECTION_END;
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

// Recalculation
void reverse_pass_kernel(block_t* const current, const block_t *next) {
  if (!current || !next) return;
  // If entry speed is already at the maximum entry speed, no need to recheck. Block is cruising.
  // If not, block in state of acceleration or deceleration. Reset entry speed to maximum and
  // check for maximum allowable speed reductions to ensure maximum possible planned speed.
  float max_entry_speed = current->max_entry_speed;
  if (current->entry_speed != max_entry_speed) {
    // If nominal length true, max junction speed is guaranteed to be reached. Only compute
    // for max allowable speed if block is decelerating and nominal length is false.
    current->entry_speed = (TEST(current->flag, BLOCK_BIT_NOMINAL_LENGTH) || max_entry_speed <= next->entry_speed)
      ? max_entry_speed
      : min(max_entry_speed, max_allowable_speed(-current->acceleration, next->entry_speed, current->millimeters));
    SBI(current->flag, BLOCK_BIT_RECALCULATE);
  }
}

void reverse_pass() {
  // Anton: Commenting this out because we are interested in moving only a single block for now.
  /*
  if (movesplanned() > 3) {

    block_t* block[3] = { NULL, NULL, NULL };

    // Make a local copy of block_buffer_tail, because the interrupt can alter it
    // Is a critical section REALLY needed for a single byte change?
    //CRITICAL_SECTION_START;
    uint8_t tail = block_buffer_tail;
    //CRITICAL_SECTION_END

    uint8_t b = BLOCK_MOD(block_buffer_head - 3);
    while (b != tail) {
      if (block[0] && TEST(block[0]->flag, BLOCK_BIT_START_FROM_FULL_HALT)) break;
      b = prev_block_index(b);
      block[2] = block[1];
      block[1] = block[0];
      block[0] = &block_buffer[b];
      reverse_pass_kernel(block[1], block[2]);
    }
  }
  */
}

// The kernel called by recalculate() when scanning the plan from first to last entry.
void forward_pass_kernel(const block_t* previous, block_t* const current) {
  if (!previous) return;

  // If the previous block is an acceleration block, but it is not long enough to complete the
  // full speed change within the block, we need to adjust the entry speed accordingly. Entry
  // speeds have already been reset, maximized, and reverse planned by reverse planner.
  // If nominal length is true, max junction speed is guaranteed to be reached. No need to recheck.
  if (!TEST(previous->flag, BLOCK_BIT_NOMINAL_LENGTH)) {
    if (previous->entry_speed < current->entry_speed) {
      float entry_speed = min(current->entry_speed,
                               max_allowable_speed(-previous->acceleration, previous->entry_speed, previous->millimeters));
      // Check for junction speed change
      if (current->entry_speed != entry_speed) {
        current->entry_speed = entry_speed;
        SBI(current->flag, BLOCK_BIT_RECALCULATE);
      }
    }
  }
}

/**
 * recalculate() needs to go over the current plan twice.
 * Once in reverse and once forward. This implements the forward pass.
 */
void forward_pass() {
  // Anton: Like the reverse pass, our planned moves are all empty.
  /*
  block_t* block[3] = { NULL, NULL, NULL };

  for (uint8_t b = block_buffer_tail; b != block_buffer_head; b = next_block_index(b)) {
    block[0] = block[1];
    block[1] = block[2];
    block[2] = &block_buffer[b];
    forward_pass_kernel(block[0], block[1]);
  }
  forward_pass_kernel(block[1], block[2]);
  */
}

void recalculate_trapezoids() {
  // Anton: I need to implement a better environment with multiple blocks and shit.
  /*
  int8_t block_index = block_buffer_tail;
  block_t *current, *next = NULL;

  while (block_index != block_buffer_head) {
    current = next;
    next = &block_buffer[block_index];
    if (current) {
      // Recalculate if current block entry or exit junction speed has changed.
      if (TEST(current->flag, BLOCK_BIT_RECALCULATE) || TEST(next->flag, BLOCK_BIT_RECALCULATE)) {
        // NOTE: Entry and exit factors always > 0 by all previous logic operations.
        float nom = current->nominal_speed;
        calculate_trapezoid_for_block(current, current->entry_speed / nom, next->entry_speed / nom);
        CBI(current->flag, BLOCK_BIT_RECALCULATE); // Reset current only to ensure next trapezoid is computed
      }
    }
    block_index = next_block_index(block_index);
  }

  // Last/newest block in buffer. Exit speed is set with MINIMUM_PLANNER_SPEED. Always recalculated.
  if (next) {
    float nom = next->nominal_speed;
    calculate_trapezoid_for_block(next, next->entry_speed / nom, (MINIMUM_PLANNER_SPEED) / nom);
    CBI(next->flag, BLOCK_BIT_RECALCULATE);
  }
  */
}

void recalculate() {
  reverse_pass();
  forward_pass();
  recalculate_trapezoids();
}

// BUFFER LINE FUNCTIONS
void _buffer_line(const float &a, const float &b, const float &c, const float &e, float fr_mm_s, const uint8_t extruder);

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
    char* p;
void _buffer_line(const float &a, const float &b, const float &c, const float &e, float fr_mm_s, const uint8_t extruder) {
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

  const long target[XYZE] = {
    lround(a * axis_steps_per_mm[X_AXIS]),
    lround(b * axis_steps_per_mm[Y_AXIS]),
    lround(c * axis_steps_per_mm[Z_AXIS]),
    lround(e * axis_steps_per_mm[E_AXIS_N])
  };

  #if ENABLED(DISTINCT_E_FACTORS)
    if (last_extruder != extruder && axis_steps_per_mm[E_AXIS_N] != axis_steps_per_mm[E_AXIS + last_extruder]) {
      position[E_AXIS] = lround(position[E_AXIS] * axis_steps_per_mm[E_AXIS_N] * steps_to_mm[E_AXIS + last_extruder]);
      last_extruder = extruder;
    }
  #endif

  #if ENABLED(LIN_ADVANCE)
    const float mm_D_float = sqrt(sq(a - position_float[X_AXIS]) + sq(b - position_float[Y_AXIS]));
  #endif

  const long da = target[X_AXIS] - pl_position[X_AXIS],
             db = target[Y_AXIS] - pl_position[Y_AXIS],
             dc = target[Z_AXIS] - pl_position[Z_AXIS];

  long de = target[E_AXIS] - pl_position[E_AXIS];

  #if ENABLED(LIN_ADVANCE)
    float de_float = e - position_float[E_AXIS];
  #endif

  // Compute direction bit-mask for this block
  uint8_t dm = 0;
  #if CORE_IS_XY
    if (da < 0) SBI(dm, X_HEAD);                // Save the real Extruder (head) direction in X Axis
    if (db < 0) SBI(dm, Y_HEAD);                // ...and Y
    if (dc < 0) SBI(dm, Z_AXIS);
    if (da + db < 0) SBI(dm, A_AXIS);           // Motor A direction
    if (CORESIGN(da - db) < 0) SBI(dm, B_AXIS); // Motor B direction
  #elif CORE_IS_XZ
    if (da < 0) SBI(dm, X_HEAD);                // Save the real Extruder (head) direction in X Axis
    if (db < 0) SBI(dm, Y_AXIS);
    if (dc < 0) SBI(dm, Z_HEAD);                // ...and Z
    if (da + dc < 0) SBI(dm, A_AXIS);           // Motor A direction
    if (CORESIGN(da - dc) < 0) SBI(dm, C_AXIS); // Motor C direction
  #elif CORE_IS_YZ
    if (da < 0) SBI(dm, X_AXIS);
    if (db < 0) SBI(dm, Y_HEAD);                // Save the real Extruder (head) direction in Y Axis
    if (dc < 0) SBI(dm, Z_HEAD);                // ...and Z
    if (db + dc < 0) SBI(dm, B_AXIS);           // Motor B direction
    if (CORESIGN(db - dc) < 0) SBI(dm, C_AXIS); // Motor C direction
  #else
    if (da < 0) SBI(dm, X_AXIS);
    if (db < 0) SBI(dm, Y_AXIS);
    if (dc < 0) SBI(dm, Z_AXIS);
  #endif
  if (de < 0) SBI(dm, E_AXIS);

  const float esteps_float = de * volumetric_multiplier[extruder] * flow_percentage[extruder] * 0.01;
  const int32_t esteps = abs(esteps_float) + 0.5;


  // MAKE THEM FAKE BLOCKS YEAH
  block_t fake;
  // block_t* block = &block_buffer[block_buffer_head];
  block_t* block = &fake;

  // Clear all flags, including the "busy" bit
  block->flag = 0;

  // Set direction bits
  block->direction_bits = dm;

  // Number of steps for each axis
  // See http://www.corexy.com/theory.html
  #if CORE_IS_XY
    block->steps[A_AXIS] = labs(da + db);
    block->steps[B_AXIS] = labs(da - db);
    block->steps[Z_AXIS] = labs(dc);
  #elif CORE_IS_XZ
    block->steps[A_AXIS] = labs(da + dc);
    block->steps[Y_AXIS] = labs(db);
    block->steps[C_AXIS] = labs(da - dc);
  #elif CORE_IS_YZ
    block->steps[X_AXIS] = labs(da);
    block->steps[B_AXIS] = labs(db + dc);
    block->steps[C_AXIS] = labs(db - dc);
  #else
    // default non-h-bot planning
    block->steps[X_AXIS] = labs(da);
    block->steps[Y_AXIS] = labs(db);
    block->steps[Z_AXIS] = labs(dc);
  #endif

  block->steps[E_AXIS] = esteps;
  block->step_event_count = MAX4(block->steps[X_AXIS], block->steps[Y_AXIS], block->steps[Z_AXIS], esteps);

  // Bail if this is a zero-length block
  if (block->step_event_count < MIN_STEPS_PER_SEGMENT) return;

  // For a mixing extruder, get a magnified step_event_count for each
  #if ENABLED(MIXING_EXTRUDER)
    for (uint8_t i = 0; i < MIXING_STEPPERS; i++)
      block->mix_event_count[i] = mixing_factor[i] * block->step_event_count;
  #endif

  #if FAN_COUNT > 0
    for (uint8_t i = 0; i < FAN_COUNT; i++) block->fan_speed[i] = fanSpeeds[i];
  #endif

  #if ENABLED(BARICUDA)
    block->valve_pressure = baricuda_valve_pressure;
    block->e_to_p_pressure = baricuda_e_to_p_pressure;
  #endif

  block->active_extruder = extruder;

  //enable active axes
  #if CORE_IS_XY
    if (block->steps[A_AXIS] || block->steps[B_AXIS]) {
      enable_X();
      enable_Y();
    }
    #if DISABLED(Z_LATE_ENABLE)
      if (block->steps[Z_AXIS]) enable_Z();
    #endif
  #elif CORE_IS_XZ
    if (block->steps[A_AXIS] || block->steps[C_AXIS]) {
      enable_X();
      enable_Z();
    }
    if (block->steps[Y_AXIS]) enable_Y();
  #elif CORE_IS_YZ
    if (block->steps[B_AXIS] || block->steps[C_AXIS]) {
      enable_Y();
      enable_Z();
    }
    if (block->steps[X_AXIS]) enable_X();
  #else
    if (block->steps[X_AXIS]) enable_X();
    if (block->steps[Y_AXIS]) enable_Y();
    #if DISABLED(Z_LATE_ENABLE)
      if (block->steps[Z_AXIS]) enable_Z();
    #endif
  #endif

  // Enable extruder(s)
  if (esteps) {

    #if ENABLED(DISABLE_INACTIVE_EXTRUDER) // Enable only the selected extruder

      #define DISABLE_IDLE_E(N) if (!g_uc_extruder_last_move[N]) disable_E##N();

      for (uint8_t i = 0; i < EXTRUDERS; i++)
        if (g_uc_extruder_last_move[i] > 0) g_uc_extruder_last_move[i]--;

      switch(extruder) {
        case 0:
          enable_E0();
          g_uc_extruder_last_move[0] = (BLOCK_BUFFER_SIZE) * 2;
          #if ENABLED(DUAL_X_CARRIAGE) || ENABLED(DUAL_NOZZLE_DUPLICATION_MODE)
            if (extruder_duplication_enabled) {
              enable_E1();
              g_uc_extruder_last_move[1] = (BLOCK_BUFFER_SIZE) * 2;
            }
          #endif
          #if EXTRUDERS > 1
            DISABLE_IDLE_E(1);
            #if EXTRUDERS > 2
              DISABLE_IDLE_E(2);
              #if EXTRUDERS > 3
                DISABLE_IDLE_E(3);
                #if EXTRUDERS > 4
                  DISABLE_IDLE_E(4);
                #endif // EXTRUDERS > 4
              #endif // EXTRUDERS > 3
            #endif // EXTRUDERS > 2
          #endif // EXTRUDERS > 1
        break;
        #if EXTRUDERS > 1
          case 1:
            enable_E1();
            g_uc_extruder_last_move[1] = (BLOCK_BUFFER_SIZE) * 2;
            DISABLE_IDLE_E(0);
            #if EXTRUDERS > 2
              DISABLE_IDLE_E(2);
              #if EXTRUDERS > 3
                DISABLE_IDLE_E(3);
                #if EXTRUDERS > 4
                  DISABLE_IDLE_E(4);
                #endif // EXTRUDERS > 4
              #endif // EXTRUDERS > 3
            #endif // EXTRUDERS > 2
          break;
          #if EXTRUDERS > 2
            case 2:
              enable_E2();
              g_uc_extruder_last_move[2] = (BLOCK_BUFFER_SIZE) * 2;
              DISABLE_IDLE_E(0);
              DISABLE_IDLE_E(1);
              #if EXTRUDERS > 3
                DISABLE_IDLE_E(3);
                #if EXTRUDERS > 4
                  DISABLE_IDLE_E(4);
                #endif
              #endif
            break;
            #if EXTRUDERS > 3
              case 3:
                enable_E3();
                g_uc_extruder_last_move[3] = (BLOCK_BUFFER_SIZE) * 2;
                DISABLE_IDLE_E(0);
                DISABLE_IDLE_E(1);
                DISABLE_IDLE_E(2);
                #if EXTRUDERS > 4
                  DISABLE_IDLE_E(4);
                #endif
              break;
              #if EXTRUDERS > 4
                case 4:
                  enable_E4();
                  g_uc_extruder_last_move[4] = (BLOCK_BUFFER_SIZE) * 2;
                  DISABLE_IDLE_E(0);
                  DISABLE_IDLE_E(1);
                  DISABLE_IDLE_E(2);
                  DISABLE_IDLE_E(3);
                break;
              #endif // EXTRUDERS > 4
            #endif // EXTRUDERS > 3
          #endif // EXTRUDERS > 2
        #endif // EXTRUDERS > 1
      }
    #else
      enable_E0();
      enable_E1();
      enable_E2();
      enable_E3();
      enable_E4();
    #endif
  }

  if (esteps)
    NOLESS(fr_mm_s, min_feedrate_mm_s);
  else
    NOLESS(fr_mm_s, min_travel_feedrate_mm_s);

  /**
   * This part of the code calculates the total length of the movement.
   * For cartesian bots, the X_AXIS is the real X movement and same for Y_AXIS.
   * But for corexy bots, that is not true. The "X_AXIS" and "Y_AXIS" motors (that should be named to A_AXIS
   * and B_AXIS) cannot be used for X and Y length, because A=X+Y and B=X-Y.
   * So we need to create other 2 "AXIS", named X_HEAD and Y_HEAD, meaning the real displacement of the Head.
   * Having the real displacement of the head, we can calculate the total movement length and apply the desired speed.
   */
  #if IS_CORE
    float delta_mm[Z_HEAD + 1];
    #if CORE_IS_XY
      delta_mm[X_HEAD] = da * steps_to_mm[A_AXIS];
      delta_mm[Y_HEAD] = db * steps_to_mm[B_AXIS];
      delta_mm[Z_AXIS] = dc * steps_to_mm[Z_AXIS];
      delta_mm[A_AXIS] = (da + db) * steps_to_mm[A_AXIS];
      delta_mm[B_AXIS] = CORESIGN(da - db) * steps_to_mm[B_AXIS];
    #elif CORE_IS_XZ
      delta_mm[X_HEAD] = da * steps_to_mm[A_AXIS];
      delta_mm[Y_AXIS] = db * steps_to_mm[Y_AXIS];
      delta_mm[Z_HEAD] = dc * steps_to_mm[C_AXIS];
      delta_mm[A_AXIS] = (da + dc) * steps_to_mm[A_AXIS];
      delta_mm[C_AXIS] = CORESIGN(da - dc) * steps_to_mm[C_AXIS];
    #elif CORE_IS_YZ
      delta_mm[X_AXIS] = da * steps_to_mm[X_AXIS];
      delta_mm[Y_HEAD] = db * steps_to_mm[B_AXIS];
      delta_mm[Z_HEAD] = dc * steps_to_mm[C_AXIS];
      delta_mm[B_AXIS] = (db + dc) * steps_to_mm[B_AXIS];
      delta_mm[C_AXIS] = CORESIGN(db - dc) * steps_to_mm[C_AXIS];
    #endif
  #else
    float delta_mm[XYZE];
    delta_mm[X_AXIS] = da * steps_to_mm[X_AXIS];
    delta_mm[Y_AXIS] = db * steps_to_mm[Y_AXIS];
    delta_mm[Z_AXIS] = dc * steps_to_mm[Z_AXIS];

  #endif
  delta_mm[E_AXIS] = esteps_float * steps_to_mm[E_AXIS_N];
  // printf("[da, db, dc, de] = [%f, %f, %f, %f]\n", da, db, dc, esteps_float);
  // printf("steps_to_mm[XYZE] = [%f, %f, %f, %f]\n",
  //        steps_to_mm[X_AXIS], steps_to_mm[Y_AXIS], steps_to_mm[Z_AXIS], steps_to_mm[E_AXIS]);

  if (block->steps[X_AXIS] < MIN_STEPS_PER_SEGMENT && block->steps[Y_AXIS] < MIN_STEPS_PER_SEGMENT && block->steps[Z_AXIS] < MIN_STEPS_PER_SEGMENT) {
    block->millimeters = fabs(delta_mm[E_AXIS]);
  }
  else {
    block->millimeters = sqrt(
      #if CORE_IS_XY
        sq(delta_mm[X_HEAD]) + sq(delta_mm[Y_HEAD]) + sq(delta_mm[Z_AXIS])
      #elif CORE_IS_XZ
        sq(delta_mm[X_HEAD]) + sq(delta_mm[Y_AXIS]) + sq(delta_mm[Z_HEAD])
      #elif CORE_IS_YZ
        sq(delta_mm[X_AXIS]) + sq(delta_mm[Y_HEAD]) + sq(delta_mm[Z_HEAD])
      #else
        sq(delta_mm[X_AXIS]) + sq(delta_mm[Y_AXIS]) + sq(delta_mm[Z_AXIS])
      #endif
    );
  }
  float inverse_millimeters = 1.0 / block->millimeters;  // Inverse millimeters to remove multiple divides

  // Calculate moves/second for this move. No divide by zero due to previous checks.
  float inverse_mm_s = fr_mm_s * inverse_millimeters;

  // const uint8_t moves_queued = movesplanned();
  const uint8_t moves_queued = 1;

  // Slow down when the buffer starts to empty, rather than wait at the corner for a buffer refill
  #if ENABLED(SLOWDOWN) || ENABLED(ULTRA_LCD) || defined(XY_FREQUENCY_LIMIT)
    // Segment time im micro seconds
    unsigned long segment_time = lround(1000000.0 / inverse_mm_s);
  #endif
  #if ENABLED(SLOWDOWN)
    if (WITHIN(moves_queued, 2, (BLOCK_BUFFER_SIZE) / 2 - 1)) {
      if (segment_time < min_segment_time) {
        // buffer is draining, add extra time.  The amount of time added increases if the buffer is still emptied more.
        inverse_mm_s = 1000000.0 / (segment_time + lround(2 * (min_segment_time - segment_time) / moves_queued));
        #if defined(XY_FREQUENCY_LIMIT) || ENABLED(ULTRA_LCD)
          segment_time = lround(1000000.0 / inverse_mm_s);
        #endif
      }
    }
  #endif

  #if ENABLED(ULTRA_LCD)
    // CRITICAL_SECTION_START
      block_buffer_runtime_us += segment_time;
    // CRITICAL_SECTION_END
  #endif

  block->nominal_speed = block->millimeters * inverse_mm_s; // (mm/sec) Always > 0
  block->nominal_rate = ceil(block->step_event_count * inverse_mm_s); // (step/sec) Always > 0

  #if ENABLED(FILAMENT_WIDTH_SENSOR)
    static float filwidth_e_count = 0, filwidth_delay_dist = 0;

    //FMM update ring buffer used for delay with filament measurements
    if (extruder == FILAMENT_SENSOR_EXTRUDER_NUM && filwidth_delay_index[1] >= 0) {  //only for extruder with filament sensor and if ring buffer is initialized

      const int MMD_CM = MAX_MEASUREMENT_DELAY + 1, MMD_MM = MMD_CM * 10;

      // increment counters with next move in e axis
      filwidth_e_count += delta_mm[E_AXIS];
      filwidth_delay_dist += delta_mm[E_AXIS];

      // Only get new measurements on forward E movement
      if (filwidth_e_count > 0.0001) {

        // Loop the delay distance counter (modulus by the mm length)
        while (filwidth_delay_dist >= MMD_MM) filwidth_delay_dist -= MMD_MM;

        // Convert into an index into the measurement array
        filwidth_delay_index[0] = (int)(filwidth_delay_dist * 0.1 + 0.0001);

        // If the index has changed (must have gone forward)...
        if (filwidth_delay_index[0] != filwidth_delay_index[1]) {
          filwidth_e_count = 0; // Reset the E movement counter
          const int8_t meas_sample = thermalManager.widthFil_to_size_ratio() - 100; // Subtract 100 to reduce magnitude - to store in a signed char
          do {
            filwidth_delay_index[1] = (filwidth_delay_index[1] + 1) % MMD_CM; // The next unused slot
            measurement_delay[filwidth_delay_index[1]] = meas_sample;         // Store the measurement
          } while (filwidth_delay_index[0] != filwidth_delay_index[1]);       // More slots to fill?
        }
      }
    }
  #endif

  // Calculate and limit speed in mm/sec for each axis
  float current_speed[NUM_AXIS], speed_factor = 1.0; // factor <1 decreases speed
  LOOP_XYZE(i) {
    const float cs = fabs(current_speed[i] = delta_mm[i] * inverse_mm_s);
    // printf("cs[%f], d_mm[%f], i_mms[%f]\n", current_speed[i], delta_mm[i], inverse_mm_s);
    #if ENABLED(DISTINCT_E_FACTORS)
      if (i == E_AXIS) i += extruder;
    #endif
    if (cs > max_feedrate_mm_s[i]) NOMORE(speed_factor, max_feedrate_mm_s[i] / cs);
  }

  // Max segment time in µs.
  #ifdef XY_FREQUENCY_LIMIT

    // Check and limit the xy direction change frequency
    const unsigned char direction_change = block->direction_bits ^ old_direction_bits;
    old_direction_bits = block->direction_bits;
    segment_time = lround((float)segment_time / speed_factor);

    long xs0 = axis_segment_time[X_AXIS][0],
         xs1 = axis_segment_time[X_AXIS][1],
         xs2 = axis_segment_time[X_AXIS][2],
         ys0 = axis_segment_time[Y_AXIS][0],
         ys1 = axis_segment_time[Y_AXIS][1],
         ys2 = axis_segment_time[Y_AXIS][2];

    if (TEST(direction_change, X_AXIS)) {
      xs2 = axis_segment_time[X_AXIS][2] = xs1;
      xs1 = axis_segment_time[X_AXIS][1] = xs0;
      xs0 = 0;
    }
    xs0 = axis_segment_time[X_AXIS][0] = xs0 + segment_time;

    if (TEST(direction_change, Y_AXIS)) {
      ys2 = axis_segment_time[Y_AXIS][2] = axis_segment_time[Y_AXIS][1];
      ys1 = axis_segment_time[Y_AXIS][1] = axis_segment_time[Y_AXIS][0];
      ys0 = 0;
    }
    ys0 = axis_segment_time[Y_AXIS][0] = ys0 + segment_time;

    const long max_x_segment_time = MAX3(xs0, xs1, xs2),
               max_y_segment_time = MAX3(ys0, ys1, ys2),
               min_xy_segment_time = min(max_x_segment_time, max_y_segment_time);
    if (min_xy_segment_time < MAX_FREQ_TIME) {
      const float low_sf = speed_factor * min_xy_segment_time / (MAX_FREQ_TIME);
      NOMORE(speed_factor, low_sf);
    }
  #endif // XY_FREQUENCY_LIMIT


    // Correct the speed
  if (speed_factor < 1.0) {
    LOOP_XYZE(i) current_speed[i] *= speed_factor;
    block->nominal_speed *= speed_factor;
    block->nominal_rate *= speed_factor;
  }

  // Compute and limit the acceleration rate for the trapezoid generator.
  const float steps_per_mm = block->step_event_count * inverse_millimeters;
  uint32_t accel;
  if (!block->steps[X_AXIS] && !block->steps[Y_AXIS] && !block->steps[Z_AXIS]) {
    // convert to: acceleration steps/sec^2
    accel = ceil(retract_acceleration * steps_per_mm);
  }
  else {
    #define LIMIT_ACCEL_LONG(AXIS,INDX) do{ \
      if (block->steps[AXIS] && max_acceleration_steps_per_s2[AXIS+INDX] < accel) { \
        const uint32_t comp = max_acceleration_steps_per_s2[AXIS+INDX] * block->step_event_count; \
        if (accel * block->steps[AXIS] > comp) accel = comp / block->steps[AXIS]; \
      } \
    }while(0)

    #define LIMIT_ACCEL_FLOAT(AXIS,INDX) do{ \
      if (block->steps[AXIS] && max_acceleration_steps_per_s2[AXIS+INDX] < accel) { \
        const float comp = (float)max_acceleration_steps_per_s2[AXIS+INDX] * (float)block->step_event_count; \
        if ((float)accel * (float)block->steps[AXIS] > comp) accel = comp / (float)block->steps[AXIS]; \
      } \
    }while(0)

    // Start with print or travel acceleration
    accel = ceil((esteps ? acceleration : travel_acceleration) * steps_per_mm);

    #if ENABLED(DISTINCT_E_FACTORS)
      #define ACCEL_IDX extruder
    #else
      #define ACCEL_IDX 0
    #endif

    // Limit acceleration per axis
    if (block->step_event_count <= cutoff_long) {
      LIMIT_ACCEL_LONG(X_AXIS, 0);
      LIMIT_ACCEL_LONG(Y_AXIS, 0);
      LIMIT_ACCEL_LONG(Z_AXIS, 0);
      LIMIT_ACCEL_LONG(E_AXIS, ACCEL_IDX);
    }
    else {
      LIMIT_ACCEL_FLOAT(X_AXIS, 0);
      LIMIT_ACCEL_FLOAT(Y_AXIS, 0);
      LIMIT_ACCEL_FLOAT(Z_AXIS, 0);
      LIMIT_ACCEL_FLOAT(E_AXIS, ACCEL_IDX);
    }
  }

  block->acceleration_steps_per_s2 = accel;
  block->acceleration = accel / steps_per_mm;
  block->acceleration_rate = (long)(accel * 16777216.0 / ((F_CPU) * 0.125)); // * 8.388608

  // Initial limit on the segment entry velocity
  float vmax_junction;

  #if 0  // Use old jerk for now
    float junction_deviation = 0.1;
    // Compute path unit vector
    double unit_vec[XYZ] = {
      delta_mm[X_AXIS] * inverse_millimeters,
      delta_mm[Y_AXIS] * inverse_millimeters,
      delta_mm[Z_AXIS] * inverse_millimeters
    };
    /*
       Compute maximum allowable entry speed at junction by centripetal acceleration approximation.
       Let a circle be tangent to both previous and current path line segments, where the junction
       deviation is defined as the distance from the junction to the closest edge of the circle,
       collinear with the circle center.
       The circular segment joining the two paths represents the path of centripetal acceleration.
       Solve for max velocity based on max acceleration about the radius of the circle, defined
       indirectly by junction deviation.
       This may be also viewed as path width or max_jerk in the previous grbl version. This approach
       does not actually deviate from path, but used as a robust way to compute cornering speeds, as
       it takes into account the nonlinearities of both the junction angle and junction velocity.
     */
    vmax_junction = MINIMUM_PLANNER_SPEED; // Set default max junction speed
    // Skip first block or when previous_nominal_speed is used as a flag for homing and offset cycles.
    if (block_buffer_head != block_buffer_tail && previous_nominal_speed > 0.0) {
      // Compute cosine of angle between previous and current path. (prev_unit_vec is negative)
      // NOTE: Max junction velocity is computed without sin() or acos() by trig half angle identity.
      float cos_theta = - previous_unit_vec[X_AXIS] * unit_vec[X_AXIS]
                        - previous_unit_vec[Y_AXIS] * unit_vec[Y_AXIS]
                        - previous_unit_vec[Z_AXIS] * unit_vec[Z_AXIS] ;
      // Skip and use default max junction speed for 0 degree acute junction.
      if (cos_theta < 0.95) {
        vmax_junction = min(previous_nominal_speed, block->nominal_speed);
        // Skip and avoid divide by zero for straight junctions at 180 degrees. Limit to min() of nominal speeds.
        if (cos_theta > -0.95) {
          // Compute maximum junction velocity based on maximum acceleration and junction deviation
          float sin_theta_d2 = sqrt(0.5 * (1.0 - cos_theta)); // Trig half angle identity. Always positive.
          NOMORE(vmax_junction, sqrt(block->acceleration * junction_deviation * sin_theta_d2 / (1.0 - sin_theta_d2)));
        }
      }
    }
  #endif


  /**
   * Adapted from Prusa MKS firmware
   *
   * Start with a safe speed (from which the machine may halt to stop immediately).
   */

  // Exit speed limited by a jerk to full halt of a previous last segment
  static float previous_safe_speed;

  float safe_speed = block->nominal_speed;
  uint8_t limited = 0;
  LOOP_XYZE(i) {
    const float jerk = fabs(current_speed[i]), maxj = max_jerk[i];
    if (jerk > maxj) {
      if (limited) {
        const float mjerk = maxj * block->nominal_speed;
        if (jerk * safe_speed > mjerk) safe_speed = mjerk / jerk;
      }
      else {
        ++limited;
        safe_speed = maxj;
      }
    }
  }

  if (moves_queued > 1 && previous_nominal_speed > 0.0001) {
    // Estimate a maximum velocity allowed at a joint of two successive segments.
    // If this maximum velocity allowed is lower than the minimum of the entry / exit safe velocities,
    // then the machine is not coasting anymore and the safe entry / exit velocities shall be used.

    // The junction velocity will be shared between successive segments. Limit the junction velocity to their minimum.
    bool prev_speed_larger = previous_nominal_speed > block->nominal_speed;
    float smaller_speed_factor = prev_speed_larger ? (block->nominal_speed / previous_nominal_speed) : (previous_nominal_speed / block->nominal_speed);
    // Pick the smaller of the nominal speeds. Higher speed shall not be achieved at the junction during coasting.
    vmax_junction = prev_speed_larger ? block->nominal_speed : previous_nominal_speed;
    // Factor to multiply the previous / current nominal velocities to get componentwise limited velocities.
    float v_factor = 1.f;
    limited = 0;
    // Now limit the jerk in all axes.
    LOOP_XYZE(axis) {
      // Limit an axis. We have to differentiate: coasting, reversal of an axis, full stop.
      float v_exit = previous_speed[axis], v_entry = current_speed[axis];
      if (prev_speed_larger) v_exit *= smaller_speed_factor;
      if (limited) {
        v_exit *= v_factor;
        v_entry *= v_factor;
      }

      // Calculate jerk depending on whether the axis is coasting in the same direction or reversing.
      const float jerk = (v_exit > v_entry)
          ? //                                  coasting             axis reversal
            ( (v_entry > 0.f || v_exit < 0.f) ? (v_exit - v_entry) : max(v_exit, -v_entry) )
          : // v_exit <= v_entry                coasting             axis reversal
            ( (v_entry < 0.f || v_exit > 0.f) ? (v_entry - v_exit) : max(-v_exit, v_entry) );

      if (jerk > max_jerk[axis]) {
        v_factor *= max_jerk[axis] / jerk;
        ++limited;
      }
    }
    if (limited) vmax_junction *= v_factor;
    // Now the transition velocity is known, which maximizes the shared exit / entry velocity while
    // respecting the jerk factors, it may be possible, that applying separate safe exit / entry velocities will achieve faster prints.
    const float vmax_junction_threshold = vmax_junction * 0.99f;
    if (previous_safe_speed > vmax_junction_threshold && safe_speed > vmax_junction_threshold) {
      // Not coasting. The machine will stop and start the movements anyway,
      // better to start the segment from start.
      SBI(block->flag, BLOCK_BIT_START_FROM_FULL_HALT);
      vmax_junction = safe_speed;
    }
  }
  else {
    SBI(block->flag, BLOCK_BIT_START_FROM_FULL_HALT);
    vmax_junction = safe_speed;
  }

  // Max entry speed of this block equals the max exit speed of the previous block.
  block->max_entry_speed = vmax_junction;

  // Initialize block entry speed. Compute based on deceleration to user-defined MINIMUM_PLANNER_SPEED.
  const float v_allowable = max_allowable_speed(-block->acceleration, MINIMUM_PLANNER_SPEED, block->millimeters);
  block->entry_speed = min(vmax_junction, v_allowable);

  // Initialize planner efficiency flags
  // Set flag if block will always reach maximum junction speed regardless of entry/exit speeds.
  // If a block can de/ac-celerate from nominal speed to zero within the length of the block, then
  // the current block and next block junction speeds are guaranteed to always be at their maximum
  // junction speeds in deceleration and acceleration, respectively. This is due to how the current
  // block nominal speed limits both the current and next maximum junction speeds. Hence, in both
  // the reverse and forward planners, the corresponding block junction speed will always be at the
  // the maximum junction speed and may always be ignored for any speed reduction checks.
  block->flag |= BLOCK_FLAG_RECALCULATE | (block->nominal_speed <= v_allowable ? BLOCK_FLAG_NOMINAL_LENGTH : 0);

  // Update previous path unit_vector and nominal speed
  COPY(previous_speed, current_speed);
  previous_nominal_speed = block->nominal_speed;
  previous_safe_speed = safe_speed;

  #if ENABLED(LIN_ADVANCE)

    //
    // Use LIN_ADVANCE for blocks if all these are true:
    //
    // esteps                                          : We have E steps todo (a printing move)
    //
    // block->steps[X_AXIS] || block->steps[Y_AXIS]    : We have a movement in XY direction (i.e., not retract / prime).
    //
    // extruder_advance_k                              : There is an advance factor set.
    //
    // block->steps[E_AXIS] != block->step_event_count : A problem occurs if the move before a retract is too small.
    //                                                   In that case, the retract and move will be executed together.
    //                                                   This leads to too many advance steps due to a huge e_acceleration.
    //                                                   The math is good, but we must avoid retract moves with advance!
    // de_float > 0.0                                  : Extruder is running forward (e.g., for "Wipe while retracting" (Slic3r) or "Combing" (Cura) moves)
    //
    block->use_advance_lead =  esteps
                            && (block->steps[X_AXIS] || block->steps[Y_AXIS])
                            && extruder_advance_k
                            && (uint32_t)esteps != block->step_event_count
                            && de_float > 0.0;
    if (block->use_advance_lead)
      block->abs_adv_steps_multiplier8 = lround(
        extruder_advance_k
        * (UNEAR_ZERO(advance_ed_ratio) ? de_float / mm_D_float : advance_ed_ratio) // Use the fixed ratio, if set
        * (block->nominal_speed / (float)block->nominal_rate)
        * axis_steps_per_mm[E_AXIS_N] * 256.0
      );

  #elif ENABLED(ADVANCE)

    // Calculate advance rate
    if (esteps && (block->steps[X_AXIS] || block->steps[Y_AXIS] || block->steps[Z_AXIS])) {
      const long acc_dist = estimate_acceleration_distance(0, block->nominal_rate, block->acceleration_steps_per_s2);
      const float advance = ((STEPS_PER_CUBIC_MM_E) * (EXTRUDER_ADVANCE_K)) * HYPOT(current_speed[E_AXIS], EXTRUSION_AREA) * 256;
      block->advance = advance;
      block->advance_rate = acc_dist ? advance / (float)acc_dist : 0;
    }
    else
      block->advance_rate = block->advance = 0;

    /**
     SERIAL_ECHO_START;
     SERIAL_ECHOPGM("advance :");
     SERIAL_ECHO(block->advance/256.0);
     SERIAL_ECHOPGM("advance rate :");
     SERIAL_ECHOLN(block->advance_rate/256.0);
     */

  #endif // ADVANCE or LIN_ADVANCE

  calculate_trapezoid_for_block(block, block->entry_speed / block->nominal_speed, safe_speed / block->nominal_speed);

  // Move buffer head
  // block_buffer_head = next_buffer_head;

  // Update the position (only when a move was queued)
  COPY(pl_position, target);
  #if ENABLED(LIN_ADVANCE)
    position_float[X_AXIS] = a;
    position_float[Y_AXIS] = b;
    position_float[Z_AXIS] = c;
    position_float[E_AXIS] = e;
  #endif

  recalculate();

  // Print the block velocity triangle information that we have:
  printf("steps[XYZE] = [%d, %d, %d, %d]\n",
         block->steps[X_AXIS], block->steps[Y_AXIS],
         block->steps[Z_AXIS], block->steps[E_AXIS]);
  // printf("max_accel_steps_per_s2[XYZE] = [%d, %d, %d, %d]\n",
  //        max_acceleration_steps_per_s2[X_AXIS],
  //        max_acceleration_steps_per_s2[Y_AXIS],
  //        max_acceleration_steps_per_s2[Z_AXIS],
  //        max_acceleration_steps_per_s2[E_AXIS]);
  printf("current_speed[XYZE]: [%f, %f, %f, %f]\n",
          current_speed[X_AXIS], current_speed[Y_AXIS],
          current_speed[Z_AXIS], current_speed[E_AXIS]);
  printf("------------------------------------------------------\n"); 

  for (int i = 0; i < XYZE; i++) { acc_steps[i] += block->steps[i]; }
} // end of _buffer_line()

/*
void _buffer_line(const float &a, const float &b, const float &c, const float &e, float fr_mm_s, const uint8_t extruder) {
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
    printf("------------------------------------------------------\n");
}
*/

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

// Planner actions?
void calculate_trapezoid_for_block(block_t* const block, const float &entry_factor, const float &exit_factor) {
  uint32_t initial_rate = ceil(block->nominal_rate * entry_factor),
           final_rate = ceil(block->nominal_rate * exit_factor); // (steps per second)

  // Limit minimal step rate (Otherwise the timer will overflow.)
  NOLESS(initial_rate, MINIMAL_STEP_RATE);
  NOLESS(final_rate, MINIMAL_STEP_RATE);

  int32_t accel = block->acceleration_steps_per_s2,
          accelerate_steps = ceil(estimate_acceleration_distance(initial_rate, block->nominal_rate, accel)),
          decelerate_steps = floor(estimate_acceleration_distance(block->nominal_rate, final_rate, -accel)),
          plateau_steps = block->step_event_count - accelerate_steps - decelerate_steps;

  // Is the Plateau of Nominal Rate smaller than nothing? That means no cruising, and we will
  // have to use intersection_distance() to calculate when to abort accel and start braking
  // in order to reach the final_rate exactly at the end of this block.
  if (plateau_steps < 0) {
    accelerate_steps = ceil(intersection_distance(initial_rate, final_rate, accel, block->step_event_count));
    NOLESS(accelerate_steps, 0); // Check limits due to numerical round-off
    accelerate_steps = min((uint32_t)accelerate_steps, block->step_event_count);//(We can cast here to unsigned, because the above line ensures that we are above zero)
    plateau_steps = 0;
  }

  // block->accelerate_until = accelerate_steps;
  // block->decelerate_after = accelerate_steps+plateau_steps;

  // CRITICAL_SECTION_START;  // Fill variables used by the stepper in a critical section
  if (!TEST(block->flag, BLOCK_BIT_BUSY)) { // Don't update variables if block is busy.
    block->accelerate_until = accelerate_steps;
    block->decelerate_after = accelerate_steps + plateau_steps;
    block->initial_rate = initial_rate;
    block->final_rate = final_rate;
    #if ENABLED(ADVANCE)
      block->initial_advance = block->advance * sq(entry_factor);
      block->final_advance = block->advance * sq(exit_factor);
    #endif
  }
  // CRITICAL_SECTION_END;
}

void calc_move(float cur[XYZE], float tgt[XYZE]) {
    for (int i = 0; i < XYZE; i++) {
        current_position[i] = cur[i];
        destination[i] = tgt[i];
    }

    LOOP_XYZE_N(i) steps_to_mm[i] = 1.0 / axis_steps_per_mm[i];

    #if IS_KINEMATIC
        prepare_kinematic_move_to(tgt);
    #else
        #if DUAL_X_CARRIAGE // Most likely need to do the ENABLED thing.
            prepare_move_to_destination_dualx()
        #else
            prepare_move_to_destination_cartesian();
        #endif
    #endif

    printf("Total steps taken: [%d, %d, %d, %d]\n",
            acc_steps[0], acc_steps[1], acc_steps[2], acc_steps[3]);

    #if IS_KINEMATIC

    #else
        float x_mm = acc_steps[0] / axis_steps_per_mm[0];
        float y_mm = acc_steps[1] / axis_steps_per_mm[1];
        float z_mm = acc_steps[2] / axis_steps_per_mm[2];;
        float e_mm = acc_steps[3] / axis_steps_per_mm[3];
        printf("mm: [%f, %f, %f, %f]\n", x_mm, y_mm, z_mm, e_mm);
    #endif
}
