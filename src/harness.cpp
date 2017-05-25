#include "harness.h"

#define _BV(PIN) (1 << PIN)

#define XYZE 4

// #include "enum.h"
// #include "macros.h"
// #include "MarlinConfig.h"

bool prepare_kinematic_move_to(float ltarget[XYZE]);
bool prepare_move_to_destination_dualx();
bool prepare_move_to_cartesian();

void calc_move(float cur[XYZE], float tgt[XYZE]) {
    #if IS_KINEMATIC
        // prepare_kinematic_move_to(tgt);
    #else
        #if DUAL_X_CARRIAGE // Most likely need to do the ENABLED thing.
            // prepare_move_to_destination_dualx()
        #else
            // prepare_move_to_cartesian();
        #endif
    #endif
}


/*
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
*/
