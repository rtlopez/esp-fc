#include "Buzzer.h"

namespace Espfc {

// put zero at end to terminate
static const uint8_t beeperSilence[] = { 0 };
static const uint8_t beeperGyroCalibrated[] = { 10, 10, 10, 0 };
static const uint8_t beeperRxLost[] = { 30, 0 };
static const uint8_t beeperDisarming[] = { 10, 10, 0 };
static const uint8_t beeperArming[] = { 20, 0 };
static const uint8_t beeperSystemInit[] = { 10, 0 };

const uint8_t* Buzzer::schemes[] = {
  //BEEPER_SILENCE
  beeperSilence,
  //BEEPER_GYRO_CALIBRATED,
  beeperGyroCalibrated,
  //BEEPER_RX_LOST,                 // Beeps when TX is turned off or signal lost (repeat until TX is okay)
  beeperRxLost,
  //BEEPER_RX_LOST_LANDING,         // Beeps SOS when armed and TX is turned off or signal lost (autolanding/autodisarm)
  beeperSilence,
  //BEEPER_DISARMING,               // Beep when disarming the board
  beeperDisarming,
  //BEEPER_ARMING,                  // Beep when arming the board
  beeperArming,
  //BEEPER_ARMING_GPS_FIX,          // Beep a special tone when arming the board and GPS has fix
  beeperSilence,
  //BEEPER_BAT_CRIT_LOW,            // Longer warning beeps when battery is critically low (repeats)
  beeperSilence,
  //BEEPER_BAT_LOW,                 // Warning beeps when battery is getting low (repeats)
  beeperSilence,
  //BEEPER_GPS_STATUS,              // FIXME **** Disable beeper when connected to USB ****
  beeperSilence,
  //BEEPER_RX_SET,                  // Beeps when aux channel is set for beep or beep sequence how many satellites has found if GPS enabled
  beeperRxLost,
  //BEEPER_ACC_CALIBRATION,         // ACC inflight calibration completed confirmation
  beeperSilence,
  //BEEPER_ACC_CALIBRATION_FAIL,    // ACC inflight calibration failed
  beeperSilence,
  //BEEPER_READY_BEEP,              // Ring a tone when GPS is locked and ready
  beeperSilence,
  //BEEPER_MULTI_BEEPS,             // Internal value used by 'beeperConfirmationBeeps()'.
  beeperSilence,
  //BEEPER_DISARM_REPEAT,           // Beeps sounded while stick held in disarm position
  beeperSilence,
  //BEEPER_ARMED,                   // Warning beeps when board is armed (repeats until board is disarmed or throttle is increased)
  beeperSilence,
  //BEEPER_SYSTEM_INIT,             // Initialisation beeps when board is powered on
  beeperSystemInit,
  //BEEPER_USB,                     // Some boards have beeper powered USB connected
  beeperSilence,
  //BEEPER_BLACKBOX_ERASE,          // Beep when blackbox erase completes
  beeperSilence,
  //BEEPER_CRASH_FLIP_MODE,         // Crash flip mode is active
  beeperSilence,
  //BEEPER_CAM_CONNECTION_OPEN,     // When the 5 key simulation stated
  beeperSilence,
  //BEEPER_CAM_CONNECTION_CLOSE,    // When the 5 key simulation stop
  beeperSilence,
  //BEEPER_ALL,                     // Turn ON or OFF all beeper conditions
  beeperSilence,
  //BEEPER_PREFERENCE,              // Save preferred beeper configuration
  beeperSilence
  // BEEPER_ALL and BEEPER_PREFERENCE must remain at the bottom of this enum
};

}
