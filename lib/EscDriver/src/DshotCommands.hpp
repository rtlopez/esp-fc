#pragma once

#include "Arduino.h"
#include "EscDriver.h"

// The official DShot Commands
enum DshotCommand
{
    DSHOT_CMD_MOTOR_STOP = 0,                    // Currently not implemented - STOP Motors
    DSHOT_CMD_BEEP1,                             // Wait at least length of beep (380ms) before next command
    DSHOT_CMD_BEEP2,                             // Wait at least length of beep (380ms) before next command
    DSHOT_CMD_BEEP3,                             // Wait at least length of beep (400ms) before next command
    DSHOT_CMD_BEEP4,                             // Wait at least length of beep (400ms) before next command
    DSHOT_CMD_BEEP5,                             // Wait at least length of beep (400ms) before next command
    DSHOT_CMD_ESC_INFO,                          // Currently not implemented
    DSHOT_CMD_SPIN_DIRECTION_1,                  // Need 6x, no wait required
    DSHOT_CMD_SPIN_DIRECTION_2,                  // Need 6x, no wait required
    DSHOT_CMD_3D_MODE_OFF,                       // Need 6x, no wait required
    DSHOT_CMD_3D_MODE_ON,                        // Need 6x, no wait required
    DSHOT_CMD_SETTINGS_REQUEST,                  // Currently not implemented
    DSHOT_CMD_SAVE_SETTINGS,                     // Need 6x, wait at least 12ms before next command
    DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,        // Need 6x, no wait required 
    DSHOT_CMD_SPIN_DIRECTION_REVERSED = 21,      // Need 6x, no wait required
    DSHOT_CMD_LED0_ON,                           // Currently not implemented
    DSHOT_CMD_LED1_ON,                           // Currently not implemented
    DSHOT_CMD_LED2_ON,                           // Currently not implemented
    DSHOT_CMD_LED3_ON,                           // Currently not implemented
    DSHOT_CMD_LED0_OFF,                          // Currently not implemented
    DSHOT_CMD_LED1_OFF,                          // Currently not implemented
    DSHOT_CMD_LED2_OFF,                          // Currently not implemented
    DSHOT_CMD_LED3_OFF,                          // Currently not implemented
    DSHOT_CMD_36,                                // Not yet assigned
    DSHOT_CMD_37,                                // Not yet assigned
    DSHOT_CMD_38,                                // Not yet assigned
    DSHOT_CMD_39,                                // Not yet assigned
    DSHOT_CMD_40,                                // Not yet assigned
    DSHOT_CMD_41,                                // Not yet assigned
    DSHOT_CMD_SIGNAL_LINE_TEMPERATURE_TELEMETRY, // No wait required
    DSHOT_CMD_SIGNAL_LINE_VOLTAGE_TELEMETRY,     // No wait required
    DSHOT_CMD_SIGNAL_LINE_CURRENT_TELEMETRY,     // No wait required
    DSHOT_CMD_SIGNAL_LINE_CONSUMPTION_TELEMETRY, // No wait required
    DSHOT_CMD_SIGNAL_LINE_ERPM_TELEMETRY,        // No wait required
    DSHOT_CMD_SIGNAL_LINE_ERPM_PERIOD_TELEMETRY, // No wait required (also command 47)
    DSHOT_CMD_MAX = 47
};
