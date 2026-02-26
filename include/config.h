#ifndef CONFIG_H
#define CONFIG_H

// ===== FIRMWARE VERSION =====
#define FIRMWARE_VERSION 0x000C

#define RESET_CALIBRATION_FLAG false
#define CRYPTO_DEBUG_MODE false
#ifndef DISABLE_CC1101
#define DISABLE_CC1101 false
#endif

// ----- Battery configuration -----
#define BATTERY_ADC_PIN 36
#define VOLTAGE_DIVIDER_RATIO 2
#define BATTERY_READ_INTERVAL_MS 1000
#define BATTERY_MIN_MILLIVOLTS 3800  // Minimum battery voltage (mV) to start balancing
// Battery encoding for transmission: 1 byte (0-255) maps to 2500-4200mV
#define BATTERY_ENCODE_MIN_MV 2500   // 0 = 2500mV
#define BATTERY_ENCODE_MAX_MV 4200   // 255 = 4200mV
// Low battery warning configuration (two independent conditions, either triggers yellow LEDs)
// Condition 1: voltage continuously below threshold for timeout period
#define BATTERY_LOW_WARNING_THRESHOLD_MV 3800         // Voltage threshold (mV) for sustained low battery warning
#define BATTERY_LOW_WARNING_TIMEOUT_MS 40000          // Time (ms) voltage must stay below threshold to trigger warning
// Condition 2: voltage dropped below critical threshold even once within a time window
#define BATTERY_DISCHARGE_WARNING_THRESHOLD_MV 3100   // If voltage drops below this (mV) even once, trigger warning
#define BATTERY_DISCHARGE_WARNING_WINDOW_MS 300000    // Time window (ms) to remember a low-voltage event (5 minutes)

// ----- Temperature sensor configuration -----
#define TEMP_SENSOR_READ_INTERVAL_MS 3000  // Interval between reading each sensor (ms)
#define TEMP_LIMIT_MAIN_BOARD_CELSIUS 55   // Maximum main board temperature before thermal shutdown
#define TEMP_LIMIT_MOTOR_CELSIUS 45        // Maximum motor board temperature before thermal shutdown
#define TEMP_SENSOR_REQUIRED false         // If true: halt on boot if sensors missing, block motors if sensors stop responding
                                           // If false: warnings only, robot operates without temperature protection
#define TEMP_SENSOR_FAILURE_TIMEOUT_MS 10000  // Time (ms) without a successful read before declaring sensor failure

// ----- Crypto configuration -----
#define ATECC_I2C_ADDR 0x60
#define ATECC_KEY_SLOT 0

// ----- Shared I2C configuration -----
#define SHARED_I2C_SDA_PIN 21
#define SHARED_I2C_SCL_PIN 22
#define SHARED_I2C_CLOCK 200000 // 200kHz, 400kHz is too fast for crypto chip

#define PRIMARY_I2C_CLOCK 400000 // 400kHz

// ----- LED configuration -----
#define LED_DATA_PIN 18
#define LED_NUM_LEDS 12
#define LED_REFRESH_INTERVAL_MS 3000  // Periodic forced LED refresh to correct EMI glitches (ms)

enum LEDPositions {
    LED_R_EYE = 0,      // R Eye
    LED_L_EYE = 1,      // L Eye
    LED_L_TOP = 2,      // L Top
    LED_L_FTOP = 3,     // L FTop
    LED_L_FBOT = 4,     // L FBot
    LED_L_BOT = 5,      // L Bot
    LED_L_BACK = 6,     // L Back
    LED_R_TOP = 7,      // R Top
    LED_R_FTOP = 8,     // R FTop
    LED_R_FBOT = 9,     // R FBot
    LED_R_BOT = 10,     // R Bot
    LED_R_BACK = 11     // R Back
};

// ----- Motor configuration -----
#define MOTOR_PP 7
#define MOTOR_R 20.0
#define MOTOR_VOLTAGE_LIMIT 9

#define MOTOR1_PHASE_A_PIN 25
#define MOTOR1_PHASE_B_PIN 33
#define MOTOR1_PHASE_C_PIN 32
#define MOTOR1_ENABLE_PIN 13

#define MOTOR2_PHASE_A_PIN 14
#define MOTOR2_PHASE_B_PIN 27
#define MOTOR2_PHASE_C_PIN 26
#define MOTOR2_ENABLE_PIN 12

// ----- Controller configuration -----
#define CONTROLLER_DEADZONE 30

// ----- Central Controller / BLE controller input timeout -----
#define BLE_CONTROLLER_INPUT_TIMEOUT_MS 1000  // BLE controller input is valid for 1 second

// ----- Scan configuration (close-proximity low-power detection) -----
#define SCAN_BURST_DURATION 1500      // ms - duration of scan burst (LEDs magenta)
#define SCAN_COOLDOWN_DURATION 3000   // ms - cooldown between scan bursts
#define SCAN_SEND_INTERVAL 200       // ms - interval between scan RF messages during burst
#define SCAN_REPORT_WINDOW_MS 3000   // ms - how long a scan event is reported in pings & dedup window
#define SCAN_SLOTS_IN_PING 4         // number of scan result slots in PingMessage

// ----- Balance Controller PID parameters -----
#define PID_STB_P 0.46
#define PID_STB_I 1.3
#define PID_STB_D 0.009
#define PID_STB_RAMP 0
#define PID_STB_INTEGRAL_LIMIT 13
#define PID_STB_LIMIT 1000

#define PID_STEERING_P 0.02
#define PID_STEERING_I 0
#define PID_STEERING_D 0.000
#define PID_STEERING_RAMP 0
#define PID_STEERING_INTEGRAL_LIMIT 0
#define PID_STEERING_LIMIT 2000

#define PID_VEL_P 0.015
#define PID_VEL_I 0.001
#define PID_VEL_D 0
#define PID_VEL_RAMP 0
#define PID_VEL_INTEGRAL_LIMIT 0.01
#define PID_VEL_LIMIT 0.4

#define PID_CRAWL_P 0.004
#define PID_CRAWL_I 0
#define PID_CRAWL_D 0.0
#define PID_CRAWL_RAMP 0
#define PID_CRAWL_INTEGRAL_LIMIT 0
#define PID_CRAWL_LIMIT 2000

// ----- Balance Controller Low Pass Filter parameters -----
#define LPF_PID_STEERING_TF 0.4
#define LPF_VEL_CTR_TF 0.08

// ----- Balance Controller velocity and steering constants -----
#define BASE_MAX_CRAWL_VELOCITY 15.0
#define BASE_MAX_BALANCE_VELOCITY_FORWARD 30.0
#define BASE_MAX_BOOST_VELOCITY_FORWARD 40.0
#define BASE_MAX_BALANCE_VELOCITY_BACKWARD 20.0
#define BASE_MIN_STEERING_SENSITIVITY 20.0
#define BASE_MAX_STEERING_SENSITIVITY 50.0
#define BASE_POWER_BOOST_AXIS_EXPONENT 1.1

// ----- Balance Controller target pitch constants -----
#define DEFAULT_TARGET_PITCH_CONSTANT -0.61
#define MAX_TARGET_PITCH_DEVIATION_CONSTANT 0.8

// ----- CC1101 Hardware pins (your wiring) -----
#define SCK_PIN         2
#define MISO_PIN        35
#define MOSI_PIN        15
#define CS_PIN          4
#define GDO0_PIN        9
#define GDO2_PIN        34

// ----- Statistics configuration -----
#define STATS_INTERVAL_MS 30000  // Print statistics every 30000ms

// ----- CC1101 RF addresses -----
#define RF_BROADCAST_ADDR 0xFFFFFFFF

// ----- CC1101 Data structure validation -----
#define DATA_STRUCTURE_VALIDATION_VALID 0
#define DATA_STRUCTURE_VALIDATION_INCORRECT_LENGTH 1
#define DATA_STRUCTURE_VALIDATION_INCORRECT_START_CANARY 2
#define DATA_STRUCTURE_VALIDATION_INCORRECT_END_CANARY 3
#define DATA_STRUCTURE_VALIDATION_UNKNOWN_TYPE 4

#define DATA_STRUCTURE_CANARY_START 0xAB
#define DATA_STRUCTURE_CANARY_END 0xCD

// ----- CC1101 configuration -----
#ifndef CC1101_FREQUENCY
#define CC1101_FREQUENCY           869.525
#endif
#define CC1101_BIT_RATE            38.4    // kbps
#define CC1101_FREQUENCY_DEVIATION 20.0     // kHz
#define CC1101_RX_BANDWIDTH        135.0    // kHz
#define CC1101_TX_POWER            10
#define CC1101_PREAMBLE_LENGTH     64       // bits 

// ----- CC1101 Data types (new protocol) -----
enum DataType {
  DATA_TYPE_PING = 0x01,                 // Robot sends: crypto_id, robot_battery (encoded mV), controller_type, controller_battery%
  DATA_TYPE_UPDATE_STATUS = 0x02,        // Central sends: crypto_id, leds[36], speed_coeff, torque_coeff
  DATA_TYPE_SCAN = 0x03                  // Robot sends: crypto_id (low power)
};

// ----- BLE Commands -----
#define BLE_CMD_KEEPALIVE            0x00  // Keepalive/heartbeat from central

// ----- BLE Subprogram Commands -----
#define BLE_CMD_LIST_SUBPROGRAMS     0x20  // Get list of all subprograms
#define BLE_CMD_CREATE_SUBPROGRAM    0x21  // Create a new blank subprogram
#define BLE_CMD_RENAME_SUBPROGRAM    0x22  // Rename existing subprogram
#define BLE_CMD_UPDATE_SUBPROGRAM    0x23  // Update subprogram data
#define BLE_CMD_SET_SUBPROGRAM_BTN   0x24  // Assign subprogram to button (1-5)
#define BLE_CMD_DELETE_SUBPROGRAM    0x26  // Delete a subprogram
#define BLE_CMD_GET_SUBPROGRAM       0x27  // Get subprogram data by ID
#define BLE_CMD_GET_PITCH_CONSTANT   0x28  // Get current target pitch constant
#define BLE_CMD_SET_PITCH_CONSTANT   0x29  // Set target pitch constant
#define BLE_CMD_RECALIBRATE          0x2A  // Clear calibration and reboot
#define BLE_CMD_READ_SUBPROGRAM_DATA 0x2B  // Read chunk of subprogram bytecode
#define BLE_CMD_START_SUBPROGRAM_WRITE 0x2C  // Start chunked subprogram write
#define BLE_CMD_WRITE_SUBPROGRAM_DATA 0x2D  // Write chunk of subprogram bytecode
#define BLE_CMD_FINISH_SUBPROGRAM_WRITE 0x2E  // Finish chunked subprogram write
#define BLE_CMD_GET_CONTROLLER_MAPPING  0x30  // Get controller mapping for a type
#define BLE_CMD_SET_CONTROLLER_MAPPING  0x31  // Set controller mapping for a type
#define BLE_CMD_RESET_CONTROLLER_MAPPING 0x32 // Reset to default mapping
#define BLE_CMD_GET_STEERING_SENSITIVITY 0x33 // Get steering sensitivity values
#define BLE_CMD_SET_STEERING_SENSITIVITY 0x34 // Set steering sensitivity values
#define BLE_CMD_GET_VELOCITY_LIMITS      0x35 // Get velocity limits (balance/boost forward)
#define BLE_CMD_SET_VELOCITY_LIMITS      0x36 // Set velocity limits (balance/boost forward)
#define BLE_CMD_CONTROLLER_INPUT         0x40 // BLE controller input (joystick/buttons from app)
#define BLE_CMD_ENABLE_CONTROLLER_STREAM 0x42 // Enable controller input streaming (robot -> app)

// Controller input streaming configuration
#define CONTROLLER_STREAM_DURATION_MS    1000  // Stream for 1 second after enable command
#define CONTROLLER_STREAM_INTERVAL_MS    100   // Send controller state every 100ms

// ----- BLE Melody Commands -----
#define BLE_CMD_READ_MELODY_DATA         0x51  // Read chunk of melody data (returns metadata on first read)
#define BLE_CMD_START_MELODY_WRITE       0x52  // Start chunked melody write
#define BLE_CMD_WRITE_MELODY_DATA        0x53  // Write chunk of melody data
#define BLE_CMD_FINISH_MELODY_WRITE      0x54  // Finish chunked melody write
#define BLE_CMD_RESET_MELODY             0x55  // Reset to default melody
#define BLE_CMD_GET_MELODY_AMPLITUDE     0x56  // Get melody amplitude (current and default)
#define BLE_CMD_SET_MELODY_AMPLITUDE     0x57  // Set melody amplitude

// ----- BLE Crypto Commands -----
#define BLE_CMD_GET_PUBLIC_KEY           0x60  // Get ECDSA public key (64 bytes)
#define BLE_CMD_SIGN_MESSAGE             0x61  // Sign a message with ECDSA

#define BLE_CHUNK_SIZE 40  // Max bytes per chunk for BLE transfers

// ----- CC1101 Message sizes (without canaries) -----
// PING: data_type(1) + crypto_id(4) + robot_battery_enc(1) + controller_type(1) + controller_battery(1) + robot_name(10) + temp_main(1) + temp_right(1) + temp_left(1) + firmware_version(2) + is_relayed(1) + leds_encoded(10) + speed_torque(1) + last_fall_ms_ago(4) + scan_crypto_ids(4*4=16) + checksum(2) = 57
#define MSG_SIZE_PING 57

// UPDATE_STATUS (Multi-Robot format): data_type(1) + N * (crypto_id(4) + leds(10) + speed_torque(1)) + checksum(2)
// Each robot entry: 4 + 10 + 1 = 15 bytes
// LEDs are indices 2-11 (excluding eyes 0,1), each 1 byte:
//   - First bit = 0: LED off (skip, don't change)
//   - First bit = 1: LED on, lower 7 bits = R*25 + G*5 + B (0-124)
//   - R,G,B are 0-4, mapping to: 0, 63, 127, 191, 255
// Speed/Torque: upper 4 bits = speed (0-15 -> 0.0-1.0), lower 4 bits = torque (0-15 -> 0.0-1.0)
// NOTE: BLE can send up to 16 robots per packet, but RF forwarding splits into smaller packets
//       due to CC1101's 64-byte FIFO limit (max 3 robots per RF packet)
#define MSG_SIZE_UPDATE_STATUS_ROBOT_ENTRY 15
#define MSG_SIZE_UPDATE_STATUS_MIN (1 + MSG_SIZE_UPDATE_STATUS_ROBOT_ENTRY + 2)  // 1 robot: 18 bytes
#define MSG_SIZE_UPDATE_STATUS_MAX (1 + 16 * MSG_SIZE_UPDATE_STATUS_ROBOT_ENTRY + 2)  // 16 robots: 243 bytes
#define UPDATE_STATUS_MAX_ROBOTS 16
#define UPDATE_STATUS_NUM_LEDS 10  // LEDs 2-11 (excluding eyes)

// SCAN: data_type(1) + timestamp(4) + crypto_id(4) + checksum(2) = 11
#define MSG_SIZE_SCAN 11
// Maximum message size (UPDATE_STATUS_MAX is 243 for 16 robots, crypto responses need 66 bytes)
#define MSG_SIZE_MAX 243

// ----- CC1101 RF configuration -----
#define RF_POWER_SHORT_RANGE (10)   // dBm
#define RF_POWER_LONG_RANGE (10)   // dBm

// ----- CC1101 RF communication -----
// PING = robot status broadcast (battery, temperature, name, controller info)
#define PING_INTERVAL 750     // ms - interval between ping broadcasts
#define JITTER_FRACTION 0.2    // fraction of interval added as random jitter to prevent collisions
#define TRIGGERED_PING_DELAY_MS 100  // ms - delay before sending triggered ping after receiving UPDATE_STATUS
// SCAN = close-proximity detection (low power RF)
#define RSSI_SCAN_THRESHOLD -65 // rssi threshold for scan detection
#define FALL_CONFIRM_MS 150        // ms - sustained tilt required before declaring fallen (debounce)
#define RF_TX_RETRIES 3

// ----- Melody configuration -----
#define MELODY_AMPLITUDE 0.5
#define MELODY_MAX_NOTES 64               // Maximum number of notes in a custom melody
#define MELODY_MAX_DATA_SIZE (MELODY_MAX_NOTES * 8)  // Each Note is 8 bytes (float f + float d)
#define MELODY_PREFS_NAMESPACE "melody"
#define MELODY_SPIFFS_PATH "/melody/custom.bin"

// ----- BLE configuration -----
#define MAX_BLE_NAME_LEN 16
#define BLE_NAME_PREFIX "RBM_"

// ----- Serial protocol configuration -----
#define SERIAL_START_BYTE       0xAA
#define SERIAL_CMD_GET_INFO     0x01  // Request version and name
#define SERIAL_CMD_SET_NAME     0x02  // Set robot name (followed by length byte and name)
#define SERIAL_CMD_GET_PUBLIC_KEY 0x03 // Get ECDSA public key (64 bytes)
#define SERIAL_CMD_SIGN_MESSAGE  0x04  // Sign a message with ECDSA
#define ROBOT_NAME_MIN_LEN      3   // Minimum name length
#define ROBOT_NAME_MAX_LEN      10  // Maximum name length
#define SIGN_MSG_MAX_LEN        60  // Maximum message length for signing (BLE and Serial)

// ----- Subprogram configuration -----
#define SUBPROGRAM_WHEEL_DIAMETER_CM    4.45f   // Wheel diameter in cm
#define SUBPROGRAM_WHEEL_BASE_CM        6.3f   // Distance between wheels in cm
#define SUBPROGRAM_MAX_PROGRAM_SIZE     512     // Maximum program size in bytes per subprogram
#define SUBPROGRAM_MAX_COUNT            8       // Maximum number of subprograms (including default)
#define SUBPROGRAM_NAME_MAX_LEN         16      // Maximum subprogram name length
#define SUBPROGRAM_PREFS_NAMESPACE      "subprog"
#define SUBPROGRAM_DEFAULT_ID           1       // ID of the default (hardcoded) subprogram

#endif // CONFIG_H
