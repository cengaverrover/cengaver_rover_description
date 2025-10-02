#ifndef __COMMUNICATION_PACKETS_H__
#define __COMMUNICATION_PACKETS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define FRAME_START					(0x31313131)
#define FRAME_END					(0x69696969)

typedef enum {
	set_velocity = 0,
	set_dutycycle = 1,
	set_maximum_dutycycle = 2,
	set_lowpass_fc = 3,
	set_pid_params = 4,
	set_pid_bounds = 5,
	set_ff_params = 6,
	set_max_velocity = 7,
	set_feedback_hz = 8,
	set_control_hz = 9,
	open_loop_enable = 10,
	control_enable = 11
} MobilityCommands;

#pragma pack(push, 1)
typedef struct {
	uint32_t frame_start;

	uint32_t overwrite_pinout;

	uint8_t lpwm_pins[4];
	uint8_t rpwm_pins[4];

	uint8_t encoder_a_pins[4];

	uint8_t motor_swap_dirs[4];

	float max_dutycycle;
	float max_velocity;
	float lowpass_fc;
	float kp;
	float ki;
	float kd;
	float p_bound;
	float i_bound;
	float d_bound;
	float feedback_hz;
	float control_hz;
	uint32_t is_open_loop;

	uint32_t frame_end;
} MobilityInitPacket;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
	uint32_t frame_start;
	int32_t command_id;   // 0: set velocity, 1: set PWM, etc.
	float value1;
	float value2;
	float value3;
	float value4;
	uint32_t frame_end;
} MobilityCommandPacket;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
	uint32_t frame_start;
	float positions[4];
	float velocities[4];
	float pwm_duties[4];
	uint32_t frame_end;
} MobilityFeedbackPacket;
#pragma pack(pop)

    

#ifdef __cplusplus
}
#endif

#endif // __COMMUNICATION_PACKETS_H__