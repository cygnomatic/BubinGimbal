#ifndef NUC_COMMU_H
#define NUC_COMMU_H

#include "struct_typedef.h"
#define SOF1 0xA5
#define SOF2 0x5A

#define NUCINFO_RX_BUF_NUM 34u
#define NUCINFO_FRAME_LENGTH 17u

enum Robot_Colors
{
	COLOR_RED,
	COLOR_BLUE
}; 

enum Task_Types
{
	NUC_SHOOT_ENEMY_CAR,
	NUC_SHOOT_ENEMY_SENTRY,
	NUC_SHOOT_ENEMY_BASE,
	NUC_WINDMILL
};

enum Robot_ID
{
	UNKNOWN,
	HERO_1,
	ENGINEER_2,
	STANDARD_3,
	STANDARD_4,
	STANDARD_5,
	GUARD_6,
	SENTRY_7
};

enum Auto_Num
{
	No,
	Normal,
	Energy,
	lob
};

enum Attack_mode
{
	Attack_forbidden,
	Attack_free
};

typedef union
{
	fp64 data;
	uint8_t bytes[8];
} RxFP64Data;

typedef union
{
	fp32 data;
	uint8_t bytes[4];
} RxFP32Data;

// 通讯协议
typedef struct
{
	RxFP32Data yaw;
	RxFP32Data pitch;
	uint8_t is_fire;
	RxFP32Data confidence;
} toSTM32_t;

extern void nuc_control_init(void);

extern const toSTM32_t *get_nuc_control_point(void);

void setNUCValid(void);
void setNUCInvalid(void);
uint32_t getNUCLastValidTime_ms(void);
int NUCIsValid(void);
extern void Encode(uint8_t* RawData, fp64 gimbal_yaw, fp64 gimbal_pitch , fp64 gimbal_roll, enum Robot_Colors self_color, enum Robot_ID self_id,  enum Auto_Num auto_num,enum Attack_mode attack_mode);
void Encode1(uint8_t* RawData, fp32 gimbal_yaw, fp32 gimbal_pitch , fp32 gimbal_roll);
extern void Encode2(uint8_t* RawData, enum Robot_Colors self_color, enum Robot_ID self_id, enum Auto_Num auto_num, enum Attack_mode attack_mode);

#endif // !NUC_COMMU_H
