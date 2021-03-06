/*
 * debug_tool.h
 *
 *  Created on: May 22, 2022
 *      Author: SSS
 */

#ifndef INC_DEBUG_TOOL_H_
#define INC_DEBUG_TOOL_H_

#ifdef __cplusplus
extern "C" {
#endif

#define DEBUG_LOG_ENABLE 1
#define LOG_LEVEL 2

enum {
	LOW = 0,
	MID = 1,
	HIGH = 2,
};

#if DEBUG_LOG_ENABLE
#define OS_LOG(level, format, ...) do {\
	if (level >= LOG_LEVEL) {\
		printf("[%3f]: " format, (float)HAL_GetTick()/1000,  ##__VA_ARGS__);\
	}\
} while (0)
#else
#define OS_LOG(level, format, ...)
#endif

enum channel {
	CHANNEL_0,
	CHANNEL_1,
	CHANNEL_2,
	CHANNEL_3,
	CHANNEL_4,
	CHANNEL_5,
	CHANNEL_6,
	CHANNEL_7,
	CHANNEL_8,
	CHANNEL_9,
	CHANNEL_MAX
};

struct db_data {
	enum channel chl;
	unsigned char isFloat;
	union {
		void *data;
		int iData;
		float fData;
	};
};

void db_creat(struct db_data * data);
void db_data_update(struct db_data * data, void* value);
void db_show_data(void);


#ifdef __cplusplus
}
#endif
#endif /* INC_DEBUG_TOOL_H_ */
