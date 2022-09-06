#pragma once

#ifdef VESCENCODER_EXPORTS
#define VESCENCODER_API __declspec(dllexport)
#else
#define VESCENCODER_API __declspec(dllimport)
#endif


typedef unsigned char uint8_t;
typedef unsigned   uint32_t;
typedef short  int16_t;
typedef unsigned short  uint16_t;

extern "C" VESCENCODER_API unsigned short crc16(unsigned char* buf, unsigned int len);

extern "C" VESCENCODER_API extern void sendAlive(uint8_t * array);

extern "C" VESCENCODER_API extern int setRpm(uint8_t * array, int rpm);

extern "C" VESCENCODER_API extern int setCurrent(uint8_t * array, int current);

extern "C" VESCENCODER_API void bufferAppendFloat32(uint8_t * buffer, float number, float scale, int* index);

extern "C" VESCENCODER_API void bufferAppendInt32(uint8_t * buffer, int number, int* index);

extern "C" VESCENCODER_API uint8_t * buildPacket(uint8_t * data, unsigned int len);

extern "C" VESCENCODER_API unsigned short crc16(unsigned char* buf, unsigned int len);

