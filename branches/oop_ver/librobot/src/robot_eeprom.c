/*
 * robot_eeprom.c
 *
 *  Created on: Jan 26, 2015
 *      Author: VyLong
 */

#include "librobot\inc\robot_eeprom.h"

void initEEPROM(void)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
	while (ROM_EEPROMInit() != EEPROM_INIT_OK)
		;
	ROM_EEPROMIntDisable(EEPROM_INT_PROGRAM);

	programEEPROM(true);
}

uint32_t getRobotIDInEEPROM(void)
{
	uint32_t ui32RobotID;

	EEPROMRead(&ui32RobotID, EEPROM_ROBOT_ID_ADDRESS, sizeof(&ui32RobotID));

	return ui32RobotID;
//	EEPROMRead(&temp, EEPROM_INTERCEPT, sizeof(&temp));
//	g_f32Intercept = temp / 65536.0;
//
//	EEPROMRead(&temp, EEPROM_SLOPE, sizeof(&temp));
//	g_f32Slope = temp / 65536.0;
}

bool writeWordToEEPROM(uint32_t ui32WordIndex, uint32_t ui32Data)
{
	ui32WordIndex <<= 2;

	EEPROMProgramNonBlocking(ui32Data, ui32WordIndex);

	while(EEPROMStatusGet() != 0)

	EEPROMIntClear(EEPROM_INT_PROGRAM);
	return true;
}

uint32_t readWordFormEEPROM(uint32_t ui32WordIndex)
{
	uint32_t pui32ReadBuffer[1] = { 0 };

	ui32WordIndex <<= 2;

	EEPROMRead(pui32ReadBuffer, ui32WordIndex, sizeof(pui32ReadBuffer));

	return pui32ReadBuffer[0];
}

void programEEPROM(bool bIsUpdate)
{
#ifdef PROGRAM_TABLE_TO_EEPROM
    /*
     * Sine Table Initilization - [0 ---> 1] step = 0.5 degree, offset *32768
     */
	  volatile unsigned short SineTableBlock0[] = {	// EEPROM start address Block 2 = 0x0080
		  0,    286,  572,  858,  1144, 1429, 1715, 2000,
		  2286, 2571, 2856, 3141, 3425, 3709, 3993, 4277,
		  4560, 4843, 5126, 5408, 5690, 5971, 6252, 6533,
		  6813, 7092, 7371, 7650, 7927, 8204, 8481, 8757
	  };

	  volatile unsigned short SineTableBlock1[] = {	// EEPROM start address Block 3 = 0x00c0
		  9032,  9307,  9580,  9854,  10126, 10397, 10668, 10938,
		  11207, 11476, 11743, 12010, 12275, 12540, 12803, 13066,
		  13328, 13589, 13848, 14107, 14365, 14621, 14876, 15131,
		  15384, 15636, 15886, 16136, 16384, 16631, 16877, 17121
	  };

	  volatile unsigned short SineTableBlock2[] = {	// EEPROM start address Block 4 = 0x0100
		  17364, 17606, 17847, 18086, 18324, 18560, 18795, 19028,
		  19261, 19491, 19720, 19948, 20174, 20399, 20622, 20843,
		  21063, 21281, 21498, 21713, 21926, 22138, 22348, 22556,
		  22763, 22967, 23170, 23372, 23571, 23769, 23965, 24159
	  };

	  volatile unsigned short SineTableBlock3[] = {	// EEPROM start address Block 5 = 0x0140
		  24351, 24542, 24730, 24917, 25102, 25285, 25466, 25645,
		  25822, 25997, 26170, 26341, 26510, 26677, 26842, 27005,
		  27166, 27325, 27482, 27636, 27789, 27939, 28088, 28234,
		  28378, 28520, 28660, 28797, 28932, 29066, 29197, 29325
	  };

	  volatile unsigned short SineTableBlock4[] = {	// EEPROM start address Block 6 = 0x0180
		  29452, 29576, 29698, 29818, 29935, 30050, 30163, 30274,
		  30382, 30488, 30592, 30693, 30792, 30888, 30983, 31075,
		  31164, 31251, 31336, 31419, 31499, 31576, 31651, 31724,
		  31795, 31863, 31928, 31991, 32052, 32110, 32166, 32219
	  };

	  volatile unsigned short SineTableBlock5[] = {	// EEPROM start address Block 7 = 0x01c0
		  32270, 32319, 32365, 32408, 32449, 32488, 32524, 32557,
		  32588, 32617, 32643, 32667, 32688, 32707, 32723, 32737,
		  32748, 32757, 32763, 32767, 32768, 0,     0,     0,
		  0, 	 0, 	0,     0,     0,     0,     0,     0
	  };

    /*
     * ArcSine Table Initilization - [0 ---> pi / 2] step = 1/180, offset *32768
     */
	  volatile unsigned short ArcSineTableBlock0[] = {	// EEPROM start address Block 8 = 0x0200
		  0,    182,  364,  546,  728,  910,  1092, 1275,
		  1457, 1639, 1821, 2004, 2186, 2369, 2551, 2734,
		  2917, 3099, 3282, 3465, 3648, 3832, 4015, 4199,
		  4382, 4566, 4750, 4934, 5118, 5302, 5487, 5672
	  };

	  volatile unsigned short ArcSineTableBlock1[] = {	// EEPROM start address Block 9 = 0x0240
		  5857,  6042,  6227,  6412,  6598,  6784,  6970,  7156,
		  7343,  7530,  7717,  7904,  8092,  8280,  8468,  8656,
		  8845,  9034,  9224,  9413,  9603,  9794,  9984,  10175,
		  10367, 10558, 10750, 10943, 11136, 11329, 11523, 11717
	  };

	  volatile unsigned short ArcSineTableBlock2[] = {	// EEPROM start address Block 10 = 0x0280
		  11911, 12106, 12302, 12498, 12694, 12891, 13088, 13286,
		  13485, 13683, 13883, 14083, 14283, 14485, 14686, 14889,
		  15091, 15295, 15499, 15704, 15909, 16116, 16323, 16530,
		  16738, 16947, 17157, 17368, 17579, 17791, 18005, 18218
	  };

	  volatile unsigned short ArcSineTableBlock3[] = {	// EEPROM start address Block 11 = 0x02c0
		  18433, 18649, 18865, 19083, 19301, 19521, 19741, 19963,
		  20185, 20409, 20633, 20859, 21086, 21314, 21544, 21774,
		  22006, 22239, 22474, 22710, 22947, 23186, 23426, 23668,
		  23912, 24157, 24404, 24652, 24902, 25154, 25408, 25664
	  };

	  volatile unsigned short ArcSineTableBlock4[] = {	// EEPROM start address Block 12 = 0x0300
		  25922, 26182, 26444, 26708, 26975, 27244, 27515, 27789,
		  28066, 28345, 28627, 28912, 29200, 29492, 29786, 30084,
		  30386, 30691, 31000, 31313, 31631, 31953, 32280, 32612,
		  32949, 33292, 33640, 33995, 34357, 34725, 35101, 35485
	  };

	  volatile unsigned short ArcSineTableBlock5[] = {	// EEPROM start address Block 13 = 0x0340
		  35878, 36280, 36693, 37116, 37551, 38000, 38463, 38942,
		  39439, 39957, 40498, 41066, 41666, 42303, 42988, 43730,
		  44551, 45481, 46583, 48016, 51472, 0,     0,     0,
		  0, 	 0, 	0,     0,     0,     0,     0,     0
	  };


	  if(bIsUpdate)
	  {
		  /*
		   * program to EEPROM process
		   */
		  uint32_t EEPROMStartAddress = 0x0080;
		  EEPROMProgram((uint32_t*)SineTableBlock0, EEPROMStartAddress, sizeof(SineTableBlock0));
		  EEPROMStartAddress += 64;
		  EEPROMProgram((uint32_t*)SineTableBlock1, EEPROMStartAddress, sizeof(SineTableBlock1));
		  EEPROMStartAddress += 64;
		  EEPROMProgram((uint32_t*)SineTableBlock2, EEPROMStartAddress, sizeof(SineTableBlock2));
		  EEPROMStartAddress += 64;
		  EEPROMProgram((uint32_t*)SineTableBlock3, EEPROMStartAddress, sizeof(SineTableBlock3));
		  EEPROMStartAddress += 64;
		  EEPROMProgram((uint32_t*)SineTableBlock4, EEPROMStartAddress, sizeof(SineTableBlock4));
		  EEPROMStartAddress += 64;
		  EEPROMProgram((uint32_t*)SineTableBlock5, EEPROMStartAddress, sizeof(SineTableBlock5));

		  EEPROMStartAddress += 64;
		  EEPROMProgram((uint32_t*)ArcSineTableBlock0, EEPROMStartAddress, sizeof(ArcSineTableBlock0));
		  EEPROMStartAddress += 64;
		  EEPROMProgram((uint32_t*)ArcSineTableBlock1, EEPROMStartAddress, sizeof(ArcSineTableBlock1));
		  EEPROMStartAddress += 64;
		  EEPROMProgram((uint32_t*)ArcSineTableBlock2, EEPROMStartAddress, sizeof(ArcSineTableBlock2));
		  EEPROMStartAddress += 64;
		  EEPROMProgram((uint32_t*)ArcSineTableBlock3, EEPROMStartAddress, sizeof(ArcSineTableBlock3));
		  EEPROMStartAddress += 64;
		  EEPROMProgram((uint32_t*)ArcSineTableBlock4, EEPROMStartAddress, sizeof(ArcSineTableBlock4));
		  EEPROMStartAddress += 64;
      	  EEPROMProgram((uint32_t*)ArcSineTableBlock5, EEPROMStartAddress, sizeof(ArcSineTableBlock5));
	}

    /*
     * EEPROM table content testing process
     */
    int i;
    volatile unsigned short readSineTableBlock[32] = { 0 };

    EEPROMRead((uint32_t*)readSineTableBlock, 0x0080, sizeof(readSineTableBlock)); // SineTableBlock0
    for(i = 0; i < 32; i++) {
      if (readSineTableBlock[i] != SineTableBlock0[i])
        while(1);
    }
    EEPROMRead((uint32_t*)readSineTableBlock, 0x00C0, sizeof(readSineTableBlock)); // SineTableBlock1
    for(i = 0; i < 32; i++) {
      if (readSineTableBlock[i] != SineTableBlock1[i])
        while(1);
    }
    EEPROMRead((uint32_t*)readSineTableBlock, 0x0100, sizeof(readSineTableBlock)); // SineTableBlock1
    for(i = 0; i < 32; i++) {
      if (readSineTableBlock[i] != SineTableBlock2[i])
        while(1);
    }
    EEPROMRead((uint32_t*)readSineTableBlock, 0x0140, sizeof(readSineTableBlock)); // SineTableBlock2
    for(i = 0; i < 32; i++) {
      if (readSineTableBlock[i] != SineTableBlock3[i])
        while(1);
    }
    EEPROMRead((uint32_t*)readSineTableBlock, 0x0180, sizeof(readSineTableBlock)); // SineTableBlock4
    for(i = 0; i < 32; i++) {
      if (readSineTableBlock[i] != SineTableBlock4[i])
        while(1);
    }
    EEPROMRead((uint32_t*)readSineTableBlock, 0x01C0, sizeof(readSineTableBlock)); // SineTableBlock5
    for(i = 0; i < 32; i++) {
      if (readSineTableBlock[i] != SineTableBlock5[i])
        while(1);
    }


    EEPROMRead((uint32_t*)readSineTableBlock, 0x0200, sizeof(readSineTableBlock)); // ArcSineTableBlock0
    for(i = 0; i < 32; i++) {
      if (readSineTableBlock[i] != ArcSineTableBlock0[i])
        while(1);
    }
    EEPROMRead((uint32_t*)readSineTableBlock, 0x240, sizeof(readSineTableBlock)); // ArcSineTableBlock1
    for(i = 0; i < 32; i++) {
      if (readSineTableBlock[i] != ArcSineTableBlock1[i])
        while(1);
    }
    EEPROMRead((uint32_t*)readSineTableBlock, 0x0280, sizeof(readSineTableBlock)); // ArcSineTableBlock2
    for(i = 0; i < 32; i++) {
      if (readSineTableBlock[i] != ArcSineTableBlock2[i])
        while(1);
    }
    EEPROMRead((uint32_t*)readSineTableBlock, 0x02C0, sizeof(readSineTableBlock)); // ArcSineTableBlock3
    for(i = 0; i < 32; i++) {
      if (readSineTableBlock[i] != ArcSineTableBlock3[i])
        while(1);
    }
    EEPROMRead((uint32_t*)readSineTableBlock, 0x0300, sizeof(readSineTableBlock)); // ArcSineTableBlock4
    for(i = 0; i < 32; i++) {
      if (readSineTableBlock[i] != ArcSineTableBlock4[i])
        while(1);
    }
    EEPROMRead((uint32_t*)readSineTableBlock, 0x0340, sizeof(readSineTableBlock)); // ArcSineTableBlock5
    for(i = 0; i < 32; i++) {
      if (readSineTableBlock[i] != ArcSineTableBlock5[i])
        while(1);
    }
#endif
}

