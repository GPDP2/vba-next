#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stddef.h>

#include "system.h"
#include "globals.h"

#ifdef __CELLOS_LV2__
#include <ppu_intrinsics.h>
#endif

#include "port.h"
#include "gba.h"
#include "memory.h"
#include "sound.h" 

#ifdef ELF
#include "elf.h"
#endif

/*============================================================
	GBA INLINE
============================================================ */

#define UPDATE_REG(address, value)	WRITE16LE(((u16 *)&ioMem[address]),value);
#define ARM_PREFETCH_NEXT		cpuPrefetch[1] = CPUReadMemoryQuick(bus.armNextPC+4);
#define THUMB_PREFETCH_NEXT		cpuPrefetch[1] = CPUReadHalfWordQuick(bus.armNextPC+2);

#define ARM_PREFETCH \
  {\
    cpuPrefetch[0] = CPUReadMemoryQuick(bus.armNextPC);\
    cpuPrefetch[1] = CPUReadMemoryQuick(bus.armNextPC+4);\
  }

#define THUMB_PREFETCH \
  {\
    cpuPrefetch[0] = CPUReadHalfWordQuick(bus.armNextPC);\
    cpuPrefetch[1] = CPUReadHalfWordQuick(bus.armNextPC+2);\
  }
 
#ifdef USE_SWITICKS
extern int SWITicks;
#endif
static int cpuNextEvent = 0;
static bool holdState = false;
static u32 cpuPrefetch[2];
static int cpuTotalTicks = 0;
static u8 memoryWait[16] =
  { 0, 0, 2, 0, 0, 0, 0, 0, 4, 4, 4, 4, 4, 4, 4, 0 };
static u8 memoryWaitSeq[16] =
  { 0, 0, 2, 0, 0, 0, 0, 0, 2, 2, 4, 4, 8, 8, 4, 0 };
static u8 memoryWait32[16] =
  { 0, 0, 5, 0, 0, 1, 1, 0, 7, 7, 9, 9, 13, 13, 4, 0 };
static u8 memoryWaitSeq32[16] =
  { 0, 0, 5, 0, 0, 1, 1, 0, 5, 5, 9, 9, 17, 17, 4, 0 };

static const int table [0x40] =
{
		0xFF10,     0,0xFF11,0xFF12,0xFF13,0xFF14,     0,     0,
		0xFF16,0xFF17,     0,     0,0xFF18,0xFF19,     0,     0,
		0xFF1A,     0,0xFF1B,0xFF1C,0xFF1D,0xFF1E,     0,     0,
		0xFF20,0xFF21,     0,     0,0xFF22,0xFF23,     0,     0,
		0xFF24,0xFF25,     0,     0,0xFF26,     0,     0,     0,
		     0,     0,     0,     0,     0,     0,     0,     0,
		0xFF30,0xFF31,0xFF32,0xFF33,0xFF34,0xFF35,0xFF36,0xFF37,
		0xFF38,0xFF39,0xFF3A,0xFF3B,0xFF3C,0xFF3D,0xFF3E,0xFF3F,
};

static int coeff[32] = {0,1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 
			11, 12, 13, 14, 15, 16, 16, 16, 16,
			16, 16, 16, 16, 16, 16, 16, 16, 16,
			16, 16, 16};

static u8 biosProtected[4];
static u8 cpuBitsSet[256];

static void CPUSwitchMode(int mode, bool saveState, bool breakLoop);
static bool N_FLAG = 0;
static bool C_FLAG = 0;
static bool Z_FLAG = 0;
static bool V_FLAG = 0;
static bool armState = true;
static bool armIrqEnable = true;
static int armMode = 0x1f;

typedef enum
{
  REG_DISPCNT = 0x000,
  REG_DISPSTAT = 0x002,
  REG_VCOUNT = 0x003,
  REG_BG0CNT = 0x004,
  REG_BG1CNT = 0x005,
  REG_BG2CNT = 0x006,
  REG_BG3CNT = 0x007,
  REG_BG0HOFS = 0x08,
  REG_BG0VOFS = 0x09,
  REG_BG1HOFS = 0x0A,
  REG_BG1VOFS = 0x0B,
  REG_BG2HOFS = 0x0C,
  REG_BG2VOFS = 0x0D,
  REG_BG3HOFS = 0x0E,
  REG_BG3VOFS = 0x0F,
  REG_BG2PA = 0x10,
  REG_BG2PB = 0x11,
  REG_BG2PC = 0x12,
  REG_BG2PD = 0x13,
  REG_BG2X_L = 0x14,
  REG_BG2X_H = 0x15,
  REG_BG2Y_L = 0x16,
  REG_BG2Y_H = 0x17,
  REG_BG3PA = 0x18,
  REG_BG3PB = 0x19,
  REG_BG3PC = 0x1A,
  REG_BG3PD = 0x1B,
  REG_BG3X_L = 0x1C,
  REG_BG3X_H = 0x1D,
  REG_BG3Y_L = 0x1E,
  REG_BG3Y_H = 0x1F,
  REG_WIN0H = 0x20,
  REG_WIN1H = 0x21,
  REG_WIN0V = 0x22,
  REG_WIN1V = 0x23,
  REG_WININ = 0x24,
  REG_WINOUT = 0x25,
  REG_BLDCNT = 0x28,
  REG_BLDALPHA = 0x29,
  REG_BLDY = 0x2A,
  REG_TM0D = 0x80,
  REG_TM0CNT = 0x81,
  REG_TM1D = 0x82,
  REG_TM1CNT = 0x83,
  REG_TM2D = 0x84,
  REG_TM2CNT = 0x85,
  REG_TM3D = 0x86,
  REG_TM3CNT = 0x87,
  REG_P1 = 0x098,
  REG_P1CNT = 0x099,
  REG_RCNT = 0x9A,
  REG_IE = 0x100,
  REG_IF = 0x101,
  REG_IME = 0x104,
  REG_HALTCNT = 0x180
} hardware_register;

static u16 io_registers[1024 * 16];

static u16 MOSAIC;

static u16 BG2X_L   = 0x0000;
static u16 BG2X_H   = 0x0000;
static u16 BG2Y_L   = 0x0000;
static u16 BG2Y_H   = 0x0000;
static u16 BG3X_L   = 0x0000;
static u16 BG3X_H   = 0x0000;
static u16 BG3Y_L   = 0x0000;
static u16 BG3Y_H   = 0x0000;
static u16 BLDMOD   = 0x0000;
static u16 COLEV    = 0x0000;
static u16 COLY     = 0x0000;
static u16 DM0SAD_L = 0x0000;
static u16 DM0SAD_H = 0x0000;
static u16 DM0DAD_L = 0x0000;
static u16 DM0DAD_H = 0x0000;
static u16 DM0CNT_L = 0x0000;
static u16 DM0CNT_H = 0x0000;
static u16 DM1SAD_L = 0x0000;
static u16 DM1SAD_H = 0x0000;
static u16 DM1DAD_L = 0x0000;
static u16 DM1DAD_H = 0x0000;
static u16 DM1CNT_L = 0x0000;
static u16 DM1CNT_H = 0x0000;
static u16 DM2SAD_L = 0x0000;
static u16 DM2SAD_H = 0x0000;
static u16 DM2DAD_L = 0x0000;
static u16 DM2DAD_H = 0x0000;
static u16 DM2CNT_L = 0x0000;
static u16 DM2CNT_H = 0x0000;
static u16 DM3SAD_L = 0x0000;
static u16 DM3SAD_H = 0x0000;
static u16 DM3DAD_L = 0x0000;
static u16 DM3DAD_H = 0x0000;
static u16 DM3CNT_L = 0x0000;
static u16 DM3CNT_H = 0x0000;

static u8 timerOnOffDelay = 0;
static u16 timer0Value = 0;
static u32 dma0Source = 0;
static u32 dma0Dest = 0;
static u32 dma1Source = 0;
static u32 dma1Dest = 0;
static u32 dma2Source = 0;
static u32 dma2Dest = 0;
static u32 dma3Source = 0;
static u32 dma3Dest = 0;
void (*cpuSaveGameFunc)(u32,u8) = flashSaveDecide;
static bool fxOn = false;
static bool windowOn = false;

static int cpuDmaTicksToUpdate = 0;

static const u32 TIMER_TICKS[4] = {0, 6, 8, 10};

static const u8 gamepakRamWaitState[4] = { 4, 3, 2, 8 };

static int IRQTicks = 0;
static bool intState = false;

static bus_t bus;
static graphics_t graphics;

static memoryMap map[256];
static int clockTicks;

static int romSize = 0x2000000;
static u32 line[6][240];
static bool gfxInWin[2][240];
static int lineOBJpixleft[128];
uint64_t joy = 0;

static int gfxBG2Changed = 0;
static int gfxBG3Changed = 0;

static int gfxBG2X = 0;
static int gfxBG2Y = 0;
static int gfxBG3X = 0;
static int gfxBG3Y = 0;

static bool ioReadable[0x400];
static int gbaSaveType = 0; // used to remember the save type on reset

//static int gfxLastVCOUNT = 0;

// Waitstates when accessing data

#define DATATICKS_ACCESS_BUS_PREFETCH(address, value) \
	int addr = (address >> 24) & 15; \
	if ((addr>=0x08) || (addr < 0x02)) \
	{ \
		bus.busPrefetchCount=0; \
		bus.busPrefetch=false; \
	} \
	else if (bus.busPrefetch) \
	{ \
		int waitState = value; \
		waitState = (1 & ~waitState) | (waitState & waitState); \
		bus.busPrefetchCount = ((bus.busPrefetchCount+1)<<waitState) - 1; \
	}

/* Waitstates when accessing data */

#define DATATICKS_ACCESS_32BIT(address)  (memoryWait32[(address >> 24) & 15])
#define DATATICKS_ACCESS_32BIT_SEQ(address) (memoryWaitSeq32[(address >> 24) & 15])
#define DATATICKS_ACCESS_16BIT(address) (memoryWait[(address >> 24) & 15])
#define DATATICKS_ACCESS_16BIT_SEQ(address) (memoryWaitSeq[(address >> 24) & 15])

// Waitstates when executing opcode
static INLINE int codeTicksAccess(u32 address, u8 bit32) // THUMB NON SEQ
{
	int addr, ret;

	addr = (address>>24) & 15;

	if (unsigned(addr - 0x08) <= 5)
	{
		if (bus.busPrefetchCount&0x1)
		{
			if (bus.busPrefetchCount&0x2)
			{
				bus.busPrefetchCount = ((bus.busPrefetchCount&0xFF)>>2) | (bus.busPrefetchCount&0xFFFFFF00);
				return 0;
			}
			bus.busPrefetchCount = ((bus.busPrefetchCount&0xFF)>>1) | (bus.busPrefetchCount&0xFFFFFF00);
			return memoryWaitSeq[addr]-1;
		}
	}
	bus.busPrefetchCount = 0;

	if(bit32)		/* ARM NON SEQ */
		ret = memoryWait32[addr];
	else			/* THUMB NON SEQ */
		ret = memoryWait[addr];

	return ret;
}

static INLINE int codeTicksAccessSeq16(u32 address) // THUMB SEQ
{
	int addr = (address>>24) & 15;

	if (unsigned(addr - 0x08) <= 5)
	{
		if (bus.busPrefetchCount&0x1)
		{
			bus.busPrefetchCount = ((bus.busPrefetchCount&0xFF)>>1) | (bus.busPrefetchCount&0xFFFFFF00);
			return 0;
		}
		else if (bus.busPrefetchCount>0xFF)
		{
			bus.busPrefetchCount=0;
			return memoryWait[addr];
		}
	}
	else
		bus.busPrefetchCount = 0;

	return memoryWaitSeq[addr];
}

static INLINE int codeTicksAccessSeq32(u32 address) // ARM SEQ
{
	int addr = (address>>24)&15;

	if (unsigned(addr - 0x08) <= 5)
	{
		if (bus.busPrefetchCount&0x1)
		{
			if (bus.busPrefetchCount&0x2)
			{
				bus.busPrefetchCount = ((bus.busPrefetchCount&0xFF)>>2) | (bus.busPrefetchCount&0xFFFFFF00);
				return 0;
			}
			bus.busPrefetchCount = ((bus.busPrefetchCount&0xFF)>>1) | (bus.busPrefetchCount&0xFFFFFF00);
			return memoryWaitSeq[addr];
		}
		else if (bus.busPrefetchCount > 0xFF)
		{
			bus.busPrefetchCount=0;
			return memoryWait32[addr];
		}
	}
	return memoryWaitSeq32[addr];
}

#define CPUReadByteQuick(addr)		map[(addr)>>24].address[(addr) & map[(addr)>>24].mask]
#define CPUReadHalfWordQuick(addr)	READ16LE(((u16*)&map[(addr)>>24].address[(addr) & map[(addr)>>24].mask]))
#define CPUReadMemoryQuick(addr)	READ32LE(((u32*)&map[(addr)>>24].address[(addr) & map[(addr)>>24].mask]))

static bool stopState = false;
extern bool cpuSramEnabled;
extern bool cpuFlashEnabled;
extern bool cpuEEPROMEnabled;
#ifdef USE_MOTION_SENSOR
extern bool cpuEEPROMSensorEnabled;
#endif
static bool timer0On = false;
static int timer0Ticks = 0;
static int timer0Reload = 0;
static int timer0ClockReload  = 0;
static u16 timer1Value = 0;
static bool timer1On = false;
static int timer1Ticks = 0;
static int timer1Reload = 0;
static int timer1ClockReload  = 0;
static u16 timer2Value = 0;
static bool timer2On = false;
static int timer2Ticks = 0;
static int timer2Reload = 0;
static int timer2ClockReload  = 0;
static u16 timer3Value = 0;
static bool timer3On = false;
static int timer3Ticks = 0;
static int timer3Reload = 0;
static int timer3ClockReload  = 0;

static const u32  objTilesAddress [3] = {0x010000, 0x014000, 0x014000};

static INLINE u32 CPUReadMemory(u32 address)
{
	u32 value;
	switch(address >> 24)
	{
		case 0:
			/* BIOS */
			if(bus.reg[15].I >> 24)
			{
				if(address < 0x4000)
					value = READ32LE(((u32 *)&biosProtected));
				else goto unreadable;
			}
			else
				value = READ32LE(((u32 *)&bios[address & 0x3FFC]));
			break;
		case 0x02:
			/* external work RAM */
			value = READ32LE(((u32 *)&workRAM[address & 0x3FFFC]));
			break;
		case 0x03:
			/* internal work RAM */
			value = READ32LE(((u32 *)&internalRAM[address & 0x7ffC]));
			break;
		case 0x04:
			/* I/O registers */
			if((address < 0x4000400) && ioReadable[address & 0x3fc])
			{
				if(ioReadable[(address & 0x3fc) + 2])
					value = READ32LE(((u32 *)&ioMem[address & 0x3fC]));
				else
					value = READ16LE(((u16 *)&ioMem[address & 0x3fc]));
			}
			else
				goto unreadable;
			break;
		case 0x05:
			/* palette RAM */
			value = READ32LE(((u32 *)&graphics.paletteRAM[address & 0x3fC]));
			break;
		case 0x06:
			/* VRAM */
			address = (address & 0x1fffc);
			if (((io_registers[REG_DISPCNT] & 7) >2) && ((address & 0x1C000) == 0x18000))
			{
				value = 0;
				break;
			}
			if ((address & 0x18000) == 0x18000)
				address &= 0x17fff;
			value = READ32LE(((u32 *)&vram[address]));
			break;
		case 0x07:
			/* OAM RAM */
			value = READ32LE(((u32 *)&oam[address & 0x3FC]));
			break;
		case 0x08:
		case 0x09:
		case 0x0A:
		case 0x0B: 
		case 0x0C: 
			/* gamepak ROM */
			value = READ32LE(((u32 *)&rom[address&0x1FFFFFC]));
			break;
		case 0x0D:
			if(cpuEEPROMEnabled)
				return eepromRead();	// no need to swap this
			goto unreadable;
		case 14:
			if(cpuFlashEnabled | cpuSramEnabled)
				return flashRead(address);	// no need to swap this
		default:
unreadable:

			if(armState)
				value = CPUReadMemoryQuick(bus.reg[15].I);
			else
			{
				value = CPUReadHalfWordQuick(bus.reg[15].I) |
					CPUReadHalfWordQuick(bus.reg[15].I) << 16;
			}
	}

	if(address & 3) {
		int shift = (address & 3) << 3;
		value = (value >> shift) | (value << (32 - shift));
	}
	return value;
}

static INLINE u32 CPUReadHalfWord(u32 address)
{
	u32 value;

	switch(address >> 24)
	{
		case 0:
			if (bus.reg[15].I >> 24)
			{
				if(address < 0x4000)
					value = READ16LE(((u16 *)&biosProtected[address&2]));
				else
					goto unreadable;
			}
			else
				value = READ16LE(((u16 *)&bios[address & 0x3FFE]));
			break;
		case 2:
			value = READ16LE(((u16 *)&workRAM[address & 0x3FFFE]));
			break;
		case 3:
			value = READ16LE(((u16 *)&internalRAM[address & 0x7ffe]));
			break;
		case 4:
			if((address < 0x4000400) && ioReadable[address & 0x3fe])
			{
				value =  READ16LE(((u16 *)&ioMem[address & 0x3fe]));
				if (((address & 0x3fe)>0xFF) && ((address & 0x3fe)<0x10E))
				{
					if (((address & 0x3fe) == 0x100) && timer0On)
						value = 0xFFFF - ((timer0Ticks-cpuTotalTicks) >> timer0ClockReload);
					else
						if (((address & 0x3fe) == 0x104) && timer1On && !(io_registers[REG_TM1CNT] & 4))
							value = 0xFFFF - ((timer1Ticks-cpuTotalTicks) >> timer1ClockReload);
						else
							if (((address & 0x3fe) == 0x108) && timer2On && !(io_registers[REG_TM2CNT] & 4))
								value = 0xFFFF - ((timer2Ticks-cpuTotalTicks) >> timer2ClockReload);
							else
								if (((address & 0x3fe) == 0x10C) && timer3On && !(io_registers[REG_TM3CNT] & 4))
									value = 0xFFFF - ((timer3Ticks-cpuTotalTicks) >> timer3ClockReload);
				}
			}
			else goto unreadable;
			break;
		case 5:
			value = READ16LE(((u16 *)&graphics.paletteRAM[address & 0x3fe]));
			break;
		case 6:
			address = (address & 0x1fffe);
			if (((io_registers[REG_DISPCNT] & 7) >2) && ((address & 0x1C000) == 0x18000))
			{
				value = 0;
				break;
			}
			if ((address & 0x18000) == 0x18000)
				address &= 0x17fff;
			value = READ16LE(((u16 *)&vram[address]));
			break;
		case 7:
			value = READ16LE(((u16 *)&oam[address & 0x3fe]));
			break;
		case 8:
		case 9:
		case 10:
		case 11:
		case 12:
			if(address == 0x80000c4 || address == 0x80000c6 || address == 0x80000c8)
				value = rtcRead(address);
			else
				value = READ16LE(((u16 *)&rom[address & 0x1FFFFFE]));
			break;
		case 13:
			if(cpuEEPROMEnabled)
				return  eepromRead();		// no need to swap this
			goto unreadable;
		case 14:
			if(cpuFlashEnabled | cpuSramEnabled)
				return flashRead(address);	// no need to swap this
		default:
unreadable:
			{
				int param = bus.reg[15].I;
				if(armState)
					param += (address & 2);
				value = CPUReadHalfWordQuick(param);
			}
			break;
	}

	if(address & 1)
		value = (value >> 8) | (value << 24);

	return value;
}

static INLINE u16 CPUReadHalfWordSigned(u32 address)
{
	u16 value = CPUReadHalfWord(address);
	if((address & 1))
		value = (s8)value;
	return value;
}

static INLINE u8 CPUReadByte(u32 address)
{
	switch(address >> 24)
	{
		case 0:
			if (bus.reg[15].I >> 24)
			{
				if(address < 0x4000)
					return biosProtected[address & 3];
				else
					goto unreadable;
			}
			return bios[address & 0x3FFF];
		case 2:
			return workRAM[address & 0x3FFFF];
		case 3:
			return internalRAM[address & 0x7fff];
		case 4:
			if((address < 0x4000400) && ioReadable[address & 0x3ff])
				return ioMem[address & 0x3ff];
			else goto unreadable;
		case 5:
			return graphics.paletteRAM[address & 0x3ff];
		case 6:
			address = (address & 0x1ffff);
			if (((io_registers[REG_DISPCNT] & 7) >2) && ((address & 0x1C000) == 0x18000))
				return 0;
			if ((address & 0x18000) == 0x18000)
				address &= 0x17fff;
			return vram[address];
		case 7:
			return oam[address & 0x3ff];
		case 8:
		case 9:
		case 10:
		case 11:
		case 12:
			return rom[address & 0x1FFFFFF];
		case 13:
			if(cpuEEPROMEnabled)
				return eepromRead();
			goto unreadable;
		case 14:
			if(cpuSramEnabled | cpuFlashEnabled)
				return flashRead(address);
#ifdef USE_MOTION_SENSOR
			if(cpuEEPROMSensorEnabled) {
				switch(address & 0x00008f00) {
					case 0x8200:
						return systemGetSensorX() & 255;
					case 0x8300:
						return (systemGetSensorX() >> 8)|0x80;
					case 0x8400:
						return systemGetSensorY() & 255;
					case 0x8500:
						return systemGetSensorY() >> 8;
				}
			}
#endif
		default:
unreadable:
			if(armState)
				return CPUReadByteQuick(bus.reg[15].I+(address & 3));
			else
				return CPUReadByteQuick(bus.reg[15].I+(address & 1));
	}
}

static INLINE void CPUWriteMemory(u32 address, u32 value)
{
	switch(address >> 24)
	{
		case 0x02:
			WRITE32LE(((u32 *)&workRAM[address & 0x3FFFC]), value);
			break;
		case 0x03:
			WRITE32LE(((u32 *)&internalRAM[address & 0x7ffC]), value);
			break;
		case 0x04:
			if(address < 0x4000400)
			{
				CPUUpdateRegister((address & 0x3FC), value & 0xFFFF);
				CPUUpdateRegister((address & 0x3FC) + 2, (value >> 16));
			}
			break;
		case 0x05:
			WRITE32LE(((u32 *)&graphics.paletteRAM[address & 0x3FC]), value);
			break;
		case 0x06:
			address = (address & 0x1fffc);
			if (((io_registers[REG_DISPCNT] & 7) >2) && ((address & 0x1C000) == 0x18000))
				return;
			if ((address & 0x18000) == 0x18000)
				address &= 0x17fff;


			WRITE32LE(((u32 *)&vram[address]), value);
			break;
		case 0x07:
			WRITE32LE(((u32 *)&oam[address & 0x3fc]), value);
			break;
		case 0x0D:
			if(cpuEEPROMEnabled) {
				eepromWrite(value);
				break;
			}
			break;
		case 0x0E:
			if((!eepromInUse) | cpuSramEnabled | cpuFlashEnabled)
				(*cpuSaveGameFunc)(address, (u8)value);
			break;
		default:
			break;
	}
}

static INLINE void CPUWriteHalfWord(u32 address, u16 value)
{
	switch(address >> 24)
	{
		case 2:
			WRITE16LE(((u16 *)&workRAM[address & 0x3FFFE]),value);
			break;
		case 3:
			WRITE16LE(((u16 *)&internalRAM[address & 0x7ffe]), value);
			break;
		case 4:
			if(address < 0x4000400)
				CPUUpdateRegister(address & 0x3fe, value);
			break;
		case 5:
			WRITE16LE(((u16 *)&graphics.paletteRAM[address & 0x3fe]), value);
			break;
		case 6:
			address = (address & 0x1fffe);
			if (((io_registers[REG_DISPCNT] & 7) >2) && ((address & 0x1C000) == 0x18000))
				return;
			if ((address & 0x18000) == 0x18000)
				address &= 0x17fff;
			WRITE16LE(((u16 *)&vram[address]), value);
			break;
		case 7:
			WRITE16LE(((u16 *)&oam[address & 0x3fe]), value);
			break;
		case 8:
		case 9:
			if(address == 0x80000c4 || address == 0x80000c6 || address == 0x80000c8)
				if(!rtcWrite(address, value))
					break;
			break;
		case 13:
			if(cpuEEPROMEnabled)
				eepromWrite((u8)value);
			break;
		case 14:
			if((!eepromInUse) | cpuSramEnabled | cpuFlashEnabled)
				(*cpuSaveGameFunc)(address, (u8)value);
			break;
		default:
			break;
	}
}

static INLINE void CPUWriteByte(u32 address, u8 b)
{
	switch(address >> 24)
	{
		case 2:
			workRAM[address & 0x3FFFF] = b;
			break;
		case 3:
			internalRAM[address & 0x7fff] = b;
			break;
		case 4:
			if(address < 0x4000400)
			{
				switch(address & 0x3FF)
				{
					case 0x60:
					case 0x61:
					case 0x62:
					case 0x63:
					case 0x64:
					case 0x65:
					case 0x68:
					case 0x69:
					case 0x6c:
					case 0x6d:
					case 0x70:
					case 0x71:
					case 0x72:
					case 0x73:
					case 0x74:
					case 0x75:
					case 0x78:
					case 0x79:
					case 0x7c:
					case 0x7d:
					case 0x80:
					case 0x81:
					case 0x84:
					case 0x85:
					case 0x90:
					case 0x91:
					case 0x92:
					case 0x93:
					case 0x94:
					case 0x95:
					case 0x96:
					case 0x97:
					case 0x98:
					case 0x99:
					case 0x9a:
					case 0x9b:
					case 0x9c:
					case 0x9d:
					case 0x9e:
					case 0x9f:
						{
							int gb_addr = table[(address & 0xFF) - 0x60];
							soundEvent_u8(gb_addr, address&0xFF, b);
						}
						break;
					case 0x301: // HALTCNT, undocumented
						if(b == 0x80)
							stopState = true;
						holdState = 1;
						cpuNextEvent = cpuTotalTicks;
						break;
					default: // every other register
						{
							u32 lowerBits = address & 0x3fe;
							u16 param;
							if(address & 1)
								param = (READ16LE(&ioMem[lowerBits]) & 0x00FF) | (b << 8);
							else
								param = (READ16LE(&ioMem[lowerBits]) & 0xFF00) | b;

							CPUUpdateRegister(lowerBits, param);
						}
					break;
				}
			}
			break;
		case 5:
			// no need to switch
			*((u16 *)&graphics.paletteRAM[address & 0x3FE]) = (b << 8) | b;
			break;
		case 6:
			address = (address & 0x1fffe);
			if (((io_registers[REG_DISPCNT] & 7) >2) && ((address & 0x1C000) == 0x18000))
				return;
			if ((address & 0x18000) == 0x18000)
				address &= 0x17fff;

			// no need to switch
			// byte writes to OBJ VRAM are ignored
			if ((address) < objTilesAddress[((io_registers[REG_DISPCNT] & 7)+1)>>2])
				*((u16 *)&vram[address]) = (b << 8) | b;
			break;
		case 7:
			// no need to switch
			// byte writes to OAM are ignored
			//    *((u16 *)&oam[address & 0x3FE]) = (b << 8) | b;
			break;
		case 13:
			if(cpuEEPROMEnabled)
				eepromWrite(b);
			break;
		case 14:
			if ((saveType != 5) && ((!eepromInUse) | cpuSramEnabled | cpuFlashEnabled))
			{
				(*cpuSaveGameFunc)(address, b);
				break;
			}
		default:
			break;
	}
}

#include "gba_bios.inl"

#define CPU_UPDATE_CPSR() \
{ \
	u32 CPSR; \
	CPSR = bus.reg[16].I & 0x40; \
	if(N_FLAG) \
		CPSR |= 0x80000000; \
	if(Z_FLAG) \
		CPSR |= 0x40000000; \
	if(C_FLAG) \
		CPSR |= 0x20000000; \
	if(V_FLAG) \
		CPSR |= 0x10000000; \
	if(!armState) \
		CPSR |= 0x00000020; \
	if(!armIrqEnable) \
		CPSR |= 0x80; \
	CPSR |= (armMode & 0x1F); \
	bus.reg[16].I = CPSR; \
}

#define CPU_SOFTWARE_INTERRUPT() \
{ \
  u32 PC = bus.reg[15].I; \
  bool savedArmState = armState; \
  if(armMode != 0x13) \
    CPUSwitchMode(0x13, true, false); \
  bus.reg[14].I = PC - (savedArmState ? 4 : 2); \
  bus.reg[15].I = 0x08; \
  armState = true; \
  armIrqEnable = false; \
  bus.armNextPC = 0x08; \
  ARM_PREFETCH; \
  bus.reg[15].I += 4; \
}

static void CPUUpdateFlags(bool breakLoop)
{
	u32 CPSR = bus.reg[16].I;

	N_FLAG = (CPSR & 0x80000000) ? true: false;
	Z_FLAG = (CPSR & 0x40000000) ? true: false;
	C_FLAG = (CPSR & 0x20000000) ? true: false;
	V_FLAG = (CPSR & 0x10000000) ? true: false;
	armState = (CPSR & 0x20) ? false : true;
	armIrqEnable = (CPSR & 0x80) ? false : true;
	if (breakLoop && armIrqEnable && (io_registers[REG_IF] & io_registers[REG_IE]) && (io_registers[REG_IME] & 1))
		cpuNextEvent = cpuTotalTicks;
}

static void CPUSoftwareInterrupt(int comment)
{
	if(armState)
		comment >>= 16;

#ifdef HAVE_HLE_BIOS
	if(useBios)
	{
		CPU_SOFTWARE_INTERRUPT();
		return;
	}
#endif

	switch(comment) {
		case 0x00:
			BIOS_SoftReset();
			ARM_PREFETCH;
			break;
		case 0x01:
			BIOS_REGISTER_RAM_RESET();
			break;
		case 0x02:
			holdState = true;
			cpuNextEvent = cpuTotalTicks;
			break;
		case 0x03:
			holdState = true;
			stopState = true;
			cpuNextEvent = cpuTotalTicks;
			break;
		case 0x04:
		case 0x05:
		case 0x06:
		case 0x07:
			CPU_SOFTWARE_INTERRUPT();
			break;
		case 0x08:
			BIOS_SQRT();
			break;
		case 0x09:
			BIOS_ArcTan();
			break;
		case 0x0A:
			BIOS_ArcTan2();
			break;
		case 0x0B:
			{
#ifdef USE_SWITICKS
				int len = (bus.reg[2].I & 0x1FFFFF) >>1;
				if (!(((bus.reg[0].I & 0xe000000) == 0) ||
							((bus.reg[0].I + len) & 0xe000000) == 0))
				{
					if ((bus.reg[2].I >> 24) & 1)
					{
						if ((bus.reg[2].I >> 26) & 1)
							SWITicks = (7 + memoryWait32[(bus.reg[1].I>>24) & 0xF]) * (len>>1);
						else
							SWITicks = (8 + memoryWait[(bus.reg[1].I>>24) & 0xF]) * (len);
					}
					else
					{
						if ((bus.reg[2].I >> 26) & 1)
							SWITicks = (10 + memoryWait32[(bus.reg[0].I>>24) & 0xF] +
									memoryWait32[(bus.reg[1].I>>24) & 0xF]) * (len>>1);
						else
							SWITicks = (11 + memoryWait[(bus.reg[0].I>>24) & 0xF] +
									memoryWait[(bus.reg[1].I>>24) & 0xF]) * len;
					}
				}
#endif
			}
			if(!(((bus.reg[0].I & 0xe000000) == 0) || ((bus.reg[0].I + (((bus.reg[2].I << 11)>>9) & 0x1fffff)) & 0xe000000) == 0))
				BIOS_CpuSet();
			break;
		case 0x0C:
			{
#ifdef USE_SWITICKS
				int len = (bus.reg[2].I & 0x1FFFFF) >>5;
				if (!(((bus.reg[0].I & 0xe000000) == 0) ||
							((bus.reg[0].I + len) & 0xe000000) == 0))
				{
					if ((bus.reg[2].I >> 24) & 1)
						SWITicks = (6 + memoryWait32[(bus.reg[1].I>>24) & 0xF] +
								7 * (memoryWaitSeq32[(bus.reg[1].I>>24) & 0xF] + 1)) * len;
					else
						SWITicks = (9 + memoryWait32[(bus.reg[0].I>>24) & 0xF] +
								memoryWait32[(bus.reg[1].I>>24) & 0xF] +
								7 * (memoryWaitSeq32[(bus.reg[0].I>>24) & 0xF] +
									memoryWaitSeq32[(bus.reg[1].I>>24) & 0xF] + 2)) * len;
				}
#endif
			}
			if(!(((bus.reg[0].I & 0xe000000) == 0) || ((bus.reg[0].I + (((bus.reg[2].I << 11)>>9) & 0x1fffff)) & 0xe000000) == 0))
				BIOS_CpuFastSet();
			break;
		case 0x0D:
			BIOS_GET_BIOS_CHECKSUM();
			break;
		case 0x0E:
			BIOS_BgAffineSet();
			break;
		case 0x0F:
			BIOS_ObjAffineSet();
			break;
		case 0x10:
			{
#ifdef USE_SWITICKS
				int len = CPUReadHalfWord(bus.reg[2].I);
				if (!(((bus.reg[0].I & 0xe000000) == 0) ||
							((bus.reg[0].I + len) & 0xe000000) == 0))
					SWITicks = (32 + memoryWait[(bus.reg[0].I>>24) & 0xF]) * len;
#endif
			}
			BIOS_BitUnPack();
			break;
		case 0x11:
#ifdef USE_SWITICKS
			{
				u32 len = CPUReadMemory(bus.reg[0].I) >> 8;
				if(!(((bus.reg[0].I & 0xe000000) == 0) ||
							((bus.reg[0].I + (len & 0x1fffff)) & 0xe000000) == 0))
					SWITicks = (9 + memoryWait[(bus.reg[1].I>>24) & 0xF]) * len;
			}
#endif
			BIOS_LZ77UnCompWram();
			break;
		case 0x12:
#ifdef USE_SWITICKS
			{
				u32 len = CPUReadMemory(bus.reg[0].I) >> 8;
				if(!(((bus.reg[0].I & 0xe000000) == 0) ||
							((bus.reg[0].I + (len & 0x1fffff)) & 0xe000000) == 0))
					SWITicks = (19 + memoryWait[(bus.reg[1].I>>24) & 0xF]) * len;
			}
#endif
			BIOS_LZ77UnCompVram();
			break;
		case 0x13:
#ifdef USE_SWITICKS
			{
				u32 len = CPUReadMemory(bus.reg[0].I) >> 8;
				if(!(((bus.reg[0].I & 0xe000000) == 0) ||
							((bus.reg[0].I + (len & 0x1fffff)) & 0xe000000) == 0))
					SWITicks = (29 + (memoryWait[(bus.reg[0].I>>24) & 0xF]<<1)) * len;
			}
#endif
			BIOS_HuffUnComp();
			break;
		case 0x14:
#ifdef USE_SWITICKS
			{
				u32 len = CPUReadMemory(bus.reg[0].I) >> 8;
				if(!(((bus.reg[0].I & 0xe000000) == 0) ||
							((bus.reg[0].I + (len & 0x1fffff)) & 0xe000000) == 0))
					SWITicks = (11 + memoryWait[(bus.reg[0].I>>24) & 0xF] +
							memoryWait[(bus.reg[1].I>>24) & 0xF]) * len;
			}
#endif
			BIOS_RLUnCompWram();
			break;
		case 0x15:
#ifdef USE_SWITICKS
			{
				u32 len = CPUReadMemory(bus.reg[0].I) >> 9;
				if(!(((bus.reg[0].I & 0xe000000) == 0) ||
							((bus.reg[0].I + (len & 0x1fffff)) & 0xe000000) == 0))
					SWITicks = (34 + (memoryWait[(bus.reg[0].I>>24) & 0xF] << 1) +
							memoryWait[(bus.reg[1].I>>24) & 0xF]) * len;
			}
#endif
			BIOS_RLUnCompVram();
			break;
		case 0x16:
#ifdef USE_SWITICKS
			{
				u32 len = CPUReadMemory(bus.reg[0].I) >> 8;
				if(!(((bus.reg[0].I & 0xe000000) == 0) ||
							((bus.reg[0].I + (len & 0x1fffff)) & 0xe000000) == 0))
					SWITicks = (13 + memoryWait[(bus.reg[0].I>>24) & 0xF] +
							memoryWait[(bus.reg[1].I>>24) & 0xF]) * len;
			}
#endif
			BIOS_Diff8bitUnFilterWram();
			break;
		case 0x17:
#ifdef USE_SWITICKS
			{
				u32 len = CPUReadMemory(bus.reg[0].I) >> 9;
				if(!(((bus.reg[0].I & 0xe000000) == 0) ||
							((bus.reg[0].I + (len & 0x1fffff)) & 0xe000000) == 0))
					SWITicks = (39 + (memoryWait[(bus.reg[0].I>>24) & 0xF]<<1) +
							memoryWait[(bus.reg[1].I>>24) & 0xF]) * len;
			}
#endif
			BIOS_Diff8bitUnFilterVram();
			break;
		case 0x18:
#ifdef USE_SWITICKS
			{
				u32 len = CPUReadMemory(bus.reg[0].I) >> 9;
				if(!(((bus.reg[0].I & 0xe000000) == 0) ||
							((bus.reg[0].I + (len & 0x1fffff)) & 0xe000000) == 0))
					SWITicks = (13 + memoryWait[(bus.reg[0].I>>24) & 0xF] +
							memoryWait[(bus.reg[1].I>>24) & 0xF]) * len;
			}
#endif
			BIOS_Diff16bitUnFilter();
			break;
		case 0x19:
			break;
		case 0x1F:
			BIOS_MIDI_KEY_2_FREQ();
			break;
		case 0x2A:
			BIOS_SND_DRIVER_JMP_TABLE_COPY();
			// let it go, because we don't really emulate this function
		default:
			break;
	}
}

#include "gba_arm_cpuexec.inl"

#include "gba_thumb_cpuexec.inl"

#include "gba_gfx.inl"

/*============================================================
	GBA.CPP
============================================================ */
int saveType = 0;
bool useBios = false;
bool skipBios = false;
bool cpuIsMultiBoot = false;
int cpuSaveType = 0;
bool enableRtc = false;
bool mirroringEnable = false;
bool skipSaveGameBattery = false;

int cpuDmaCount = 0;

u8 *bios = 0;
u8 *rom = 0;
u8 *internalRAM = 0;
u8 *workRAM = 0;
u8 *vram = 0;
u16 *pix = 0;
u8 *oam = 0;
u8 *ioMem = 0;

#ifdef USE_SWITICKS
int SWITicks = 0;
#endif

bool cpuSramEnabled = true;
bool cpuFlashEnabled = true;
bool cpuEEPROMEnabled = true;
bool cpuEEPROMSensorEnabled = false;

#ifndef LSB_FIRST
bool cpuBiosSwapped = false;
#endif

u32 myROM[] = {
0xEA000006,
0xEA000093,
0xEA000006,
0x00000000,
0x00000000,
0x00000000,
0xEA000088,
0x00000000,
0xE3A00302,
0xE1A0F000,
0xE92D5800,
0xE55EC002,
0xE28FB03C,
0xE79BC10C,
0xE14FB000,
0xE92D0800,
0xE20BB080,
0xE38BB01F,
0xE129F00B,
0xE92D4004,
0xE1A0E00F,
0xE12FFF1C,
0xE8BD4004,
0xE3A0C0D3,
0xE129F00C,
0xE8BD0800,
0xE169F00B,
0xE8BD5800,
0xE1B0F00E,
0x0000009C,
0x0000009C,
0x0000009C,
0x0000009C,
0x000001F8,
0x000001F0,
0x000000AC,
0x000000A0,
0x000000FC,
0x00000168,
0xE12FFF1E,
0xE1A03000,
0xE1A00001,
0xE1A01003,
0xE2113102,
0x42611000,
0xE033C040,
0x22600000,
0xE1B02001,
0xE15200A0,
0x91A02082,
0x3AFFFFFC,
0xE1500002,
0xE0A33003,
0x20400002,
0xE1320001,
0x11A020A2,
0x1AFFFFF9,
0xE1A01000,
0xE1A00003,
0xE1B0C08C,
0x22600000,
0x42611000,
0xE12FFF1E,
0xE92D0010,
0xE1A0C000,
0xE3A01001,
0xE1500001,
0x81A000A0,
0x81A01081,
0x8AFFFFFB,
0xE1A0000C,
0xE1A04001,
0xE3A03000,
0xE1A02001,
0xE15200A0,
0x91A02082,
0x3AFFFFFC,
0xE1500002,
0xE0A33003,
0x20400002,
0xE1320001,
0x11A020A2,
0x1AFFFFF9,
0xE0811003,
0xE1B010A1,
0xE1510004,
0x3AFFFFEE,
0xE1A00004,
0xE8BD0010,
0xE12FFF1E,
0xE0010090,
0xE1A01741,
0xE2611000,
0xE3A030A9,
0xE0030391,
0xE1A03743,
0xE2833E39,
0xE0030391,
0xE1A03743,
0xE2833C09,
0xE283301C,
0xE0030391,
0xE1A03743,
0xE2833C0F,
0xE28330B6,
0xE0030391,
0xE1A03743,
0xE2833C16,
0xE28330AA,
0xE0030391,
0xE1A03743,
0xE2833A02,
0xE2833081,
0xE0030391,
0xE1A03743,
0xE2833C36,
0xE2833051,
0xE0030391,
0xE1A03743,
0xE2833CA2,
0xE28330F9,
0xE0000093,
0xE1A00840,
0xE12FFF1E,
0xE3A00001,
0xE3A01001,
0xE92D4010,
0xE3A03000,
0xE3A04001,
0xE3500000,
0x1B000004,
0xE5CC3301,
0xEB000002,
0x0AFFFFFC,
0xE8BD4010,
0xE12FFF1E,
0xE3A0C301,
0xE5CC3208,
0xE15C20B8,
0xE0110002,
0x10222000,
0x114C20B8,
0xE5CC4208,
0xE12FFF1E,
0xE92D500F,
0xE3A00301,
0xE1A0E00F,
0xE510F004,
0xE8BD500F,
0xE25EF004,
0xE59FD044,
0xE92D5000,
0xE14FC000,
0xE10FE000,
0xE92D5000,
0xE3A0C302,
0xE5DCE09C,
0xE35E00A5,
0x1A000004,
0x05DCE0B4,
0x021EE080,
0xE28FE004,
0x159FF018,
0x059FF018,
0xE59FD018,
0xE8BD5000,
0xE169F00C,
0xE8BD5000,
0xE25EF004,
0x03007FF0,
0x09FE2000,
0x09FFC000,
0x03007FE0
};

static variable_desc saveGameStruct[] = {
	{ &io_registers[REG_DISPCNT]  , sizeof(u16) },
	{ &io_registers[REG_DISPSTAT] , sizeof(u16) },
	{ &io_registers[REG_VCOUNT]   , sizeof(u16) },
	{ &io_registers[REG_BG0CNT]   , sizeof(u16) },
	{ &io_registers[REG_BG1CNT]   , sizeof(u16) },
	{ &io_registers[REG_BG2CNT]   , sizeof(u16) },
	{ &io_registers[REG_BG3CNT]   , sizeof(u16) },
	{ &io_registers[REG_BG0HOFS]  , sizeof(u16) },
	{ &io_registers[REG_BG0VOFS]  , sizeof(u16) },
	{ &io_registers[REG_BG1HOFS]  , sizeof(u16) },
	{ &io_registers[REG_BG1VOFS]  , sizeof(u16) },
	{ &io_registers[REG_BG2HOFS]  , sizeof(u16) },
	{ &io_registers[REG_BG2VOFS]  , sizeof(u16) },
	{ &io_registers[REG_BG3HOFS]  , sizeof(u16) },
	{ &io_registers[REG_BG3VOFS]  , sizeof(u16) },
	{ &io_registers[REG_BG2PA]    , sizeof(u16) },
	{ &io_registers[REG_BG2PB]    , sizeof(u16) },
	{ &io_registers[REG_BG2PC]    , sizeof(u16) },
	{ &io_registers[REG_BG2PD]    , sizeof(u16) },
	{ &BG2X_L   , sizeof(u16) },
	{ &BG2X_H   , sizeof(u16) },
	{ &BG2Y_L   , sizeof(u16) },
	{ &BG2Y_H   , sizeof(u16) },
	{ &io_registers[REG_BG3PA]    , sizeof(u16) },
	{ &io_registers[REG_BG3PB]    , sizeof(u16) },
	{ &io_registers[REG_BG3PC]    , sizeof(u16) },
	{ &io_registers[REG_BG3PD]    , sizeof(u16) },
	{ &BG3X_L   , sizeof(u16) },
	{ &BG3X_H   , sizeof(u16) },
	{ &BG3Y_L   , sizeof(u16) },
	{ &BG3Y_H   , sizeof(u16) },
	{ &io_registers[REG_WIN0H]    , sizeof(u16) },
	{ &io_registers[REG_WIN1H]    , sizeof(u16) },
	{ &io_registers[REG_WIN0V]    , sizeof(u16) },
	{ &io_registers[REG_WIN1V]    , sizeof(u16) },
	{ &io_registers[REG_WININ]    , sizeof(u16) },
	{ &io_registers[REG_WINOUT]   , sizeof(u16) },
	{ &MOSAIC   , sizeof(u16) },
	{ &BLDMOD   , sizeof(u16) },
	{ &COLEV    , sizeof(u16) },
	{ &COLY     , sizeof(u16) },
	{ &DM0SAD_L , sizeof(u16) },
	{ &DM0SAD_H , sizeof(u16) },
	{ &DM0DAD_L , sizeof(u16) },
	{ &DM0DAD_H , sizeof(u16) },
	{ &DM0CNT_L , sizeof(u16) },
	{ &DM0CNT_H , sizeof(u16) },
	{ &DM1SAD_L , sizeof(u16) },
	{ &DM1SAD_H , sizeof(u16) },
	{ &DM1DAD_L , sizeof(u16) },
	{ &DM1DAD_H , sizeof(u16) },
	{ &DM1CNT_L , sizeof(u16) },
	{ &DM1CNT_H , sizeof(u16) },
	{ &DM2SAD_L , sizeof(u16) },
	{ &DM2SAD_H , sizeof(u16) },
	{ &DM2DAD_L , sizeof(u16) },
	{ &DM2DAD_H , sizeof(u16) },
	{ &DM2CNT_L , sizeof(u16) },
	{ &DM2CNT_H , sizeof(u16) },
	{ &DM3SAD_L , sizeof(u16) },
	{ &DM3SAD_H , sizeof(u16) },
	{ &DM3DAD_L , sizeof(u16) },
	{ &DM3DAD_H , sizeof(u16) },
	{ &DM3CNT_L , sizeof(u16) },
	{ &DM3CNT_H , sizeof(u16) },
	{ &io_registers[REG_TM0D]     , sizeof(u16) },
	{ &io_registers[REG_TM0CNT]   , sizeof(u16) },
	{ &io_registers[REG_TM1D]     , sizeof(u16) },
	{ &io_registers[REG_TM1CNT]   , sizeof(u16) },
	{ &io_registers[REG_TM2D]     , sizeof(u16) },
	{ &io_registers[REG_TM2CNT]   , sizeof(u16) },
	{ &io_registers[REG_TM3D]     , sizeof(u16) },
	{ &io_registers[REG_TM3CNT]   , sizeof(u16) },
	{ &io_registers[REG_P1]       , sizeof(u16) },
	{ &io_registers[REG_IE]       , sizeof(u16) },
	{ &io_registers[REG_IF]       , sizeof(u16) },
	{ &io_registers[REG_IME]      , sizeof(u16) },
	{ &holdState, sizeof(bool) },
	{ &graphics.lcdTicks, sizeof(int) },
	{ &timer0On , sizeof(bool) },
	{ &timer0Ticks , sizeof(int) },
	{ &timer0Reload , sizeof(int) },
	{ &timer0ClockReload  , sizeof(int) },
	{ &timer1On , sizeof(bool) },
	{ &timer1Ticks , sizeof(int) },
	{ &timer1Reload , sizeof(int) },
	{ &timer1ClockReload  , sizeof(int) },
	{ &timer2On , sizeof(bool) },
	{ &timer2Ticks , sizeof(int) },
	{ &timer2Reload , sizeof(int) },
	{ &timer2ClockReload  , sizeof(int) },
	{ &timer3On , sizeof(bool) },
	{ &timer3Ticks , sizeof(int) },
	{ &timer3Reload , sizeof(int) },
	{ &timer3ClockReload  , sizeof(int) },
	{ &dma0Source , sizeof(u32) },
	{ &dma0Dest , sizeof(u32) },
	{ &dma1Source , sizeof(u32) },
	{ &dma1Dest , sizeof(u32) },
	{ &dma2Source , sizeof(u32) },
	{ &dma2Dest , sizeof(u32) },
	{ &dma3Source , sizeof(u32) },
	{ &dma3Dest , sizeof(u32) },
	{ &fxOn, sizeof(bool) },
	{ &windowOn, sizeof(bool) },
	{ &N_FLAG , sizeof(bool) },
	{ &C_FLAG , sizeof(bool) },
	{ &Z_FLAG , sizeof(bool) },
	{ &V_FLAG , sizeof(bool) },
	{ &armState , sizeof(bool) },
	{ &armIrqEnable , sizeof(bool) },
	{ &bus.armNextPC , sizeof(u32) },
	{ &armMode , sizeof(int) },
	{ &saveType , sizeof(int) },
	{ NULL, 0 }
};

static INLINE int CPUUpdateTicks (void)
{
	int cpuLoopTicks = graphics.lcdTicks;

	if(soundTicks < cpuLoopTicks)
		cpuLoopTicks = soundTicks;

	if(timer0On && (timer0Ticks < cpuLoopTicks))
		cpuLoopTicks = timer0Ticks;

	if(timer1On && !(io_registers[REG_TM1CNT] & 4) && (timer1Ticks < cpuLoopTicks))
		cpuLoopTicks = timer1Ticks;

	if(timer2On && !(io_registers[REG_TM2CNT] & 4) && (timer2Ticks < cpuLoopTicks))
		cpuLoopTicks = timer2Ticks;

	if(timer3On && !(io_registers[REG_TM3CNT] & 4) && (timer3Ticks < cpuLoopTicks))
		cpuLoopTicks = timer3Ticks;

#ifdef USE_SWITICKS
	if (SWITicks)
	{
		if (SWITicks < cpuLoopTicks)
			cpuLoopTicks = SWITicks;
	}
#endif

	if (IRQTicks)
	{
		if (IRQTicks < cpuLoopTicks)
			cpuLoopTicks = IRQTicks;
	}

	return cpuLoopTicks;
}

#define CPUUpdateWindow0() \
{ \
  int x00_window0 = io_registers[REG_WIN0H] >>8; \
  int x01_window0 = io_registers[REG_WIN0H] & 255; \
  int x00_lte_x01 = x00_window0 <= x01_window0; \
  for(int i = 0; i < 240; i++) \
      gfxInWin[0][i] = ((i >= x00_window0 && i < x01_window0) & x00_lte_x01) | ((i >= x00_window0 || i < x01_window0) & ~x00_lte_x01); \
}

#define CPUUpdateWindow1() \
{ \
  int x00_window1 = io_registers[REG_WIN1H]>>8; \
  int x01_window1 = io_registers[REG_WIN1H] & 255; \
  int x00_lte_x01 = x00_window1 <= x01_window1; \
  for(int i = 0; i < 240; i++) \
   gfxInWin[1][i] = ((i >= x00_window1 && i < x01_window1) & x00_lte_x01) | ((i >= x00_window1 || i < x01_window1) & ~x00_lte_x01); \
}

#define CPUCompareVCOUNT() \
  if(io_registers[REG_VCOUNT] == (io_registers[REG_DISPSTAT] >> 8)) \
  { \
    io_registers[REG_DISPSTAT] |= 4; \
    UPDATE_REG(0x04, io_registers[REG_DISPSTAT]); \
    if(io_registers[REG_DISPSTAT] & 0x20) \
    { \
      io_registers[REG_IF] |= 4; \
      UPDATE_REG(0x202, io_registers[REG_IF]); \
    } \
  } \
  else \
  { \
    io_registers[REG_DISPSTAT] &= 0xFFFB; \
    UPDATE_REG(0x4, io_registers[REG_DISPSTAT]); \
  } \
  if (graphics.layerEnableDelay > 0) \
  { \
      graphics.layerEnableDelay--; \
      if (graphics.layerEnableDelay == 1) \
          graphics.layerEnable = io_registers[REG_DISPCNT]; \
  }


unsigned CPUWriteState(u8* data, unsigned size)
{
	u8 *orig = data;

	utilWriteIntMem(data, SAVE_GAME_VERSION);
	utilWriteMem(data, &rom[0xa0], 16);
	utilWriteIntMem(data, useBios);
	utilWriteMem(data, &bus.reg[0], sizeof(bus.reg));

	utilWriteDataMem(data, saveGameStruct);

	utilWriteIntMem(data, stopState);
	utilWriteIntMem(data, IRQTicks);

	utilWriteMem(data, internalRAM, 0x8000);
	utilWriteMem(data, graphics.paletteRAM, 0x400);
	utilWriteMem(data, workRAM, 0x40000);
	utilWriteMem(data, vram, 0x20000);
	utilWriteMem(data, oam, 0x400);
	utilWriteMem(data, pix, 4 * PIX_BUFFER_SCREEN_WIDTH * 160);
	utilWriteMem(data, ioMem, 0x400);

	eepromSaveGameMem(data);
	flashSaveGameMem(data);
	soundSaveGameMem(data);
	rtcSaveGameMem(data);

	return (ptrdiff_t)data - (ptrdiff_t)orig;
}

bool CPUWriteBatteryFile(const char *fileName)
{
	if(gbaSaveType == 0)
	{
		if(eepromInUse)
			gbaSaveType = 3;
		else
			switch(saveType)
			{
				case 1:
					gbaSaveType = 1;
					break;
				case 2:
					gbaSaveType = 2;
					break;
			}
	}

	if((gbaSaveType) && (gbaSaveType!=5))
	{
		FILE *file = fopen(fileName, "wb");

		if(!file) {
			systemMessage("Error creating file %s", fileName);
			return false;
		}

		// only save if Flash/Sram in use or EEprom in use
		if(gbaSaveType != 3) {
			if(gbaSaveType == 2) {
				if(fwrite(flashSaveMemory, 1, flashSize, file) != (size_t)flashSize) {
					fclose(file);
					return false;
				}
			} else {
				if(fwrite(flashSaveMemory, 1, 0x10000, file) != 0x10000) {
					fclose(file);
					return false;
				}
			}
		} else {
			if(fwrite(eepromData, 1, eepromSize, file) != (size_t)eepromSize) {
				fclose(file);
				return false;
			}
		}
		fclose(file);
	}
	return true;
}

bool CPUReadBatteryFile(const char *fileName)
{
	FILE *file = fopen(fileName, "rb");

	if(!file)
		return false;

	// check file size to know what we should read
	fseek(file, 0, SEEK_END);

	long size = ftell(file);
	fseek(file, 0, SEEK_SET);

	if(size == 512 || size == 0x2000) {
		if(fread(eepromData, 1, size, file) != (size_t)size) {
			fclose(file);
			return false;
		}
	} else {
		if(size == 0x20000) {
			if(fread(flashSaveMemory, 1, 0x20000, file) != 0x20000) {
				fclose(file);
				return false;
			}
			flashSetSize(0x20000);
		} else {
			if(fread(flashSaveMemory, 1, 0x10000, file) != 0x10000) {
				fclose(file);
				return false;
			}
			flashSetSize(0x10000);
		}
	}
	fclose(file);
	return true;
}

#ifdef HAVE_HLE_BIOS
static bool CPUIsGBABios(const char * file)
{
	if(strlen(file) > 4)
	{
		const char * p = strrchr(file,'.');

		if(p != NULL)
		{
			if(strcasecmp(p, ".gba") == 0)
				return true;
			if(strcasecmp(p, ".agb") == 0)
				return true;
			if(strcasecmp(p, ".bin") == 0)
				return true;
			if(strcasecmp(p, ".bios") == 0)
				return true;
			if(strcasecmp(p, ".rom") == 0)
				return true;
		}
	}

	return false;
}
#endif

#ifdef ELF
static bool CPUIsELF(const char *file)
{
	if(file == NULL)
		return false;

	if(strlen(file) > 4)
	{
		const char * p = strrchr(file,'.');

		if(p != NULL)
		{
			if(strcasecmp(p, ".elf") == 0)
				return true;
		}
	}
	return false;
}
#endif

static void CPUCleanUp (void)
{
	if(rom != NULL) {
		free(rom);
		rom = NULL;
	}

	if(vram != NULL) {
		free(vram);
		vram = NULL;
	}

	if(graphics.paletteRAM != NULL) {
		free(graphics.paletteRAM);
		graphics.paletteRAM = NULL;
	}

	if(internalRAM != NULL) {
		free(internalRAM);
		internalRAM = NULL;
	}

	if(workRAM != NULL) {
		free(workRAM);
		workRAM = NULL;
	}

	if(bios != NULL) {
		free(bios);
		bios = NULL;
	}

	if(pix != NULL) {
		free(pix);
		pix = NULL;
	}

	if(oam != NULL) {
		free(oam);
		oam = NULL;
	}

	if(ioMem != NULL) {
		free(ioMem);
		ioMem = NULL;
	}
}

int CPULoadRom(const char * file)
{
	romSize = 0x2000000;
	if(rom != NULL)
		CPUCleanUp();

	rom = (u8 *)malloc(0x2000000);

	if(rom == NULL)
		return 0;

	workRAM = (u8 *)calloc(1, 0x40000);

	if(workRAM == NULL)
		return 0;

	u8 *whereToLoad = cpuIsMultiBoot ? workRAM : rom;

		if(file != NULL)
		{
			if(!utilLoad(file,
						utilIsGBAImage,
						whereToLoad,
						romSize)) {
				free(rom);
				rom = NULL;
				free(workRAM);
				workRAM = NULL;
				return 0;
			}
		}

	u16 *temp = (u16 *)(rom+((romSize+1)&~1));
	int i;
	for(i = (romSize+1)&~1; i < 0x2000000; i+=2) {
		WRITE16LE(temp, (i >> 1) & 0xFFFF);
		temp++;
	}

	bios = (u8 *)calloc(1,0x4000);
	if(bios == NULL) {
		CPUCleanUp();
		return 0;
	}
	internalRAM = (u8 *)calloc(1,0x8000);
	if(internalRAM == NULL) {
		CPUCleanUp();
		return 0;
	}
	graphics.paletteRAM = (u8 *)calloc(1,0x400);
	if(graphics.paletteRAM == NULL) {
		CPUCleanUp();
		return 0;
	}
	vram = (u8 *)calloc(1, 0x20000);
	if(vram == NULL) {
		CPUCleanUp();
		return 0;
	}
	oam = (u8 *)calloc(1, 0x400);
	if(oam == NULL) {
		CPUCleanUp();
		return 0;
	}
	pix = (u16 *)calloc(1, 4 * PIX_BUFFER_SCREEN_WIDTH * 160);
	if(pix == NULL) {
		CPUCleanUp();
		return 0;
	}
	ioMem = (u8 *)calloc(1, 0x400);
	if(ioMem == NULL) {
		CPUCleanUp();
		return 0;
	}

	flashInit();
	eepromInit();

	memset(line[0], -1, 240 * sizeof(u32));
	memset(line[1], -1, 240 * sizeof(u32));
	memset(line[2], -1, 240 * sizeof(u32));
	memset(line[3], -1, 240 * sizeof(u32));

	return romSize;
}

void doMirroring (bool b)
{
	u32 mirroredRomSize = (((romSize)>>20) & 0x3F)<<20;
	u32 mirroredRomAddress = romSize;
	if ((mirroredRomSize <=0x800000) && (b))
	{
		mirroredRomAddress = mirroredRomSize;
		if (mirroredRomSize==0)
			mirroredRomSize=0x100000;
		while (mirroredRomAddress<0x01000000)
		{
			memcpy((u16 *)(rom+mirroredRomAddress), (u16 *)(rom), mirroredRomSize);
			mirroredRomAddress+=mirroredRomSize;
		}
	}
}

#define brightness_switch() \
      switch((BLDMOD >> 6) & 3) \
      { \
         case 2: \
               color = gfxIncreaseBrightness(color, coeff[COLY & 0x1F]); \
               break; \
         case 3: \
               color = gfxDecreaseBrightness(color, coeff[COLY & 0x1F]); \
               break; \
      }

#define alpha_blend_brightness_switch() \
      if(top2 & (BLDMOD>>8)) \
	if(color < 0x80000000) \
	{ \
		GFX_ALPHA_BLEND(color, back, coeff[COLEV & 0x1F], coeff[(COLEV >> 8) & 0x1F]); \
	} \
      else if(BLDMOD & top) \
      { \
         brightness_switch(); \
      }

/* we only use 16bit color depth */
#define INIT_COLOR_DEPTH_LINE_MIX() u16 * lineMix = (pix + PIX_BUFFER_SCREEN_WIDTH * io_registers[REG_VCOUNT])

#include "gba_mode0.inl"
#include "gba_mode1.inl"
#include "gba_mode2.inl"
#include "gba_mode3.inl"
#include "gba_mode4.inl"
#include "gba_mode5.inl"

static void (*renderLine)(void) = mode0RenderLine;
static bool render_line_all_enabled = false;

#define CPUUpdateRender() \
  render_line_all_enabled = false; \
  switch(io_registers[REG_DISPCNT] & 7) { \
  case 0: \
    if((!fxOn && !windowOn && !(graphics.layerEnable & 0x8000))) \
      renderLine = mode0RenderLine; \
    else if(fxOn && !windowOn && !(graphics.layerEnable & 0x8000)) \
      renderLine = mode0RenderLineNoWindow; \
    else { \
      renderLine = mode0RenderLineAll; \
      render_line_all_enabled = true; \
    } \
    break; \
  case 1: \
    if((!fxOn && !windowOn && !(graphics.layerEnable & 0x8000))) \
      renderLine = mode1RenderLine; \
    else if(fxOn && !windowOn && !(graphics.layerEnable & 0x8000)) \
      renderLine = mode1RenderLineNoWindow; \
    else { \
      renderLine = mode1RenderLineAll; \
      render_line_all_enabled = true; \
    } \
    break; \
  case 2: \
    if((!fxOn && !windowOn && !(graphics.layerEnable & 0x8000))) \
      renderLine = mode2RenderLine; \
    else if(fxOn && !windowOn && !(graphics.layerEnable & 0x8000)) \
      renderLine = mode2RenderLineNoWindow; \
    else { \
      renderLine = mode2RenderLineAll; \
      render_line_all_enabled = true; \
    } \
    break; \
  case 3: \
    if((!fxOn && !windowOn && !(graphics.layerEnable & 0x8000))) \
      renderLine = mode3RenderLine; \
    else if(fxOn && !windowOn && !(graphics.layerEnable & 0x8000)) \
      renderLine = mode3RenderLineNoWindow; \
    else { \
      renderLine = mode3RenderLineAll; \
      render_line_all_enabled = true; \
    } \
    break; \
  case 4: \
    if((!fxOn && !windowOn && !(graphics.layerEnable & 0x8000))) \
      renderLine = mode4RenderLine; \
    else if(fxOn && !windowOn && !(graphics.layerEnable & 0x8000)) \
      renderLine = mode4RenderLineNoWindow; \
    else { \
      renderLine = mode4RenderLineAll; \
      render_line_all_enabled = true; \
    } \
    break; \
  case 5: \
    if((!fxOn && !windowOn && !(graphics.layerEnable & 0x8000))) \
      renderLine = mode5RenderLine; \
    else if(fxOn && !windowOn && !(graphics.layerEnable & 0x8000)) \
      renderLine = mode5RenderLineNoWindow; \
    else { \
      renderLine = mode5RenderLineAll; \
      render_line_all_enabled = true; \
    } \
  }

bool CPUReadState(const u8* data, unsigned size)
{
	// Don't really care about version.
	int version = utilReadIntMem(data);
	if (version != SAVE_GAME_VERSION)
		return false;

	char romname[16];
	utilReadMem(romname, data, 16);
	if (memcmp(&rom[0xa0], romname, 16) != 0)
		return false;

	// Don't care about use bios ...
	utilReadIntMem(data);

	utilReadMem(&bus.reg[0], data, sizeof(bus.reg));

	utilReadDataMem(data, saveGameStruct);

	stopState = utilReadIntMem(data) ? true : false;

	IRQTicks = utilReadIntMem(data);
	if (IRQTicks > 0)
		intState = true;
	else
	{
		intState = false;
		IRQTicks = 0;
	}

	utilReadMem(internalRAM, data, 0x8000);
	utilReadMem(graphics.paletteRAM, data, 0x400);
	utilReadMem(workRAM, data, 0x40000);
	utilReadMem(vram, data, 0x20000);
	utilReadMem(oam, data, 0x400);
	utilReadMem(pix, data, 4* PIX_BUFFER_SCREEN_WIDTH * 160);
	utilReadMem(ioMem, data, 0x400);

	eepromReadGameMem(data, version);
	flashReadGameMem(data, version);
	soundReadGameMem(data, version);
	rtcReadGameMem(data);

	//// Copypasta stuff ...
	// set pointers!
	graphics.layerEnable = io_registers[REG_DISPCNT];

	CPUUpdateRender();

	memset(line[0], -1, 240 * sizeof(u32));
	memset(line[1], -1, 240 * sizeof(u32));
	memset(line[2], -1, 240 * sizeof(u32));
	memset(line[3], -1, 240 * sizeof(u32));

	CPUUpdateWindow0();
	CPUUpdateWindow1();
	gbaSaveType = 0;
	switch(saveType) {
		case 0:
			cpuSaveGameFunc = flashSaveDecide;
			break;
		case 1:
			cpuSaveGameFunc = sramWrite;
			gbaSaveType = 1;
			break;
		case 2:
			cpuSaveGameFunc = flashWrite;
			gbaSaveType = 2;
			break;
		case 3:
			break;
		case 5:
			gbaSaveType = 5;
			break;
		default:
			break;
	}
	if(eepromInUse)
		gbaSaveType = 3;

	if(armState) {
		ARM_PREFETCH;
	} else {
		THUMB_PREFETCH;
	}

	CPUUpdateRegister(0x204, CPUReadHalfWordQuick(0x4000204));

	return true;
}


#define CPUSwap(a, b) \
a ^= b; \
b ^= a; \
a ^= b;

static void CPUSwitchMode(int mode, bool saveState, bool breakLoop)
{
	CPU_UPDATE_CPSR();

	switch(armMode) {
		case 0x10:
		case 0x1F:
			bus.reg[R13_USR].I = bus.reg[13].I;
			bus.reg[R14_USR].I = bus.reg[14].I;
			bus.reg[17].I = bus.reg[16].I;
			break;
		case 0x11:
			CPUSwap(bus.reg[R8_FIQ].I, bus.reg[8].I);
			CPUSwap(bus.reg[R9_FIQ].I, bus.reg[9].I);
			CPUSwap(bus.reg[R10_FIQ].I, bus.reg[10].I);
			CPUSwap(bus.reg[R11_FIQ].I, bus.reg[11].I);
			CPUSwap(bus.reg[R12_FIQ].I, bus.reg[12].I);
			bus.reg[R13_FIQ].I = bus.reg[13].I;
			bus.reg[R14_FIQ].I = bus.reg[14].I;
			bus.reg[SPSR_FIQ].I = bus.reg[17].I;
			break;
		case 0x12:
			bus.reg[R13_IRQ].I  = bus.reg[13].I;
			bus.reg[R14_IRQ].I  = bus.reg[14].I;
			bus.reg[SPSR_IRQ].I =  bus.reg[17].I;
			break;
		case 0x13:
			bus.reg[R13_SVC].I  = bus.reg[13].I;
			bus.reg[R14_SVC].I  = bus.reg[14].I;
			bus.reg[SPSR_SVC].I =  bus.reg[17].I;
			break;
		case 0x17:
			bus.reg[R13_ABT].I  = bus.reg[13].I;
			bus.reg[R14_ABT].I  = bus.reg[14].I;
			bus.reg[SPSR_ABT].I =  bus.reg[17].I;
			break;
		case 0x1b:
			bus.reg[R13_UND].I  = bus.reg[13].I;
			bus.reg[R14_UND].I  = bus.reg[14].I;
			bus.reg[SPSR_UND].I =  bus.reg[17].I;
			break;
	}

	u32 CPSR = bus.reg[16].I;
	u32 SPSR = bus.reg[17].I;

	switch(mode) {
		case 0x10:
		case 0x1F:
			bus.reg[13].I = bus.reg[R13_USR].I;
			bus.reg[14].I = bus.reg[R14_USR].I;
			bus.reg[16].I = SPSR;
			break;
		case 0x11:
			CPUSwap(bus.reg[8].I, bus.reg[R8_FIQ].I);
			CPUSwap(bus.reg[9].I, bus.reg[R9_FIQ].I);
			CPUSwap(bus.reg[10].I, bus.reg[R10_FIQ].I);
			CPUSwap(bus.reg[11].I, bus.reg[R11_FIQ].I);
			CPUSwap(bus.reg[12].I, bus.reg[R12_FIQ].I);
			bus.reg[13].I = bus.reg[R13_FIQ].I;
			bus.reg[14].I = bus.reg[R14_FIQ].I;
			if(saveState)
				bus.reg[17].I = CPSR; else
				bus.reg[17].I = bus.reg[SPSR_FIQ].I;
			break;
		case 0x12:
			bus.reg[13].I = bus.reg[R13_IRQ].I;
			bus.reg[14].I = bus.reg[R14_IRQ].I;
			bus.reg[16].I = SPSR;
			if(saveState)
				bus.reg[17].I = CPSR;
			else
				bus.reg[17].I = bus.reg[SPSR_IRQ].I;
			break;
		case 0x13:
			bus.reg[13].I = bus.reg[R13_SVC].I;
			bus.reg[14].I = bus.reg[R14_SVC].I;
			bus.reg[16].I = SPSR;
			if(saveState)
				bus.reg[17].I = CPSR;
			else
				bus.reg[17].I = bus.reg[SPSR_SVC].I;
			break;
		case 0x17:
			bus.reg[13].I = bus.reg[R13_ABT].I;
			bus.reg[14].I = bus.reg[R14_ABT].I;
			bus.reg[16].I = SPSR;
			if(saveState)
				bus.reg[17].I = CPSR;
			else
				bus.reg[17].I = bus.reg[SPSR_ABT].I;
			break;
		case 0x1b:
			bus.reg[13].I = bus.reg[R13_UND].I;
			bus.reg[14].I = bus.reg[R14_UND].I;
			bus.reg[16].I = SPSR;
			if(saveState)
				bus.reg[17].I = CPSR;
			else
				bus.reg[17].I = bus.reg[SPSR_UND].I;
			break;
		default:
			break;
	}
	armMode = mode;
	CPUUpdateFlags(breakLoop);
	CPU_UPDATE_CPSR();
}



void doDMA(u32 &s, u32 &d, u32 si, u32 di, u32 c, int transfer32)
{
	int sm = s >> 24;
	int dm = d >> 24;
	int sw = 0;
	int dw = 0;
	int sc = c;

	cpuDmaCount = c;
	// This is done to get the correct waitstates.
	int32_t sm_gt_15_mask = ((sm>15) | -(sm>15)) >> 31;
	int32_t dm_gt_15_mask = ((dm>15) | -(dm>15)) >> 31;
	sm = ((((15) & sm_gt_15_mask) | ((((sm) & ~(sm_gt_15_mask))))));
	dm = ((((15) & dm_gt_15_mask) | ((((dm) & ~(dm_gt_15_mask))))));

	//if ((sm>=0x05) && (sm<=0x07) || (dm>=0x05) && (dm <=0x07))
	//    blank = (((io_registers[REG_DISPSTAT] | ((io_registers[REG_DISPSTAT] >> 1)&1))==1) ?  true : false);

	if(transfer32)
	{
		s &= 0xFFFFFFFC;
		if(s < 0x02000000 && (bus.reg[15].I >> 24))
		{
			do
			{
				CPUWriteMemory(d, 0);
				d += di;
				c--;
			}while(c != 0);
		}
		else
		{
			do {
				CPUWriteMemory(d, CPUReadMemory(s));
				d += di;
				s += si;
				c--;
			}while(c != 0);
		}
	}
	else
	{
		s &= 0xFFFFFFFE;
		si = (int)si >> 1;
		di = (int)di >> 1;
		if(s < 0x02000000 && (bus.reg[15].I >> 24))
		{
			do {
				CPUWriteHalfWord(d, 0);
				d += di;
				c--;
			}while(c != 0);
		}
		else
		{
			do{
				CPUWriteHalfWord(d, CPUReadHalfWord(s));
				d += di;
				s += si;
				c--;
			}while(c != 0);
		}
	}

	cpuDmaCount = 0;

	if(transfer32)
	{
		sw = 1+memoryWaitSeq32[sm & 15];
		dw = 1+memoryWaitSeq32[dm & 15];
		cpuDmaTicksToUpdate += (sw+dw)*(sc-1) + 6 + memoryWait32[sm & 15] + memoryWaitSeq32[dm & 15];
	}
	else
	{
		sw = 1+memoryWaitSeq[sm & 15];
		dw = 1+memoryWaitSeq[dm & 15];
		cpuDmaTicksToUpdate += (sw+dw)*(sc-1) + 6 + memoryWait[sm & 15] + memoryWaitSeq[dm & 15];
	}
}


void CPUCheckDMA(int reason, int dmamask)
{
	u32 arrayval[] = {4, (u32)-4, 0, 4};
	// DMA 0
	if((DM0CNT_H & 0x8000) && (dmamask & 1))
	{
		if(((DM0CNT_H >> 12) & 3) == reason)
		{
			u32 sourceIncrement, destIncrement;
			u32 condition1 = ((DM0CNT_H >> 7) & 3);
			u32 condition2 = ((DM0CNT_H >> 5) & 3);
			sourceIncrement = arrayval[condition1];
			destIncrement = arrayval[condition2];
			doDMA(dma0Source, dma0Dest, sourceIncrement, destIncrement,
					DM0CNT_L ? DM0CNT_L : 0x4000,
					DM0CNT_H & 0x0400);

			if(DM0CNT_H & 0x4000)
			{
				io_registers[REG_IF] |= 0x0100;
				UPDATE_REG(0x202, io_registers[REG_IF]);
				cpuNextEvent = cpuTotalTicks;
			}

			if(((DM0CNT_H >> 5) & 3) == 3) {
				dma0Dest = DM0DAD_L | (DM0DAD_H << 16);
			}

			if(!(DM0CNT_H & 0x0200) || (reason == 0)) {
				DM0CNT_H &= 0x7FFF;
				UPDATE_REG(0xBA, DM0CNT_H);
			}
		}
	}

	// DMA 1
	if((DM1CNT_H & 0x8000) && (dmamask & 2)) {
		if(((DM1CNT_H >> 12) & 3) == reason) {
			u32 sourceIncrement, destIncrement;
			u32 condition1 = ((DM1CNT_H >> 7) & 3);
			u32 condition2 = ((DM1CNT_H >> 5) & 3);
			sourceIncrement = arrayval[condition1];
			destIncrement = arrayval[condition2];
			u32 di_value, c_value, transfer_value;
			if(reason == 3)
			{
				di_value = 0;
				c_value = 4;
				transfer_value = 0x0400;
			}
			else
			{
				di_value = destIncrement;
				c_value = DM1CNT_L ? DM1CNT_L : 0x4000;
				transfer_value = DM1CNT_H & 0x0400;
			}
			doDMA(dma1Source, dma1Dest, sourceIncrement, di_value, c_value, transfer_value);

			if(DM1CNT_H & 0x4000) {
				io_registers[REG_IF] |= 0x0200;
				UPDATE_REG(0x202, io_registers[REG_IF]);
				cpuNextEvent = cpuTotalTicks;
			}

			if(((DM1CNT_H >> 5) & 3) == 3) {
				dma1Dest = DM1DAD_L | (DM1DAD_H << 16);
			}

			if(!(DM1CNT_H & 0x0200) || (reason == 0)) {
				DM1CNT_H &= 0x7FFF;
				UPDATE_REG(0xC6, DM1CNT_H);
			}
		}
	}

	// DMA 2
	if((DM2CNT_H & 0x8000) && (dmamask & 4)) {
		if(((DM2CNT_H >> 12) & 3) == reason) {
			u32 sourceIncrement, destIncrement;
			u32 condition1 = ((DM2CNT_H >> 7) & 3);
			u32 condition2 = ((DM2CNT_H >> 5) & 3);
			sourceIncrement = arrayval[condition1];
			destIncrement = arrayval[condition2];
			u32 di_value, c_value, transfer_value;
			if(reason == 3)
			{
				di_value = 0;
				c_value = 4;
				transfer_value = 0x0400;
			}
			else
			{
				di_value = destIncrement;
				c_value = DM2CNT_L ? DM2CNT_L : 0x4000;
				transfer_value = DM2CNT_H & 0x0400;
			}
			doDMA(dma2Source, dma2Dest, sourceIncrement, di_value, c_value, transfer_value);

			if(DM2CNT_H & 0x4000) {
				io_registers[REG_IF] |= 0x0400;
				UPDATE_REG(0x202, io_registers[REG_IF]);
				cpuNextEvent = cpuTotalTicks;
			}

			if(((DM2CNT_H >> 5) & 3) == 3) {
				dma2Dest = DM2DAD_L | (DM2DAD_H << 16);
			}

			if(!(DM2CNT_H & 0x0200) || (reason == 0)) {
				DM2CNT_H &= 0x7FFF;
				UPDATE_REG(0xD2, DM2CNT_H);
			}
		}
	}

	// DMA 3
	if((DM3CNT_H & 0x8000) && (dmamask & 8))
	{
		if(((DM3CNT_H >> 12) & 3) == reason)
		{
			u32 sourceIncrement, destIncrement;
			u32 condition1 = ((DM3CNT_H >> 7) & 3);
			u32 condition2 = ((DM3CNT_H >> 5) & 3);
			sourceIncrement = arrayval[condition1];
			destIncrement = arrayval[condition2];
			doDMA(dma3Source, dma3Dest, sourceIncrement, destIncrement,
					DM3CNT_L ? DM3CNT_L : 0x10000,
					DM3CNT_H & 0x0400);
			if(DM3CNT_H & 0x4000) {
				io_registers[REG_IF] |= 0x0800;
				UPDATE_REG(0x202, io_registers[REG_IF]);
				cpuNextEvent = cpuTotalTicks;
			}

			if(((DM3CNT_H >> 5) & 3) == 3) {
				dma3Dest = DM3DAD_L | (DM3DAD_H << 16);
			}

			if(!(DM3CNT_H & 0x0200) || (reason == 0)) {
				DM3CNT_H &= 0x7FFF;
				UPDATE_REG(0xDE, DM3CNT_H);
			}
		}
	}
}

static u16 *address_lut[0x300];

void CPUUpdateRegister(u32 address, u16 value)
{
	switch(address)
	{
		case 0x00:
			{
				if((value & 7) > 5) // display modes above 0-5 are prohibited
					io_registers[REG_DISPCNT] = (value & 7);

				bool change = (0 != ((io_registers[REG_DISPCNT] ^ value) & 0x80));
				bool changeBG = (0 != ((io_registers[REG_DISPCNT] ^ value) & 0x0F00));
				u16 changeBGon = ((~io_registers[REG_DISPCNT]) & value) & 0x0F00; // these layers are being activated

				io_registers[REG_DISPCNT] = (value & 0xFFF7); // bit 3 can only be accessed by the BIOS to enable GBC mode
				UPDATE_REG(0x00, io_registers[REG_DISPCNT]);

				graphics.layerEnable = value;

				if(changeBGon)
				{
					graphics.layerEnableDelay = 4;
					graphics.layerEnable &= ~changeBGon;
				}

				windowOn = (graphics.layerEnable & 0x6000) ? true : false;
				if(change && !((value & 0x80)))
				{
					if(!(io_registers[REG_DISPSTAT] & 1))
					{
						graphics.lcdTicks = 1008;
						io_registers[REG_DISPSTAT] &= 0xFFFC;
						UPDATE_REG(0x04, io_registers[REG_DISPSTAT]);
						CPUCompareVCOUNT();
					}
				}
				CPUUpdateRender();
				// we only care about changes in BG0-BG3
				if(changeBG)
				{
					if(!(graphics.layerEnable & 0x0100))
						memset(line[0], -1, 240 * sizeof(u32));
					if(!(graphics.layerEnable & 0x0200))
						memset(line[1], -1, 240 * sizeof(u32));
					if(!(graphics.layerEnable & 0x0400))
						memset(line[2], -1, 240 * sizeof(u32));
					if(!(graphics.layerEnable & 0x0800))
						memset(line[3], -1, 240 * sizeof(u32));
				}
				break;
			}
		case 0x04:
			io_registers[REG_DISPSTAT] = (value & 0xFF38) | (io_registers[REG_DISPSTAT] & 7);
			UPDATE_REG(0x04, io_registers[REG_DISPSTAT]);
			break;
		case 0x06:
			// not writable
			break;
		case 0x08: /* BG0CNT */
		case 0x0A: /* BG1CNT */
			*address_lut[address] = (value & 0xDFCF);
			UPDATE_REG(address, *address_lut[address]);
			break;
		case 0x0C: /* BG2CNT */
		case 0x0E: /* BG3CNT */
			*address_lut[address] = (value & 0xFFCF);
			UPDATE_REG(address, *address_lut[address]);
			break;
		case 0x10: /* BG0HOFS */
		case 0x12: /* BG0VOFS */
		case 0x14: /* BG1HOFS */
		case 0x16: /* BG1VOFS */
		case 0x18: /* BG2HOFS */
		case 0x1A: /* BG2VOFS */
		case 0x1C: /* BG3HOFS */
		case 0x1E: /* BG3VOFS */
			*address_lut[address] = value & 511;
			UPDATE_REG(address, *address_lut[address]);
			break;
		case 0x20: /* BG2PA */
		case 0x22: /* BG2PB */
		case 0x24: /* BG2PC */
		case 0x26: /* BG2PD */
			*address_lut[address] = value;
			UPDATE_REG(address, *address_lut[address]);
			break;
		case 0x28:
			BG2X_L = value;
			UPDATE_REG(0x28, BG2X_L);
			gfxBG2Changed |= 1;
			break;
		case 0x2A:
			BG2X_H = (value & 0xFFF);
			UPDATE_REG(0x2A, BG2X_H);
			gfxBG2Changed |= 1;
			break;
		case 0x2C:
			BG2Y_L = value;
			UPDATE_REG(0x2C, BG2Y_L);
			gfxBG2Changed |= 2;
			break;
		case 0x2E:
			BG2Y_H = value & 0xFFF;
			UPDATE_REG(0x2E, BG2Y_H);
			gfxBG2Changed |= 2;
			break;
		case 0x30: /* BG3PA */
		case 0x32: /* BG3PB */
		case 0x34: /* BG3PC */
		case 0x36: /* BG3PD */
			*address_lut[address] = value;
			UPDATE_REG(address, *address_lut[address]);
			break;
		case 0x38:
			BG3X_L = value;
			UPDATE_REG(0x38, BG3X_L);
			gfxBG3Changed |= 1;
			break;
		case 0x3A:
			BG3X_H = value & 0xFFF;
			UPDATE_REG(0x3A, BG3X_H);
			gfxBG3Changed |= 1;
			break;
		case 0x3C:
			BG3Y_L = value;
			UPDATE_REG(0x3C, BG3Y_L);
			gfxBG3Changed |= 2;
			break;
		case 0x3E:
			BG3Y_H = value & 0xFFF;
			UPDATE_REG(0x3E, BG3Y_H);
			gfxBG3Changed |= 2;
			break;
		case 0x40:
			io_registers[REG_WIN0H] = value;
			UPDATE_REG(0x40, io_registers[REG_WIN0H]);
			CPUUpdateWindow0();
			break;
		case 0x42:
			io_registers[REG_WIN1H] = value;
			UPDATE_REG(0x42, io_registers[REG_WIN1H]);
			CPUUpdateWindow1();
			break;
		case 0x44:
		case 0x46:
			*address_lut[address] = value;
			UPDATE_REG(address, *address_lut[address]);
			break;
		case 0x48: /* WININ */
		case 0x4A: /* WINOUT */
			*address_lut[address] = value & 0x3F3F;
			UPDATE_REG(address, *address_lut[address]);
			break;
		case 0x4C:
			MOSAIC = value;
			UPDATE_REG(0x4C, MOSAIC);
			break;
		case 0x50:
			BLDMOD = value & 0x3FFF;
			UPDATE_REG(0x50, BLDMOD);
			fxOn = ((BLDMOD>>6)&3) != 0;
			CPUUpdateRender();
			break;
		case 0x52:
			COLEV = value & 0x1F1F;
			UPDATE_REG(0x52, COLEV);
			break;
		case 0x54:
			COLY = value & 0x1F;
			UPDATE_REG(0x54, COLY);
			break;
		case 0x60:
		case 0x62:
		case 0x64:
		case 0x68:
		case 0x6c:
		case 0x70:
		case 0x72:
		case 0x74:
		case 0x78:
		case 0x7c:
		case 0x80:
		case 0x84:
			{
				int gb_addr[2] = {address & 0xFF, (address & 0xFF) + 1};
				u32 address_array[2] = {address & 0xFF, (address&0xFF)+1};
				u8 data_array[2] = {(u8)(value & 0xFF), (u8)(value>>8)};
				gb_addr[0] = table[gb_addr[0] - 0x60];
				gb_addr[1] = table[gb_addr[1] - 0x60];
				soundEvent_u8_parallel(gb_addr, address_array, data_array);
				break;
			}
		case 0x82:
		case 0x88:
		case 0xa0:
		case 0xa2:
		case 0xa4:
		case 0xa6:
		case 0x90:
		case 0x92:
		case 0x94:
		case 0x96:
		case 0x98:
		case 0x9a:
		case 0x9c:
		case 0x9e:
			soundEvent_u16(address&0xFF, value);
			break;
		case 0xB0:
			DM0SAD_L = value;
			UPDATE_REG(0xB0, DM0SAD_L);
			break;
		case 0xB2:
			DM0SAD_H = value & 0x07FF;
			UPDATE_REG(0xB2, DM0SAD_H);
			break;
		case 0xB4:
			DM0DAD_L = value;
			UPDATE_REG(0xB4, DM0DAD_L);
			break;
		case 0xB6:
			DM0DAD_H = value & 0x07FF;
			UPDATE_REG(0xB6, DM0DAD_H);
			break;
		case 0xB8:
			DM0CNT_L = value & 0x3FFF;
			UPDATE_REG(0xB8, 0);
			break;
		case 0xBA:
			{
				bool start = ((DM0CNT_H ^ value) & 0x8000) ? true : false;
				value &= 0xF7E0;

				DM0CNT_H = value;
				UPDATE_REG(0xBA, DM0CNT_H);

				if(start && (value & 0x8000))
				{
					dma0Source = DM0SAD_L | (DM0SAD_H << 16);
					dma0Dest = DM0DAD_L | (DM0DAD_H << 16);
					CPUCheckDMA(0, 1);
				}
			}
			break;
		case 0xBC:
			DM1SAD_L = value;
			UPDATE_REG(0xBC, DM1SAD_L);
			break;
		case 0xBE:
			DM1SAD_H = value & 0x0FFF;
			UPDATE_REG(0xBE, DM1SAD_H);
			break;
		case 0xC0:
			DM1DAD_L = value;
			UPDATE_REG(0xC0, DM1DAD_L);
			break;
		case 0xC2:
			DM1DAD_H = value & 0x07FF;
			UPDATE_REG(0xC2, DM1DAD_H);
			break;
		case 0xC4:
			DM1CNT_L = value & 0x3FFF;
			UPDATE_REG(0xC4, 0);
			break;
		case 0xC6:
			{
				bool start = ((DM1CNT_H ^ value) & 0x8000) ? true : false;
				value &= 0xF7E0;

				DM1CNT_H = value;
				UPDATE_REG(0xC6, DM1CNT_H);

				if(start && (value & 0x8000))
				{
					dma1Source = DM1SAD_L | (DM1SAD_H << 16);
					dma1Dest = DM1DAD_L | (DM1DAD_H << 16);
					CPUCheckDMA(0, 2);
				}
			}
			break;
		case 0xC8:
			DM2SAD_L = value;
			UPDATE_REG(0xC8, DM2SAD_L);
			break;
		case 0xCA:
			DM2SAD_H = value & 0x0FFF;
			UPDATE_REG(0xCA, DM2SAD_H);
			break;
		case 0xCC:
			DM2DAD_L = value;
			UPDATE_REG(0xCC, DM2DAD_L);
			break;
		case 0xCE:
			DM2DAD_H = value & 0x07FF;
			UPDATE_REG(0xCE, DM2DAD_H);
			break;
		case 0xD0:
			DM2CNT_L = value & 0x3FFF;
			UPDATE_REG(0xD0, 0);
			break;
		case 0xD2:
			{
				bool start = ((DM2CNT_H ^ value) & 0x8000) ? true : false;

				value &= 0xF7E0;

				DM2CNT_H = value;
				UPDATE_REG(0xD2, DM2CNT_H);

				if(start && (value & 0x8000)) {
					dma2Source = DM2SAD_L | (DM2SAD_H << 16);
					dma2Dest = DM2DAD_L | (DM2DAD_H << 16);

					CPUCheckDMA(0, 4);
				}
			}
			break;
		case 0xD4:
			DM3SAD_L = value;
			UPDATE_REG(0xD4, DM3SAD_L);
			break;
		case 0xD6:
			DM3SAD_H = value & 0x0FFF;
			UPDATE_REG(0xD6, DM3SAD_H);
			break;
		case 0xD8:
			DM3DAD_L = value;
			UPDATE_REG(0xD8, DM3DAD_L);
			break;
		case 0xDA:
			DM3DAD_H = value & 0x0FFF;
			UPDATE_REG(0xDA, DM3DAD_H);
			break;
		case 0xDC:
			DM3CNT_L = value;
			UPDATE_REG(0xDC, 0);
			break;
		case 0xDE:
			{
				bool start = ((DM3CNT_H ^ value) & 0x8000) ? true : false;

				value &= 0xFFE0;

				DM3CNT_H = value;
				UPDATE_REG(0xDE, DM3CNT_H);

				if(start && (value & 0x8000)) {
					dma3Source = DM3SAD_L | (DM3SAD_H << 16);
					dma3Dest = DM3DAD_L | (DM3DAD_H << 16);
					CPUCheckDMA(0,8);
				}
			}
			break;
		case 0x100:
			timer0Reload = value;
			break;
		case 0x102:
			timer0Value = value;
			timerOnOffDelay|=1;
			cpuNextEvent = cpuTotalTicks;
			break;
		case 0x104:
			timer1Reload = value;
			break;
		case 0x106:
			timer1Value = value;
			timerOnOffDelay|=2;
			cpuNextEvent = cpuTotalTicks;
			break;
		case 0x108:
			timer2Reload = value;
			break;
		case 0x10A:
			timer2Value = value;
			timerOnOffDelay|=4;
			cpuNextEvent = cpuTotalTicks;
			break;
		case 0x10C:
			timer3Reload = value;
			break;
		case 0x10E:
			timer3Value = value;
			timerOnOffDelay|=8;
			cpuNextEvent = cpuTotalTicks;
			break;
		case 0x130:
			io_registers[REG_P1] |= (value & 0x3FF);
			UPDATE_REG(0x130, io_registers[REG_P1]);
			break;
		case 0x132:
			UPDATE_REG(0x132, value & 0xC3FF);
			break;


		case 0x200:
			io_registers[REG_IE] = value & 0x3FFF;
			UPDATE_REG(0x200, io_registers[REG_IE]);
			if ((io_registers[REG_IME] & 1) && (io_registers[REG_IF] & io_registers[REG_IE]) && armIrqEnable)
				cpuNextEvent = cpuTotalTicks;
			break;
		case 0x202:
			io_registers[REG_IF] ^= (value & io_registers[REG_IF]);
			UPDATE_REG(0x202, io_registers[REG_IF]);
			break;
		case 0x204:
			{
				memoryWait[0x0e] = memoryWaitSeq[0x0e] = gamepakRamWaitState[value & 3];

				memoryWait[0x08] = memoryWait[0x09] = 3;
				memoryWaitSeq[0x08] = memoryWaitSeq[0x09] = 1;

				memoryWait[0x0a] = memoryWait[0x0b] = 3;
				memoryWaitSeq[0x0a] = memoryWaitSeq[0x0b] = 1;

				memoryWait[0x0c] = memoryWait[0x0d] = 3;
				memoryWaitSeq[0x0c] = memoryWaitSeq[0x0d] = 1;

				memoryWait32[8] = memoryWait[8] + memoryWaitSeq[8] + 1;
				memoryWaitSeq32[8] = memoryWaitSeq[8]*2 + 1;

				memoryWait32[9] = memoryWait[9] + memoryWaitSeq[9] + 1;
				memoryWaitSeq32[9] = memoryWaitSeq[9]*2 + 1;

				memoryWait32[10] = memoryWait[10] + memoryWaitSeq[10] + 1;
				memoryWaitSeq32[10] = memoryWaitSeq[10]*2 + 1;

				memoryWait32[11] = memoryWait[11] + memoryWaitSeq[11] + 1;
				memoryWaitSeq32[11] = memoryWaitSeq[11]*2 + 1;

				memoryWait32[12] = memoryWait[12] + memoryWaitSeq[12] + 1;
				memoryWaitSeq32[12] = memoryWaitSeq[12]*2 + 1;

				memoryWait32[13] = memoryWait[13] + memoryWaitSeq[13] + 1;
				memoryWaitSeq32[13] = memoryWaitSeq[13]*2 + 1;

				memoryWait32[14] = memoryWait[14] + memoryWaitSeq[14] + 1;
				memoryWaitSeq32[14] = memoryWaitSeq[14]*2 + 1;

				if((value & 0x4000) == 0x4000)
					bus.busPrefetchEnable = true;
				else
					bus.busPrefetchEnable = false;

				bus.busPrefetch = false;
				bus.busPrefetchCount = 0;

				UPDATE_REG(0x204, value & 0x7FFF);

			}
			break;
		case 0x208:
			io_registers[REG_IME] = value & 1;
			UPDATE_REG(0x208, io_registers[REG_IME]);
			if ((io_registers[REG_IME] & 1) && (io_registers[REG_IF] & io_registers[REG_IE]) && armIrqEnable)
				cpuNextEvent = cpuTotalTicks;
			break;
		case 0x300:
			if(value != 0)
				value &= 0xFFFE;
			UPDATE_REG(0x300, value);
			break;
		default:
			UPDATE_REG(address&0x3FE, value);
			break;
	}
}


void CPUInit(const char *biosFileName, bool useBiosFile)
{
#ifndef LSB_FIRST
	if(!cpuBiosSwapped) {
		for(unsigned int i = 0; i < sizeof(myROM)/4; i++) {
			WRITE32LE(&myROM[i], myROM[i]);
		}
		cpuBiosSwapped = true;
	}
#endif
	gbaSaveType = 0;
	eepromInUse = 0;
	saveType = 0;
	useBios = false;

#ifdef HAVE_HLE_BIOS
	if(useBiosFile)
	{
		int size = 0x4000;
		if(utilLoad(biosFileName, CPUIsGBABios, bios, size))
		{
			if(size == 0x4000)
				useBios = true;
		}
	}
#endif

#ifdef HAVE_HLE_BIOS
	if(!useBios)
#endif
		memcpy(bios, myROM, sizeof(myROM));

	int i = 0;

	biosProtected[0] = 0x00;
	biosProtected[1] = 0xf0;
	biosProtected[2] = 0x29;
	biosProtected[3] = 0xe1;

	for(i = 0; i < 256; i++)
	{
		int count = 0;
		int j;
		for(j = 0; j < 8; j++)
			if(i & (1 << j))
				count++;
		cpuBitsSet[i] = count;

		for(j = 0; j < 8; j++)
			if(i & (1 << j))
				break;
	}

	for(i = 0; i < 0x400; i++)
		ioReadable[i] = true;
	for(i = 0x10; i < 0x48; i++)
		ioReadable[i] = false;
	for(i = 0x4c; i < 0x50; i++)
		ioReadable[i] = false;
	for(i = 0x54; i < 0x60; i++)
		ioReadable[i] = false;
	for(i = 0x8c; i < 0x90; i++)
		ioReadable[i] = false;
	for(i = 0xa0; i < 0xb8; i++)
		ioReadable[i] = false;
	for(i = 0xbc; i < 0xc4; i++)
		ioReadable[i] = false;
	for(i = 0xc8; i < 0xd0; i++)
		ioReadable[i] = false;
	for(i = 0xd4; i < 0xdc; i++)
		ioReadable[i] = false;
	for(i = 0xe0; i < 0x100; i++)
		ioReadable[i] = false;
	for(i = 0x110; i < 0x120; i++)
		ioReadable[i] = false;
	for(i = 0x12c; i < 0x130; i++)
		ioReadable[i] = false;
	for(i = 0x138; i < 0x140; i++)
		ioReadable[i] = false;
	for(i = 0x144; i < 0x150; i++)
		ioReadable[i] = false;
	for(i = 0x15c; i < 0x200; i++)
		ioReadable[i] = false;
	for(i = 0x20c; i < 0x300; i++)
		ioReadable[i] = false;
	for(i = 0x304; i < 0x400; i++)
		ioReadable[i] = false;

	if(romSize < 0x1fe2000) {
		*((u16 *)&rom[0x1fe209c]) = 0xdffa; // SWI 0xFA
		*((u16 *)&rom[0x1fe209e]) = 0x4770; // BX LR
	}

	graphics.layerEnable = 0xff00;
	graphics.layerEnableDelay = 1;
	io_registers[REG_DISPCNT] = 0x0080;
	io_registers[REG_DISPSTAT] = 0;
	graphics.lcdTicks = (useBios && !skipBios) ? 1008 : 208;

	/* address lut for use in CPUUpdateRegister */
	address_lut[0x08] = &io_registers[REG_BG0CNT];
	address_lut[0x0A] = &io_registers[REG_BG1CNT];
	address_lut[0x0C] = &io_registers[REG_BG2CNT];
	address_lut[0x0E] = &io_registers[REG_BG3CNT];
	address_lut[0x10] = &io_registers[REG_BG0HOFS];
	address_lut[0x12] = &io_registers[REG_BG0VOFS];
	address_lut[0x14] = &io_registers[REG_BG1HOFS];
	address_lut[0x16] = &io_registers[REG_BG1VOFS];
	address_lut[0x18] = &io_registers[REG_BG2HOFS];
	address_lut[0x1A] = &io_registers[REG_BG2VOFS];
	address_lut[0x1C] = &io_registers[REG_BG3HOFS];
	address_lut[0x1E] = &io_registers[REG_BG3VOFS];
	address_lut[0x20] = &io_registers[REG_BG2PA];
	address_lut[0x22] = &io_registers[REG_BG2PB];
	address_lut[0x24] = &io_registers[REG_BG2PC];
	address_lut[0x26] = &io_registers[REG_BG2PD];
	address_lut[0x48] = &io_registers[REG_WININ];
	address_lut[0x4A] = &io_registers[REG_WINOUT];
	address_lut[0x30] = &io_registers[REG_BG3PA];
	address_lut[0x32] = &io_registers[REG_BG3PB];
	address_lut[0x34] = &io_registers[REG_BG3PC];
	address_lut[0x36] = &io_registers[REG_BG3PD];
        address_lut[0x40] = &io_registers[REG_WIN0H];
        address_lut[0x42] = &io_registers[REG_WIN1H];
        address_lut[0x44] = &io_registers[REG_WIN0V];
        address_lut[0x46] = &io_registers[REG_WIN1V];
}

void CPUReset (void)
{
	if(gbaSaveType == 0)
	{
		if(eepromInUse)
			gbaSaveType = 3;
		else
			switch(saveType)
			{
				case 1:
					gbaSaveType = 1;
					break;
				case 2:
					gbaSaveType = 2;
					break;
			}
	}
	rtcReset();
	memset(&bus.reg[0], 0, sizeof(bus.reg));	// clean registers
	memset(oam, 0, 0x400);				// clean OAM
	memset(graphics.paletteRAM, 0, 0x400);		// clean palette
	memset(pix, 0, 4 * 160 * 240);			// clean picture
	memset(vram, 0, 0x20000);			// clean vram
	memset(ioMem, 0, 0x400);			// clean io memory

	io_registers[REG_DISPCNT]  = 0x0080;
	io_registers[REG_DISPSTAT] = 0x0000;
	io_registers[REG_VCOUNT]   = (useBios && !skipBios) ? 0 :0x007E;
	io_registers[REG_BG0CNT]   = 0x0000;
	io_registers[REG_BG1CNT]   = 0x0000;
	io_registers[REG_BG2CNT]   = 0x0000;
	io_registers[REG_BG3CNT]   = 0x0000;
	io_registers[REG_BG0HOFS]  = 0x0000;
	io_registers[REG_BG0VOFS]  = 0x0000;
	io_registers[REG_BG1HOFS]  = 0x0000;
	io_registers[REG_BG1VOFS]  = 0x0000;
	io_registers[REG_BG2HOFS]  = 0x0000;
	io_registers[REG_BG2VOFS]  = 0x0000;
	io_registers[REG_BG3HOFS]  = 0x0000;
	io_registers[REG_BG3VOFS]  = 0x0000;
	io_registers[REG_BG2PA]    = 0x0100;
	io_registers[REG_BG2PB]    = 0x0000;
	io_registers[REG_BG2PC]    = 0x0000;
	io_registers[REG_BG2PD]    = 0x0100;
	BG2X_L   = 0x0000;
	BG2X_H   = 0x0000;
	BG2Y_L   = 0x0000;
	BG2Y_H   = 0x0000;
	io_registers[REG_BG3PA]    = 0x0100;
	io_registers[REG_BG3PB]    = 0x0000;
	io_registers[REG_BG3PC]    = 0x0000;
	io_registers[REG_BG3PD]    = 0x0100;
	BG3X_L   = 0x0000;
	BG3X_H   = 0x0000;
	BG3Y_L   = 0x0000;
	BG3Y_H   = 0x0000;
	io_registers[REG_WIN0H]    = 0x0000;
	io_registers[REG_WIN1H]    = 0x0000;
	io_registers[REG_WIN0V]    = 0x0000;
	io_registers[REG_WIN1V]    = 0x0000;
	io_registers[REG_WININ]    = 0x0000;
	io_registers[REG_WINOUT]   = 0x0000;
	MOSAIC   = 0x0000;
	BLDMOD   = 0x0000;
	COLEV    = 0x0000;
	COLY     = 0x0000;
	DM0SAD_L = 0x0000;
	DM0SAD_H = 0x0000;
	DM0DAD_L = 0x0000;
	DM0DAD_H = 0x0000;
	DM0CNT_L = 0x0000;
	DM0CNT_H = 0x0000;
	DM1SAD_L = 0x0000;
	DM1SAD_H = 0x0000;
	DM1DAD_L = 0x0000;
	DM1DAD_H = 0x0000;
	DM1CNT_L = 0x0000;
	DM1CNT_H = 0x0000;
	DM2SAD_L = 0x0000;
	DM2SAD_H = 0x0000;
	DM2DAD_L = 0x0000;
	DM2DAD_H = 0x0000;
	DM2CNT_L = 0x0000;
	DM2CNT_H = 0x0000;
	DM3SAD_L = 0x0000;
	DM3SAD_H = 0x0000;
	DM3DAD_L = 0x0000;
	DM3DAD_H = 0x0000;
	DM3CNT_L = 0x0000;
	DM3CNT_H = 0x0000;
	io_registers[REG_TM0D]     = 0x0000;
	io_registers[REG_TM0CNT]   = 0x0000;
	io_registers[REG_TM1D]     = 0x0000;
	io_registers[REG_TM1CNT]   = 0x0000;
	io_registers[REG_TM2D]     = 0x0000;
	io_registers[REG_TM2CNT]   = 0x0000;
	io_registers[REG_TM3D]     = 0x0000;
	io_registers[REG_TM3CNT]   = 0x0000;
	io_registers[REG_P1]       = 0x03FF;
	io_registers[REG_IE]       = 0x0000;
	io_registers[REG_IF]       = 0x0000;
	io_registers[REG_IME]      = 0x0000;

	armMode = 0x1F;

	if(cpuIsMultiBoot) {
		bus.reg[13].I = 0x03007F00;
		bus.reg[15].I = 0x02000000;
		bus.reg[16].I = 0x00000000;
		bus.reg[R13_IRQ].I = 0x03007FA0;
		bus.reg[R13_SVC].I = 0x03007FE0;
		armIrqEnable = true;
	} else {
#ifdef HAVE_HLE_BIOS
		if(useBios && !skipBios)
		{
			bus.reg[15].I = 0x00000000;
			armMode = 0x13;
			armIrqEnable = false;
		}
		else
		{
#endif
			bus.reg[13].I = 0x03007F00;
			bus.reg[15].I = 0x08000000;
			bus.reg[16].I = 0x00000000;
			bus.reg[R13_IRQ].I = 0x03007FA0;
			bus.reg[R13_SVC].I = 0x03007FE0;
			armIrqEnable = true;
#ifdef HAVE_HLE_BIOS
		}
#endif
	}
	armState = true;
	C_FLAG = V_FLAG = N_FLAG = Z_FLAG = false;
	UPDATE_REG(0x00, io_registers[REG_DISPCNT]);
	UPDATE_REG(0x06, io_registers[REG_VCOUNT]);
	UPDATE_REG(0x20, io_registers[REG_BG2PA]);
	UPDATE_REG(0x26, io_registers[REG_BG2PD]);
	UPDATE_REG(0x30, io_registers[REG_BG3PA]);
	UPDATE_REG(0x36, io_registers[REG_BG3PD]);
	UPDATE_REG(0x130, io_registers[REG_P1]);
	UPDATE_REG(0x88, 0x200);

	// disable FIQ
	bus.reg[16].I |= 0x40;

	CPU_UPDATE_CPSR();

	bus.armNextPC = bus.reg[15].I;
	bus.reg[15].I += 4;

	// reset internal state
	holdState = false;

	biosProtected[0] = 0x00;
	biosProtected[1] = 0xf0;
	biosProtected[2] = 0x29;
	biosProtected[3] = 0xe1;

	graphics.lcdTicks = (useBios && !skipBios) ? 1008 : 208;
	timer0On = false;
	timer0Ticks = 0;
	timer0Reload = 0;
	timer0ClockReload  = 0;
	timer1On = false;
	timer1Ticks = 0;
	timer1Reload = 0;
	timer1ClockReload  = 0;
	timer2On = false;
	timer2Ticks = 0;
	timer2Reload = 0;
	timer2ClockReload  = 0;
	timer3On = false;
	timer3Ticks = 0;
	timer3Reload = 0;
	timer3ClockReload  = 0;
	dma0Source = 0;
	dma0Dest = 0;
	dma1Source = 0;
	dma1Dest = 0;
	dma2Source = 0;
	dma2Dest = 0;
	dma3Source = 0;
	dma3Dest = 0;
	cpuSaveGameFunc = flashSaveDecide;
	renderLine = mode0RenderLine;
	fxOn = false;
	windowOn = false;
	saveType = 0;
	graphics.layerEnable = io_registers[REG_DISPCNT];

	memset(line[0], -1, 240 * sizeof(u32));
	memset(line[1], -1, 240 * sizeof(u32));
	memset(line[2], -1, 240 * sizeof(u32));
	memset(line[3], -1, 240 * sizeof(u32));

	for(int i = 0; i < 256; i++) {
		map[i].address = 0;
		map[i].mask = 0;
	}

	map[0].address = bios;
	map[0].mask = 0x3FFF;
	map[2].address = workRAM;
	map[2].mask = 0x3FFFF;
	map[3].address = internalRAM;
	map[3].mask = 0x7FFF;
	map[4].address = ioMem;
	map[4].mask = 0x3FF;
	map[5].address = graphics.paletteRAM;
	map[5].mask = 0x3FF;
	map[6].address = vram;
	map[6].mask = 0x1FFFF;
	map[7].address = oam;
	map[7].mask = 0x3FF;
	map[8].address = rom;
	map[8].mask = 0x1FFFFFF;
	map[9].address = rom;
	map[9].mask = 0x1FFFFFF;
	map[10].address = rom;
	map[10].mask = 0x1FFFFFF;
	map[12].address = rom;
	map[12].mask = 0x1FFFFFF;
	map[14].address = flashSaveMemory;
	map[14].mask = 0xFFFF;

	eepromReset();
	flashReset();

	soundReset();

	CPUUpdateWindow0();
	CPUUpdateWindow1();

	// make sure registers are correctly initialized if not using BIOS
	if(cpuIsMultiBoot)
		BIOS_RegisterRamReset(0xfe);
	else if(!useBios && !cpuIsMultiBoot)
		BIOS_RegisterRamReset(0xff);
		
	switch(cpuSaveType) {
		case 0: // automatic
			cpuSramEnabled = true;
			cpuFlashEnabled = true;
			cpuEEPROMEnabled = true;
			cpuEEPROMSensorEnabled = false;
			saveType = gbaSaveType = 0;
			break;
		case 1: // EEPROM
			cpuSramEnabled = false;
			cpuFlashEnabled = false;
			cpuEEPROMEnabled = true;
			cpuEEPROMSensorEnabled = false;
			saveType = gbaSaveType = 3;
			// EEPROM usage is automatically detected
			break;
		case 2: // SRAM
			cpuSramEnabled = true;
			cpuFlashEnabled = false;
			cpuEEPROMEnabled = false;
			cpuEEPROMSensorEnabled = false;
			cpuSaveGameFunc = sramDelayedWrite; // to insure we detect the write
			saveType = gbaSaveType = 1;
			break;
		case 3: // FLASH
			cpuSramEnabled = false;
			cpuFlashEnabled = true;
			cpuEEPROMEnabled = false;
			cpuEEPROMSensorEnabled = false;
			cpuSaveGameFunc = flashDelayedWrite; // to insure we detect the write
			saveType = gbaSaveType = 2;
			break;
		case 4: // EEPROM+Sensor
			cpuSramEnabled = false;
			cpuFlashEnabled = false;
			cpuEEPROMEnabled = true;
			cpuEEPROMSensorEnabled = true;
			// EEPROM usage is automatically detected
			saveType = gbaSaveType = 3;
			break;
		case 5: // NONE
			cpuSramEnabled = false;
			cpuFlashEnabled = false;
			cpuEEPROMEnabled = false;
			cpuEEPROMSensorEnabled = false;
			// no save at all
			saveType = gbaSaveType = 5;
			break;
	}

	ARM_PREFETCH;

#ifdef USE_SWITICKS
	SWITicks = 0;
#endif
}

static void CPUInterrupt(void)
{
	u32 PC = bus.reg[15].I;
	bool savedState = armState;

	if(armMode != 0x12 )
		CPUSwitchMode(0x12, true, false);

	bus.reg[14].I = PC;
	if(!savedState)
		bus.reg[14].I += 2;
	bus.reg[15].I = 0x18;
	armState = true;
	armIrqEnable = false;

	bus.armNextPC = bus.reg[15].I;
	bus.reg[15].I += 4;
	ARM_PREFETCH;

	//  if(!holdState)
	biosProtected[0] = 0x02;
	biosProtected[1] = 0xc0;
	biosProtected[2] = 0x5e;
	biosProtected[3] = 0xe5;
}

void CPULoop (void)
{
	bus.busPrefetchCount = 0;
	int ticks = 250000;
	int timerOverflow = 0;
	// variable used by the CPU core
	cpuTotalTicks = 0;

	cpuNextEvent = CPUUpdateTicks();
	if(cpuNextEvent > ticks)
		cpuNextEvent = ticks;


	do
	{
		if(!holdState)
		{
			if(armState)
			{
				if (!armExecute())
					return;
			}
			else
			{
				if (!thumbExecute())
					return;
			}
			clockTicks = 0;
		}
		else
			clockTicks = CPUUpdateTicks();

		cpuTotalTicks += clockTicks;


		if(cpuTotalTicks >= cpuNextEvent) {
			int remainingTicks = cpuTotalTicks - cpuNextEvent;

#ifdef USE_SWITICKS
			if (SWITicks)
			{
				SWITicks-=clockTicks;
				if (SWITicks<0)
					SWITicks = 0;
			}
#endif

			clockTicks = cpuNextEvent;
			cpuTotalTicks = 0;

updateLoop:

			if (IRQTicks)
			{
				IRQTicks -= clockTicks;
				if (IRQTicks<0)
					IRQTicks = 0;
			}

			graphics.lcdTicks -= clockTicks;

			if(graphics.lcdTicks <= 0)
			{
				if(io_registers[REG_DISPSTAT] & 1)
				{ // V-BLANK
					// if in V-Blank mode, keep computing...
					if(io_registers[REG_DISPSTAT] & 2)
					{
						graphics.lcdTicks += 1008;
						io_registers[REG_VCOUNT] += 1;
						UPDATE_REG(0x06, io_registers[REG_VCOUNT]);
						io_registers[REG_DISPSTAT] &= 0xFFFD;
						UPDATE_REG(0x04, io_registers[REG_DISPSTAT]);
						CPUCompareVCOUNT();
					}
					else
					{
						graphics.lcdTicks += 224;
						io_registers[REG_DISPSTAT] |= 2;
						UPDATE_REG(0x04, io_registers[REG_DISPSTAT]);
						if(io_registers[REG_DISPSTAT] & 16)
						{
							io_registers[REG_IF] |= 2;
							UPDATE_REG(0x202, io_registers[REG_IF]);
						}
					}

					if(io_registers[REG_VCOUNT] >= 228)
					{
						//Reaching last line
						io_registers[REG_DISPSTAT] &= 0xFFFC;
						UPDATE_REG(0x04, io_registers[REG_DISPSTAT]);
						io_registers[REG_VCOUNT] = 0;
						UPDATE_REG(0x06, io_registers[REG_VCOUNT]);
						CPUCompareVCOUNT();
					}
				}
				else if(io_registers[REG_DISPSTAT] & 2)
				{
					// if in H-Blank, leave it and move to drawing mode
					io_registers[REG_VCOUNT] += 1;
					UPDATE_REG(0x06, io_registers[REG_VCOUNT]);

					graphics.lcdTicks += 1008;
					io_registers[REG_DISPSTAT] &= 0xFFFD;
					if(io_registers[REG_VCOUNT] == 160)
					{
						/* update joystick information */
						io_registers[REG_P1] = 0x03FF ^ (joy & 0x3FF);
#if 0
						if(cpuEEPROMSensorEnabled)
							systemUpdateMotionSensor();
#endif
						UPDATE_REG(0x130, io_registers[REG_P1]);
						io_registers[REG_P1CNT] = READ16LE(((u16 *)&ioMem[0x132]));

						// this seems wrong, but there are cases where the game
						// can enter the stop state without requesting an IRQ from
						// the joypad.
						if((io_registers[REG_P1CNT] & 0x4000) || stopState) {
							u16 p1 = (0x3FF ^ io_registers[REG_P1CNT]) & 0x3FF;
							if(io_registers[REG_P1CNT] & 0x8000) {
								if(p1 == (io_registers[REG_P1CNT] & 0x3FF)) {
									io_registers[REG_IF] |= 0x1000;
									UPDATE_REG(0x202, io_registers[REG_IF]);
								}
							} else {
								if(p1 & io_registers[REG_P1CNT]) {
									io_registers[REG_IF] |= 0x1000;
									UPDATE_REG(0x202, io_registers[REG_IF]);
								}
							}
						}

						io_registers[REG_DISPSTAT] |= 1;
						io_registers[REG_DISPSTAT] &= 0xFFFD;
						UPDATE_REG(0x04, io_registers[REG_DISPSTAT]);
						if(io_registers[REG_DISPSTAT] & 0x0008)
						{
							io_registers[REG_IF] |= 1;
							UPDATE_REG(0x202, io_registers[REG_IF]);
						}
						CPUCheckDMA(1, 0x0f);
						systemDrawScreen();
					}

					UPDATE_REG(0x04, io_registers[REG_DISPSTAT]);
					CPUCompareVCOUNT();
				}
				else
				{
					bool draw_objwin = (graphics.layerEnable & 0x9000) == 0x9000;
					bool draw_sprites = graphics.layerEnable & 0x1000;
					memset(line[4], -1, 240 * sizeof(u32));	// erase all sprites

					if(draw_sprites)
						gfxDrawSprites();

					if(render_line_all_enabled)
					{
						memset(line[5], -1, 240 * sizeof(u32));	// erase all OBJ Win 
						if(draw_objwin)
							gfxDrawOBJWin();
					}

					(*renderLine)();

					// entering H-Blank
					io_registers[REG_DISPSTAT] |= 2;
					UPDATE_REG(0x04, io_registers[REG_DISPSTAT]);
					graphics.lcdTicks += 224;
					CPUCheckDMA(2, 0x0f);
					if(io_registers[REG_DISPSTAT] & 16)
					{
						io_registers[REG_IF] |= 2;
						UPDATE_REG(0x202, io_registers[REG_IF]);
					}
				}
			}

			// we shouldn't be doing sound in stop state, but we lose synchronization
			// if sound is disabled, so in stop state, soundTick will just produce
			// mute sound
			soundTicks -= clockTicks;
			if(!soundTicks)
			{
				process_sound_tick_fn();
				soundTicks += SOUND_CLOCK_TICKS;
			}

			if(!stopState) {
				if(timer0On) {
					timer0Ticks -= clockTicks;
					if(timer0Ticks <= 0) {
						timer0Ticks += (0x10000 - timer0Reload) << timer0ClockReload;
						timerOverflow |= 1;
						soundTimerOverflow(0);
						if(io_registers[REG_TM0CNT] & 0x40) {
							io_registers[REG_IF] |= 0x08;
							UPDATE_REG(0x202, io_registers[REG_IF]);
						}
					}
					io_registers[REG_TM0D] = 0xFFFF - (timer0Ticks >> timer0ClockReload);
					UPDATE_REG(0x100, io_registers[REG_TM0D]);
				}

				if(timer1On) {
					if(io_registers[REG_TM1CNT] & 4) {
						if(timerOverflow & 1) {
							io_registers[REG_TM1D]++;
							if(io_registers[REG_TM1D] == 0) {
								io_registers[REG_TM1D] += timer1Reload;
								timerOverflow |= 2;
								soundTimerOverflow(1);
								if(io_registers[REG_TM1CNT] & 0x40) {
									io_registers[REG_IF] |= 0x10;
									UPDATE_REG(0x202, io_registers[REG_IF]);
								}
							}
							UPDATE_REG(0x104, io_registers[REG_TM1D]);
						}
					} else {
						timer1Ticks -= clockTicks;
						if(timer1Ticks <= 0) {
							timer1Ticks += (0x10000 - timer1Reload) << timer1ClockReload;
							timerOverflow |= 2;
							soundTimerOverflow(1);
							if(io_registers[REG_TM1CNT] & 0x40) {
								io_registers[REG_IF] |= 0x10;
								UPDATE_REG(0x202, io_registers[REG_IF]);
							}
						}
						io_registers[REG_TM1D] = 0xFFFF - (timer1Ticks >> timer1ClockReload);
						UPDATE_REG(0x104, io_registers[REG_TM1D]);
					}
				}

				if(timer2On) {
					if(io_registers[REG_TM2CNT] & 4) {
						if(timerOverflow & 2) {
							io_registers[REG_TM2D]++;
							if(io_registers[REG_TM2D] == 0) {
								io_registers[REG_TM2D] += timer2Reload;
								timerOverflow |= 4;
								if(io_registers[REG_TM2CNT] & 0x40) {
									io_registers[REG_IF] |= 0x20;
									UPDATE_REG(0x202, io_registers[REG_IF]);
								}
							}
							UPDATE_REG(0x108, io_registers[REG_TM2D]);
						}
					} else {
						timer2Ticks -= clockTicks;
						if(timer2Ticks <= 0) {
							timer2Ticks += (0x10000 - timer2Reload) << timer2ClockReload;
							timerOverflow |= 4;
							if(io_registers[REG_TM2CNT] & 0x40) {
								io_registers[REG_IF] |= 0x20;
								UPDATE_REG(0x202, io_registers[REG_IF]);
							}
						}
						io_registers[REG_TM2D] = 0xFFFF - (timer2Ticks >> timer2ClockReload);
						UPDATE_REG(0x108, io_registers[REG_TM2D]);
					}
				}

				if(timer3On) {
					if(io_registers[REG_TM3CNT] & 4) {
						if(timerOverflow & 4) {
							io_registers[REG_TM3D]++;
							if(io_registers[REG_TM3D] == 0) {
								io_registers[REG_TM3D] += timer3Reload;
								if(io_registers[REG_TM3CNT] & 0x40) {
									io_registers[REG_IF] |= 0x40;
									UPDATE_REG(0x202, io_registers[REG_IF]);
								}
							}
							UPDATE_REG(0x10C, io_registers[REG_TM3D]);
						}
					} else {
						timer3Ticks -= clockTicks;
						if(timer3Ticks <= 0) {
							timer3Ticks += (0x10000 - timer3Reload) << timer3ClockReload;
							if(io_registers[REG_TM3CNT] & 0x40) {
								io_registers[REG_IF] |= 0x40;
								UPDATE_REG(0x202, io_registers[REG_IF]);
							}
						}
						io_registers[REG_TM3D] = 0xFFFF - (timer3Ticks >> timer3ClockReload);
						UPDATE_REG(0x10C, io_registers[REG_TM3D]);
					}
				}
			}

			timerOverflow = 0;
			ticks -= clockTicks;
			cpuNextEvent = CPUUpdateTicks();

			if(cpuDmaTicksToUpdate > 0)
			{
				if(cpuDmaTicksToUpdate > cpuNextEvent)
					clockTicks = cpuNextEvent;
				else
					clockTicks = cpuDmaTicksToUpdate;
				cpuDmaTicksToUpdate -= clockTicks;
				if(cpuDmaTicksToUpdate < 0)
					cpuDmaTicksToUpdate = 0;
				goto updateLoop;
			}

			if(io_registers[REG_IF] && (io_registers[REG_IME] & 1) && armIrqEnable)
			{
				int res = io_registers[REG_IF] & io_registers[REG_IE];
				if(stopState)
					res &= 0x3080;
				if(res)
				{
					if (intState)
					{
						if (!IRQTicks)
						{
							CPUInterrupt();
							intState = false;
							holdState = false;
							stopState = false;
						}
					}
					else
					{
						if (!holdState)
						{
							intState = true;
							IRQTicks=7;
							if (cpuNextEvent> IRQTicks)
								cpuNextEvent = IRQTicks;
						}
						else
						{
							CPUInterrupt();
							holdState = false;
							stopState = false;
						}
					}

#ifdef USE_SWITICKS
					// Stops the SWI Ticks emulation if an IRQ is executed
					//(to avoid problems with nested IRQ/SWI)
					if (SWITicks)
						SWITicks = 0;
#endif
				}
			}

			if(remainingTicks > 0) {
				if(remainingTicks > cpuNextEvent)
					clockTicks = cpuNextEvent;
				else
					clockTicks = remainingTicks;
				remainingTicks -= clockTicks;
				if(remainingTicks < 0)
					remainingTicks = 0;
				goto updateLoop;
			}

			if (timerOnOffDelay)
			{
				// Apply Timer
				if (timerOnOffDelay & 1)
				{
					timer0ClockReload = TIMER_TICKS[timer0Value & 3];
					if(!timer0On && (timer0Value & 0x80)) {
						// reload the counter
						io_registers[REG_TM0D] = timer0Reload;
						timer0Ticks = (0x10000 - io_registers[REG_TM0D]) << timer0ClockReload;
						UPDATE_REG(0x100, io_registers[REG_TM0D]);
					}
					timer0On = timer0Value & 0x80 ? true : false;
					io_registers[REG_TM0CNT] = timer0Value & 0xC7;
					UPDATE_REG(0x102, io_registers[REG_TM0CNT]);
				}
				if (timerOnOffDelay & 2)
				{
					timer1ClockReload = TIMER_TICKS[timer1Value & 3];
					if(!timer1On && (timer1Value & 0x80)) {
						// reload the counter
						io_registers[REG_TM1D] = timer1Reload;
						timer1Ticks = (0x10000 - io_registers[REG_TM1D]) << timer1ClockReload;
						UPDATE_REG(0x104, io_registers[REG_TM1D]);
					}
					timer1On = timer1Value & 0x80 ? true : false;
					io_registers[REG_TM1CNT] = timer1Value & 0xC7;
					UPDATE_REG(0x106, io_registers[REG_TM1CNT]);
				}
				if (timerOnOffDelay & 4)
				{
					timer2ClockReload = TIMER_TICKS[timer2Value & 3];
					if(!timer2On && (timer2Value & 0x80)) {
						// reload the counter
						io_registers[REG_TM2D] = timer2Reload;
						timer2Ticks = (0x10000 - io_registers[REG_TM2D]) << timer2ClockReload;
						UPDATE_REG(0x108, io_registers[REG_TM2D]);
					}
					timer2On = timer2Value & 0x80 ? true : false;
					io_registers[REG_TM2CNT] = timer2Value & 0xC7;
					UPDATE_REG(0x10A, io_registers[REG_TM2CNT]);
				}
				if (timerOnOffDelay & 8)
				{
					timer3ClockReload = TIMER_TICKS[timer3Value & 3];
					if(!timer3On && (timer3Value & 0x80)) {
						// reload the counter
						io_registers[REG_TM3D] = timer3Reload;
						timer3Ticks = (0x10000 - io_registers[REG_TM3D]) << timer3ClockReload;
						UPDATE_REG(0x10C, io_registers[REG_TM3D]);
					}
					timer3On = timer3Value & 0x80 ? true : false;
					io_registers[REG_TM3CNT] = timer3Value & 0xC7;
					UPDATE_REG(0x10E, io_registers[REG_TM3CNT]);
				}
				cpuNextEvent = CPUUpdateTicks();
				timerOnOffDelay = 0;
				// End of Apply Timer
			}

			if(cpuNextEvent > ticks)
				cpuNextEvent = ticks;

			if(ticks <= 0)
				break;

		}
	}while(1);
}
