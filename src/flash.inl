/*============================================================
	FLASH
============================================================ */
#define FLASH_READ_ARRAY         0
#define FLASH_CMD_1              1
#define FLASH_CMD_2              2
#define FLASH_AUTOSELECT         3
#define FLASH_CMD_3              4
#define FLASH_CMD_4              5
#define FLASH_CMD_5              6
#define FLASH_ERASE_COMPLETE     7
#define FLASH_PROGRAM            8
#define FLASH_SETBANK            9

#ifdef __LIBRETRO__
extern u8 libretro_save_buf[0x20000 + 0x2000];
u8 *flashSaveMemory = libretro_save_buf;
#else
u8 flashSaveMemory[FLASH_128K_SZ];
#endif

int flashState = FLASH_READ_ARRAY;
int flashReadState = FLASH_READ_ARRAY;
int flashSize = 0x10000;
int flashDeviceID = 0x1b;
int flashManufacturerID = 0x32;
int flashBank = 0;

static variable_desc flashSaveData3[] = {
  { &flashState, sizeof(int) },
  { &flashReadState, sizeof(int) },
  { &flashSize, sizeof(int) },
  { &flashBank, sizeof(int) },
  { &flashSaveMemory[0], 0x20000 },
  { NULL, 0 }
};

void flashInit (void)
{
#ifdef __LIBRETRO__
	memset(flashSaveMemory, 0xff, 0x20000);
#else
	memset(flashSaveMemory, 0xff, sizeof(flashSaveMemory));
#endif
}

void flashReset (void)
{
	flashState = FLASH_READ_ARRAY;
	flashReadState = FLASH_READ_ARRAY;
	flashBank = 0;
}

void flashSaveGameMem(u8 *& data)
{
	utilWriteDataMem(data, flashSaveData3);
}

void flashReadGameMem(const u8 *& data, int)
{
	utilReadDataMem(data, flashSaveData3);
}

void flashSetSize(int size)
{
	if(size == 0x10000)
   {
		flashDeviceID = 0x1b;
		flashManufacturerID = 0x32;
	}
   else
   {
		flashDeviceID = 0x13; //0x09;
		flashManufacturerID = 0x62; //0xc2;
	}
	// Added to make 64k saves compatible with 128k ones
	// (allow wrongfuly set 64k saves to work for Pokemon games)
	if ((size == 0x20000) && (flashSize == 0x10000))
		memcpy((u8 *)(flashSaveMemory+0x10000), (u8 *)(flashSaveMemory), 0x10000);
	flashSize = size;
}

u8 flashRead(u32 address)
{
	address &= 0xFFFF;

	switch(flashReadState)
   {
		case FLASH_READ_ARRAY:
			return flashSaveMemory[(flashBank << 16) + address];
		case FLASH_AUTOSELECT:
			switch(address & 0xFF)
			{
				case 0:
					return flashManufacturerID;
				case 1:
					return flashDeviceID;
			}
			break;
		case FLASH_ERASE_COMPLETE:
			flashState = FLASH_READ_ARRAY;
			flashReadState = FLASH_READ_ARRAY;
			return 0xFF;
	}

	return 0;
}

void flashSaveDecide(u32 address, u8 byte)
{
	if(address == 0x0e005555)
   {
		saveType = 2;
		cpuSaveGameFunc = flashWrite;
	}
   else
   {
		saveType = 1;
		cpuSaveGameFunc = sramWrite;
	}

	(*cpuSaveGameFunc)(address, byte);
}

void flashDelayedWrite(u32 address, u8 byte)
{
  saveType = 2;
  cpuSaveGameFunc = flashWrite;
  flashWrite(address, byte);
}

void flashWrite(u32 address, u8 byte)
{
	address &= 0xFFFF;
	switch(flashState) {
		case FLASH_READ_ARRAY:
			if(address == 0x5555 && byte == 0xAA)
				flashState = FLASH_CMD_1;
			break;
		case FLASH_CMD_1:
			if(address == 0x2AAA && byte == 0x55)
				flashState = FLASH_CMD_2;
			else
				flashState = FLASH_READ_ARRAY;
			break;
		case FLASH_CMD_2:
			if(address == 0x5555) {
				if(byte == 0x90) {
					flashState = FLASH_AUTOSELECT;
					flashReadState = FLASH_AUTOSELECT;
				} else if(byte == 0x80) {
					flashState = FLASH_CMD_3;
				} else if(byte == 0xF0) {
					flashState = FLASH_READ_ARRAY;
					flashReadState = FLASH_READ_ARRAY;
				} else if(byte == 0xA0) {
					flashState = FLASH_PROGRAM;
				} else if(byte == 0xB0 && flashSize == 0x20000) {
					flashState = FLASH_SETBANK;
				} else {
					flashState = FLASH_READ_ARRAY;
					flashReadState = FLASH_READ_ARRAY;
				}
			} else {
				flashState = FLASH_READ_ARRAY;
				flashReadState = FLASH_READ_ARRAY;
			}
			break;
		case FLASH_CMD_3:
			if(address == 0x5555 && byte == 0xAA)
				flashState = FLASH_CMD_4;
			else
         {
				flashState = FLASH_READ_ARRAY;
				flashReadState = FLASH_READ_ARRAY;
			}
			break;
		case FLASH_CMD_4:
			if(address == 0x2AAA && byte == 0x55)
				flashState = FLASH_CMD_5;
			else
         {
				flashState = FLASH_READ_ARRAY;
				flashReadState = FLASH_READ_ARRAY;
			}
			break;
		case FLASH_CMD_5:
			if(byte == 0x30) {
				// SECTOR ERASE
				memset(&flashSaveMemory[(flashBank << 16) + (address & 0xF000)],
						0,
						0x1000);
				flashReadState = FLASH_ERASE_COMPLETE;
			} else if(byte == 0x10) {
				// CHIP ERASE
				memset(flashSaveMemory, 0, flashSize);
				flashReadState = FLASH_ERASE_COMPLETE;
			} else {
				flashState = FLASH_READ_ARRAY;
				flashReadState = FLASH_READ_ARRAY;
			}
			break;
		case FLASH_AUTOSELECT:
			if(byte == 0xF0)
         {
				flashState = FLASH_READ_ARRAY;
				flashReadState = FLASH_READ_ARRAY;
			}
         else if(address == 0x5555 && byte == 0xAA)
				flashState = FLASH_CMD_1;
			else
         {
				flashState = FLASH_READ_ARRAY;
				flashReadState = FLASH_READ_ARRAY;
			}
			break;
		case FLASH_PROGRAM:
			flashSaveMemory[(flashBank<<16)+address] = byte;
			flashState = FLASH_READ_ARRAY;
			flashReadState = FLASH_READ_ARRAY;
			break;
		case FLASH_SETBANK:
			if(address == 0)
				flashBank = (byte & 1);
			flashState = FLASH_READ_ARRAY;
			flashReadState = FLASH_READ_ARRAY;
			break;
	}
}
