/*============================================================
	EEPROM
============================================================ */
int eepromMode = EEPROM_IDLE;
int eepromByte = 0;
int eepromBits = 0;
int eepromAddress = 0;

#ifdef __LIBRETRO__
// Workaround for broken-by-design GBA save semantics.
extern u8 libretro_save_buf[0x20000 + 0x2000];
u8 *eepromData = libretro_save_buf + 0x20000;
#else
u8 eepromData[0x2000];
#endif

u8 eepromBuffer[16];
bool eepromInUse = false;
int eepromSize = 512;

variable_desc eepromSaveData[] = {
  { &eepromMode, sizeof(int) },
  { &eepromByte, sizeof(int) },
  { &eepromBits , sizeof(int) },
  { &eepromAddress , sizeof(int) },
  { &eepromInUse, sizeof(bool) },
  { &eepromData[0], 512 },
  { &eepromBuffer[0], 16 },
  { NULL, 0 }
};

void eepromInit (void)
{
#ifdef __LIBRETRO__
	memset(eepromData, 255, 0x2000);
#else
	memset(eepromData, 255, sizeof(eepromData));
#endif
}

void eepromReset (void)
{
	eepromMode = EEPROM_IDLE;
	eepromByte = 0;
	eepromBits = 0;
	eepromAddress = 0;
	eepromInUse = false;
	eepromSize = 512;
}

void eepromSaveGameMem(uint8_t *& data)
{
	utilWriteDataMem(data, eepromSaveData);
	utilWriteIntMem(data, eepromSize);
	utilWriteMem(data, eepromData, 0x2000);
}

void eepromReadGameMem(const uint8_t *& data, int version)
{
	utilReadDataMem(data, eepromSaveData);
	eepromSize = utilReadIntMem(data);
	utilReadMem(eepromData, data, 0x2000);
}

int eepromRead (void)
{
	switch(eepromMode)
	{
		case EEPROM_IDLE:
		case EEPROM_READADDRESS:
		case EEPROM_WRITEDATA:
			return 1;
		case EEPROM_READDATA:
			{
				eepromBits++;
				if(eepromBits == 4)
            {
					eepromMode = EEPROM_READDATA2;
					eepromBits = 0;
					eepromByte = 0;
				}
				return 0;
			}
		case EEPROM_READDATA2:
			{
				int data = 0;
				int address = eepromAddress << 3;
				int mask = 1 << (7 - (eepromBits & 7));
				data = (eepromData[address+eepromByte] & mask) ? 1 : 0;
				eepromBits++;
				if((eepromBits & 7) == 0)
					eepromByte++;
				if(eepromBits == 0x40)
					eepromMode = EEPROM_IDLE;
				return data;
			}
		default:
			return 0;
	}
	return 1;
}

void eepromWrite(u8 value)
{
	if(cpuDmaCount == 0)
		return;
	int bit = value & 1;
	switch(eepromMode) {
		case EEPROM_IDLE:
			eepromByte = 0;
			eepromBits = 1;
			eepromBuffer[eepromByte] = bit;
			eepromMode = EEPROM_READADDRESS;
			break;
		case EEPROM_READADDRESS:
			eepromBuffer[eepromByte] <<= 1;
			eepromBuffer[eepromByte] |= bit;
			eepromBits++;
			if((eepromBits & 7) == 0) {
				eepromByte++;
			}
			if(cpuDmaCount == 0x11 || cpuDmaCount == 0x51) {
				if(eepromBits == 0x11) {
					eepromInUse = true;
					eepromSize = 0x2000;
					eepromAddress = ((eepromBuffer[0] & 0x3F) << 8) |
						((eepromBuffer[1] & 0xFF));
					if(!(eepromBuffer[0] & 0x40)) {
						eepromBuffer[0] = bit;
						eepromBits = 1;
						eepromByte = 0;
						eepromMode = EEPROM_WRITEDATA;
					} else {
						eepromMode = EEPROM_READDATA;
						eepromByte = 0;
						eepromBits = 0;
					}
				}
			} else {
				if(eepromBits == 9) {
					eepromInUse = true;
					eepromAddress = (eepromBuffer[0] & 0x3F);
					if(!(eepromBuffer[0] & 0x40)) {
						eepromBuffer[0] = bit;
						eepromBits = 1;
						eepromByte = 0;
						eepromMode = EEPROM_WRITEDATA;
					} else {
						eepromMode = EEPROM_READDATA;
						eepromByte = 0;
						eepromBits = 0;
					}
				}
			}
			break;
		case EEPROM_READDATA:
		case EEPROM_READDATA2:
			// should we reset here?
			eepromMode = EEPROM_IDLE;
			break;
		case EEPROM_WRITEDATA:
			eepromBuffer[eepromByte] <<= 1;
			eepromBuffer[eepromByte] |= bit;
			eepromBits++;
			if((eepromBits & 7) == 0)
				eepromByte++;
			if(eepromBits == 0x40)
			{
				eepromInUse = true;
				// write data;
				for(int i = 0; i < 8; i++)
					eepromData[(eepromAddress << 3) + i] = eepromBuffer[i];
			}
			else if(eepromBits == 0x41)
			{
				eepromMode = EEPROM_IDLE;
				eepromByte = 0;
				eepromBits = 0;
			}
			break;
	}
}
