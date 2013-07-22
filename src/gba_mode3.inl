/*
Mode 3 is a 15-bit (32768) colour bitmap graphics mode.
It has a single layer, background layer 2, the same size as the screen.
It doesn't support paging, scrolling, flipping, rotation or tiles.

These routines only render a single line at a time, because of the way the GBA does events.
*/

static void mode3RenderLine (void)
{
#ifdef REPORT_VIDEO_MODES
	fprintf(stderr, "MODE 3: Render Line\n");
#endif
	u16 *lineMix = (pix + PIX_BUFFER_SCREEN_WIDTH * io_registers[REG_VCOUNT]);
	u16 *palette = (u16*)graphics.paletteRAM;

	if(graphics.layerEnable & 0x0400) {
		int changed = gfxBG2Changed;

#if 0
		if(gfxLastVCOUNT > io_registers[REG_VCOUNT])
			changed = 3;
#endif

		gfxDrawRotScreen16Bit(gfxBG2X, gfxBG2Y, changed);
	}

	uint32_t background = (READ16LE(&palette[0]) | 0x30000000);

	for(int x = 0; x < 240; ++x) {
		uint32_t color = background;
		uint8_t top = 0x20;

		if(line[2][x] < color) {
			color = line[2][x];
			top = 0x04;
		}

		if((uint8_t)(line[4][x]>>24) < (uint8_t)(color >>24)) {
			color = line[4][x];
			top = 0x10;

			if(color & 0x00010000) {
				// semi-transparent OBJ
				uint32_t back = background;
				uint8_t top2 = 0x20;

				if(line[2][x] < background) {
					back = line[2][x];
					top2 = 0x04;
				}

				alpha_blend_brightness_switch();
			}
		}


		lineMix[x] = CONVERT_COLOR(color);
	}
	gfxBG2Changed = 0;
	//gfxLastVCOUNT = io_registers[REG_VCOUNT];
}

static void mode3RenderLineNoWindow (void)
{
#ifdef REPORT_VIDEO_MODES
	fprintf(stderr, "MODE 3: Render Line No Window\n");
#endif
	u16 *lineMix = (pix + PIX_BUFFER_SCREEN_WIDTH * io_registers[REG_VCOUNT]);
	u16 *palette = (u16*)graphics.paletteRAM;

	if(graphics.layerEnable & 0x0400) {
		int changed = gfxBG2Changed;

#if 0
		if(gfxLastVCOUNT > io_registers[REG_VCOUNT])
			changed = 3;
#endif

		gfxDrawRotScreen16Bit(gfxBG2X, gfxBG2Y, changed);
	}

	uint32_t background = (READ16LE(&palette[0]) | 0x30000000);

	for(int x = 0; x < 240; ++x) {
		uint32_t color = background;
		uint8_t top = 0x20;

		if(line[2][x] < background) {
			color = line[2][x];
			top = 0x04;
		}

		if((uint8_t)(line[4][x]>>24) < (uint8_t)(color >>24)) {
			color = line[4][x];
			top = 0x10;
		}

		if(!(color & 0x00010000)) {
			switch((BLDMOD >> 6) & 3) {
				case 0:
					break;
				case 1:
					if(top & BLDMOD)
					{
						uint32_t back = background;
						uint8_t top2 = 0x20;

						if(top != 0x04 && (line[2][x] < background) ) {
							back = line[2][x];
							top2 = 0x04;
						}

						if(top != 0x10 && ((uint8_t)(line[4][x]>>24) < (uint8_t)(back >> 24))) {
							back = line[4][x];
							top2 = 0x10;
						}

						if(top2 & (BLDMOD>>8) && color < 0x80000000)
						{
							GFX_ALPHA_BLEND(color, back, coeff[COLEV & 0x1F], coeff[(COLEV >> 8) & 0x1F]);
						}

					}
					break;
				case 2:
					if(BLDMOD & top)
						color = gfxIncreaseBrightness(color, coeff[COLY & 0x1F]);
					break;
				case 3:
					if(BLDMOD & top)
						color = gfxDecreaseBrightness(color, coeff[COLY & 0x1F]);
					break;
			}
		} else {
			// semi-transparent OBJ
			uint32_t back = background;
			uint8_t top2 = 0x20;

			if(line[2][x] < background) {
				back = line[2][x];
				top2 = 0x04;
			}

			alpha_blend_brightness_switch();
		}

		lineMix[x] = CONVERT_COLOR(color);
	}
	gfxBG2Changed = 0;
	//gfxLastVCOUNT = io_registers[REG_VCOUNT];
}

static void mode3RenderLineAll (void)
{
#ifdef REPORT_VIDEO_MODES
	fprintf(stderr, "MODE 3: Render Line All\n");
#endif
	u16 *lineMix = (pix + PIX_BUFFER_SCREEN_WIDTH * io_registers[REG_VCOUNT]);
	u16 *palette = (u16*)graphics.paletteRAM;

	bool inWindow0 = false;
	bool inWindow1 = false;

	if(graphics.layerEnable & 0x2000)
	{
		uint8_t v0 = io_registers[REG_WIN0V] >> 8;
		uint8_t v1 = io_registers[REG_WIN0V] & 255;
		inWindow0 = ((v0 == v1) && (v0 >= 0xe8));
#ifndef ORIGINAL_BRANCHES
		uint32_t condition = v1 >= v0;
		int32_t condition_mask = ((condition) | -(condition)) >> 31;
		inWindow0 = (((inWindow0 | (io_registers[REG_VCOUNT] >= v0 && io_registers[REG_VCOUNT] < v1)) & condition_mask) | (((inWindow0 | (io_registers[REG_VCOUNT] >= v0 || io_registers[REG_VCOUNT] < v1)) & ~(condition_mask))));
#else
		if(v1 >= v0)
			inWindow0 |= (io_registers[REG_VCOUNT] >= v0 && io_registers[REG_VCOUNT] < v1);
		else
			inWindow0 |= (io_registers[REG_VCOUNT] >= v0 || io_registers[REG_VCOUNT] < v1);
#endif
	}

	if(graphics.layerEnable & 0x4000)
	{
		uint8_t v0 = io_registers[REG_WIN1V] >> 8;
		uint8_t v1 = io_registers[REG_WIN1V] & 255;
		inWindow1 = ((v0 == v1) && (v0 >= 0xe8));
#ifndef ORIGINAL_BRANCHES
		uint32_t condition = v1 >= v0;
		int32_t condition_mask = ((condition) | -(condition)) >> 31;
		inWindow1 = (((inWindow1 | (io_registers[REG_VCOUNT] >= v0 && io_registers[REG_VCOUNT] < v1)) & condition_mask) | (((inWindow1 | (io_registers[REG_VCOUNT] >= v0 || io_registers[REG_VCOUNT] < v1)) & ~(condition_mask))));
#else
		if(v1 >= v0)
			inWindow1 |= (io_registers[REG_VCOUNT] >= v0 && io_registers[REG_VCOUNT] < v1);
		else
			inWindow1 |= (io_registers[REG_VCOUNT] >= v0 || io_registers[REG_VCOUNT] < v1);
#endif
	}

	if(graphics.layerEnable & 0x0400) {
		int changed = gfxBG2Changed;

#if 0
		if(gfxLastVCOUNT > io_registers[REG_VCOUNT])
			changed = 3;
#endif

		gfxDrawRotScreen16Bit(gfxBG2X, gfxBG2Y, changed);
	}

	uint8_t inWin0Mask = io_registers[REG_WININ] & 0xFF;
	uint8_t inWin1Mask = io_registers[REG_WININ] >> 8;
	uint8_t outMask = io_registers[REG_WINOUT] & 0xFF;

	uint32_t background = (READ16LE(&palette[0]) | 0x30000000);

	for(int x = 0; x < 240; ++x) {
		uint32_t color = background;
		uint8_t top = 0x20;
		uint8_t mask = outMask;

		if(!(line[5][x] & 0x80000000)) {
			mask = io_registers[REG_WINOUT] >> 8;
		}

		int32_t window1_mask = ((inWindow1 & gfxInWin[1][x]) | -(inWindow1 & gfxInWin[1][x])) >> 31;
		int32_t window0_mask = ((inWindow0 & gfxInWin[0][x]) | -(inWindow0 & gfxInWin[0][x])) >> 31;
		mask = (inWin1Mask & window1_mask) | (mask & ~window1_mask);
		mask = (inWin0Mask & window0_mask) | (mask & ~window0_mask);

		if((mask & 4) && line[2][x] < background) {
			color = line[2][x];
			top = 0x04;
		}

		if((mask & 16) && ((uint8_t)(line[4][x]>>24) < (uint8_t)(color >>24))) {
			color = line[4][x];
			top = 0x10;
		}

		if(color & 0x00010000) {
			// semi-transparent OBJ
			uint32_t back = background;
			uint8_t top2 = 0x20;

			if((mask & 4) && line[2][x] < background) {
				back = line[2][x];
				top2 = 0x04;
			}

			alpha_blend_brightness_switch();
		} else if(mask & 32) {
			switch((BLDMOD >> 6) & 3) {
				case 0:
					break;
				case 1:
					if(top & BLDMOD)
					{
						uint32_t back = background;
						uint8_t top2 = 0x20;

						if((mask & 4) && (top != 0x04) && line[2][x] < back) {
							back = line[2][x];
							top2 = 0x04;
						}

						if((mask & 16) && (top != 0x10) && (uint8_t)(line[4][x]>>24) < (uint8_t)(back >> 24)) {
							back = line[4][x];
							top2 = 0x10;
						}

						if(top2 & (BLDMOD>>8) && color < 0x80000000)
						{
							GFX_ALPHA_BLEND(color, back, coeff[COLEV & 0x1F], coeff[(COLEV >> 8) & 0x1F]);
						}
					}
					break;
				case 2:
					if(BLDMOD & top)
						color = gfxIncreaseBrightness(color, coeff[COLY & 0x1F]);
					break;
				case 3:
					if(BLDMOD & top)
						color = gfxDecreaseBrightness(color, coeff[COLY & 0x1F]);
					break;
			}
		}

		lineMix[x] = CONVERT_COLOR(color);
	}
	gfxBG2Changed = 0;
	//gfxLastVCOUNT = io_registers[REG_VCOUNT];
}
