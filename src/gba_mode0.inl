static void mode0RenderLine (void)
{
#ifdef REPORT_VIDEO_MODES
	fprintf(stderr, "MODE 0: Render Line\n");
#endif
	u16 *lineMix = (pix + PIX_BUFFER_SCREEN_WIDTH * io_registers[REG_VCOUNT]);
	u16 *palette = (u16*)graphics.paletteRAM;

	bool	process_layers[4];

	process_layers[0] = graphics.layerEnable & 0x0100;
	process_layers[1] = graphics.layerEnable & 0x0200;
	process_layers[2] = graphics.layerEnable & 0x0400;
	process_layers[3] = graphics.layerEnable & 0x0800;

	if(process_layers[0] || process_layers[1] || process_layers[2] || process_layers[3])
		gfxDrawTextScreen(process_layers[0], process_layers[1], process_layers[2], process_layers[3]);


	uint32_t backdrop = (READ16LE(&palette[0]) | 0x30000000);

	for(int x = 0; x < 240; x++)
	{
		uint32_t color = backdrop;
		uint8_t top = 0x20;

		if(line[0][x] < color) {
			color = line[0][x];
			top = 0x01;
		}

		if((uint8_t)(line[1][x]>>24) < (uint8_t)(color >> 24)) {
			color = line[1][x];
			top = 0x02;
		}

		if((uint8_t)(line[2][x]>>24) < (uint8_t)(color >> 24)) {
			color = line[2][x];
			top = 0x04;
		}

		if((uint8_t)(line[3][x]>>24) < (uint8_t)(color >> 24)) {
			color = line[3][x];
			top = 0x08;
		}

		if((uint8_t)(line[4][x]>>24) < (uint8_t)(color >> 24)) {
			color = line[4][x];
			top = 0x10;

			if(color & 0x00010000) {
				// semi-transparent OBJ
				uint32_t back = backdrop;
				uint8_t top2 = 0x20;

				if((uint8_t)(line[0][x]>>24) < (uint8_t)(back >> 24)) {
					back = line[0][x];
					top2 = 0x01;
				}

				if((uint8_t)(line[1][x]>>24) < (uint8_t)(back >> 24)) {
					back = line[1][x];
					top2 = 0x02;
				}

				if((uint8_t)(line[2][x]>>24) < (uint8_t)(back >> 24)) {
					back = line[2][x];
					top2 = 0x04;
				}

				if((uint8_t)(line[3][x]>>24) < (uint8_t)(back >> 24)) {
					back = line[3][x];
					top2 = 0x08;
				}

				alpha_blend_brightness_switch();
			}
		}


		lineMix[x] = CONVERT_COLOR(color);
	}
}

static void mode0RenderLineNoWindow (void)
{
#ifdef REPORT_VIDEO_MODES
	fprintf(stderr, "MODE 0: Render Line No Window\n");
#endif
	u16 *lineMix = (pix + PIX_BUFFER_SCREEN_WIDTH * io_registers[REG_VCOUNT]);
	u16 *palette = (u16*)graphics.paletteRAM;

	bool	process_layers[4];

	process_layers[0] = graphics.layerEnable & 0x0100;
	process_layers[1] = graphics.layerEnable & 0x0200;
	process_layers[2] = graphics.layerEnable & 0x0400;
	process_layers[3] = graphics.layerEnable & 0x0800;

	if(process_layers[0] || process_layers[1] || process_layers[2] || process_layers[3])
		gfxDrawTextScreen(process_layers[0], process_layers[1], process_layers[2], process_layers[3]);

	uint32_t backdrop = (READ16LE(&palette[0]) | 0x30000000);

	int effect = (BLDMOD >> 6) & 3;

	for(int x = 0; x < 240; x++) {
		uint32_t color = backdrop;
		uint8_t top = 0x20;

		if(line[0][x] < color) {
			color = line[0][x];
			top = 0x01;
		}

		if(line[1][x] < (color & 0xFF000000)) {
			color = line[1][x];
			top = 0x02;
		}

		if(line[2][x] < (color & 0xFF000000)) {
			color = line[2][x];
			top = 0x04;
		}

		if(line[3][x] < (color & 0xFF000000)) {
			color = line[3][x];
			top = 0x08;
		}

		if(line[4][x] < (color & 0xFF000000)) {
			color = line[4][x];
			top = 0x10;
		}

		if(!(color & 0x00010000)) {
			switch(effect) {
				case 0:
					break;
				case 1:
					if(top & BLDMOD)
					{
						uint32_t back = backdrop;
						uint8_t top2 = 0x20;
						if((line[0][x] < back) && (top != 0x01))
						{
							back = line[0][x];
							top2 = 0x01;
						}

						if((line[1][x] < (back & 0xFF000000)) && (top != 0x02))
						{
							back = line[1][x];
							top2 = 0x02;
						}

						if((line[2][x] < (back & 0xFF000000)) && (top != 0x04))
						{
							back = line[2][x];
							top2 = 0x04;
						}

						if((line[3][x] < (back & 0xFF000000)) && (top != 0x08))
						{
							back = line[3][x];
							top2 = 0x08;
						}

						if((line[4][x] < (back & 0xFF000000)) && (top != 0x10))
						{
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
			uint32_t back = backdrop;
			uint8_t top2 = 0x20;

			if(line[0][x] < back) {
				back = line[0][x];
				top2 = 0x01;
			}

			if(line[1][x] < (back & 0xFF000000)) {
				back = line[1][x];
				top2 = 0x02;
			}

			if(line[2][x] < (back & 0xFF000000)) {
				back = line[2][x];
				top2 = 0x04;
			}

			if(line[3][x] < (back & 0xFF000000)) {
				back = line[3][x];
				top2 = 0x08;
			}

			alpha_blend_brightness_switch();
		}

		lineMix[x] = CONVERT_COLOR(color);
	}
}

static void mode0RenderLineAll (void)
{
#ifdef REPORT_VIDEO_MODES
	fprintf(stderr, "MODE 0: Render Line All\n");
#endif
	u16 *lineMix = (pix + PIX_BUFFER_SCREEN_WIDTH * io_registers[REG_VCOUNT]);
	u16 *palette = (u16*)graphics.paletteRAM;

	bool inWindow0 = false;
	bool inWindow1 = false;

	if(graphics.layerEnable & 0x2000) {
		uint8_t v0 = io_registers[REG_WIN0V] >> 8;
		uint8_t v1 = io_registers[REG_WIN0V] & 255;
		inWindow0 = ((v0 == v1) && (v0 >= 0xe8));
		if(v1 >= v0)
			inWindow0 |= (io_registers[REG_VCOUNT] >= v0 && io_registers[REG_VCOUNT] < v1);
		else
			inWindow0 |= (io_registers[REG_VCOUNT] >= v0 || io_registers[REG_VCOUNT] < v1);
	}
	if(graphics.layerEnable & 0x4000) {
		uint8_t v0 = io_registers[REG_WIN1V] >> 8;
		uint8_t v1 = io_registers[REG_WIN1V] & 255;
		inWindow1 = ((v0 == v1) && (v0 >= 0xe8));
		if(v1 >= v0)
			inWindow1 |= (io_registers[REG_VCOUNT] >= v0 && io_registers[REG_VCOUNT] < v1);
		else
			inWindow1 |= (io_registers[REG_VCOUNT] >= v0 || io_registers[REG_VCOUNT] < v1);
	}

	bool	process_layers[4];

	process_layers[0] = graphics.layerEnable & 0x0100;
	process_layers[1] = graphics.layerEnable & 0x0200;
	process_layers[2] = graphics.layerEnable & 0x0400;
	process_layers[3] = graphics.layerEnable & 0x0800;

	if(process_layers[0] || process_layers[1] || process_layers[2] || process_layers[3])
		gfxDrawTextScreen(process_layers[0], process_layers[1], process_layers[2], process_layers[3]);


	uint32_t backdrop = (READ16LE(&palette[0]) | 0x30000000);

	uint8_t inWin0Mask = io_registers[REG_WININ] & 0xFF;
	uint8_t inWin1Mask = io_registers[REG_WININ] >> 8;
	uint8_t outMask = io_registers[REG_WINOUT] & 0xFF;

	for(int x = 0; x < 240; x++) {
		uint32_t color = backdrop;
		uint8_t top = 0x20;
		uint8_t mask = outMask;

		if(!(line[5][x] & 0x80000000)) {
			mask = io_registers[REG_WINOUT] >> 8;
		}

		int32_t window1_mask = ((inWindow1 & gfxInWin[1][x]) | -(inWindow1 & gfxInWin[1][x])) >> 31;
		int32_t window0_mask = ((inWindow0 & gfxInWin[0][x]) | -(inWindow0 & gfxInWin[0][x])) >> 31;
		mask = (inWin1Mask & window1_mask) | (mask & ~window1_mask);
		mask = (inWin0Mask & window0_mask) | (mask & ~window0_mask);

		if((mask & 1) && (line[0][x] < color)) {
			color = line[0][x];
			top = 0x01;
		}

		if((mask & 2) && ((uint8_t)(line[1][x]>>24) < (uint8_t)(color >> 24))) {
			color = line[1][x];
			top = 0x02;
		}

		if((mask & 4) && ((uint8_t)(line[2][x]>>24) < (uint8_t)(color >> 24))) {
			color = line[2][x];
			top = 0x04;
		}

		if((mask & 8) && ((uint8_t)(line[3][x]>>24) < (uint8_t)(color >> 24))) {
			color = line[3][x];
			top = 0x08;
		}

		if((mask & 16) && ((uint8_t)(line[4][x]>>24) < (uint8_t)(color >> 24))) {
			color = line[4][x];
			top = 0x10;
		}

		if(color & 0x00010000)
		{
			// semi-transparent OBJ
			uint32_t back = backdrop;
			uint8_t top2 = 0x20;

			if((mask & 1) && ((uint8_t)(line[0][x]>>24) < (uint8_t)(back >> 24))) {
				back = line[0][x];
				top2 = 0x01;
			}

			if((mask & 2) && ((uint8_t)(line[1][x]>>24) < (uint8_t)(back >> 24))) {
				back = line[1][x];
				top2 = 0x02;
			}

			if((mask & 4) && ((uint8_t)(line[2][x]>>24) < (uint8_t)(back >> 24))) {
				back = line[2][x];
				top2 = 0x04;
			}

			if((mask & 8) && ((uint8_t)(line[3][x]>>24) < (uint8_t)(back >> 24))) {
				back = line[3][x];
				top2 = 0x08;
			}

			alpha_blend_brightness_switch();
		}
		else if((mask & 32) && (top & BLDMOD))
		{
			// special FX on in the window
			switch((BLDMOD >> 6) & 3)
			{
				case 0:
					break;
				case 1:
					{
						uint32_t back = backdrop;
						uint8_t top2 = 0x20;
						if(((mask & 1) && (uint8_t)(line[0][x]>>24) < (uint8_t)(back >> 24)) && top != 0x01)
						{
							back = line[0][x];
							top2 = 0x01;
						}

						if(((mask & 2) && (uint8_t)(line[1][x]>>24) < (uint8_t)(back >> 24)) && top != 0x02)
						{
							back = line[1][x];
							top2 = 0x02;
						}

						if(((mask & 4) && (uint8_t)(line[2][x]>>24) < (uint8_t)(back >> 24)) && top != 0x04)
						{
							back = line[2][x];
							top2 = 0x04;
						}

						if(((mask & 8) && (uint8_t)(line[3][x]>>24) < (uint8_t)(back >> 24)) && top != 0x08)
						{
							back = line[3][x];
							top2 = 0x08;
						}

						if(((mask & 16) && (uint8_t)(line[4][x]>>24) < (uint8_t)(back >> 24)) && top != 0x10) {
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
					color = gfxIncreaseBrightness(color, coeff[COLY & 0x1F]);
					break;
				case 3:
					color = gfxDecreaseBrightness(color, coeff[COLY & 0x1F]);
					break;
			}
		}

		lineMix[x] = CONVERT_COLOR(color);
	}
}
