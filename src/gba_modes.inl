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
      { \
	if(color < 0x80000000) \
	{ \
		GFX_ALPHA_BLEND(color, back, coeff[COLEV & 0x1F], coeff[(COLEV >> 8) & 0x1F]); \
	} \
      } \
      else if(BLDMOD & top) \
      { \
         brightness_switch(); \
      }
