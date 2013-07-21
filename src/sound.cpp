/* Copyright (C) 2003-2007 Shay Green. This module is free software; you
can redistribute it and/or modify it under the terms of the GNU Lesser
General Public License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version. This
module is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more
details. You should have received a copy of the GNU Lesser General Public
License along with this module; if not, write to the Free Software Foundation,
Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <math.h>

#include "sound.h"

#include "gba.h"
#include "globals.h"

#include "memory.h"
#include "port.h"
#include "system.h"

#define NR10 0x60
#define NR11 0x62
#define NR12 0x63
#define NR13 0x64
#define NR14 0x65
#define NR21 0x68
#define NR22 0x69
#define NR23 0x6c
#define NR24 0x6d
#define NR30 0x70
#define NR31 0x72
#define NR32 0x73
#define NR33 0x74
#define NR34 0x75
#define NR41 0x78
#define NR42 0x79
#define NR43 0x7c
#define NR44 0x7d
#define NR50 0x80
#define NR51 0x81
#define NR52 0x84

/* 1/100th of a second */
#define SOUND_CLOCK_TICKS_ 167772 
#define SOUNDVOLUME 0.5f
#define SOUNDVOLUME_ -1

/*============================================================
	CLASS DECLS
============================================================ */

class Blip_Buffer
{
	public:
		const char * set_sample_rate( long samples_per_sec, int msec_length = 1000 / 4 );
		void clear( void);
		void save_state( blip_buffer_state_t* out );
		void load_state( blip_buffer_state_t const& in );
		uint32_t clock_rate_factor( long clock_rate ) const;
		long clock_rate_;
		int length_;		/* Length of buffer in milliseconds*/
		long sample_rate_;	/* Current output sample rate*/
		uint32_t factor_;
		uint32_t offset_;
		int32_t * buffer_;
		int32_t buffer_size_;
		int32_t reader_accum_;
		Blip_Buffer();
		~Blip_Buffer();
	private:
		Blip_Buffer( const Blip_Buffer& );
		Blip_Buffer& operator = ( const Blip_Buffer& );
};

class Blip_Synth
{
	public:
	Blip_Buffer* buf;
	int delta_factor;

	Blip_Synth();

	void volume( double v ) { delta_factor = int ((v * 1.0) * (1L << BLIP_SAMPLE_BITS) + 0.5); }
	void offset( int32_t, int delta, Blip_Buffer* ) const;
	void offset_resampled( uint32_t, int delta, Blip_Buffer* ) const;
	void offset_inline( int32_t t, int delta, Blip_Buffer* buf ) const {
		offset_resampled( t * buf->factor_ + buf->offset_, delta, buf );
	}
	void offset_inline( int32_t t, int delta ) const {
		offset_resampled( t * buf->factor_ + buf->offset_, delta, buf );
	}
};

class Gb_Osc
{
	public:
	Blip_Buffer* outputs [4];	/* NULL, right, left, center*/
	Blip_Buffer* output;		/* where to output sound*/
	uint8_t * regs;			/* osc's 5 registers*/
	int mode;			/* mode_dmg, mode_cgb, mode_agb*/
	int dac_off_amp;		/* amplitude when DAC is off*/
	int last_amp;			/* current amplitude in Blip_Buffer*/
	Blip_Synth const* good_synth;
	Blip_Synth  const* med_synth;

	int delay;			/* clocks until frequency timer expires*/
	int length_ctr;			/* length counter*/
	unsigned phase;			/* waveform phase (or equivalent)*/
	bool enabled;			/* internal enabled flag*/

	void clock_length();
	void reset();
	protected:
	void update_amp( int32_t, int new_amp );
	int write_trig( int frame_phase, int max_len, int old_data );
};

class Gb_Env : public Gb_Osc
{
	public:
	int  env_delay;
	int  volume;
	bool env_enabled;

	void clock_envelope();
	bool write_register( int frame_phase, int reg, int old_data, int data );

	void reset()
	{
		env_delay = 0;
		volume    = 0;
		Gb_Osc::reset();
	}
	private:
	void zombie_volume( int old, int data );
	int reload_env_timer();
};

class Gb_Square : public Gb_Env
{
	public:
	bool write_register( int frame_phase, int reg, int old_data, int data );
	void run( int32_t, int32_t );

	void reset()
	{
		Gb_Env::reset();
		delay = 0x40000000; /* TODO: something less hacky (never clocked until first trigger)*/
	}
	private:
	/* Frequency timer period*/
	int period() const { return (2048 - GB_OSC_FREQUENCY()) * (CLK_MUL_MUL_4); }
};

class Gb_Sweep_Square : public Gb_Square
{
	public:
	int  sweep_freq;
	int  sweep_delay;
	bool sweep_enabled;
	bool sweep_neg;

	void clock_sweep();
	void write_register( int frame_phase, int reg, int old_data, int data );

	void reset()
	{
		sweep_freq    = 0;
		sweep_delay   = 0;
		sweep_enabled = false;
		sweep_neg     = false;
		Gb_Square::reset();
	}
	private:
	void calc_sweep( bool update );
};

class Gb_Noise : public Gb_Env
{
	public:
	int divider; /* noise has more complex frequency divider setup*/

	void run( int32_t, int32_t );
	void write_register( int frame_phase, int reg, int old_data, int data );

	void reset()
	{
		divider = 0;
		Gb_Env::reset();
		delay = CLK_MUL_MUL_4; /* TODO: remove?*/
	}
};

class Gb_Wave : public Gb_Osc
{
	public:
	int sample_buf;		/* last wave RAM byte read (hardware has this as well)*/
	int agb_mask;		/* 0xFF if AGB features enabled, 0 otherwise*/
	uint8_t* wave_ram;	/* 32 bytes (64 nybbles), stored in APU*/

	void write_register( int frame_phase, int reg, int old_data, int data );
	void run( int32_t, int32_t );

	/* Reads/writes wave RAM*/
	int read( unsigned addr ) const;
	void write( unsigned addr, int data );

	void reset()
	{
		sample_buf = 0;
		Gb_Osc::reset();
	}

	private:
	friend class Gb_Apu;

	/* Frequency timer period*/
	int period() const { return (2048 - GB_OSC_FREQUENCY()) * (CLK_MUL_MUL_2); }

	void corrupt_wave();

	/* Wave index that would be accessed, or -1 if no access would occur*/
	int access( unsigned addr ) const;
};

/*============================================================
	INLINE CLASS FUNCS
============================================================ */

INLINE void Blip_Synth::offset_resampled( uint32_t time, int delta, Blip_Buffer* blip_buf ) const
{
	int32_t left, right, phase;
	int32_t *buf;

	delta *= delta_factor;
	buf = blip_buf->buffer_ + (time >> BLIP_BUFFER_ACCURACY);
	phase = (int) (time >> (BLIP_BUFFER_ACCURACY - BLIP_PHASE_BITS) & BLIP_RES_MIN_ONE);

	left = buf [0] + delta;

	right = (delta >> BLIP_PHASE_BITS) * phase;

	left  -= right;
	right += buf [1];

	buf [0] = left;
	buf [1] = right;
}

INLINE void Blip_Synth::offset( int32_t t, int delta, Blip_Buffer* buf ) const
{
   offset_resampled( t * buf->factor_ + buf->offset_, delta, buf );
}

INLINE int Gb_Wave::read( unsigned addr ) const
{
	int index;

	if(enabled)
		index = access( addr );
	else
		index = addr & 0x0F;
	
	unsigned char const * wave_bank = &wave_ram[(~regs[0] & BANK40_MASK) >> 2 & agb_mask];

	return (index < 0 ? 0xFF : wave_bank[index]);
}

INLINE void Gb_Wave::write( unsigned addr, int data )
{
	int index;

	if(enabled)
		index = access( addr );
	else
		index = addr & 0x0F;
	
	unsigned char * wave_bank = &wave_ram[(~regs[0] & BANK40_MASK) >> 2 & agb_mask];

	if ( index >= 0 )
		wave_bank[index] = data;;
}

static int16_t   soundFinalWave [1600];
long  soundSampleRate    = 22050;
int   SOUND_CLOCK_TICKS  = SOUND_CLOCK_TICKS_;
int   soundTicks         = SOUND_CLOCK_TICKS_;

static int soundEnableFlag   = 0x3ff; /* emulator channels enabled*/
static float const apu_vols [4] = { -0.25f, -0.5f, -1.0f, -0.25f };

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

typedef struct
{
	int last_amp;
	int last_time;
	int shift;
	Blip_Buffer* output;
} gba_pcm_t;

typedef struct
{
	bool enabled;
	uint8_t   fifo [32];
	int  count;
	int  dac;
	int  readIndex;
	int  writeIndex;
	int     which;
	int  timer;
	gba_pcm_t pcm;
} gba_pcm_fifo_t;

static gba_pcm_fifo_t   pcm [2];


static Blip_Synth pcm_synth; // 32 kHz, 16 kHz, 8 kHz

static Blip_Buffer bufs_buffer [BUFS_SIZE];
static int mixer_samples_read;

static void gba_pcm_init (void)
{
	pcm[0].pcm.output    = 0;
	pcm[0].pcm.last_time = 0;
	pcm[0].pcm.last_amp  = 0;
	pcm[0].pcm.shift     = 0;

	pcm[1].pcm.output    = 0;
	pcm[1].pcm.last_time = 0;
	pcm[1].pcm.last_amp  = 0;
	pcm[1].pcm.shift     = 0;
}

static void gba_pcm_apply_control( int pcm_idx, int idx )
{
	int ch = 0;
	pcm[pcm_idx].pcm.shift = ~ioMem [SGCNT0_H] >> (2 + idx) & 1;

	if ( (ioMem [NR52] & 0x80) )
		ch = ioMem [SGCNT0_H+1] >> (idx << 2) & 3;

	Blip_Buffer* out = 0;
	switch ( ch )
	{
		case 1:
			out = &bufs_buffer[1];
			break;
		case 2:
			out = &bufs_buffer[0];
			break;
		case 3:
			out = &bufs_buffer[2];
			break;
	}

	if ( pcm[pcm_idx].pcm.output != out )
	{
		if ( pcm[pcm_idx].pcm.output )
			pcm_synth.offset( SOUND_CLOCK_TICKS - soundTicks, -pcm[pcm_idx].pcm.last_amp, pcm[pcm_idx].pcm.output );
		pcm[pcm_idx].pcm.last_amp = 0;
		pcm[pcm_idx].pcm.output = out;
	}
}

#include "sound_gb_apu.inl"

#include "sound_gb_osc.inl"

#include "sound_blip_buffer.inl"

/*============================================================
	STEREO BUFFER
============================================================ */

/* Uses three buffers (one for center) and outputs stereo sample pairs. */

#define STEREO_BUFFER_SAMPLES_AVAILABLE() ((long)(bufs_buffer[0].offset_ -  mixer_samples_read) << 1)
#define stereo_buffer_samples_avail() ((((bufs_buffer [0].offset_ >> BLIP_BUFFER_ACCURACY) - mixer_samples_read) << 1))


static const char * stereo_buffer_set_sample_rate( long rate, int msec )
{
        mixer_samples_read = 0;
        for ( int i = BUFS_SIZE; --i >= 0; )
                RETURN_ERR( bufs_buffer [i].set_sample_rate( rate, msec ) );
        return 0; 
}

static void stereo_buffer_clock_rate( long rate )
{
	bufs_buffer[2].factor_ = bufs_buffer [2].clock_rate_factor( rate );
	bufs_buffer[1].factor_ = bufs_buffer [1].clock_rate_factor( rate );
	bufs_buffer[0].factor_ = bufs_buffer [0].clock_rate_factor( rate );
}

static void stereo_buffer_clear (void)
{
        mixer_samples_read = 0;
	bufs_buffer [2].clear();
	bufs_buffer [1].clear();
	bufs_buffer [0].clear();
}

/* mixers use a single index value to improve performance on register-challenged processors
 * offset goes from negative to zero*/

static INLINE void stereo_buffer_mixer_read_pairs( int16_t* out, int count )
{
	/* TODO: if caller never marks buffers as modified, uses mono*/
	/* except that buffer isn't cleared, so caller can encounter*/
	/* subtle problems and not realize the cause.*/
	mixer_samples_read += count;
	int16_t* outtemp = out + count * STEREO;

	/* do left + center and right + center separately to reduce register load*/
	Blip_Buffer* buf = &bufs_buffer [2];
	{
		--buf;
		--outtemp;

		BLIP_READER_BEGIN( side,   *buf );
		BLIP_READER_BEGIN( center, bufs_buffer[2] );

		BLIP_READER_ADJ_( side,   mixer_samples_read );
		BLIP_READER_ADJ_( center, mixer_samples_read );

		int offset = -count;
		do
		{
			int s = (center_reader_accum + side_reader_accum) >> 14;
			BLIP_READER_NEXT_IDX_( side,   offset );
			BLIP_READER_NEXT_IDX_( center, offset );
			BLIP_CLAMP( s, s );

			++offset; /* before write since out is decremented to slightly before end*/
			outtemp [offset * STEREO] = (int16_t) s;
		}while ( offset );

		BLIP_READER_END( side,   *buf );
	}
	{
		--buf;
		--outtemp;

		BLIP_READER_BEGIN( side,   *buf );
		BLIP_READER_BEGIN( center, bufs_buffer[2] );

		BLIP_READER_ADJ_( side,   mixer_samples_read );
		BLIP_READER_ADJ_( center, mixer_samples_read );

		int offset = -count;
		do
		{
			int s = (center_reader_accum + side_reader_accum) >> 14;
			BLIP_READER_NEXT_IDX_( side,   offset );
			BLIP_READER_NEXT_IDX_( center, offset );
			BLIP_CLAMP( s, s );

			++offset; /* before write since out is decremented to slightly before end*/
			outtemp [offset * STEREO] = (int16_t) s;
		}while ( offset );

		BLIP_READER_END( side,   *buf );

		/* only end center once*/
		BLIP_READER_END( center, bufs_buffer[2] );
	}
}

static void blip_buffer_remove_all_samples( long count )
{
	uint32_t new_offset = (uint32_t)count << BLIP_BUFFER_ACCURACY;
	/* BLIP BUFFER #1 */
	bufs_buffer[0].offset_ -= new_offset;
	bufs_buffer[1].offset_ -= new_offset;
	bufs_buffer[2].offset_ -= new_offset;

	/* copy remaining samples to beginning and clear old samples*/
	long remain = (bufs_buffer[0].offset_ >> BLIP_BUFFER_ACCURACY) + BLIP_BUFFER_EXTRA_;
	memmove( bufs_buffer[0].buffer_, bufs_buffer[0].buffer_ + count, remain * sizeof *bufs_buffer[0].buffer_ );
	memset( bufs_buffer[0].buffer_ + remain, 0, count * sizeof(*bufs_buffer[0].buffer_));

	remain = (bufs_buffer[1].offset_ >> BLIP_BUFFER_ACCURACY) + BLIP_BUFFER_EXTRA_;
	memmove( bufs_buffer[1].buffer_, bufs_buffer[1].buffer_ + count, remain * sizeof *bufs_buffer[1].buffer_ );
	memset( bufs_buffer[1].buffer_ + remain, 0, count * sizeof(*bufs_buffer[1].buffer_));

	remain = (bufs_buffer[2].offset_ >> BLIP_BUFFER_ACCURACY) + BLIP_BUFFER_EXTRA_;
	memmove( bufs_buffer[2].buffer_, bufs_buffer[2].buffer_ + count, remain * sizeof *bufs_buffer[2].buffer_ );
	memset( bufs_buffer[2].buffer_ + remain, 0, count * sizeof(*bufs_buffer[2].buffer_));
}

static long stereo_buffer_read_samples( int16_t * out, long out_size )
{
	int pair_count;

        out_size = (STEREO_BUFFER_SAMPLES_AVAILABLE() < out_size) ? STEREO_BUFFER_SAMPLES_AVAILABLE() : out_size;

        pair_count = int (out_size >> 1);
        if ( pair_count )
	{
		stereo_buffer_mixer_read_pairs( out, pair_count );
		blip_buffer_remove_all_samples( mixer_samples_read );
		mixer_samples_read = 0;
	}
        return out_size;
}

static void gba_to_gb_sound_parallel( int * __restrict addr, int * __restrict addr2 )
{
	uint32_t addr1_table = *addr - 0x60;
	uint32_t addr2_table = *addr2 - 0x60;
	*addr = table [addr1_table];
	*addr2 = table [addr2_table];
}

static void pcm_fifo_write_control( int data, int data2)
{
	pcm[0].enabled = (data & 0x0300) ? true : false;
	pcm[0].timer   = (data & 0x0400) ? 1 : 0;

	if ( data & 0x0800 )
	{
		// Reset
		pcm[0].writeIndex = 0;
		pcm[0].readIndex  = 0;
		pcm[0].count      = 0;
		pcm[0].dac        = 0;
		memset(pcm[0].fifo, 0, sizeof(pcm[0].fifo));
	}

	gba_pcm_apply_control( 0, pcm[0].which );

	if(pcm[0].pcm.output)
	{
		int time = SOUND_CLOCK_TICKS -  soundTicks;

		pcm[0].dac = (int8_t)pcm[0].dac >> pcm[0].pcm.shift;
		int delta = pcm[0].dac - pcm[0].pcm.last_amp;
		if ( delta )
		{
			pcm[0].pcm.last_amp = pcm[0].dac;
			pcm_synth.offset( time, delta, pcm[0].pcm.output );
		}
		pcm[0].pcm.last_time = time;
	}

	pcm[1].enabled = (data2 & 0x0300) ? true : false;
	pcm[1].timer   = (data2 & 0x0400) ? 1 : 0;

	if ( data2 & 0x0800 )
	{
		// Reset
		pcm[1].writeIndex = 0;
		pcm[1].readIndex  = 0;
		pcm[1].count      = 0;
		pcm[1].dac        = 0;
		memset( pcm[1].fifo, 0, sizeof(pcm[1].fifo));
	}

	gba_pcm_apply_control( 1, pcm[1].which );

	if(pcm[1].pcm.output)
	{
		int time = SOUND_CLOCK_TICKS -  soundTicks;

		pcm[1].dac = (int8_t)pcm[1].dac >> pcm[1].pcm.shift;
		int delta = pcm[1].dac - pcm[1].pcm.last_amp;
		if ( delta )
		{
			pcm[1].pcm.last_amp = pcm[1].dac;
			pcm_synth.offset( time, delta, pcm[1].pcm.output );
		}
		pcm[1].pcm.last_time = time;
	}
}

static void soundEvent_u16_parallel(uint32_t address[])
{
	for(int i = 0; i < 8; i++)
	{
		switch ( address[i] )
		{
			case SGCNT0_H:
				//Begin of Write SGCNT0_H
				WRITE16LE( &ioMem [SGCNT0_H], 0 & 0x770F );
				pcm_fifo_write_control(0, 0);

				gb_apu_volume( apu_vols [ioMem [SGCNT0_H] & 3] );
				//End of SGCNT0_H
				break;

			case FIFOA_L:
			case FIFOA_H:
				pcm[0].fifo [pcm[0].writeIndex  ] = 0;
				pcm[0].fifo [pcm[0].writeIndex+1] = 0;
				pcm[0].count += 2;
				pcm[0].writeIndex = (pcm[0].writeIndex + 2) & 31;
				WRITE16LE( &ioMem[address[i]], 0 );
				break;

			case FIFOB_L:
			case FIFOB_H:
				pcm[1].fifo [pcm[1].writeIndex  ] = 0;
				pcm[1].fifo [pcm[1].writeIndex+1] = 0;
				pcm[1].count += 2;
				pcm[1].writeIndex = (pcm[1].writeIndex + 2) & 31;
				WRITE16LE( &ioMem[address[i]], 0 );
				break;

			case 0x88:
				WRITE16LE( &ioMem[address[i]], 0 );
				break;

			default:
				{
					int gb_addr[2]	= {address[i] & ~1, address[i] | 1};
					uint32_t address_array[2] = {address[i] & ~ 1, address[i] | 1};
					uint8_t data_array[2] = {0};
					gba_to_gb_sound_parallel(&gb_addr[0], &gb_addr[1]);
					soundEvent_u8_parallel(gb_addr, address_array, data_array);
					break;
				}
		}
	}
}

static void gba_pcm_fifo_timer_overflowed( unsigned pcm_idx )
{
	if ( pcm[pcm_idx].count <= 16 )
	{
		// Need to fill FIFO
		CPUCheckDMA( 3, pcm[pcm_idx].which ? 4 : 2 );

		if ( pcm[pcm_idx].count <= 16 )
		{
			// Not filled by DMA, so fill with 16 bytes of silence
			int reg = pcm[pcm_idx].which ? FIFOB_L : FIFOA_L;

			uint32_t address_array[8] = {reg, reg+2, reg, reg+2, reg, reg+2, reg, reg+2};
			soundEvent_u16_parallel(address_array);
		}
	}

	// Read next sample from FIFO
	pcm[pcm_idx].count--;
	pcm[pcm_idx].dac = pcm[pcm_idx].fifo [pcm[pcm_idx].readIndex];
	pcm[pcm_idx].readIndex = (pcm[pcm_idx].readIndex + 1) & 31;

	if(pcm[pcm_idx].pcm.output)
	{
		int time = SOUND_CLOCK_TICKS -  soundTicks;

		pcm[pcm_idx].dac = (int8_t)pcm[pcm_idx].dac >> pcm[pcm_idx].pcm.shift;
		int delta = pcm[pcm_idx].dac - pcm[pcm_idx].pcm.last_amp;
		if ( delta )
		{
			pcm[pcm_idx].pcm.last_amp = pcm[pcm_idx].dac;
			pcm_synth.offset( time, delta, pcm[pcm_idx].pcm.output );
		}
		pcm[pcm_idx].pcm.last_time = time;
	}
}

void soundEvent_u8_parallel(int gb_addr[], uint32_t address[], uint8_t data[])
{
	for(uint32_t i = 0; i < 2; i++)
	{
		ioMem[address[i]] = data[i];
		gb_apu_write_register( SOUND_CLOCK_TICKS -  soundTicks, gb_addr[i], data[i] );

		if ( address[i] == NR52 )
		{
			gba_pcm_apply_control(0, 0 );
			gba_pcm_apply_control(1, 1 );
		}
		// TODO: what about byte writes to SGCNT0_H etc.?
	}
}

void soundEvent_u8(int gb_addr, uint32_t address, uint8_t data)
{
	ioMem[address] = data;
	gb_apu_write_register( SOUND_CLOCK_TICKS -  soundTicks, gb_addr, data );

	if ( address == NR52 )
	{
		gba_pcm_apply_control(0, 0 );
		gba_pcm_apply_control(1, 1 );
	}
	// TODO: what about byte writes to SGCNT0_H etc.?
}


void soundEvent_u16(uint32_t address, uint16_t data)
{
	switch ( address )
	{
		case SGCNT0_H:
			//Begin of Write SGCNT0_H
			WRITE16LE( &ioMem [SGCNT0_H], data & 0x770F );
			pcm_fifo_write_control( data, data >> 4);

			gb_apu_volume( apu_vols [ioMem [SGCNT0_H] & 3] );
			//End of SGCNT0_H
			break;

		case FIFOA_L:
		case FIFOA_H:
			pcm[0].fifo [pcm[0].writeIndex  ] = data & 0xFF;
			pcm[0].fifo [pcm[0].writeIndex+1] = data >> 8;
			pcm[0].count += 2;
			pcm[0].writeIndex = (pcm[0].writeIndex + 2) & 31;
			WRITE16LE( &ioMem[address], data );
			break;

		case FIFOB_L:
		case FIFOB_H:
			pcm[1].fifo [pcm[1].writeIndex  ] = data & 0xFF;
			pcm[1].fifo [pcm[1].writeIndex+1] = data >> 8;
			pcm[1].count += 2;
			pcm[1].writeIndex = (pcm[1].writeIndex + 2) & 31;
			WRITE16LE( &ioMem[address], data );
			break;

		case 0x88:
			data &= 0xC3FF;
			WRITE16LE( &ioMem[address], data );
			break;

		default:
			{
				int gb_addr[2]	= {address & ~1, address | 1};
				uint32_t address_array[2] = {address & ~ 1, address | 1};
				uint8_t data_array[2] = {(uint8_t)data, (uint8_t)(data >> 8)};
				gba_to_gb_sound_parallel(&gb_addr[0], &gb_addr[1]);
				soundEvent_u8_parallel(gb_addr, address_array, data_array);
				break;
			}
	}
}

void soundTimerOverflow(int timer)
{
	if ( timer == pcm[0].timer && pcm[0].enabled )
		gba_pcm_fifo_timer_overflowed(0);
	if ( timer == pcm[1].timer && pcm[1].enabled )
		gba_pcm_fifo_timer_overflowed(1);
}

void process_sound_tick_fn (void)
{
	// Run sound hardware to present
	pcm[0].pcm.last_time -= SOUND_CLOCK_TICKS;
	if ( pcm[0].pcm.last_time < -2048 )
		pcm[0].pcm.last_time = -2048;

	pcm[1].pcm.last_time -= SOUND_CLOCK_TICKS;
	if ( pcm[1].pcm.last_time < -2048 )
		pcm[1].pcm.last_time = -2048;

	/* Emulates sound hardware up to a specified time, ends current time
	frame, then starts a new frame at time 0 */

	if(SOUND_CLOCK_TICKS > gb_apu.last_time)
		gb_apu_run_until_( SOUND_CLOCK_TICKS );

	gb_apu.frame_time -= SOUND_CLOCK_TICKS;
	gb_apu.last_time -= SOUND_CLOCK_TICKS;

	bufs_buffer[2].offset_ += SOUND_CLOCK_TICKS * bufs_buffer[2].factor_;
	bufs_buffer[1].offset_ += SOUND_CLOCK_TICKS * bufs_buffer[1].factor_;
	bufs_buffer[0].offset_ += SOUND_CLOCK_TICKS * bufs_buffer[0].factor_;


	// dump all the samples available
	// VBA will only ever store 1 frame worth of samples
	int numSamples = stereo_buffer_read_samples( (int16_t*) soundFinalWave, stereo_buffer_samples_avail());
	systemOnWriteDataToSoundBuffer(soundFinalWave, numSamples);
}

static void apply_muting (void)
{
	// PCM
	gba_pcm_apply_control(1, 0 );
	gba_pcm_apply_control(1, 1 );

	// APU
	gb_apu_set_output( &bufs_buffer[2], &bufs_buffer[0], &bufs_buffer[1], 0 );
	gb_apu_set_output( &bufs_buffer[2], &bufs_buffer[0], &bufs_buffer[1], 1 );
	gb_apu_set_output( &bufs_buffer[2], &bufs_buffer[0], &bufs_buffer[1], 2 );
	gb_apu_set_output( &bufs_buffer[2], &bufs_buffer[0], &bufs_buffer[1], 3 );
}


static void remake_stereo_buffer (void)
{
	if ( !ioMem )
		return;

	// Clears pointers kept to old stereo_buffer
	gba_pcm_init();

	// Stereo_Buffer

        mixer_samples_read = 0;
	stereo_buffer_set_sample_rate( soundSampleRate, BLIP_DEFAULT_LENGTH );
	stereo_buffer_clock_rate( CLOCK_RATE );

	// PCM
	pcm [0].which = 0;
	pcm [1].which = 1;

	// APU
	gb_apu_new();
	gb_apu_reset( MODE_AGB, true );

	stereo_buffer_clear();

	soundTicks = SOUND_CLOCK_TICKS;

	apply_muting();

	gb_apu_volume(apu_vols [ioMem [SGCNT0_H] & 3] );

	pcm_synth.volume( 0.66 / 256 * SOUNDVOLUME_ );
}

void soundReset (void)
{
	remake_stereo_buffer();
	//Begin of Reset APU
	gb_apu_reset( MODE_AGB, true );

	stereo_buffer_clear();

	soundTicks = SOUND_CLOCK_TICKS;
	//End of Reset APU

	SOUND_CLOCK_TICKS = SOUND_CLOCK_TICKS_;
	soundTicks        = SOUND_CLOCK_TICKS_;

	// Sound Event (NR52)
	int gb_addr = table[NR52 - 0x60];
	if ( gb_addr )
	{
		ioMem[NR52] = 0x80;
		gb_apu_write_register( SOUND_CLOCK_TICKS -  soundTicks, gb_addr, 0x80 );

		gba_pcm_apply_control(0, 0 );
		gba_pcm_apply_control(1, 1 );
	}

	// TODO: what about byte writes to SGCNT0_H etc.?
	// End of Sound Event (NR52)
}

void soundSetSampleRate(long sampleRate)
{
	if ( soundSampleRate != sampleRate )
	{
		soundSampleRate      = sampleRate;
		remake_stereo_buffer();
	}
}

static int dummy_state [16];

#define SKIP( type, name ) { dummy_state, sizeof (type) }

#define LOAD( type, name ) { &name, sizeof (type) }

static struct {
	gb_apu_state_t apu;

	// old state
	int soundDSBValue;
	uint8_t soundDSAValue;
} state;

// New state format
static variable_desc gba_state [] =
{
	// PCM
	LOAD( int, pcm [0].readIndex ),
	LOAD( int, pcm [0].count ),
	LOAD( int, pcm [0].writeIndex ),
	LOAD(uint8_t[32],pcm[0].fifo ),
	LOAD( int, pcm [0].dac ),

	SKIP( int [4], room_for_expansion ),

	LOAD( int, pcm [1].readIndex ),
	LOAD( int, pcm [1].count ),
	LOAD( int, pcm [1].writeIndex ),
	LOAD(uint8_t[32],pcm[1].fifo ),
	LOAD( int, pcm [1].dac ),

	SKIP( int [4], room_for_expansion ),

	// APU
	LOAD( uint8_t [0x40], state.apu.regs ),      // last values written to registers and wave RAM (both banks)
	LOAD( int, state.apu.frame_time ),      // clocks until next frame sequencer action
	LOAD( int, state.apu.frame_phase ),     // next step frame sequencer will run

	LOAD( int, state.apu.sweep_freq ),      // sweep's internal frequency register
	LOAD( int, state.apu.sweep_delay ),     // clocks until next sweep action
	LOAD( int, state.apu.sweep_enabled ),
	LOAD( int, state.apu.sweep_neg ),       // obscure internal flag
	LOAD( int, state.apu.noise_divider ),
	LOAD( int, state.apu.wave_buf ),        // last read byte of wave RAM

	LOAD( int [4], state.apu.delay ),       // clocks until next channel action
	LOAD( int [4], state.apu.length_ctr ),
	LOAD( int [4], state.apu.phase ),       // square/wave phase, noise LFSR
	LOAD( int [4], state.apu.enabled ),     // internal enabled flag

	LOAD( int [3], state.apu.env_delay ),   // clocks until next envelope action
	LOAD( int [3], state.apu.env_volume ),
	LOAD( int [3], state.apu.env_enabled ),

	SKIP( int [13], room_for_expansion ),

	// Emulator
	LOAD( int, soundEnableFlag ),

	SKIP( int [15], room_for_expansion ),

	{ NULL, 0 }
};

void soundSaveGameMem(uint8_t *& data)
{
	gb_apu_save_state(&state.apu);
	memset(dummy_state, 0, sizeof dummy_state);
	utilWriteDataMem(data, gba_state);
}

void soundReadGameMem(const uint8_t *& in_data, int)
{
	// Prepare APU and default state

	//Begin of Reset APU
	gb_apu_reset( MODE_AGB, true );

	stereo_buffer_clear();

	soundTicks = SOUND_CLOCK_TICKS;
	//End of Reset APU

	gb_apu_save_state( &state.apu );

	utilReadDataMem( in_data, gba_state );

	gb_apu_load_state( state.apu );
	//Begin of Write SGCNT0_H
	int data = (READ16LE( &ioMem [SGCNT0_H] ) & 0x770F);
	WRITE16LE( &ioMem [SGCNT0_H], data & 0x770F );
	pcm_fifo_write_control( data, data >> 4 );

	gb_apu_volume(apu_vols [ioMem [SGCNT0_H] & 3] );
	//End of SGCNT0_H
}

