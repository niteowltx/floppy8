#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include <libgen.h>

//	ext --- extract sector data from floppy given timestamp files for each track

// Data was collected from the floppy as pulses separated by 2-4us, with some variation.
// FM data pulses are either 2us or 4us.  MFM data pulses arrive with deltas of 2, 3 or 4us.
// On the disk, locations where the recording started or stopped may have very large or
// very smal deltas.  Drive speed rotation may also affect the samples pulse width values.

// The capture device (Teensy 4.1) runs at 600Mhz and samples are divided by 16 so 1 us equals 37.5 counts

#define TWO_US		75
#define ONE_US		(TWO_US/2)
#define	THREE_US	(TWO_US+ONE_US)
#define	FOUR_US		(TWO_US*2)
#define	FIVE_US		((TWO_US*2)+ONE_US)

#define	HALF_US		(ONE_US/2)
#define	ONEP5_US	(ONE_US+HALF_US)
#define	TWOP5_US	(TWO_US+HALF_US)
#define	THREEP5_US	(THREE_US+HALF_US)
#define	FOURP5_US	(FOUR_US+HALF_US)

#define	FM_SPLIT	THREE_US	// FM has 2 ranges: 2 and 4 us
#define	MFM_SPLIT_LO	TWOP5_US	// MFM has 3 ranges: 2, 3 and 4 us
#define	MFM_SPLIT_HI	THREEP5_US

#define  MAX_US    6		// samples are classified into 1us buckets.   +/- 0.5us

// For FM disks, track layout is:
//
// Once at beginning of track:
//      40      FF
//      6       00
//      1       FC      Index mark (clock D7)
//      26      FF
//
// For each sector:
//      6       00
//      1       FE      Address ID Mark (clock C7)
//      1       TT      Track (00-4C)
//      1       SD      Side (00)
//      1       ST      Sector (01-1A)
//      1       SS      Sector Size (00=128, 01=256, 02=512, 03=1024)
//      2       CRC
//      11      FF
//      6       00
//      1       FB      Data Mark (Clock C7) (Or F8 if 'deleted data')
//      ???     ??      Data
//      2       CRC
//      27      FF
//
// Once at end of data:
//      247     FF      count is approximate

// For MFM disks, track layout is:
//
// Once at beginning of track:
//      80      4E
//      12      00
//      3       C2
//      1       FC      Index Mark
//      50      4E
//
// For each sector:
//      12      00
//      3       A1
//      1       FE      Address ID Mark
//      1       TT      Track (00-4C)
//      1       SD      Side (00)
//      1       ST      Sector (01-1A)
//      1       SS      Sector Size (as above)
//      2       CRC
//      22      4E
//      12      00
//      3       A1
//      1       FB      Data Mark (Or F8 if 'deleted data')
//      ???     ??      Data
//      2       CRC
//      54      4E
//
// Once at end of data:
//      598     4E      count is approximate

// CRC bytes are 16-bits, polynomial is X**16 + X**12 + X**5 + 1.  CRC is calculated
// so that Address/Data mark + data + CRC == 0x0000

#define	NTRACKS		77
#define	NSIDES		1
#define NSECTORS	33	// sectors range from 0 to NSECTORS-1
#define NSIZES		4	// sector size is 128 << size
#define MAX_SSIZE	1024	// sector size can be 128/256/512/1024

bool		Verbose = false;
bool		Json_show = false;
unsigned int	Last_track = NTRACKS;
unsigned int	Last_side;
unsigned int	Last_sector;
unsigned int	Last_size;

// Raw track data is classified into types before deltas are categorized
#define TT_FM	1
#define TT_MFM	2

#define MAX_SAMPLES 200000
typedef unsigned int sample_t;

// Special FM marks
uint8_t FM_indx_mark[] = { 1,1,1,0,1,1,0,1,1,1,0,0 };	// Data 0xFC,Clock 0xD7
uint8_t FM_addr_mark[] = { 1,1,1,0,0,0,1,1,1,1,1,0 };	// Data 0xFE,Clock 0xC7
uint8_t FM_data_mark[] = { 1,1,1,0,0,0,1,0,1,1,1,1 };	// Data 0xFB,Clock 0xC7
uint8_t FM_deld_mark[] = { 1,1,1,0,0,0,1,0,0,0,1 };	// Data 0xF8,Clock 0xC7

// Special MFM marks
uint8_t MFM_indx_mark[] = { 0xC2, 0xC2, 0xC2, 0xFC };
uint8_t MFM_addr_mark[] = { 0xA1, 0xA1, 0xA1, 0xFE };
uint8_t MFM_data_mark[] = { 0xA1, 0xA1, 0xA1, 0xFB };
uint8_t MFM_deld_mark[] = { 0xA1, 0xA1, 0xA1, 0xF8 };

// Pad decode buffers by this much in case the sample buffer ends with a valid 'mark'
// This MIGHT fail if the padded area happens to have a correct CRC
#define	DECODE_PAD	(2*8*(MAX_SSIZE))

typedef struct sector {
	unsigned int size;
	uint8_t	*data;
} sector_t;
sector_t Disk[NTRACKS][NSECTORS];

static inline void
fatal(const char *s)
{
	printf("# FATAL: %s\n",s);
	exit(1);
}

static inline void
error(const char *s)
{
	printf("# ERROR: %s\n",s);
}

// invalidate last known sector info
static inline void
sector_none()
{
	Last_track  = NTRACKS;
	Last_sector = NSECTORS;
	Last_side   = NSIDES;
	Last_size   = 0;
}

static inline bool
valid_size(const unsigned int size)
{
	unsigned int i;

	for(i=0;i<NSIZES;i++){
		if( size == (128u<<i) )
			return true;
		}
	return false;
}

// add one sectors worth of data to the overall disk image
static inline void
disk_add(const unsigned int track, const unsigned int side, const unsigned int sector, const unsigned int size, const uint8_t *data)
{
	sector_t *s;

	if(track>=NTRACKS || side>=NSIDES || sector>=NSECTORS || !valid_size(size) ){
		printf("# ERROR: invalid params Track:%u Side:%u Sector:%u Size:%u\n",track,side,sector,size);
		return;
		}
	if(data==NULL){
		error("missing data");
		return;
		}
	s = &Disk[track][sector];
	if( s->size==0 && s->data == NULL ){	// first time seen
		s->data = (uint8_t *)malloc(size);
		memcpy(s->data,data,size);
		s->size = size;
		}

	if( s->size != size )
		error("Inconsistent sector size");
	if( memcmp(s->data,data,s->size) != 0 )
		error("Inconsistent sector data");
	if(Verbose)
		printf("OK\n");
}

// is the sector all the same value?
static inline bool
sector_filled(const uint8_t *buf, const unsigned int count)
{
	unsigned int i;

	for(i=1;i<count;i++)
		if( buf[0]!=buf[i] )
			return false;
	return true;
}

#define	DSTEP	32
static inline void
sector_dump(const uint8_t *buf, const unsigned int count)
{
	unsigned int i,j;

	for(i=0;i<count;i+=DSTEP){
		printf("# ");
		for(j=0;j<DSTEP;j++){
			if( buf[i+j] )
				printf("%02X ",buf[i+j]);
			else
				printf("__ ");
			}
		printf("| ");
		for(j=0;j<DSTEP;j++)
			printf("%c",isprint(buf[i+j]) ? buf[i+j] : '_');
		printf("\n");
		}
}

static inline char
size_to_let(unsigned int size)
{
	if(size==0)return '.';
	if(size==128)return '1';
	if(size==256)return '2';
	if(size==512)return '3';
	if(size==1024)return '4';
	return '?';
}

// show one sector in human readable form
static inline void
human_show(sector_t *s, unsigned int track, unsigned int sector)
{
	printf("# Track:%-2u Sector:%-2u Size:%-4u Status:",track,sector,s->size);
	if( s->size==0 || s->data==NULL )
		printf("MISSING\n");
	else if( sector_filled(s->data,s->size) ){
		if( s->data[0]==0 )
			printf("ZERO\n");
		else
			printf("FILL=0x%02X\n",s->data[0]);
		}
	else{
		printf("DATA\n");
		sector_dump(s->data,s->size);
		}
}

// show one sector in JSON format
static inline void
json_show(sector_t *s, unsigned int track, unsigned int sector)
{
	unsigned int i;

	printf("{\n");
	printf(" \"track\": %u,",track);
	printf(" \"sector\": %u,",sector);
	printf(" \"size\": %u,",s->size);
	printf(" \"data\":[\n");
	for(i=0;s->data && i<s->size;i++){
		printf("0x%X,",s->data[i]);
		if( (i%32)==31 )
			printf("\n");
		}
	printf(" ],\n");
	printf("}\n");
}

static inline void
disk_show()
{
	unsigned int sector_min = NSECTORS;
	unsigned int sector_max = 0;
	unsigned int track;
	unsigned int sector;
	sector_t *s;

	// establish sector min/max
	for(track=0;track<NTRACKS;track++)
	for(sector=0;sector<NSECTORS;sector++){
		s = &Disk[track][sector];
		if( s->size ){
			if(sector<sector_min)
				sector_min = sector;
			if(sector>sector_max)
				sector_max = sector;
			}
		}
	// sector_min is always either 0 or 1?
	if(sector_min > 1)
		sector_min=1;

	printf("# Track/Sector map: .=Missing, 1=128, 2=256, 3=512, 4=1014\n");
	for(sector=sector_min;sector<=sector_max;sector++){
		printf("#\t%2u: ",sector);
		for(track=0;track<NTRACKS;track++)
			printf("%c",size_to_let(Disk[track][sector].size));
		printf("\n");
		}

	for(track=0;track<NTRACKS;track++)
	for(sector=sector_min;sector<=sector_max;sector++){
		s = &Disk[track][sector];
		if(Json_show)
			json_show(s,track,sector);
		else
			human_show(s,track,sector);
		}
}

// Does a piece of decoded data match one of the special mark patterns?
static inline bool
mark_match (const uint8_t *s, const uint8_t *pat, const unsigned int len)
{
	unsigned int i;

	for(i=0;i<len;i++)
		if (s[i] != pat[i])
			return false;
	return true;
}

static inline unsigned short
crc16 (const uint8_t *buf, const unsigned int count)
{
	unsigned int i;
	uint8_t x;
	unsigned short crc = 0xffff;

	for(i=0; i<count; i++){
		x = crc >> 8 ^ buf[i];
		x ^= x >> 4;
		crc = (crc << 8) ^ ((unsigned short) (x << 12)) ^ ((unsigned short) (x << 5)) ^ ((unsigned short) x);
	}
	return crc;
}

// Given a sample value in ticks, return which microsecond bucket it falls into
static unsigned int
sample_to_us (sample_t s)
{
	unsigned int us = (s + (ONE_US / 2)) / ONE_US;

	return (us < MAX_US) ? us : MAX_US - 1;
}

// Look at samples and decide if it looks like FM or MFM encoding
// FM has 2 peaks at 2us and 4us.  MFM has peaks at 2, 3 and 4us.
// If there are more than about 5% of the samples at 3us, its probably MFM.
static inline int
determine_format (const sample_t *samples, const unsigned int n)
{
	unsigned int i;
	unsigned int fmt;
	unsigned int histogram[MAX_US];
	unsigned int s;

	for (i = 0; i < MAX_US; i++)
		histogram[i] = 0;
	for (i = 0; i < n; i++){
		s = sample_to_us(samples[i]);
		histogram[s]++;
		}
	fmt = (((histogram[3] * 100) / n) > 5) ? TT_MFM : TT_FM;

	if(Verbose){
		printf("# Histogram:\n");
		for(i=0;i<MAX_US;i++)
			printf("# %2u: %u\n",i,histogram[i]);
		printf("# Track Format: %s\n", (fmt==TT_FM) ? "FM":"MFM");
		}

	return fmt;
}

static inline void
mark_used(uint8_t *buf, unsigned int count, char tag)
{
	unsigned int i;

	for(i=0;i<count;i++)
		buf[i] = tag;
}

#if 0
static inline unsigned int
count_repeat(const uint8_t *buf, const unsigned int count)
{
	unsigned int i;

	for(i=1;i<count;i++)
		if( buf[i] != buf[0] )
			break;
	return i;
}

// show used/unused areas in decoded track
// bytes are either 0/1 or marked with a printable letter
static inline void
track_map(uint8_t *buf, unsigned int count)
{
	unsigned int i,j;
	unsigned int repeat;

	printf("Track use map\n");
	for(i=0;i<count;i+=repeat){
		repeat = count_repeat(&buf[i],count-i);
		if(buf[i]==0){
			if( repeat>=100 )
				printf(" 0:%u",repeat);
			else{
				printf(" ");
				for(j=0;j<repeat;j++)
					printf("0");
				}
			}
		else if(buf[i]==1){
			if( repeat>=100 )
				printf(" 1:%u",repeat);
			else{
				printf(" ");
				for(j=0;j<repeat;j++)
					printf("1");
				}
			}
		else if(isprint(buf[i]))
			printf("\n%c:%u",buf[i],repeat);
		else
			printf("_:%u",repeat);
		}
	printf("\n");
}
#endif

// fetch an FM encoded byte, return value, update buf pointer
static inline uint8_t
fm_fetch_byte (uint8_t ** buf)
{
	unsigned int i;
	unsigned int byte = 0;
	uint8_t *p = *buf;

	for (i = 0; i < 8; i++) {
		byte <<= 1;
		byte |= *p;
		if( p[0]==1 && p[1]==1 )
			p += 2;
		else
			p += 1;
	}
	*buf = p;
	return byte;
}

// fetch bytes from in, save to out, return updated in ptr
static inline uint8_t *
fm_fetch_bytes (uint8_t * in, uint8_t *out, unsigned int count)
{
	unsigned int i;

	for(i=0;i<count;i++)
		out[i] = fm_fetch_byte(&in);
	return in;
}

// Examine an address mark and see if it is valid.  Return the number of input bytes consumed
// and fill in track,sector,side,ssize if true
static inline unsigned int
fm_valid_addr (uint8_t *buf, unsigned int *track, unsigned int *side, unsigned int *sector, unsigned int *size)
{
	uint8_t *bufend;
	uint8_t addr[1+4+2];	// Address mark, Track, Side, Sector, Size, 2 CRC

	addr[0] = 0xFE;
	bufend = fm_fetch_bytes (buf, &addr[1], 6);
	if (crc16 (addr, sizeof (addr)) != 0)
		return 0;
	if( addr[1] >= NTRACKS )
		return 0;
	if( addr[2] >= NSIDES )
		return 0;
	if( addr[3] >= NSECTORS )
		return 0;
	if (addr[4] >= NSIZES)
		return 0;	// invalid ssize code

	*track  = addr[1];
	*side   = addr[2]; // always 0 for SA-800
	*sector = addr[3];
	*size   = 128 << addr[4];
	return bufend-buf;
}

static inline unsigned int
fm_valid_data (uint8_t *buf, unsigned int sector_size, uint8_t *sector_data)
{
	uint8_t *bufend;
	uint8_t data[1 + MAX_SSIZE + 2];	// Data mark, data bytes, 2 CRC

	if(sector_size>MAX_SSIZE)
		return 0;
	data[0] = 0xFB;

	bufend = fm_fetch_bytes (buf, &data[1], sector_size+2);
	if (crc16 (data, 1 + sector_size + 2) != 0)
		return 0;
	memcpy(sector_data,&data[1],sector_size);
	return bufend-buf;
}

static inline unsigned int
fm_valid_deld (uint8_t *buf, unsigned int sector_size, uint8_t *sector_data)
{
	uint8_t *bufend;
	uint8_t data[1 + MAX_SSIZE + 2];	// Data mark, data bytes, 2 CRC

	if(sector_size>MAX_SSIZE)
		return 0;
	data[0] = 0xF8;	// deleted data

	bufend = fm_fetch_bytes (buf, &data[1], sector_size+2);
	if (crc16 (data, 1 + sector_size + 2) != 0)
		return 0;
	memcpy(sector_data,&data[1],sector_size);
	return bufend-buf;
}

static inline unsigned int
fm_indx(uint8_t *buf, unsigned int i)
{
	(void)buf;
	if(Verbose)
		printf("# %06u: INDX\n",i);
	sector_none();
	return 0;
}

static inline unsigned int
fm_addr(uint8_t *buf, unsigned int i)
{
	unsigned int consumed = fm_valid_addr(&buf[i],&Last_track,&Last_side,&Last_sector,&Last_size);

	if( consumed ){
		if(Verbose)
			printf("# %06u: ADDR Track:%02u Side:%u Sector:%02u Size:%u\n",i,Last_track,Last_side,Last_sector,Last_size);
		}
	else
		sector_none();
	return consumed;
}

static inline unsigned int
fm_data(uint8_t *buf, unsigned int i)
{
	uint8_t	sector_data[MAX_SSIZE];
	unsigned int consumed = fm_valid_data(&buf[i],Last_size,sector_data);

	if( consumed ){
		if(Verbose)
			printf("# %06u: DATA ",i);
		disk_add(Last_track,Last_side,Last_sector,Last_size,sector_data);
		sector_none();
		}
	return consumed;
}

static inline unsigned int
fm_deld(uint8_t *buf, unsigned int i)
{
	uint8_t	sector_data[MAX_SSIZE];
	unsigned int consumed = fm_valid_deld(&buf[i],Last_size,sector_data);

	if( consumed ){
		if(Verbose)
			printf("# %06u: DELD ",i);
		disk_add(Last_track,Last_side,Last_sector,Last_size,sector_data);
		sector_none();
		}
	return consumed;
}

static inline void
fm_decode (const sample_t *samples, const unsigned int n, const sample_t split)
{
	unsigned int i;
	unsigned int consumed;
	uint8_t	*decode = (uint8_t *)malloc(n+DECODE_PAD);		// decoded stream is the same size as samples

	// Convert sample us to just 0/1
	for (i = 0; i < n; i++)
		decode[i] = (samples[i] < split) ? 1 : 0;
	for(i=0;i<DECODE_PAD;i++)
		decode[n+i] = 0;

	// Identify index/addr/data areas and extract
	for (i = 0; i < n; i++) {
		if( mark_match (&decode[i], FM_indx_mark, sizeof (FM_indx_mark))){
			(void)fm_indx(decode,i+sizeof(FM_indx_mark));	// just to print
			consumed = sizeof(FM_indx_mark);
			mark_used(&decode[i],consumed,'I');
			}
		else if( mark_match (&decode[i], FM_addr_mark, sizeof (FM_addr_mark))){
			consumed = fm_addr(decode,i+sizeof(FM_addr_mark));
			if( consumed ){
				consumed += sizeof(FM_addr_mark);
				mark_used(&decode[i],consumed,'A');
				}
			}
		else if( mark_match (&decode[i], FM_data_mark, sizeof (FM_data_mark))){
			consumed = fm_data(decode,i+sizeof(FM_data_mark));
			if( consumed ){
				consumed += sizeof(FM_data_mark);
				mark_used(&decode[i],consumed,'D');
				}
			}
		else if( mark_match (&decode[i], FM_deld_mark, sizeof (FM_deld_mark))){
			consumed = fm_deld(decode,i+sizeof(FM_deld_mark));
			if( consumed ){
				consumed += sizeof(FM_deld_mark);
				mark_used(&decode[i],consumed,'d');
				}
			}
		else
			consumed = 0;
		i += consumed;
	}
	//track_map(decode,n);	// DEBUG
	free(decode);
}

// Convert pairs of mfm bits to final values: 00->0, 01->1, 10->0, 11->invalid
static inline uint8_t
mfm_fetch_bit(uint8_t *buf)
{
	unsigned int pair = (buf[0]<<1)+(buf[1]<<0);

	switch(pair){
	case 0:
	case 2:
		return 0;
	case 1:
		return 1;
	}

	error("Invalid MFM bit");
	return 0;	// NOTREACHED
}

// fetch an MFM encoded byte, return value, update buf pointer
static inline uint8_t
mfm_fetch_byte (uint8_t ** buf)
{
	unsigned int i;
	unsigned int byte = 0;
	uint8_t *p = *buf;

	for (i = 0; i < 8; i++) {
		byte <<= 1;
		byte |= mfm_fetch_bit(p);
		p += 2;
	}
	*buf = p;
	return byte;
}

static inline uint8_t *
mfm_fetch_bytes (uint8_t * in, uint8_t *out, unsigned int count)
{
	unsigned int i;

	for(i=0;i<count;i++)
		out[i] = mfm_fetch_byte(&in);
	return in;
}

// Examine an address mark and see if it is valid.  Return number of consumed input bytes
// and fill in track,sector,side,ssize if true
static inline unsigned int
mfm_valid_addr (uint8_t *buf, unsigned int *track, unsigned int *side, unsigned int *sector, unsigned int *size)
{
	uint8_t *bufend;
	uint8_t addr[4+4+2];	// Address mark, Track, Side, Sector, Size, 2 CRC

	bufend = mfm_fetch_bytes (buf, addr, sizeof(addr));
	if ( crc16 (addr, sizeof(addr)) != 0)
		return 0;
	if( addr[4] >= NTRACKS )
		return 0;
	if( addr[5] >= NSIDES )
		return 0;
	if( addr[6] >= NSECTORS )
		return 0;
	if (addr[7] >= NSIZES)
		return 0;	// invalid ssize code

	*track  = addr[4];
	*side   = addr[5];
	*sector = addr[6];
	*size   = 128 << addr[7];
	return bufend-buf;
}

static inline unsigned int
mfm_valid_data (uint8_t *buf, unsigned int sector_size, uint8_t *sector_data)
{
	uint8_t *bufend;
	uint8_t data[4 + MAX_SSIZE + 2];	// Data mark, data bytes, 2 CRC

	if(sector_size>MAX_SSIZE)
		return 0;
	bufend = mfm_fetch_bytes (buf, data, 4 + sector_size + 2);
	if (crc16 (data, 4 + sector_size + 2) != 0)
		return 0;
	memcpy(sector_data,&data[4],sector_size);
	return bufend-buf;
}

static inline unsigned int
mfm_valid_deld (uint8_t *buf, unsigned int sector_size, uint8_t *sector_data)
{
	uint8_t *bufend;
	uint8_t data[4 + MAX_SSIZE + 2];	// Data mark, data bytes, 2 CRC

	if(sector_size>MAX_SSIZE)
		return 0;
	bufend = mfm_fetch_bytes (buf, data, 4 + sector_size + 2);
	if (crc16 (data, 4 + sector_size + 2) != 0)
		return 0;
	memcpy(sector_data,&data[4],sector_size);
	return bufend-buf;
}

static inline unsigned int
mfm_indx(uint8_t *buf, unsigned int i)
{
	(void)buf;
	if(Verbose)
		printf("# %06u: INDX\n",i);
	sector_none();
	return 0;
}

static inline unsigned int
mfm_addr(uint8_t *buf, unsigned int i)
{
	unsigned int consumed = mfm_valid_addr(&buf[i],&Last_track,&Last_side,&Last_sector,&Last_size);

	if( consumed ){
		if(Verbose)
			printf("# %06u: ADDR Track:%02u Side:%u Sector:%02u Size:%u\n",i,Last_track,Last_side,Last_sector,Last_size);
		}
	else
		sector_none();
	return consumed;
}

static inline unsigned int
mfm_data(uint8_t *buf, unsigned int i)
{
	uint8_t	sector_data[MAX_SSIZE];
	unsigned int consumed = mfm_valid_data(&buf[i],Last_size,sector_data);

	if( consumed ){
		if(Verbose)
			printf("# %06u: DATA ",i);
		disk_add(Last_track,Last_side,Last_sector,Last_size,sector_data);
		sector_none();
		}
	return consumed;
}

static inline unsigned int
mfm_deld(uint8_t *buf, unsigned int i)
{
	uint8_t	sector_data[MAX_SSIZE];
	unsigned int consumed = mfm_valid_data(&buf[i],Last_size,sector_data);

	if( consumed ){
		if(Verbose)
			printf("# %06u: DELD ",i);
		disk_add(Last_track,Last_side,Last_sector,Last_size,sector_data);
		sector_none();
		}
	return consumed;
}

static inline void
mfm_decode (const sample_t *samples, const unsigned int n, const sample_t split_lo, const sample_t split_hi)
{
	unsigned int i;
	unsigned int consumed;
	sample_t s;
	uint8_t *dptr;
	unsigned int nact;		// actual number of decoded bits, typically about 2.5x of the input
	uint8_t mark[4];		// for finding index/addr/data marks
	uint8_t *decode = (uint8_t *)malloc((n*4)+DECODE_PAD);	// worse case, decoded bits are 4x the number of samples

	// convert to RLL format
	dptr = decode;
	for(i=0;i<n;i++){
		s = samples[i];
		*dptr++ = 1;
		if( s>=split_hi ){		// 4us
			*dptr++ = 0;
			*dptr++ = 0;
			*dptr++ = 0;
			}
		else if( s>=split_lo ){		// 3us
			*dptr++ = 0;
			*dptr++ = 0;
			}
		else{				// 2us
			*dptr++ = 0;
			}
		}
	nact = dptr-decode;	// how much expanded data
	for(i=0;i<DECODE_PAD;i++)
		*dptr++ = 0;
	if(Verbose)
		printf("# MFM decode expanded to %u samples\n",nact);

	// Identify index/addr/data areas and extract
	for (i = 0; i < nact; i+=consumed) {
		mfm_fetch_bytes(&decode[i],mark,sizeof(mark));
		if( mark_match (mark, MFM_indx_mark, sizeof(mark))){
			(void)mfm_indx(decode,i);
			consumed = sizeof(mark)*8*2;	// each byte consumes 8 pairs of bits
			mark_used(&decode[i],consumed,'I');
			}
		else if( mark_match (mark, MFM_addr_mark, sizeof(mark))){
			consumed = mfm_addr(decode,i);
			if( consumed )
				mark_used(&decode[i],consumed,'A');
			else
				consumed=1;
			}
		else if( mark_match (mark, MFM_data_mark, sizeof(mark))){
			consumed = mfm_data(decode,i);
			if(consumed)
				mark_used(&decode[i],consumed,'D');
			else
				consumed=1;
			}
		else if( mark_match (mark, MFM_deld_mark, sizeof(mark))){
			consumed = mfm_deld(decode,i);
			if(consumed)
				mark_used(&decode[i],consumed,'d');
			else
				consumed=1;
			}
		else
			consumed = 1;	// advance the search
	}
	//track_map(decode,nact);	// DEBUG
	free(decode);
}

// load track data from file into sample array, return actual number of samples
static inline unsigned int
track_load(const char *s, sample_t *samples, const unsigned int n)
{
	FILE *fp = fopen(s,"r");
	unsigned int i;
	unsigned int v;

	if(fp==NULL)
		return 0;
	for(i=0;i<n;i++){
		if( fscanf(fp,"%u",&v)!=1 )
			break;
		samples[i] = v;
		}
	fclose(fp);
	return i;
}

static inline void
process(char *s)
{
	unsigned int	n;		// number of samples loaded
	sample_t	*samples = (sample_t *)malloc(sizeof(sample_t)*MAX_SAMPLES);

	if(Verbose)
		printf("# Load %s, ",s);
	n = track_load(s,samples,MAX_SAMPLES);
	if(Verbose)
		printf("%u samples\n",n);
	if( n==0 ){
		free(samples);
		return;
		}
	sector_none();
	switch (determine_format(samples,n)) {
	case TT_FM:
		fm_decode (samples,n,FM_SPLIT);
		break;
	case TT_MFM:
		mfm_decode (samples,n,MFM_SPLIT_LO,MFM_SPLIT_HI);
		break;
	default:
		error("Cannot determine track format");
		break;
	}
	free(samples);
}

int
main(int argc, char **argv)
{
	char *arg;

	setbuf(stdout,NULL);
	while(--argc){
		arg = *++argv;
		if( strcmp(arg,"-v")==0 )
			Verbose=true;
		else if( strcmp(arg,"-j")==0 )
			Json_show=true;
		else
			process(arg);
		}
	disk_show();
	return 0;
}
