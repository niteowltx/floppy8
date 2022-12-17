#include "cyclecount.h"
#include "sa800.h"
#include "fat.h"

//
//	floppy8 --- read contents of ancient 8" floppy from Shugart SA-800 disk drive
//
//	Captures all data bits coming from the drive and
//	write the deltas between each bit in one file per track
//
//	Uses a Teensy 4.1 and uses its SD card to save captures

#define	ONE_US		600				// Teensy 4.1 clock is 600Mhz
#define	CAPTURE_TIMEOUT	(3*SA800_ONE_REV*ONE_US)	// max time to capture pulses (in cpu cycle counts)
#define	INDEX_TIMEOUT	(4*SA800_ONE_REV*ONE_US)	// max time to wait for index pulse

#define	DISK_FMT	"Disk%03lu"	// directory name format
#define	TRACK_FMT	"Track%02lu.raw"	// track name format
#define	USER_DELAY	1000		// waiting for user to do something (in ms)

#define SAMPLE_SIZE	200000		// theoretically there can be no more than SA800_ONE_REV/2 pulses per track (roughly 84000)
#define	SAMPLE_SHIFT	0		// if desired, shift raw capture delta right by this much if low bits are not very important
#define	SAMPLE_MAX	0xFFFF		// differences are clamped to this so it fits within a sample_t
typedef uint16_t sample_t;		// a sample is the CPU cyclecounter difference between 2 falling edges of data, shifted by SAMPLE_SHIFT, capped at SAMPLE_MAX
sample_t Samples[SAMPLE_SIZE];		// sample buffer (cannot be PSRAM, unfortunately)

uint32_t Disk = 0;	// counts the number of disks scanned
uint32_t Last_capture;	// how long (in cycles) did the last capture take?
uint32_t One_us = 0;

// establish how many cpu cycles are in one us
void
one_us_init()
{
	uint32_t start = cycle_count();

	delay(1000);	// might vary a little since interrupts are enabled
	One_us = cycle_count()-start;
	if( One_us == 0 )
		Serial.printf("Cycle count not working?\r\n");
	else{
		One_us /= 1000000;
		Serial.printf("One us = %u cycles",One_us);
		}
}

void
setup ()
{
	Serial.begin (115200);
	while (!Serial) {
		;			// wait for serial port to connect.
	}
	cycle_init ();
	one_us_init();
	sa800_init ();
	fat_init ();
}

// write all collected sample to a file, one sample per line
boolean
save_data (uint32_t disk, uint32_t track, sample_t * buf, uint32_t count)
{
	uint32_t i;
	File fp;
	char path[128];
	char item[32];

	sprintf (path, "/" DISK_FMT "/" TRACK_FMT, disk, track);
	fp = SD.open (path, FILE_WRITE_BEGIN);
	if (!fp)
		return false;

	for (i = 0; i < count; i++) {
		sprintf (item, "%u\n", buf[i]);
		fp.write (item, strlen (item));	// TODO: check that write worked
	}
	fp.close ();
	return true;
}

// find next unused directory name on SD card and create it
uint32_t
next_disk_slot (uint32_t mindisk)
{
	uint32_t d;
	char path[128];

	for (d = mindisk; d < 1000000; d++) {
		sprintf (path, "/" DISK_FMT, d);
		if (SD.exists (path) )	// try the next one
			continue;
		if (SD.mkdir (path))
			return d;
		break;	// serious problem
	}
	// SD write protected, full or corrupted?
	Serial.printf ("Failed to create directory %s???\r\n", path);
	while (true)
		delay (USER_DELAY);
	return d;		// NOTREACHED
}

// Wait for pin to transition hi-to-lo
// Return false if more than 'timeout' cpu cycles pass before the edge is seen
inline boolean
wait_for_edge (const int pin, const uint32_t start, const uint32_t timeout)
{
	while (digitalReadFast (pin) == LOW) {
		if (cycle_since (start) >= timeout)
			return false;
	}
	while (digitalReadFast (pin) == HIGH) {
		if (cycle_since (start) >= timeout)
			return false;
	}
	return true;
}

// convert the difference between 2 cpu cycle counts to a sample_t
inline sample_t
sample_cvt(const uint32_t curr, const uint32_t prev)
{
	uint32_t	delta = curr-prev;

	delta >>= SAMPLE_SHIFT;	// low bits are not so useful?
	if (delta > SAMPLE_MAX)
		delta = SAMPLE_MAX;
	return (sample_t)delta;
}

// Wait for INDEX pulse, then capture cpu cycle difference between falling READ_DATA edges
// Stop when the buffer fills up or the timeout (in cpu cycles) is reached
// Disable interrupts to prevent clock ticks and/or other activity from
// disturbing the capture so that the timing is as accurate as possible.
// Cannot use delay() since interrupts are disabled
// Return the number of samples collected
uint32_t
capture (volatile sample_t * buf, const uint32_t count)
{
	volatile sample_t *s = buf;
	volatile sample_t *send = buf + count;
	volatile uint32_t start, curr, prev;

	noInterrupts ();
	if (!wait_for_edge (INDEX, cycle_count(), INDEX_TIMEOUT)){
		interrupts();
		Last_capture = 0;
		return 0;
		}
	start = prev = cycle_count ();	// remember start of capture, establish 'prev'
	for (; s < send; s++) {
		if (!wait_for_edge (READ_DATA, start, CAPTURE_TIMEOUT))
			break;	// timed out
		curr = cycle_count ();
		*s = sample_cvt(curr,prev);
		prev = curr;
	}
	Last_capture = cycle_count()-start;
	interrupts ();
	return s - buf;
}

// return number of cpu cycles for one disk revolution, or zero if not spinning
uint32_t
spinning()
{
	volatile uint32_t start;

	// not masking interrupts, so there may be some variation in the reading
	if (!wait_for_edge (INDEX, cycle_count(), INDEX_TIMEOUT))
		return 0;

	start = cycle_count();
	if (!wait_for_edge (INDEX, start, INDEX_TIMEOUT))
		return 0;
	return cycle_count()-start;
}

void
loop ()
{
	uint32_t track;
	uint32_t actual;
	uint32_t one_rev;

	sa800_drive_select ();
	if (!sa800_drive_ready ()) {
		sa800_head_unload ();
		Serial.printf ("\rInsert Disk...");
		delay (USER_DELAY);
		return;
	}

	// drive is selected and reports ready, check if its producing index pulses
	one_rev = spinning();
	if( one_rev == 0 ){
		Serial.printf(" Not spinning?\r\n");
		delay (USER_DELAY);
		return;
		}
	Serial.printf("Ready. Spinning at %u us/rev\r\n",one_rev/One_us);	// expecting roughly SA800_ONE_REV

	Disk = next_disk_slot (Disk);	// find and create next directory to save captures
	Serial.printf ("Capture to " DISK_FMT "\r\n", Disk);

	sa800_seek_track00 ();
	sa800_head_load ();
	for (track = 0; track < SA800_NTRACKS; track++) {
		Serial.printf ("    " TRACK_FMT " Capture...", track);
		actual = capture (Samples, SAMPLE_SIZE);
		Serial.printf(" Took %u cycles (%u/us)... ",Last_capture,Last_capture/One_us);
		Serial.printf ("Save %u samples...", actual);
		if (save_data (Disk, track, Samples, actual))
			Serial.printf ("OK\r\n");
		else
			Serial.printf ("FAILED\r\n");
		sa800_step_in ();
	}
	sa800_head_unload ();
	sa800_seek_track00 ();

	// wait for user to remove disk
	while (sa800_drive_ready ()) {
		Serial.printf ("\rRemove Disk...");
		delay (USER_DELAY);
	}
}
