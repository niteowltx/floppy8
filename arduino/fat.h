#include <SD.h>
#include <SPI.h>
#include <stdio.h>

Sd2Card Card;
SdVolume Volume;
const int ChipSelect = BUILTIN_SDCARD;

#define	MAXDEPTH	20
#define	MAXDIRNAME	32
static char Fat_path[MAXDEPTH][MAXDIRNAME];

// show all files in directory, recursively
static void
fat_directory (File dir, unsigned int level)
{
	File entry;
	DateTimeFields tm;
	unsigned int m;
	unsigned int i;
	static const char *months[] = { "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec", "***" };

	while (true) {
		entry = dir.openNextFile ();
		if (!entry)
			break;
		if (entry.isDirectory ()) {
			if (level < MAXDEPTH)
				strncpy (Fat_path[level], entry.name (), MAXDIRNAME - 1);
			fat_directory (entry, level + 1);
		}
		else {
			if (!entry.getModifyTime (tm)) {
				tm.year = tm.mon = tm.mday = tm.hour = tm.min = 0;
			}
			m = (tm.mon < 12) ? tm.mon : 12;
			Serial.printf ("%4d-%s-%02d %02d:%02d %10llu ", tm.year + 1900, months[m], tm.mday, tm.hour, tm.min, entry.size ());
			for (i = 0; i < level && i < MAXDEPTH; i++)
				Serial.printf ("%s/", Fat_path[i]);
			if (i < level)
				Serial.printf (".../");
			Serial.printf ("%s\r\n", entry.name ());
		}
		entry.close ();
	}
}

static inline void
fat_ls ()
{
	File root = SD.open ("/");

	fat_directory (root, 0);
	root.close ();
}

static inline void
fat_init ()
{
	uint32_t volumesize;	// overflows at 2TB drive size

	Serial.print ("SD card:");

	if (!Card.init (SPI_HALF_SPEED, ChipSelect)) {
		Serial.printf (" CardInit FAILED\r\n");
		return;
	}

	switch (Card.type ()) {
	case SD_CARD_TYPE_SD1:
		Serial.printf (" SD1");
		break;
	case SD_CARD_TYPE_SD2:
		Serial.printf (" SD2");
		break;
	case SD_CARD_TYPE_SDHC:
		Serial.printf (" SDHC");
		break;
	default:
		Serial.printf (" ?SD%d?", Card.type ());
		break;
	}

	if (!Volume.init (Card)) {
		Serial.printf (" VolumeInit FAILED\r\n");
		return;
	}

	Serial.printf (" FAT%d", Volume.fatType ());

	volumesize = Volume.blocksPerCluster ();	// clusters are collections of blocks
	volumesize *= Volume.clusterCount ();	// we'll have a lot of clusters
	volumesize /= 2 * 1024;	// convert block count to MB (block size is always 512 bytes)
	Serial.printf (" %u Mbytes\r\n", volumesize);

	//fat_ls ();		// optional
}
