floppy8 - extract sector data from ancient 8" floppy disks

The arduino sub-directory contains code for a Teensy 4.1 SBC which
is directly connected to 8 control/data pins of an 8" floppy disk.
This program does a track-by-track extraction of the raw data pulses
from the disk and stores them as timestamp deltas.

The extract program then reads the timestamp delta data for each track and reconstructs
the sector data (FM or MFM formatted) and prints it in 'hexdump' format.

A sample capture disk is stored in data_dir for testing the extract program.
