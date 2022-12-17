
// SA-800 

// There is a 50-pin connector for all Control and Status lines
#define HEAD_LOAD	21	// 18 - Head Load       (Control)
#define STEP_DIR	20	// 34 - Step Direction  (Control)
#define STEP		19	// 36 - Step            (Control)
#define DRIVE_SELECT	18	// 26 - Drive Select    (Control)
#define READY		17	// 22 - Ready           (Status)
#define READ_DATA	16	// 46 - Read Data       (Status)
#define TRACK_00	15	// 42 - Track 00        (Status)
#define INDEX		14	// 20 - Index           (Status)

// SA800 parameters
#define SA800_RPM	360	// disk rotatation speed in revolutions/minute
#define	SA800_ONE_REV	166667	// one revolution in us
#define SA800_NTRACKS	77	// total tracks, numbered 0-76

#define STEP_IN		LOW	// direction to step in
#define STEP_OUT	HIGH	// direction to step out
#define EXTRA		3	// wiggle stepper a bit when finding track 0

// various delays (in ms)
#define DRIVE_SELECT_DELAY	100
#define HEAD_LOAD_DELAY	700	// spec doesn't say how long this should be?
#define STEP_DELAY	25	// spec says 8ms min
#define STEP_PULSE	1	// spec says 1us
#define STEP_SETTLE	1	// spec says 12us from step to track00 valid

// Set Control pin to level, with optional delay
// All inputs to the SA-800 have +5V pullups so to set HIGH, set pinMode() as an input with no pullup
// to program pin LOW, set pinMode() to output mode and set to zero
static inline void
set_pin_delay (const int pin, const int level, const int d)
{
	if(level==HIGH)
		pinMode (pin, INPUT);
	else{
		digitalWrite(pin,0);
		pinMode (pin, OUTPUT);
		}
	if (d)
		delay (d);
}

static inline void
sa800_drive_select ()
{
	set_pin_delay (DRIVE_SELECT, LOW, DRIVE_SELECT_DELAY);
}

static inline void
sa800_drive_unselect ()
{
	set_pin_delay (DRIVE_SELECT, HIGH, DRIVE_SELECT_DELAY);
}

static inline void
sa800_head_load ()
{
	set_pin_delay (HEAD_LOAD, LOW, HEAD_LOAD_DELAY);
}

static inline void
sa800_head_unload ()
{
	set_pin_delay (HEAD_LOAD, HIGH, HEAD_LOAD_DELAY);
}

static inline void
sa800_step (const int dir)
{
	set_pin_delay (STEP_DIR, dir, 0);
	set_pin_delay (STEP, LOW, STEP_PULSE);
	set_pin_delay (STEP, HIGH, STEP_DELAY);
}

static inline void
sa800_step_out ()
{
	sa800_step (STEP_OUT);
}

static inline void
sa800_step_in ()
{
	sa800_step (STEP_IN);
}

// read one of the inputs.  All inputs are active low
static inline boolean
sa800_track00 ()
{
	return digitalReadFast (TRACK_00) == LOW;
}

static inline boolean
sa800_drive_ready ()
{
	return digitalReadFast (READY) == LOW;
}

static inline boolean
sa800_index ()
{
	return digitalReadFast (INDEX) == LOW;
}

static inline boolean
sa800_read_data ()
{
	return digitalReadFast (READ_DATA) == LOW;
}

// move to the outermost track
static inline boolean
sa800_seek_track00 ()
{
	uint32_t i;

	// cannot seek if drive is not ready
	if( !sa800_drive_ready() )
		return false;

	// Step out until the TRACK_00 signal is asserted.
	for (i = 0; i < SA800_NTRACKS + EXTRA; i++) {
		if (sa800_track00 ())
			break;
		sa800_step_out ();
	}
	if (!sa800_track00 ())
		return false;

	// Step in EXTRA tracks, make sure TRACK_00 is NOT asserted,
	for (i = 0; i < EXTRA; i++)
		sa800_step_in ();
	if (sa800_track00 ())
		return false;

	// step back out and verify TRACK_00 is again asserted.
	for (i = 0; i < EXTRA; i++) {
		if (sa800_track00 ())
			break;
		sa800_step_out ();
	}
	return sa800_track00();
}

static inline void
sa800_status(const char *tag)
{
	Serial.printf("\r\nDriveSelect:%d HeadLoad:%d StepDir:%d Step:%d | Index:%d Ready:%d Track00:%d ReadData:%d  %s",
		digitalReadFast(DRIVE_SELECT),
		digitalReadFast(HEAD_LOAD),
		digitalReadFast(STEP_DIR),
		digitalReadFast(STEP),
		sa800_index(),
		sa800_drive_ready(),
		sa800_track00(),
		sa800_read_data(),
		tag);
}

static inline void
sa800_debug()
{
        if( Serial.available() == 0 )
                return;
        switch( Serial.read() & 0x7F ){
	case '0':	Serial.printf("Track00");	sa800_seek_track00();	break;
        case '+':       Serial.printf("StepIn");	sa800_step_in();	break;
        case '-':       Serial.printf("StepOut");	sa800_step_out();	break;
	case 's':	Serial.printf("Unselect");	sa800_drive_unselect();	break;
	case 'S':	Serial.printf("Select");	sa800_drive_select();	break;
	case 'h':	Serial.printf("Unload");	sa800_head_unload();	break;
	case 'H':	Serial.printf("Load");		sa800_head_load();	break;
	case '\r':
	case '\n':
	case ' ':
		break;
        default:
                Serial.printf("Expected 0 + - s S h H");
                break;
        }
	sa800_status("");
}

static inline void
sa800_init ()
{
	set_pin_delay(DRIVE_SELECT,HIGH,0);
	set_pin_delay(HEAD_LOAD,HIGH,0);
	set_pin_delay(STEP_DIR,HIGH,0);
	set_pin_delay(STEP,HIGH,0);
	pinMode (INDEX, INPUT_PULLUP);
	pinMode (READY, INPUT_PULLUP);
	pinMode (TRACK_00, INPUT_PULLUP);
	pinMode (READ_DATA, INPUT_PULLUP);
}
