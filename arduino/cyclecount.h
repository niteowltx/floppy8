
// Core Debug registers
#define DEMCR		(*((volatile uint32_t *)0xE000EDFC))
#define DEMCR_TRCENA	0x01000000
#define DWT_CTRL	(*(volatile uint32_t *)0xE0001000)
#define CYCCNTENA	(1u<<0)
#define DWT_CYCCNT	((volatile uint32_t *)0xE0001004)
#define CPU_CYCLES	(*DWT_CYCCNT)

// get current 32-bit cpu cycle counter
static inline volatile uint32_t
cycle_count ()
{
	return CPU_CYCLES;
}

// busy wait for some cycles to elapse
static inline void
cycle_delay (const uint32_t d)
{
	volatile uint32_t start = cycle_count ();

	while ((cycle_count () - start) < d) {
	}
}

// how many cycles have elapsed since some previous reading
static inline uint32_t
cycle_since (const uint32_t prev)
{
	return cycle_count () - prev;
}

// turn on the cpu cycle counter
static inline void
cycle_init (void)
{
	DEMCR |= DEMCR_TRCENA;	// Enable DWT
	*DWT_CYCCNT = 0;
	DWT_CTRL |= CYCCNTENA;	// Enable CPU cycle counter
}
