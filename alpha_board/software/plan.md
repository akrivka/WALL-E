# General notes
	gonna use Timer 2


# Code outline

InitServo():
	set up pins
		pin 23
			PORT_ID = 1Bh (PORT_EVENT4)
		pin 24 (AUXIO 25!!)
			PORT_ID = 8h (AUX_IO)


	set up timer
		CFG
			16-bit
		TAMR
			TAMR = periodic
			TAAMS = PWM
			TAPLO = legacy
			TACINTD = time-out interrupt disabled
		CTL
			enable
		interval: 
			48 Mhz / (256 * (1/20)) = 3,750,000 (should fit in 24-bits)
		match register 
			gonna be set by SetServo

	set up ADC
		AUX_ADI4 (0x400C B000)
			MUX3
				one-hot 0100 0000 or maybe 0000 0010
			ADC0
				EN = 1h
				RESET_N = 1h
				SMPL_CYCLE_EXP = 3h (2.7us)
		AUX_AIODIO3 (0x400C F000)
			IOMODE
				IO1 = 1h (input mode)
		AUX_ANAIF (0x400C 9000)
			ADCCTL
				CMD = 1h (enable)
				START_SRC = 3Fh (NO_EVENT)
		AUX_SYSIF (0x400C 6000)
			ADCCLKCTL REQ = 1 (enabled)
			(wait for ADCCLKCTL ACK = 1?)


SetServo(pos):
	test if pos is withing -90, 90 (if not return FAIL)
	change PWM pulse by writing to the match register of the timer
	return SUCCESS


ReleaseServo():
	disable/stop timer
	return void


GetServo():
	flush FIFO
		AUX_ANAIF ADCCTL 3h
	trigger ADC conversion
		AUX_ANAIF ADCTRIG 1h
	wait for it to finish
		AUX_ANAIF ADCFIFOSTAT EMPTY 0h?
	read from from 
		AUX_ANAIF ADCFIFO
	convert it to position
	return it


TestServo():
	test table
	set up interrupt to display servo position
	loop
		set position
		release servo
		wait (in debug)

Main loop():
	init power and clocks
	init LCD
	init Servo
	TestServo()

# Implementation plan

1. SetServo and basic test loop
2. ReleaseServo (should be quick)
3. ADC init (connect to power supply? or set up a simple pot circuit)
4. GetServo
5. LCD