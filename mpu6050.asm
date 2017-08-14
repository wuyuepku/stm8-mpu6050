;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 3.4.0 #8981 (Jul 11 2014) (Linux)
; This file was generated Mon Aug 14 14:32:24 2017
;--------------------------------------------------------
	.module mpu6050
	.optsdcc -mstm8
	
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _segmentMap
	.globl _main
	.globl _getBMP280Humidity
	.globl _getBMP280Temperature
	.globl _initBMP280
	.globl _readBMP280
	.globl _tm1637DisplayDecimal
	.globl _tm1637Init
	.globl _InitializeUART
	.globl _InitializeI2C
	.globl _i2c_read_register
	.globl _print_byte_hex
	.globl _i2c_set_start_ack
	.globl _i2c_send_address
	.globl _UARTPrintF
	.globl _i2c_send_reg
	.globl _i2c_set_stop
	.globl _i2c_set_nak
	.globl _i2c_read
	.globl _delay
	.globl _InitializeSystemClock
	.globl _delayTenMicro
	.globl _tm1637SetBrightness
	.globl __tm1637Start
	.globl __tm1637Stop
	.globl __tm1637ReadResult
	.globl __tm1637WriteByte
	.globl __tm1637ClkHigh
	.globl __tm1637ClkLow
	.globl __tm1637DioHigh
	.globl __tm1637DioLow
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area DATA
_dig_T1:
	.ds 4
_dig_T2:
	.ds 4
_dig_T3:
	.ds 4
_dig_H1:
	.ds 4
_dig_H2:
	.ds 4
_dig_H3:
	.ds 4
_dig_H4:
	.ds 4
_dig_H5:
	.ds 4
_dig_H6:
	.ds 4
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area INITIALIZED
_BMP280_ADDR:
	.ds 2
;--------------------------------------------------------
; Stack segment in internal ram 
;--------------------------------------------------------
	.area	SSEG
__start__stack:
	.ds	1

;--------------------------------------------------------
; absolute external ram data
;--------------------------------------------------------
	.area DABS (ABS)
;--------------------------------------------------------
; interrupt vector 
;--------------------------------------------------------
	.area HOME
__interrupt_vect:
	int s_GSINIT ;reset
	int 0x0000 ;trap
	int 0x0000 ;int0
	int 0x0000 ;int1
	int 0x0000 ;int2
	int 0x0000 ;int3
	int 0x0000 ;int4
	int 0x0000 ;int5
	int 0x0000 ;int6
	int 0x0000 ;int7
	int 0x0000 ;int8
	int 0x0000 ;int9
	int 0x0000 ;int10
	int 0x0000 ;int11
	int 0x0000 ;int12
	int 0x0000 ;int13
	int 0x0000 ;int14
	int 0x0000 ;int15
	int 0x0000 ;int16
	int 0x0000 ;int17
	int 0x0000 ;int18
	int 0x0000 ;int19
	int 0x0000 ;int20
	int 0x0000 ;int21
	int 0x0000 ;int22
	int 0x0000 ;int23
	int 0x0000 ;int24
	int 0x0000 ;int25
	int 0x0000 ;int26
	int 0x0000 ;int27
	int 0x0000 ;int28
	int 0x0000 ;int29
;--------------------------------------------------------
; global & static initialisations
;--------------------------------------------------------
	.area HOME
	.area GSINIT
	.area GSFINAL
	.area GSINIT
__sdcc_gs_init_startup:
__sdcc_init_data:
; stm8_genXINIT() start
	ldw x, #l_DATA
	jreq	00002$
00001$:
	clr (s_DATA - 1, x)
	decw x
	jrne	00001$
00002$:
	ldw	x, #l_INITIALIZER
	jreq	00004$
00003$:
	ld	a, (s_INITIALIZER - 1, x)
	ld	(s_INITIALIZED - 1, x), a
	decw	x
	jrne	00003$
00004$:
; stm8_genXINIT() end
	.area GSFINAL
	jp	__sdcc_program_startup
;--------------------------------------------------------
; Home
;--------------------------------------------------------
	.area HOME
	.area HOME
__sdcc_program_startup:
	jp	_main
;	return from main will return to caller
;--------------------------------------------------------
; code
;--------------------------------------------------------
	.area CODE
;	mpu6050.c: 18: void delayTenMicro (void) {
;	-----------------------------------------
;	 function delayTenMicro
;	-----------------------------------------
_delayTenMicro:
;	mpu6050.c: 20: for (a = 0; a < 50; ++a)
	ld	a, #0x32
00104$:
;	mpu6050.c: 21: __asm__("nop");
	nop
	dec	a
;	mpu6050.c: 20: for (a = 0; a < 50; ++a)
	tnz	a
	jrne	00104$
	ret
;	mpu6050.c: 24: void InitializeSystemClock() {
;	-----------------------------------------
;	 function InitializeSystemClock
;	-----------------------------------------
_InitializeSystemClock:
;	mpu6050.c: 25: CLK_ICKR = 0;                       //  Reset the Internal Clock Register.
	ldw	x, #0x50c0
	clr	(x)
;	mpu6050.c: 26: CLK_ICKR = CLK_HSIEN;               //  Enable the HSI.
	ldw	x, #0x50c0
	ld	a, #0x01
	ld	(x), a
;	mpu6050.c: 27: CLK_ECKR = 0;                       //  Disable the external clock.
	ldw	x, #0x50c1
	clr	(x)
;	mpu6050.c: 28: while ((CLK_ICKR & CLK_HSIRDY) == 0);       //  Wait for the HSI to be ready for use.
00101$:
	ldw	x, #0x50c0
	ld	a, (x)
	bcp	a, #0x02
	jreq	00101$
;	mpu6050.c: 29: CLK_CKDIVR = 0;                     //  Ensure the clocks are running at full speed.
	ldw	x, #0x50c6
	clr	(x)
;	mpu6050.c: 30: CLK_PCKENR1 = 0xff;                 //  Enable all peripheral clocks.
	ldw	x, #0x50c7
	ld	a, #0xff
	ld	(x), a
;	mpu6050.c: 31: CLK_PCKENR2 = 0xff;                 //  Ditto.
	ldw	x, #0x50ca
	ld	a, #0xff
	ld	(x), a
;	mpu6050.c: 32: CLK_CCOR = 0;                       //  Turn off CCO.
	ldw	x, #0x50c9
	clr	(x)
;	mpu6050.c: 33: CLK_HSITRIMR = 0;                   //  Turn off any HSIU trimming.
	ldw	x, #0x50cc
	clr	(x)
;	mpu6050.c: 34: CLK_SWIMCCR = 0;                    //  Set SWIM to run at clock / 2.
	ldw	x, #0x50cd
	clr	(x)
;	mpu6050.c: 35: CLK_SWR = 0xe1;                     //  Use HSI as the clock source.
	ldw	x, #0x50c4
	ld	a, #0xe1
	ld	(x), a
;	mpu6050.c: 36: CLK_SWCR = 0;                       //  Reset the clock switch control register.
	ldw	x, #0x50c5
	clr	(x)
;	mpu6050.c: 37: CLK_SWCR = CLK_SWEN;                //  Enable switching.
	ldw	x, #0x50c5
	ld	a, #0x02
	ld	(x), a
;	mpu6050.c: 38: while ((CLK_SWCR & CLK_SWBSY) != 0);        //  Pause while the clock switch is busy.
00104$:
	ldw	x, #0x50c5
	ld	a, (x)
	srl	a
	jrc	00104$
	ret
;	mpu6050.c: 40: void delay (int time_ms) {
;	-----------------------------------------
;	 function delay
;	-----------------------------------------
_delay:
	sub	sp, #10
;	mpu6050.c: 42: for (x = 0; x < 1036*time_ms; ++x)
	clrw	x
	ldw	(0x03, sp), x
	ldw	(0x01, sp), x
	ldw	x, (0x0d, sp)
	pushw	x
	push	#0x0c
	push	#0x04
	call	__mulint
	addw	sp, #4
	ldw	(0x09, sp), x
00103$:
	ldw	y, (0x09, sp)
	ldw	(0x07, sp), y
	ld	a, (0x07, sp)
	rlc	a
	clr	a
	sbc	a, #0x00
	ld	(0x06, sp), a
	ld	(0x05, sp), a
	ldw	x, (0x03, sp)
	cpw	x, (0x07, sp)
	ld	a, (0x02, sp)
	sbc	a, (0x06, sp)
	ld	a, (0x01, sp)
	sbc	a, (0x05, sp)
	jrsge	00105$
;	mpu6050.c: 43: __asm__("nop");
	nop
;	mpu6050.c: 42: for (x = 0; x < 1036*time_ms; ++x)
	ldw	y, (0x03, sp)
	addw	y, #0x0001
	ld	a, (0x02, sp)
	adc	a, #0x00
	ld	xl, a
	ld	a, (0x01, sp)
	adc	a, #0x00
	ld	xh, a
	ldw	(0x03, sp), y
	ldw	(0x01, sp), x
	jra	00103$
00105$:
	addw	sp, #10
	ret
;	mpu6050.c: 45: void i2c_read (unsigned char *x) {
;	-----------------------------------------
;	 function i2c_read
;	-----------------------------------------
_i2c_read:
;	mpu6050.c: 46: while ((I2C_SR1 & I2C_RXNE) == 0);
00101$:
	ldw	x, #0x5217
	ld	a, (x)
	bcp	a, #0x40
	jreq	00101$
;	mpu6050.c: 47: *x = I2C_DR;
	ldw	y, (0x03, sp)
	ldw	x, #0x5216
	ld	a, (x)
	ld	(y), a
	ret
;	mpu6050.c: 49: void i2c_set_nak (void) {
;	-----------------------------------------
;	 function i2c_set_nak
;	-----------------------------------------
_i2c_set_nak:
;	mpu6050.c: 50: I2C_CR2 &= ~I2C_ACK;
	ldw	x, #0x5211
	ld	a, (x)
	and	a, #0xfb
	ld	(x), a
	ret
;	mpu6050.c: 52: void i2c_set_stop (void) {
;	-----------------------------------------
;	 function i2c_set_stop
;	-----------------------------------------
_i2c_set_stop:
;	mpu6050.c: 53: I2C_CR2 |= I2C_STOP;
	ldw	x, #0x5211
	ld	a, (x)
	or	a, #0x02
	ld	(x), a
	ret
;	mpu6050.c: 55: void i2c_send_reg (UCHAR addr) {
;	-----------------------------------------
;	 function i2c_send_reg
;	-----------------------------------------
_i2c_send_reg:
	sub	sp, #2
;	mpu6050.c: 57: reg = I2C_SR1;
	ldw	x, #0x5217
	ld	a, (x)
	clrw	x
	ld	xl, a
	ldw	(0x01, sp), x
;	mpu6050.c: 58: reg = I2C_SR3;
	ldw	x, #0x5219
	ld	a, (x)
	clrw	x
	ld	xl, a
	ldw	(0x01, sp), x
;	mpu6050.c: 59: I2C_DR = addr;
	ldw	x, #0x5216
	ld	a, (0x05, sp)
	ld	(x), a
;	mpu6050.c: 60: while ((I2C_SR1 & I2C_TXE) == 0);
00101$:
	ldw	x, #0x5217
	ld	a, (x)
	sll	a
	jrnc	00101$
	addw	sp, #2
	ret
;	mpu6050.c: 64: void UARTPrintF (char *message) {
;	-----------------------------------------
;	 function UARTPrintF
;	-----------------------------------------
_UARTPrintF:
;	mpu6050.c: 65: char *ch = message;
	ldw	y, (0x03, sp)
;	mpu6050.c: 66: while (*ch) {
00104$:
	ld	a, (y)
	tnz	a
	jreq	00107$
;	mpu6050.c: 67: UART1_DR = (unsigned char) *ch;     //  Put the next character into the data transmission register.
	ldw	x, #0x5231
	ld	(x), a
;	mpu6050.c: 68: while ((UART1_SR & SR_TXE) == 0);   //  Wait for transmission to complete.
00101$:
	ldw	x, #0x5230
	ld	a, (x)
	sll	a
	jrnc	00101$
;	mpu6050.c: 69: ch++;                               //  Grab the next character.
	incw	y
	jra	00104$
00107$:
	ret
;	mpu6050.c: 75: void i2c_send_address (UCHAR addr, UCHAR mode) {
;	-----------------------------------------
;	 function i2c_send_address
;	-----------------------------------------
_i2c_send_address:
	sub	sp, #3
;	mpu6050.c: 77: reg = I2C_SR1;
	ldw	x, #0x5217
	ld	a, (x)
	clrw	x
	ld	xl, a
	ldw	(0x01, sp), x
;	mpu6050.c: 78: I2C_DR = (addr << 1) | mode;
	ld	a, (0x06, sp)
	sll	a
	or	a, (0x07, sp)
	ldw	x, #0x5216
	ld	(x), a
;	mpu6050.c: 79: if (mode == I2C_READ) {
	ld	a, (0x07, sp)
	cp	a, #0x01
	jrne	00127$
	ld	a, #0x01
	ld	(0x03, sp), a
	jra	00128$
00127$:
	clr	(0x03, sp)
00128$:
	tnz	(0x03, sp)
	jreq	00103$
;	mpu6050.c: 80: I2C_OARL = 0;
	ldw	x, #0x5213
	clr	(x)
;	mpu6050.c: 81: I2C_OARH = 0;
	ldw	x, #0x5214
	clr	(x)
;	mpu6050.c: 84: while ((I2C_SR1 & I2C_ADDR) == 0);
00103$:
;	mpu6050.c: 77: reg = I2C_SR1;
	ldw	x, #0x5217
	ld	a, (x)
;	mpu6050.c: 84: while ((I2C_SR1 & I2C_ADDR) == 0);
	bcp	a, #0x02
	jreq	00103$
;	mpu6050.c: 85: if (mode == I2C_READ)
	tnz	(0x03, sp)
	jreq	00108$
;	mpu6050.c: 86: UNSET (I2C_SR1, I2C_ADDR);
	and	a, #0xfd
	ldw	x, #0x5217
	ld	(x), a
00108$:
	addw	sp, #3
	ret
;	mpu6050.c: 89: void i2c_set_start_ack (void) {
;	-----------------------------------------
;	 function i2c_set_start_ack
;	-----------------------------------------
_i2c_set_start_ack:
;	mpu6050.c: 90: I2C_CR2 = I2C_ACK | I2C_START;
	ldw	x, #0x5211
	ld	a, #0x05
	ld	(x), a
;	mpu6050.c: 91: while ((I2C_SR1 & I2C_SB) == 0);
00101$:
	ldw	x, #0x5217
	ld	a, (x)
	srl	a
	jrnc	00101$
	ret
;	mpu6050.c: 98: void print_byte_hex (unsigned char buffer) {
;	-----------------------------------------
;	 function print_byte_hex
;	-----------------------------------------
_print_byte_hex:
	sub	sp, #12
;	mpu6050.c: 101: a = (buffer >> 4);
	ld	a, (0x0f, sp)
	swap	a
	and	a, #0x0f
	clrw	x
	ld	xl, a
;	mpu6050.c: 102: if (a > 9)
	cpw	x, #0x0009
	jrsle	00102$
;	mpu6050.c: 103: a = a + 'a' - 10;
	addw	x, #0x0057
	ldw	(0x03, sp), x
	jra	00103$
00102$:
;	mpu6050.c: 105: a += '0'; 
	addw	x, #0x0030
	ldw	(0x03, sp), x
00103$:
;	mpu6050.c: 106: b = buffer & 0x0f;
	ld	a, (0x0f, sp)
	and	a, #0x0f
	clrw	x
	ld	xl, a
;	mpu6050.c: 107: if (b > 9)
	cpw	x, #0x0009
	jrsle	00105$
;	mpu6050.c: 108: b = b + 'a' - 10;
	addw	x, #0x0057
	ldw	(0x01, sp), x
	jra	00106$
00105$:
;	mpu6050.c: 110: b += '0'; 
	addw	x, #0x0030
	ldw	(0x01, sp), x
00106$:
;	mpu6050.c: 111: message[0] = a;
	ldw	y, sp
	addw	y, #5
	ld	a, (0x04, sp)
	ld	(y), a
;	mpu6050.c: 112: message[1] = b;
	ldw	x, y
	incw	x
	ld	a, (0x02, sp)
	ld	(x), a
;	mpu6050.c: 113: message[2] = 0;
	ldw	x, y
	incw	x
	incw	x
	clr	(x)
;	mpu6050.c: 114: UARTPrintF (message);
	pushw	y
	call	_UARTPrintF
	addw	sp, #2
	addw	sp, #12
	ret
;	mpu6050.c: 118: unsigned char i2c_read_register (UCHAR addr, UCHAR rg) {
;	-----------------------------------------
;	 function i2c_read_register
;	-----------------------------------------
_i2c_read_register:
	sub	sp, #2
;	mpu6050.c: 121: i2c_set_start_ack ();
	call	_i2c_set_start_ack
;	mpu6050.c: 122: i2c_send_address (addr, I2C_WRITE);
	push	#0x00
	ld	a, (0x06, sp)
	push	a
	call	_i2c_send_address
	addw	sp, #2
;	mpu6050.c: 123: i2c_send_reg (rg);
	ld	a, (0x06, sp)
	push	a
	call	_i2c_send_reg
	pop	a
;	mpu6050.c: 124: i2c_set_start_ack ();
	call	_i2c_set_start_ack
;	mpu6050.c: 125: i2c_send_address (addr, I2C_READ);
	push	#0x01
	ld	a, (0x06, sp)
	push	a
	call	_i2c_send_address
	addw	sp, #2
;	mpu6050.c: 126: reg = I2C_SR1;
	ldw	x, #0x5217
	ld	a, (x)
	ld	(0x02, sp), a
;	mpu6050.c: 127: reg = I2C_SR3;
	ldw	x, #0x5219
	ld	a, (x)
	ld	(0x02, sp), a
;	mpu6050.c: 128: i2c_set_nak ();
	call	_i2c_set_nak
;	mpu6050.c: 129: i2c_set_stop ();
	call	_i2c_set_stop
;	mpu6050.c: 130: i2c_read (&x);
	ldw	x, sp
	incw	x
	pushw	x
	call	_i2c_read
	addw	sp, #2
;	mpu6050.c: 131: return (x);
	ld	a, (0x01, sp)
	addw	sp, #2
	ret
;	mpu6050.c: 134: void InitializeI2C (void) {
;	-----------------------------------------
;	 function InitializeI2C
;	-----------------------------------------
_InitializeI2C:
;	mpu6050.c: 135: I2C_CR1 = 0;   //  Disable I2C before configuration starts. PE bit is bit 0
	ldw	x, #0x5210
	clr	(x)
;	mpu6050.c: 139: I2C_FREQR = 16;                     //  Set the internal clock frequency (MHz).
	ldw	x, #0x5212
	ld	a, #0x10
	ld	(x), a
;	mpu6050.c: 140: UNSET (I2C_CCRH, I2C_FS);           //  I2C running is standard mode.
	bres	0x521c, #7
;	mpu6050.c: 142: I2C_CCRL = 0xa0;                    //  SCL clock speed is 50 kHz.
	ldw	x, #0x521b
	ld	a, #0xa0
	ld	(x), a
;	mpu6050.c: 144: I2C_CCRH &= 0x00;	// Clears lower 4 bits "CCR"
	ldw	x, #0x521c
	clr	(x)
;	mpu6050.c: 148: UNSET (I2C_OARH, I2C_ADDMODE);      //  7 bit address mode.
	bres	0x5214, #7
;	mpu6050.c: 149: SET (I2C_OARH, I2C_ADDCONF);        //  Docs say this must always be 1.
	ldw	x, #0x5214
	ld	a, (x)
	or	a, #0x40
	ld	(x), a
;	mpu6050.c: 153: I2C_TRISER = 17;
	ldw	x, #0x521d
	ld	a, #0x11
	ld	(x), a
;	mpu6050.c: 161: I2C_CR1 = I2C_PE;	// Enables port
	ldw	x, #0x5210
	ld	a, #0x01
	ld	(x), a
	ret
;	mpu6050.c: 167: void InitializeUART() {
;	-----------------------------------------
;	 function InitializeUART
;	-----------------------------------------
_InitializeUART:
;	mpu6050.c: 177: UART1_CR1 = 0;
	ldw	x, #0x5234
	clr	(x)
;	mpu6050.c: 178: UART1_CR2 = 0;
	ldw	x, #0x5235
	clr	(x)
;	mpu6050.c: 179: UART1_CR4 = 0;
	ldw	x, #0x5237
	clr	(x)
;	mpu6050.c: 180: UART1_CR3 = 0;
	ldw	x, #0x5236
	clr	(x)
;	mpu6050.c: 181: UART1_CR5 = 0;
	ldw	x, #0x5238
	clr	(x)
;	mpu6050.c: 182: UART1_GTR = 0;
	ldw	x, #0x5239
	clr	(x)
;	mpu6050.c: 183: UART1_PSCR = 0;
	ldw	x, #0x523a
	clr	(x)
;	mpu6050.c: 187: UNSET (UART1_CR1, CR1_M);        //  8 Data bits.
	ldw	x, #0x5234
	ld	a, (x)
	and	a, #0xef
	ld	(x), a
;	mpu6050.c: 188: UNSET (UART1_CR1, CR1_PCEN);     //  Disable parity.
	ldw	x, #0x5234
	ld	a, (x)
	and	a, #0xfb
	ld	(x), a
;	mpu6050.c: 189: UNSET (UART1_CR3, CR3_STOPH);    //  1 stop bit.
	ldw	x, #0x5236
	ld	a, (x)
	and	a, #0xdf
	ld	(x), a
;	mpu6050.c: 190: UNSET (UART1_CR3, CR3_STOPL);    //  1 stop bit.
	ldw	x, #0x5236
	ld	a, (x)
	and	a, #0xef
	ld	(x), a
;	mpu6050.c: 191: UART1_BRR2 = 0x0a;      //  Set the baud rate registers to 115200 baud
	ldw	x, #0x5233
	ld	a, #0x0a
	ld	(x), a
;	mpu6050.c: 192: UART1_BRR1 = 0x08;      //  based upon a 16 MHz system clock.
	ldw	x, #0x5232
	ld	a, #0x08
	ld	(x), a
;	mpu6050.c: 196: UNSET (UART1_CR2, CR2_TEN);      //  Disable transmit.
	ldw	x, #0x5235
	ld	a, (x)
	and	a, #0xf7
	ld	(x), a
;	mpu6050.c: 197: UNSET (UART1_CR2, CR2_REN);      //  Disable receive.
	ldw	x, #0x5235
	ld	a, (x)
	and	a, #0xfb
	ld	(x), a
;	mpu6050.c: 201: SET (UART1_CR3, CR3_CPOL);
	ldw	x, #0x5236
	ld	a, (x)
	or	a, #0x04
	ld	(x), a
;	mpu6050.c: 202: SET (UART1_CR3, CR3_CPHA);
	ldw	x, #0x5236
	ld	a, (x)
	or	a, #0x02
	ld	(x), a
;	mpu6050.c: 203: SET (UART1_CR3, CR3_LBCL);
	bset	0x5236, #0
;	mpu6050.c: 207: SET (UART1_CR2, CR2_TEN);
	ldw	x, #0x5235
	ld	a, (x)
	or	a, #0x08
	ld	(x), a
;	mpu6050.c: 208: SET (UART1_CR2, CR2_REN);
	ldw	x, #0x5235
	ld	a, (x)
	or	a, #0x04
	ld	(x), a
;	mpu6050.c: 209: UART1_CR3 = CR3_CLKEN;
	ldw	x, #0x5236
	ld	a, #0x08
	ld	(x), a
	ret
;	mpu6050.c: 237: void tm1637Init(void)
;	-----------------------------------------
;	 function tm1637Init
;	-----------------------------------------
_tm1637Init:
;	mpu6050.c: 239: tm1637SetBrightness(8);
	push	#0x08
	call	_tm1637SetBrightness
	pop	a
	ret
;	mpu6050.c: 244: void tm1637DisplayDecimal(long TT,unsigned int displaySeparator)
;	-----------------------------------------
;	 function tm1637DisplayDecimal
;	-----------------------------------------
_tm1637DisplayDecimal:
	sub	sp, #19
;	mpu6050.c: 246: unsigned int v = TT & 0x0000FFFF;
	ld	a, (0x19, sp)
	ld	xl, a
	ld	a, (0x18, sp)
	ld	xh, a
	clr	(0x0d, sp)
	clr	a
	ldw	(0x05, sp), x
;	mpu6050.c: 252: for (ii = 0; ii < 4; ++ii) {
	ldw	x, sp
	incw	x
	ldw	(0x10, sp), x
	ldw	x, #_segmentMap+0
	ldw	(0x12, sp), x
	clrw	y
00106$:
;	mpu6050.c: 253: digitArr[ii] = segmentMap[v % 10];
	ldw	x, y
	addw	x, (0x10, sp)
	ldw	(0x09, sp), x
	pushw	y
	ldw	x, (0x07, sp)
	ldw	y, #0x000a
	divw	x, y
	ldw	x, y
	popw	y
	addw	x, (0x12, sp)
	ld	a, (x)
	ldw	x, (0x09, sp)
	ld	(x), a
;	mpu6050.c: 254: if (ii == 2 && displaySeparator) {
	cpw	y, #0x0002
	jrne	00102$
	ldw	x, (0x1a, sp)
	jreq	00102$
;	mpu6050.c: 255: digitArr[ii] |= 1 << 7;
	ldw	x, (0x09, sp)
	ld	a, (x)
	or	a, #0x80
	ldw	x, (0x09, sp)
	ld	(x), a
00102$:
;	mpu6050.c: 257: v /= 10;
	pushw	y
	ldw	x, (0x07, sp)
	ldw	y, #0x000a
	divw	x, y
	popw	y
	ldw	(0x05, sp), x
;	mpu6050.c: 252: for (ii = 0; ii < 4; ++ii) {
	incw	y
	cpw	y, #0x0004
	jrc	00106$
;	mpu6050.c: 260: _tm1637Start();
	call	__tm1637Start
;	mpu6050.c: 261: _tm1637WriteByte(0x40);
	push	#0x40
	call	__tm1637WriteByte
	pop	a
;	mpu6050.c: 262: _tm1637ReadResult();
	call	__tm1637ReadResult
;	mpu6050.c: 263: _tm1637Stop();
	call	__tm1637Stop
;	mpu6050.c: 265: _tm1637Start();
	call	__tm1637Start
;	mpu6050.c: 266: _tm1637WriteByte(0xc0);
	push	#0xc0
	call	__tm1637WriteByte
	pop	a
;	mpu6050.c: 267: _tm1637ReadResult();
	call	__tm1637ReadResult
;	mpu6050.c: 269: for (ii = 0; ii < 4; ++ii) {
	clrw	x
	ldw	(0x07, sp), x
00108$:
;	mpu6050.c: 270: _tm1637WriteByte(digitArr[3 - ii]);
	ld	a, (0x08, sp)
	ld	(0x0b, sp), a
	ld	a, #0x03
	sub	a, (0x0b, sp)
	clrw	x
	ld	xl, a
	addw	x, (0x10, sp)
	ld	a, (x)
	push	a
	call	__tm1637WriteByte
	pop	a
;	mpu6050.c: 271: _tm1637ReadResult();
	call	__tm1637ReadResult
;	mpu6050.c: 269: for (ii = 0; ii < 4; ++ii) {
	ldw	x, (0x07, sp)
	incw	x
	ldw	(0x07, sp), x
	ldw	x, (0x07, sp)
	cpw	x, #0x0004
	jrc	00108$
;	mpu6050.c: 274: _tm1637Stop();
	call	__tm1637Stop
	addw	sp, #19
	ret
;	mpu6050.c: 279: void tm1637SetBrightness(char brightness)
;	-----------------------------------------
;	 function tm1637SetBrightness
;	-----------------------------------------
_tm1637SetBrightness:
;	mpu6050.c: 286: _tm1637Start();
	call	__tm1637Start
;	mpu6050.c: 287: _tm1637WriteByte(0x87 + brightness);
	ld	a, (0x03, sp)
	add	a, #0x87
	push	a
	call	__tm1637WriteByte
	pop	a
;	mpu6050.c: 288: _tm1637ReadResult();
	call	__tm1637ReadResult
;	mpu6050.c: 289: _tm1637Stop();
	jp	__tm1637Stop
;	mpu6050.c: 292: void _tm1637Start(void)
;	-----------------------------------------
;	 function _tm1637Start
;	-----------------------------------------
__tm1637Start:
;	mpu6050.c: 294: _tm1637ClkHigh();
	call	__tm1637ClkHigh
;	mpu6050.c: 295: _tm1637DioHigh();
	call	__tm1637DioHigh
;	mpu6050.c: 296: delay(5);
	push	#0x05
	push	#0x00
	call	_delay
	addw	sp, #2
;	mpu6050.c: 297: _tm1637DioLow();
	jp	__tm1637DioLow
;	mpu6050.c: 300: void _tm1637Stop(void)
;	-----------------------------------------
;	 function _tm1637Stop
;	-----------------------------------------
__tm1637Stop:
;	mpu6050.c: 302: _tm1637ClkLow();
	call	__tm1637ClkLow
;	mpu6050.c: 303: delay(5);
	push	#0x05
	push	#0x00
	call	_delay
	addw	sp, #2
;	mpu6050.c: 304: _tm1637DioLow();
	call	__tm1637DioLow
;	mpu6050.c: 305: delay(5);
	push	#0x05
	push	#0x00
	call	_delay
	addw	sp, #2
;	mpu6050.c: 306: _tm1637ClkHigh();
	call	__tm1637ClkHigh
;	mpu6050.c: 307: delay(5);
	push	#0x05
	push	#0x00
	call	_delay
	addw	sp, #2
;	mpu6050.c: 308: _tm1637DioHigh();
	jp	__tm1637DioHigh
;	mpu6050.c: 311: void _tm1637ReadResult(void)
;	-----------------------------------------
;	 function _tm1637ReadResult
;	-----------------------------------------
__tm1637ReadResult:
;	mpu6050.c: 313: _tm1637ClkLow();
	call	__tm1637ClkLow
;	mpu6050.c: 314: delay(5);
	push	#0x05
	push	#0x00
	call	_delay
	addw	sp, #2
;	mpu6050.c: 316: _tm1637ClkHigh();
	call	__tm1637ClkHigh
;	mpu6050.c: 317: delay(5);
	push	#0x05
	push	#0x00
	call	_delay
	addw	sp, #2
;	mpu6050.c: 318: _tm1637ClkLow();
	jp	__tm1637ClkLow
;	mpu6050.c: 321: void _tm1637WriteByte(unsigned char b)
;	-----------------------------------------
;	 function _tm1637WriteByte
;	-----------------------------------------
__tm1637WriteByte:
	sub	sp, #2
;	mpu6050.c: 323: for (ii = 0; ii < 8; ++ii) {
	clrw	x
	ldw	(0x01, sp), x
00105$:
;	mpu6050.c: 324: _tm1637ClkLow();
	call	__tm1637ClkLow
;	mpu6050.c: 325: if (b & 0x01) {
	ld	a, (0x05, sp)
	srl	a
	jrnc	00102$
;	mpu6050.c: 326: _tm1637DioHigh();
	call	__tm1637DioHigh
	jra	00103$
00102$:
;	mpu6050.c: 329: _tm1637DioLow();
	call	__tm1637DioLow
00103$:
;	mpu6050.c: 331: delay(15);
	push	#0x0f
	push	#0x00
	call	_delay
	addw	sp, #2
;	mpu6050.c: 332: b >>= 1;
	ld	a, (0x05, sp)
	srl	a
	ld	(0x05, sp), a
;	mpu6050.c: 333: _tm1637ClkHigh();
	call	__tm1637ClkHigh
;	mpu6050.c: 334: delay(15);
	push	#0x0f
	push	#0x00
	call	_delay
	addw	sp, #2
;	mpu6050.c: 323: for (ii = 0; ii < 8; ++ii) {
	ldw	x, (0x01, sp)
	incw	x
	ldw	(0x01, sp), x
	ldw	x, (0x01, sp)
	cpw	x, #0x0008
	jrslt	00105$
	addw	sp, #2
	ret
;	mpu6050.c: 340: void _tm1637ClkHigh(void)
;	-----------------------------------------
;	 function _tm1637ClkHigh
;	-----------------------------------------
__tm1637ClkHigh:
;	mpu6050.c: 345: PD_ODR |= 1 << 2;
	ldw	x, #0x500f
	ld	a, (x)
	or	a, #0x04
	ld	(x), a
	ret
;	mpu6050.c: 348: void _tm1637ClkLow(void)
;	-----------------------------------------
;	 function _tm1637ClkLow
;	-----------------------------------------
__tm1637ClkLow:
;	mpu6050.c: 352: PD_ODR &= ~(1 << 2);
	ldw	x, #0x500f
	ld	a, (x)
	and	a, #0xfb
	ld	(x), a
	ret
;	mpu6050.c: 358: void _tm1637DioHigh(void)
;	-----------------------------------------
;	 function _tm1637DioHigh
;	-----------------------------------------
__tm1637DioHigh:
;	mpu6050.c: 362: PD_ODR |= 1 << 3;
	ldw	x, #0x500f
	ld	a, (x)
	or	a, #0x08
	ld	(x), a
	ret
;	mpu6050.c: 366: void _tm1637DioLow(void)
;	-----------------------------------------
;	 function _tm1637DioLow
;	-----------------------------------------
__tm1637DioLow:
;	mpu6050.c: 368: PD_ODR &= ~(1 << 3);
	ldw	x, #0x500f
	ld	a, (x)
	and	a, #0xf7
	ld	(x), a
	ret
;	mpu6050.c: 377: long readBMP280(unsigned short regAddr, short amount){
;	-----------------------------------------
;	 function readBMP280
;	-----------------------------------------
_readBMP280:
	sub	sp, #16
;	mpu6050.c: 378: long result = 0;
	clrw	x
	ldw	(0x07, sp), x
	ldw	(0x05, sp), x
;	mpu6050.c: 389: while(amount > 0){
00103$:
	ldw	x, (0x15, sp)
	cpw	x, #0x0000
	jrsle	00105$
;	mpu6050.c: 396: x = i2c_read_register (BMP280_ADDR, regAddr);
	ld	a, (0x14, sp)
	ld	xl, a
	ld	a, _BMP280_ADDR+1
	pushw	x
	addw	sp, #1
	push	a
	call	_i2c_read_register
	addw	sp, #2
;	mpu6050.c: 399: byteRead=x;
	ld	(0x04, sp), a
	clr	a
	clrw	x
	ldw	(0x01, sp), x
;	mpu6050.c: 400: result |= byteRead << ((amount-1) * 8);
	ldw	y, (0x15, sp)
	decw	y
	ldw	x, y
	sllw	x
	sllw	x
	sllw	x
	push	a
	ld	a, (0x05, sp)
	ld	(0x0d, sp), a
	ld	a, (0x03, sp)
	ld	(0x0b, sp), a
	ld	a, (0x02, sp)
	ld	(0x0a, sp), a
	ld	a, xl
	tnz	a
	jreq	00122$
00121$:
	sll	(0x0d, sp)
	rlc	(1, sp)
	rlc	(0x0b, sp)
	rlc	(0x0a, sp)
	dec	a
	jrne	00121$
00122$:
	pop	a
	or	a, (0x07, sp)
	ld	xh, a
	ld	a, (0x08, sp)
	or	a, (0x0c, sp)
	ld	xl, a
	ld	a, (0x06, sp)
	or	a, (0x0a, sp)
	ld	(0x0e, sp), a
	ld	a, (0x05, sp)
	or	a, (0x09, sp)
	ldw	(0x07, sp), x
	ld	(0x05, sp), a
	ld	a, (0x0e, sp)
	ld	(0x06, sp), a
;	mpu6050.c: 401: amount --;
	ldw	(0x15, sp), y
;	mpu6050.c: 403: if(amount == 0){
	ldw	x, (0x15, sp)
	jrne	00103$
;	mpu6050.c: 404: i2c_set_nak();
	call	_i2c_set_nak
	jp	00103$
00105$:
;	mpu6050.c: 412: i2c_set_stop();
	call	_i2c_set_stop
;	mpu6050.c: 413: return result;
	ldw	x, (0x07, sp)
	ldw	y, (0x05, sp)
	addw	sp, #16
	ret
;	mpu6050.c: 447: void initBMP280(){
;	-----------------------------------------
;	 function initBMP280
;	-----------------------------------------
_initBMP280:
	sub	sp, #28
;	mpu6050.c: 468: i2c_set_start_ack();
	call	_i2c_set_start_ack
;	mpu6050.c: 469: i2c_send_address (BMP280_ADDR, I2C_WRITE);
	ld	a, _BMP280_ADDR+1
	push	#0x00
	push	a
	call	_i2c_send_address
	addw	sp, #2
;	mpu6050.c: 470: i2c_send_reg(0xF4);
	push	#0xf4
	call	_i2c_send_reg
	pop	a
;	mpu6050.c: 471: i2c_send_reg(0x23);
	push	#0x23
	call	_i2c_send_reg
	pop	a
;	mpu6050.c: 472: i2c_set_stop();
	call	_i2c_set_stop
;	mpu6050.c: 474: dig_T1_2 = readBMP280(0x88,1);
	push	#0x01
	push	#0x00
	push	#0x88
	push	#0x00
	call	_readBMP280
	addw	sp, #4
	ldw	(0x05, sp), x
;	mpu6050.c: 475: dig_T1_1 = readBMP280(0x89,1);
	push	#0x01
	push	#0x00
	push	#0x89
	push	#0x00
	call	_readBMP280
	addw	sp, #4
	ldw	(0x07, sp), x
;	mpu6050.c: 476: dig_T2_1 = readBMP280(0x8B,1);
	push	#0x01
	push	#0x00
	push	#0x8b
	push	#0x00
	call	_readBMP280
	addw	sp, #4
	ldw	(0x03, sp), x
;	mpu6050.c: 477: dig_T2_2 = readBMP280(0x8A,1);
	push	#0x01
	push	#0x00
	push	#0x8a
	push	#0x00
	call	_readBMP280
	addw	sp, #4
	ldw	(0x01, sp), x
;	mpu6050.c: 478: dig_T3_1 = readBMP280(0x8D,1);
	push	#0x01
	push	#0x00
	push	#0x8d
	push	#0x00
	call	_readBMP280
	addw	sp, #4
	ldw	(0x0d, sp), x
;	mpu6050.c: 479: dig_T3_2 = readBMP280(0x8C,1);
	push	#0x01
	push	#0x00
	push	#0x8c
	push	#0x00
	call	_readBMP280
	addw	sp, #4
	ldw	(0x0b, sp), x
;	mpu6050.c: 481: dig_T1 = (dig_T1_1 << 8) | dig_T1_2;
	ldw	x, (0x07, sp)
	sllw	x
	sllw	x
	sllw	x
	sllw	x
	sllw	x
	sllw	x
	sllw	x
	sllw	x
	ld	a, xl
	or	a, (0x06, sp)
	ld	xl, a
	ld	a, xh
	or	a, (0x05, sp)
	ld	xh, a
	clrw	y
	ldw	_dig_T1+2, x
	ldw	_dig_T1+0, y
;	mpu6050.c: 482: dig_T2 = (dig_T2_1 << 8) | dig_T2_2;
	ldw	x, (0x03, sp)
	sllw	x
	sllw	x
	sllw	x
	sllw	x
	sllw	x
	sllw	x
	sllw	x
	sllw	x
	ld	a, xl
	or	a, (0x02, sp)
	ld	xl, a
	ld	a, xh
	or	a, (0x01, sp)
	ld	xh, a
	clrw	y
	ldw	_dig_T2+2, x
	ldw	_dig_T2+0, y
;	mpu6050.c: 483: dig_T3 = (dig_T3_1 << 8) | dig_T3_2;
	ldw	x, (0x0d, sp)
	sllw	x
	sllw	x
	sllw	x
	sllw	x
	sllw	x
	sllw	x
	sllw	x
	sllw	x
	ld	a, xl
	or	a, (0x0c, sp)
	ld	xl, a
	ld	a, xh
	or	a, (0x0b, sp)
	ld	xh, a
	clrw	y
	ldw	_dig_T3+2, x
	ldw	_dig_T3+0, y
;	mpu6050.c: 486: dig_H1_1 = readBMP280(0xA1,1);
	push	#0x01
	push	#0x00
	push	#0xa1
	push	#0x00
	call	_readBMP280
	addw	sp, #4
	ldw	(0x09, sp), x
;	mpu6050.c: 487: dig_H2_1 = readBMP280(0xE2,1);
	push	#0x01
	push	#0x00
	push	#0xe2
	push	#0x00
	call	_readBMP280
	addw	sp, #4
;	mpu6050.c: 488: dig_H2_2 = readBMP280(0xE1,1);
	push	#0x01
	push	#0x00
	push	#0xe1
	push	#0x00
	call	_readBMP280
	addw	sp, #4
	ldw	(0x15, sp), x
;	mpu6050.c: 489: dig_H3_1 = readBMP280(0xE3,1);
	push	#0x01
	push	#0x00
	push	#0xe3
	push	#0x00
	call	_readBMP280
	addw	sp, #4
	ldw	(0x13, sp), x
;	mpu6050.c: 490: dig_H4_1 = readBMP280(0xE5,1);
	push	#0x01
	push	#0x00
	push	#0xe5
	push	#0x00
	call	_readBMP280
	addw	sp, #4
	ldw	(0x11, sp), x
;	mpu6050.c: 491: dig_H4_2 = readBMP280(0xE4,1);
	push	#0x01
	push	#0x00
	push	#0xe4
	push	#0x00
	call	_readBMP280
	addw	sp, #4
;	mpu6050.c: 492: dig_H5_1 = readBMP280(0xE6,1);
	push	#0x01
	push	#0x00
	push	#0xe6
	push	#0x00
	call	_readBMP280
	addw	sp, #4
	ldw	(0x0f, sp), x
;	mpu6050.c: 493: dig_H5_2 = readBMP280(0xE5,1);
	push	#0x01
	push	#0x00
	push	#0xe5
	push	#0x00
	call	_readBMP280
	addw	sp, #4
	ldw	(0x19, sp), x
;	mpu6050.c: 494: dig_H6_1 = readBMP280(0xE7,1);
	push	#0x01
	push	#0x00
	push	#0xe7
	push	#0x00
	call	_readBMP280
	addw	sp, #4
	ldw	(0x17, sp), x
;	mpu6050.c: 495: dig_H1 = dig_H1_1; 
	ldw	y, (0x09, sp)
	clrw	x
	ldw	_dig_H1+2, y
	ldw	_dig_H1+0, x
;	mpu6050.c: 496: dig_H2 = (dig_H1_1 << 8) | dig_H2_2; 
	ldw	x, (0x09, sp)
	sllw	x
	sllw	x
	sllw	x
	sllw	x
	sllw	x
	sllw	x
	sllw	x
	sllw	x
	ld	a, xl
	or	a, (0x16, sp)
	ld	xl, a
	ld	a, xh
	or	a, (0x15, sp)
	ld	xh, a
	clrw	y
	ldw	_dig_H2+2, x
	ldw	_dig_H2+0, y
;	mpu6050.c: 497: dig_H3 = dig_H3_1; 
	ldw	y, (0x13, sp)
	clrw	x
	ldw	_dig_H3+2, y
	ldw	_dig_H3+0, x
;	mpu6050.c: 498: dig_H4 = dig_H4_1 & 0x0F;
	ld	a, (0x12, sp)
	and	a, #0x0f
	ld	xl, a
	clr	a
	ld	xh, a
	clrw	y
	ldw	_dig_H4+2, x
	ldw	_dig_H4+0, y
;	mpu6050.c: 499: dig_H5 = (dig_H5_1 << 4) | ((dig_H5_2 & 0xF0)>>4);
	ldw	x, (0x0f, sp)
	sllw	x
	sllw	x
	sllw	x
	sllw	x
	ldw	(0x1b, sp), x
	ld	a, (0x1a, sp)
	and	a, #0xf0
	ld	xl, a
	clr	a
	ld	xh, a
	ld	a, #0x10
	div	x, a
	ld	a, xl
	or	a, (0x1c, sp)
	ld	xl, a
	ld	a, xh
	or	a, (0x1b, sp)
	ld	xh, a
	clrw	y
	ldw	_dig_H5+2, x
	ldw	_dig_H5+0, y
;	mpu6050.c: 500: dig_H6 = dig_H6_1;
	ldw	y, (0x17, sp)
	clrw	x
	ldw	_dig_H6+2, y
	ldw	_dig_H6+0, x
	addw	sp, #28
	ret
;	mpu6050.c: 503: long getBMP280Temperature(){
;	-----------------------------------------
;	 function getBMP280Temperature
;	-----------------------------------------
_getBMP280Temperature:
	sub	sp, #16
;	mpu6050.c: 504: if(dig_T1 != 0 && dig_T2 != 0 && dig_T3 != 0){
	ldw	x, _dig_T1+2
	jrne	00120$
	ldw	x, _dig_T1+0
	jrne	00120$
	jp	00102$
00120$:
	ldw	x, _dig_T2+2
	jrne	00121$
	ldw	x, _dig_T2+0
	jrne	00121$
	jp	00102$
00121$:
	ldw	x, _dig_T3+2
	jrne	00122$
	ldw	x, _dig_T3+0
	jrne	00122$
	jp	00102$
00122$:
;	mpu6050.c: 505: long T = readBMP280(0xFA,3) >> 4;
	push	#0x03
	push	#0x00
	push	#0xfa
	push	#0x00
	call	_readBMP280
	addw	sp, #4
	ld	a, #0x04
	tnz	a
	jreq	00124$
00123$:
	sraw	y
	rrcw	x
	dec	a
	jrne	00123$
00124$:
	ldw	(0x07, sp), x
	ldw	(0x05, sp), y
;	mpu6050.c: 506: long part1 = (T >> 3) - (dig_T1 << 1);
	ldw	y, (0x07, sp)
	ldw	(0x0f, sp), y
	ld	a, (0x05, sp)
	ld	(0x0d, sp), a
	ld	a, (0x06, sp)
	push	a
	ld	a, #0x03
	tnz	a
	jreq	00126$
00125$:
	sra	(0x0e, sp)
	rrc	(1, sp)
	rrc	(0x10, sp)
	rrc	(0x11, sp)
	dec	a
	jrne	00125$
00126$:
	pop	a
	ldw	y, _dig_T1+2
	ldw	(0x0b, sp), y
	ldw	x, _dig_T1+0
	sll	(0x0c, sp)
	rlc	(0x0b, sp)
	rlcw	x
	ldw	y, (0x0f, sp)
	subw	y, (0x0b, sp)
	pushw	x
	sbc	a, (#2, sp)
	popw	x
	ld	xl, a
	ld	a, (0x0d, sp)
	pushw	x
	sbc	a, (#1, sp)
	popw	x
	ld	xh, a
;	mpu6050.c: 507: long var1 = (part1 * dig_T2) >> 11;
	push	_dig_T2+3
	push	_dig_T2+2
	push	_dig_T2+1
	push	_dig_T2+0
	pushw	y
	pushw	x
	call	__mullong
	addw	sp, #8
	ld	a, #0x0b
	tnz	a
	jreq	00128$
00127$:
	sraw	y
	rrcw	x
	dec	a
	jrne	00127$
00128$:
	ldw	(0x03, sp), x
	ldw	(0x01, sp), y
;	mpu6050.c: 508: long part2 = (T >> 4) - dig_T1;
	ldw	y, (0x07, sp)
	ldw	x, (0x05, sp)
	ld	a, #0x04
	tnz	a
	jreq	00130$
00129$:
	sraw	x
	rrcw	y
	dec	a
	jrne	00129$
00130$:
	subw	y, _dig_T1+2
	ld	a, xl
	sbc	a, _dig_T1+1
	ld	xl, a
	ld	a, xh
	sbc	a, _dig_T1+0
	ld	xh, a
;	mpu6050.c: 509: long var2 = (((part2 * part2) >> 12) * dig_T3) >> 14;
	pushw	y
	pushw	x
	pushw	y
	pushw	x
	call	__mullong
	addw	sp, #8
	ld	a, #0x0c
	tnz	a
	jreq	00132$
00131$:
	sraw	y
	rrcw	x
	dec	a
	jrne	00131$
00132$:
	push	_dig_T3+3
	push	_dig_T3+2
	push	_dig_T3+1
	push	_dig_T3+0
	pushw	x
	pushw	y
	call	__mullong
	addw	sp, #8
	exgw	x, y
	ld	a, #0x0e
	tnz	a
	jreq	00134$
00133$:
	sraw	x
	rrcw	y
	dec	a
	jrne	00133$
00134$:
;	mpu6050.c: 510: long t_fine = var1 + var2;
	addw	y, (0x03, sp)
	ld	a, xl
	adc	a, (0x02, sp)
	ld	xl, a
	ld	a, xh
	adc	a, (0x01, sp)
	ld	xh, a
;	mpu6050.c: 511: long calc = (t_fine * 5 + 128) >> 8;
	pushw	y
	pushw	x
	push	#0x05
	clrw	x
	pushw	x
	push	#0x00
	call	__mullong
	addw	sp, #8
	exgw	x, y
	addw	y, #0x0080
	ld	a, xl
	adc	a, #0x00
	ld	xl, a
	ld	a, xh
	adc	a, #0x00
	ld	xh, a
	sraw	x
	rrcw	y
	sraw	x
	rrcw	y
	sraw	x
	rrcw	y
	sraw	x
	rrcw	y
	sraw	x
	rrcw	y
	sraw	x
	rrcw	y
	sraw	x
	rrcw	y
	sraw	x
	rrcw	y
;	mpu6050.c: 512: return calc;
	exgw	x, y
	jra	00106$
00102$:
;	mpu6050.c: 514: return 0;
	clrw	x
	clrw	y
00106$:
	addw	sp, #16
	ret
;	mpu6050.c: 517: long getBMP280Humidity(){
;	-----------------------------------------
;	 function getBMP280Humidity
;	-----------------------------------------
_getBMP280Humidity:
;	mpu6050.c: 518: }
	ret
;	mpu6050.c: 520: int main () {
;	-----------------------------------------
;	 function main
;	-----------------------------------------
_main:
	sub	sp, #22
;	mpu6050.c: 527: InitializeSystemClock();
	call	_InitializeSystemClock
;	mpu6050.c: 530: PD_DDR = (1 << 3) | (1 << 2); // output mode
	ldw	x, #0x5011
	ld	a, #0x0c
	ld	(x), a
;	mpu6050.c: 531: PB_DDR = (1 << 4);
	ldw	x, #0x5007
	ld	a, #0x10
	ld	(x), a
;	mpu6050.c: 532: PB_DDR = (1 << 5);
	ldw	x, #0x5007
	ld	a, #0x20
	ld	(x), a
;	mpu6050.c: 533: PB_ODR &= ~(1 << 4);
	ldw	x, #0x5005
	ld	a, (x)
	and	a, #0xef
	ld	(x), a
;	mpu6050.c: 534: PB_ODR &= ~(1 << 5);
	ldw	x, #0x5005
	ld	a, (x)
	and	a, #0xdf
	ld	(x), a
;	mpu6050.c: 535: PB_CR1 = (1 << 4) | (1 << 5); // push-pull
	ldw	x, #0x5008
	ld	a, #0x30
	ld	(x), a
;	mpu6050.c: 541: InitializeUART();
	call	_InitializeUART
;	mpu6050.c: 542: UARTPrintF("uart initialised \n\r");
	ldw	x, #___str_0+0
	pushw	x
	call	_UARTPrintF
	addw	sp, #2
;	mpu6050.c: 543: InitializeI2C();
	call	_InitializeI2C
;	mpu6050.c: 544: delay(200);
	push	#0xc8
	push	#0x00
	call	_delay
	addw	sp, #2
;	mpu6050.c: 546: initBMP280();
	call	_initBMP280
;	mpu6050.c: 550: while (1) {
00114$:
;	mpu6050.c: 551: objTemp = getBMP280Temperature();
	call	_getBMP280Temperature
	pushw	x
	pushw	y
	call	___slong2fs
	addw	sp, #4
	ldw	(0x03, sp), x
	ldw	(0x01, sp), y
;	mpu6050.c: 555: while (objTemp > 1000) {
	clrw	x
	ldw	(0x05, sp), x
00101$:
	clrw	x
	pushw	x
	push	#0x7a
	push	#0x44
	ldw	x, (0x07, sp)
	pushw	x
	ldw	x, (0x07, sp)
	pushw	x
	call	___fsgt
	addw	sp, #8
	tnz	a
	jreq	00127$
;	mpu6050.c: 556: vierde+=1;
	ldw	x, (0x05, sp)
	incw	x
	ldw	(0x05, sp), x
;	mpu6050.c: 557: objTemp-=1000;
	clrw	x
	pushw	x
	push	#0x7a
	push	#0x44
	ldw	x, (0x07, sp)
	pushw	x
	ldw	x, (0x07, sp)
	pushw	x
	call	___fssub
	addw	sp, #8
	ldw	(0x03, sp), x
	ldw	(0x01, sp), y
	jra	00101$
;	mpu6050.c: 559: while (objTemp > 100) {
00127$:
	ldw	y, (0x05, sp)
	ldw	(0x15, sp), y
	clrw	x
	ldw	(0x09, sp), x
00104$:
	clrw	x
	pushw	x
	push	#0xc8
	push	#0x42
	ldw	x, (0x07, sp)
	pushw	x
	ldw	x, (0x07, sp)
	pushw	x
	call	___fsgt
	addw	sp, #8
	tnz	a
	jreq	00128$
;	mpu6050.c: 560: derde+=1;
	ldw	x, (0x09, sp)
	incw	x
	ldw	(0x09, sp), x
;	mpu6050.c: 561: objTemp-=100;
	clrw	x
	pushw	x
	push	#0xc8
	push	#0x42
	ldw	x, (0x07, sp)
	pushw	x
	ldw	x, (0x07, sp)
	pushw	x
	call	___fssub
	addw	sp, #8
	ldw	(0x03, sp), x
	ldw	(0x01, sp), y
	jra	00104$
;	mpu6050.c: 563: while (objTemp > 10) {
00128$:
	ldw	y, (0x09, sp)
	ldw	(0x13, sp), y
	clrw	x
	ldw	(0x0b, sp), x
00107$:
	clrw	x
	pushw	x
	push	#0x20
	push	#0x41
	ldw	x, (0x07, sp)
	pushw	x
	ldw	x, (0x07, sp)
	pushw	x
	call	___fsgt
	addw	sp, #8
	tnz	a
	jreq	00129$
;	mpu6050.c: 564: tweede+=1;
	ldw	x, (0x0b, sp)
	incw	x
	ldw	(0x0b, sp), x
;	mpu6050.c: 565: objTemp-=10;
	clrw	x
	pushw	x
	push	#0x20
	push	#0x41
	ldw	x, (0x07, sp)
	pushw	x
	ldw	x, (0x07, sp)
	pushw	x
	call	___fssub
	addw	sp, #8
	ldw	(0x03, sp), x
	ldw	(0x01, sp), y
	jra	00107$
;	mpu6050.c: 567: while (objTemp > 0)
00129$:
	ldw	y, (0x0b, sp)
	ldw	(0x11, sp), y
	clrw	x
	ldw	(0x07, sp), x
00110$:
	clrw	x
	pushw	x
	clrw	x
	pushw	x
	ldw	x, (0x07, sp)
	pushw	x
	ldw	x, (0x07, sp)
	pushw	x
	call	___fsgt
	addw	sp, #8
	tnz	a
	jreq	00112$
;	mpu6050.c: 569: eerste+=1;
	ldw	x, (0x07, sp)
	incw	x
	ldw	(0x07, sp), x
;	mpu6050.c: 570: objTemp-=1;
	clrw	x
	pushw	x
	push	#0x80
	push	#0x3f
	ldw	x, (0x07, sp)
	pushw	x
	ldw	x, (0x07, sp)
	pushw	x
	call	___fssub
	addw	sp, #8
	ldw	(0x03, sp), x
	ldw	(0x01, sp), y
	jra	00110$
00112$:
;	mpu6050.c: 581: utemp=vierde*1000+derde*100+tweede*10+eerste;
	ldw	x, (0x15, sp)
	pushw	x
	push	#0xe8
	push	#0x03
	call	__mulint
	addw	sp, #4
	ldw	(0x0f, sp), x
	ldw	x, (0x13, sp)
	pushw	x
	push	#0x64
	push	#0x00
	call	__mulint
	addw	sp, #4
	addw	x, (0x0f, sp)
	ldw	(0x0d, sp), x
	ldw	x, (0x11, sp)
	pushw	x
	push	#0x0a
	push	#0x00
	call	__mulint
	addw	sp, #4
	addw	x, (0x0d, sp)
	addw	x, (0x07, sp)
	clrw	y
	tnzw	x
	jrpl	00162$
	decw	y
00162$:
;	mpu6050.c: 584: tm1637DisplayDecimal(utemp, 1); // eg 37:12
	push	#0x01
	push	#0x00
	pushw	x
	pushw	y
	call	_tm1637DisplayDecimal
	addw	sp, #6
;	mpu6050.c: 587: delayTenMicro();
	call	_delayTenMicro
	jp	00114$
	addw	sp, #22
	ret
	.area CODE
_segmentMap:
	.db #0x3F	;  63
	.db #0x06	;  6
	.db #0x5B	;  91
	.db #0x4F	;  79	'O'
	.db #0x66	;  102	'f'
	.db #0x6D	;  109	'm'
	.db #0x7D	;  125
	.db #0x07	;  7
	.db #0x7F	;  127
	.db #0x6F	;  111	'o'
	.db #0x77	;  119	'w'
	.db #0x7C	;  124
	.db #0x39	;  57	'9'
	.db #0x5E	;  94
	.db #0x79	;  121	'y'
	.db #0x71	;  113	'q'
	.db #0x00	;  0
___str_0:
	.ascii "uart initialised "
	.db 0x0A
	.db 0x0D
	.db 0x00
	.area INITIALIZER
__xinit__BMP280_ADDR:
	.dw #0x0076
	.area CABS (ABS)
