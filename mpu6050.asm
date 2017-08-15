;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 3.4.0 #8981 (Jul  5 2014) (Linux)
; This file was generated Tue Aug 15 10:20:35 2017
;--------------------------------------------------------
	.module mpu6050
	.optsdcc -mstm8
	
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _segmentMap
	.globl _main
	.globl _getMPU6050
	.globl _initMPU6050
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
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area INITIALIZED
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
;	mpu6050.c: 17: void delayTenMicro (void) {
;	-----------------------------------------
;	 function delayTenMicro
;	-----------------------------------------
_delayTenMicro:
;	mpu6050.c: 19: for (a = 0; a < 50; ++a)
	ld	a, #0x32
00104$:
;	mpu6050.c: 20: __asm__("nop");
	nop
	dec	a
;	mpu6050.c: 19: for (a = 0; a < 50; ++a)
	tnz	a
	jrne	00104$
	ret
;	mpu6050.c: 23: void InitializeSystemClock() {
;	-----------------------------------------
;	 function InitializeSystemClock
;	-----------------------------------------
_InitializeSystemClock:
;	mpu6050.c: 24: CLK_ICKR = 0;                       //  Reset the Internal Clock Register.
	ldw	x, #0x50c0
	clr	(x)
;	mpu6050.c: 25: CLK_ICKR = CLK_HSIEN;               //  Enable the HSI.
	ldw	x, #0x50c0
	ld	a, #0x01
	ld	(x), a
;	mpu6050.c: 26: CLK_ECKR = 0;                       //  Disable the external clock.
	ldw	x, #0x50c1
	clr	(x)
;	mpu6050.c: 27: while ((CLK_ICKR & CLK_HSIRDY) == 0);       //  Wait for the HSI to be ready for use.
00101$:
	ldw	x, #0x50c0
	ld	a, (x)
	bcp	a, #0x02
	jreq	00101$
;	mpu6050.c: 28: CLK_CKDIVR = 0;                     //  Ensure the clocks are running at full speed.
	ldw	x, #0x50c6
	clr	(x)
;	mpu6050.c: 29: CLK_PCKENR1 = 0xff;                 //  Enable all peripheral clocks.
	ldw	x, #0x50c7
	ld	a, #0xff
	ld	(x), a
;	mpu6050.c: 30: CLK_PCKENR2 = 0xff;                 //  Ditto.
	ldw	x, #0x50ca
	ld	a, #0xff
	ld	(x), a
;	mpu6050.c: 31: CLK_CCOR = 0;                       //  Turn off CCO.
	ldw	x, #0x50c9
	clr	(x)
;	mpu6050.c: 32: CLK_HSITRIMR = 0;                   //  Turn off any HSIU trimming.
	ldw	x, #0x50cc
	clr	(x)
;	mpu6050.c: 33: CLK_SWIMCCR = 0;                    //  Set SWIM to run at clock / 2.
	ldw	x, #0x50cd
	clr	(x)
;	mpu6050.c: 34: CLK_SWR = 0xe1;                     //  Use HSI as the clock source.
	ldw	x, #0x50c4
	ld	a, #0xe1
	ld	(x), a
;	mpu6050.c: 35: CLK_SWCR = 0;                       //  Reset the clock switch control register.
	ldw	x, #0x50c5
	clr	(x)
;	mpu6050.c: 36: CLK_SWCR = CLK_SWEN;                //  Enable switching.
	ldw	x, #0x50c5
	ld	a, #0x02
	ld	(x), a
;	mpu6050.c: 37: while ((CLK_SWCR & CLK_SWBSY) != 0);        //  Pause while the clock switch is busy.
00104$:
	ldw	x, #0x50c5
	ld	a, (x)
	srl	a
	jrc	00104$
	ret
;	mpu6050.c: 39: void delay (int time_ms) {
;	-----------------------------------------
;	 function delay
;	-----------------------------------------
_delay:
	sub	sp, #10
;	mpu6050.c: 41: for (x = 0; x < 1036*time_ms; ++x)
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
;	mpu6050.c: 42: __asm__("nop");
	nop
;	mpu6050.c: 41: for (x = 0; x < 1036*time_ms; ++x)
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
;	mpu6050.c: 44: void i2c_read (unsigned char *x) {
;	-----------------------------------------
;	 function i2c_read
;	-----------------------------------------
_i2c_read:
;	mpu6050.c: 45: while ((I2C_SR1 & I2C_RXNE) == 0);
00101$:
	ldw	x, #0x5217
	ld	a, (x)
	bcp	a, #0x40
	jreq	00101$
;	mpu6050.c: 46: *x = I2C_DR;
	ldw	y, (0x03, sp)
	ldw	x, #0x5216
	ld	a, (x)
	ld	(y), a
	ret
;	mpu6050.c: 48: void i2c_set_nak (void) {
;	-----------------------------------------
;	 function i2c_set_nak
;	-----------------------------------------
_i2c_set_nak:
;	mpu6050.c: 49: I2C_CR2 &= ~I2C_ACK;
	ldw	x, #0x5211
	ld	a, (x)
	and	a, #0xfb
	ld	(x), a
	ret
;	mpu6050.c: 51: void i2c_set_stop (void) {
;	-----------------------------------------
;	 function i2c_set_stop
;	-----------------------------------------
_i2c_set_stop:
;	mpu6050.c: 52: I2C_CR2 |= I2C_STOP;
	ldw	x, #0x5211
	ld	a, (x)
	or	a, #0x02
	ld	(x), a
	ret
;	mpu6050.c: 54: void i2c_send_reg (UCHAR addr) {
;	-----------------------------------------
;	 function i2c_send_reg
;	-----------------------------------------
_i2c_send_reg:
	sub	sp, #2
;	mpu6050.c: 56: reg = I2C_SR1;
	ldw	x, #0x5217
	ld	a, (x)
	clrw	x
	ld	xl, a
	ldw	(0x01, sp), x
;	mpu6050.c: 57: reg = I2C_SR3;
	ldw	x, #0x5219
	ld	a, (x)
	clrw	x
	ld	xl, a
	ldw	(0x01, sp), x
;	mpu6050.c: 58: I2C_DR = addr;
	ldw	x, #0x5216
	ld	a, (0x05, sp)
	ld	(x), a
;	mpu6050.c: 59: while ((I2C_SR1 & I2C_TXE) == 0);
00101$:
	ldw	x, #0x5217
	ld	a, (x)
	sll	a
	jrnc	00101$
	addw	sp, #2
	ret
;	mpu6050.c: 63: void UARTPrintF (char *message) {
;	-----------------------------------------
;	 function UARTPrintF
;	-----------------------------------------
_UARTPrintF:
;	mpu6050.c: 64: char *ch = message;
	ldw	y, (0x03, sp)
;	mpu6050.c: 65: while (*ch) {
00104$:
	ld	a, (y)
	tnz	a
	jreq	00107$
;	mpu6050.c: 66: UART1_DR = (unsigned char) *ch;     //  Put the next character into the data transmission register.
	ldw	x, #0x5231
	ld	(x), a
;	mpu6050.c: 67: while ((UART1_SR & SR_TXE) == 0);   //  Wait for transmission to complete.
00101$:
	ldw	x, #0x5230
	ld	a, (x)
	sll	a
	jrnc	00101$
;	mpu6050.c: 68: ch++;                               //  Grab the next character.
	incw	y
	jra	00104$
00107$:
	ret
;	mpu6050.c: 74: void i2c_send_address (UCHAR addr, UCHAR mode) {
;	-----------------------------------------
;	 function i2c_send_address
;	-----------------------------------------
_i2c_send_address:
	sub	sp, #3
;	mpu6050.c: 76: reg = I2C_SR1;
	ldw	x, #0x5217
	ld	a, (x)
	clrw	x
	ld	xl, a
	ldw	(0x01, sp), x
;	mpu6050.c: 77: I2C_DR = (addr << 1) | mode;
	ld	a, (0x06, sp)
	sll	a
	or	a, (0x07, sp)
	ldw	x, #0x5216
	ld	(x), a
;	mpu6050.c: 78: if (mode == I2C_READ) {
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
;	mpu6050.c: 79: I2C_OARL = 0;
	ldw	x, #0x5213
	clr	(x)
;	mpu6050.c: 80: I2C_OARH = 0;
	ldw	x, #0x5214
	clr	(x)
;	mpu6050.c: 83: while ((I2C_SR1 & I2C_ADDR) == 0);
00103$:
;	mpu6050.c: 76: reg = I2C_SR1;
	ldw	x, #0x5217
	ld	a, (x)
;	mpu6050.c: 83: while ((I2C_SR1 & I2C_ADDR) == 0);
	bcp	a, #0x02
	jreq	00103$
;	mpu6050.c: 84: if (mode == I2C_READ)
	tnz	(0x03, sp)
	jreq	00108$
;	mpu6050.c: 85: UNSET (I2C_SR1, I2C_ADDR);
	and	a, #0xfd
	ldw	x, #0x5217
	ld	(x), a
00108$:
	addw	sp, #3
	ret
;	mpu6050.c: 88: void i2c_set_start_ack (void) {
;	-----------------------------------------
;	 function i2c_set_start_ack
;	-----------------------------------------
_i2c_set_start_ack:
;	mpu6050.c: 89: I2C_CR2 = I2C_ACK | I2C_START;
	ldw	x, #0x5211
	ld	a, #0x05
	ld	(x), a
;	mpu6050.c: 90: while ((I2C_SR1 & I2C_SB) == 0);
00101$:
	ldw	x, #0x5217
	ld	a, (x)
	srl	a
	jrnc	00101$
	ret
;	mpu6050.c: 97: void print_byte_hex (unsigned char buffer) {
;	-----------------------------------------
;	 function print_byte_hex
;	-----------------------------------------
_print_byte_hex:
	sub	sp, #12
;	mpu6050.c: 100: a = (buffer >> 4);
	ld	a, (0x0f, sp)
	swap	a
	and	a, #0x0f
	clrw	x
	ld	xl, a
;	mpu6050.c: 101: if (a > 9)
	cpw	x, #0x0009
	jrsle	00102$
;	mpu6050.c: 102: a = a + 'a' - 10;
	addw	x, #0x0057
	ldw	(0x03, sp), x
	jra	00103$
00102$:
;	mpu6050.c: 104: a += '0'; 
	addw	x, #0x0030
	ldw	(0x03, sp), x
00103$:
;	mpu6050.c: 105: b = buffer & 0x0f;
	ld	a, (0x0f, sp)
	and	a, #0x0f
	clrw	x
	ld	xl, a
;	mpu6050.c: 106: if (b > 9)
	cpw	x, #0x0009
	jrsle	00105$
;	mpu6050.c: 107: b = b + 'a' - 10;
	addw	x, #0x0057
	ldw	(0x01, sp), x
	jra	00106$
00105$:
;	mpu6050.c: 109: b += '0'; 
	addw	x, #0x0030
	ldw	(0x01, sp), x
00106$:
;	mpu6050.c: 110: message[0] = a;
	ldw	y, sp
	addw	y, #5
	ld	a, (0x04, sp)
	ld	(y), a
;	mpu6050.c: 111: message[1] = b;
	ldw	x, y
	incw	x
	ld	a, (0x02, sp)
	ld	(x), a
;	mpu6050.c: 112: message[2] = 0;
	ldw	x, y
	incw	x
	incw	x
	clr	(x)
;	mpu6050.c: 113: UARTPrintF (message);
	pushw	y
	call	_UARTPrintF
	addw	sp, #2
	addw	sp, #12
	ret
;	mpu6050.c: 117: unsigned char i2c_read_register (UCHAR addr, UCHAR rg) {
;	-----------------------------------------
;	 function i2c_read_register
;	-----------------------------------------
_i2c_read_register:
	sub	sp, #2
;	mpu6050.c: 120: i2c_set_start_ack ();
	call	_i2c_set_start_ack
;	mpu6050.c: 121: i2c_send_address (addr, I2C_WRITE);
	push	#0x00
	ld	a, (0x06, sp)
	push	a
	call	_i2c_send_address
	addw	sp, #2
;	mpu6050.c: 122: i2c_send_reg (rg);
	ld	a, (0x06, sp)
	push	a
	call	_i2c_send_reg
	pop	a
;	mpu6050.c: 123: i2c_set_start_ack ();
	call	_i2c_set_start_ack
;	mpu6050.c: 124: i2c_send_address (addr, I2C_READ);
	push	#0x01
	ld	a, (0x06, sp)
	push	a
	call	_i2c_send_address
	addw	sp, #2
;	mpu6050.c: 125: reg = I2C_SR1;
	ldw	x, #0x5217
	ld	a, (x)
	ld	(0x01, sp), a
;	mpu6050.c: 126: reg = I2C_SR3;
	ldw	x, #0x5219
	ld	a, (x)
	ld	(0x01, sp), a
;	mpu6050.c: 127: i2c_set_nak ();
	call	_i2c_set_nak
;	mpu6050.c: 128: i2c_set_stop ();
	call	_i2c_set_stop
;	mpu6050.c: 129: i2c_read (&x);
	ldw	x, sp
	incw	x
	incw	x
	pushw	x
	call	_i2c_read
	addw	sp, #2
;	mpu6050.c: 130: return (x);
	ld	a, (0x02, sp)
	addw	sp, #2
	ret
;	mpu6050.c: 133: void InitializeI2C (void) {
;	-----------------------------------------
;	 function InitializeI2C
;	-----------------------------------------
_InitializeI2C:
;	mpu6050.c: 134: I2C_CR1 = 0;   //  Disable I2C before configuration starts. PE bit is bit 0
	ldw	x, #0x5210
	clr	(x)
;	mpu6050.c: 138: I2C_FREQR = 16;                     //  Set the internal clock frequency (MHz).
	ldw	x, #0x5212
	ld	a, #0x10
	ld	(x), a
;	mpu6050.c: 139: UNSET (I2C_CCRH, I2C_FS);           //  I2C running is standard mode.
	bres	0x521c, #7
;	mpu6050.c: 141: I2C_CCRL = 0xa0;                    //  SCL clock speed is 50 kHz.
	ldw	x, #0x521b
	ld	a, #0xa0
	ld	(x), a
;	mpu6050.c: 143: I2C_CCRH &= 0x00;	// Clears lower 4 bits "CCR"
	ldw	x, #0x521c
	clr	(x)
;	mpu6050.c: 147: UNSET (I2C_OARH, I2C_ADDMODE);      //  7 bit address mode.
	bres	0x5214, #7
;	mpu6050.c: 148: SET (I2C_OARH, I2C_ADDCONF);        //  Docs say this must always be 1.
	ldw	x, #0x5214
	ld	a, (x)
	or	a, #0x40
	ld	(x), a
;	mpu6050.c: 152: I2C_TRISER = 17;
	ldw	x, #0x521d
	ld	a, #0x11
	ld	(x), a
;	mpu6050.c: 160: I2C_CR1 = I2C_PE;	// Enables port
	ldw	x, #0x5210
	ld	a, #0x01
	ld	(x), a
	ret
;	mpu6050.c: 166: void InitializeUART() {
;	-----------------------------------------
;	 function InitializeUART
;	-----------------------------------------
_InitializeUART:
;	mpu6050.c: 176: UART1_CR1 = 0;
	ldw	x, #0x5234
	clr	(x)
;	mpu6050.c: 177: UART1_CR2 = 0;
	ldw	x, #0x5235
	clr	(x)
;	mpu6050.c: 178: UART1_CR4 = 0;
	ldw	x, #0x5237
	clr	(x)
;	mpu6050.c: 179: UART1_CR3 = 0;
	ldw	x, #0x5236
	clr	(x)
;	mpu6050.c: 180: UART1_CR5 = 0;
	ldw	x, #0x5238
	clr	(x)
;	mpu6050.c: 181: UART1_GTR = 0;
	ldw	x, #0x5239
	clr	(x)
;	mpu6050.c: 182: UART1_PSCR = 0;
	ldw	x, #0x523a
	clr	(x)
;	mpu6050.c: 186: UNSET (UART1_CR1, CR1_M);        //  8 Data bits.
	ldw	x, #0x5234
	ld	a, (x)
	and	a, #0xef
	ld	(x), a
;	mpu6050.c: 187: UNSET (UART1_CR1, CR1_PCEN);     //  Disable parity.
	ldw	x, #0x5234
	ld	a, (x)
	and	a, #0xfb
	ld	(x), a
;	mpu6050.c: 188: UNSET (UART1_CR3, CR3_STOPH);    //  1 stop bit.
	ldw	x, #0x5236
	ld	a, (x)
	and	a, #0xdf
	ld	(x), a
;	mpu6050.c: 189: UNSET (UART1_CR3, CR3_STOPL);    //  1 stop bit.
	ldw	x, #0x5236
	ld	a, (x)
	and	a, #0xef
	ld	(x), a
;	mpu6050.c: 190: UART1_BRR2 = 0x0a;      //  Set the baud rate registers to 115200 baud
	ldw	x, #0x5233
	ld	a, #0x0a
	ld	(x), a
;	mpu6050.c: 191: UART1_BRR1 = 0x08;      //  based upon a 16 MHz system clock.
	ldw	x, #0x5232
	ld	a, #0x08
	ld	(x), a
;	mpu6050.c: 195: UNSET (UART1_CR2, CR2_TEN);      //  Disable transmit.
	ldw	x, #0x5235
	ld	a, (x)
	and	a, #0xf7
	ld	(x), a
;	mpu6050.c: 196: UNSET (UART1_CR2, CR2_REN);      //  Disable receive.
	ldw	x, #0x5235
	ld	a, (x)
	and	a, #0xfb
	ld	(x), a
;	mpu6050.c: 200: SET (UART1_CR3, CR3_CPOL);
	ldw	x, #0x5236
	ld	a, (x)
	or	a, #0x04
	ld	(x), a
;	mpu6050.c: 201: SET (UART1_CR3, CR3_CPHA);
	ldw	x, #0x5236
	ld	a, (x)
	or	a, #0x02
	ld	(x), a
;	mpu6050.c: 202: SET (UART1_CR3, CR3_LBCL);
	bset	0x5236, #0
;	mpu6050.c: 206: SET (UART1_CR2, CR2_TEN);
	ldw	x, #0x5235
	ld	a, (x)
	or	a, #0x08
	ld	(x), a
;	mpu6050.c: 207: SET (UART1_CR2, CR2_REN);
	ldw	x, #0x5235
	ld	a, (x)
	or	a, #0x04
	ld	(x), a
;	mpu6050.c: 208: UART1_CR3 = CR3_CLKEN;
	ldw	x, #0x5236
	ld	a, #0x08
	ld	(x), a
	ret
;	mpu6050.c: 236: void tm1637Init(void)
;	-----------------------------------------
;	 function tm1637Init
;	-----------------------------------------
_tm1637Init:
;	mpu6050.c: 238: tm1637SetBrightness(8);
	push	#0x08
	call	_tm1637SetBrightness
	pop	a
	ret
;	mpu6050.c: 243: void tm1637DisplayDecimal(long TT,unsigned int displaySeparator)
;	-----------------------------------------
;	 function tm1637DisplayDecimal
;	-----------------------------------------
_tm1637DisplayDecimal:
	sub	sp, #19
;	mpu6050.c: 245: unsigned int v = TT & 0x0000FFFF;
	ld	a, (0x19, sp)
	ld	xl, a
	ld	a, (0x18, sp)
	ld	xh, a
	clr	(0x0f, sp)
	clr	a
	ldw	(0x05, sp), x
;	mpu6050.c: 251: for (ii = 0; ii < 4; ++ii) {
	ldw	x, sp
	incw	x
	ldw	(0x12, sp), x
	ldw	x, #_segmentMap+0
	ldw	(0x0b, sp), x
	clrw	y
00106$:
;	mpu6050.c: 252: digitArr[ii] = segmentMap[v % 10];
	ldw	x, y
	addw	x, (0x12, sp)
	ldw	(0x09, sp), x
	pushw	y
	ldw	x, (0x07, sp)
	ldw	y, #0x000a
	divw	x, y
	ldw	x, y
	popw	y
	addw	x, (0x0b, sp)
	ld	a, (x)
	ldw	x, (0x09, sp)
	ld	(x), a
;	mpu6050.c: 253: if (ii == 2 && displaySeparator) {
	cpw	y, #0x0002
	jrne	00102$
	ldw	x, (0x1a, sp)
	jreq	00102$
;	mpu6050.c: 254: digitArr[ii] |= 1 << 7;
	ldw	x, (0x09, sp)
	ld	a, (x)
	or	a, #0x80
	ldw	x, (0x09, sp)
	ld	(x), a
00102$:
;	mpu6050.c: 256: v /= 10;
	pushw	y
	ldw	x, (0x07, sp)
	ldw	y, #0x000a
	divw	x, y
	popw	y
	ldw	(0x05, sp), x
;	mpu6050.c: 251: for (ii = 0; ii < 4; ++ii) {
	incw	y
	cpw	y, #0x0004
	jrc	00106$
;	mpu6050.c: 259: _tm1637Start();
	call	__tm1637Start
;	mpu6050.c: 260: _tm1637WriteByte(0x40);
	push	#0x40
	call	__tm1637WriteByte
	pop	a
;	mpu6050.c: 261: _tm1637ReadResult();
	call	__tm1637ReadResult
;	mpu6050.c: 262: _tm1637Stop();
	call	__tm1637Stop
;	mpu6050.c: 264: _tm1637Start();
	call	__tm1637Start
;	mpu6050.c: 265: _tm1637WriteByte(0xc0);
	push	#0xc0
	call	__tm1637WriteByte
	pop	a
;	mpu6050.c: 266: _tm1637ReadResult();
	call	__tm1637ReadResult
;	mpu6050.c: 268: for (ii = 0; ii < 4; ++ii) {
	clrw	x
	ldw	(0x07, sp), x
00108$:
;	mpu6050.c: 269: _tm1637WriteByte(digitArr[3 - ii]);
	ld	a, (0x08, sp)
	ld	(0x0d, sp), a
	ld	a, #0x03
	sub	a, (0x0d, sp)
	clrw	x
	ld	xl, a
	addw	x, (0x12, sp)
	ld	a, (x)
	push	a
	call	__tm1637WriteByte
	pop	a
;	mpu6050.c: 270: _tm1637ReadResult();
	call	__tm1637ReadResult
;	mpu6050.c: 268: for (ii = 0; ii < 4; ++ii) {
	ldw	x, (0x07, sp)
	incw	x
	ldw	(0x07, sp), x
	ldw	x, (0x07, sp)
	cpw	x, #0x0004
	jrc	00108$
;	mpu6050.c: 273: _tm1637Stop();
	call	__tm1637Stop
	addw	sp, #19
	ret
;	mpu6050.c: 278: void tm1637SetBrightness(char brightness)
;	-----------------------------------------
;	 function tm1637SetBrightness
;	-----------------------------------------
_tm1637SetBrightness:
;	mpu6050.c: 285: _tm1637Start();
	call	__tm1637Start
;	mpu6050.c: 286: _tm1637WriteByte(0x87 + brightness);
	ld	a, (0x03, sp)
	add	a, #0x87
	push	a
	call	__tm1637WriteByte
	pop	a
;	mpu6050.c: 287: _tm1637ReadResult();
	call	__tm1637ReadResult
;	mpu6050.c: 288: _tm1637Stop();
	jp	__tm1637Stop
;	mpu6050.c: 291: void _tm1637Start(void)
;	-----------------------------------------
;	 function _tm1637Start
;	-----------------------------------------
__tm1637Start:
;	mpu6050.c: 293: _tm1637ClkHigh();
	call	__tm1637ClkHigh
;	mpu6050.c: 294: _tm1637DioHigh();
	call	__tm1637DioHigh
;	mpu6050.c: 295: delay(5);
	push	#0x05
	push	#0x00
	call	_delay
	addw	sp, #2
;	mpu6050.c: 296: _tm1637DioLow();
	jp	__tm1637DioLow
;	mpu6050.c: 299: void _tm1637Stop(void)
;	-----------------------------------------
;	 function _tm1637Stop
;	-----------------------------------------
__tm1637Stop:
;	mpu6050.c: 301: _tm1637ClkLow();
	call	__tm1637ClkLow
;	mpu6050.c: 302: delay(5);
	push	#0x05
	push	#0x00
	call	_delay
	addw	sp, #2
;	mpu6050.c: 303: _tm1637DioLow();
	call	__tm1637DioLow
;	mpu6050.c: 304: delay(5);
	push	#0x05
	push	#0x00
	call	_delay
	addw	sp, #2
;	mpu6050.c: 305: _tm1637ClkHigh();
	call	__tm1637ClkHigh
;	mpu6050.c: 306: delay(5);
	push	#0x05
	push	#0x00
	call	_delay
	addw	sp, #2
;	mpu6050.c: 307: _tm1637DioHigh();
	jp	__tm1637DioHigh
;	mpu6050.c: 310: void _tm1637ReadResult(void)
;	-----------------------------------------
;	 function _tm1637ReadResult
;	-----------------------------------------
__tm1637ReadResult:
;	mpu6050.c: 312: _tm1637ClkLow();
	call	__tm1637ClkLow
;	mpu6050.c: 313: delay(5);
	push	#0x05
	push	#0x00
	call	_delay
	addw	sp, #2
;	mpu6050.c: 315: _tm1637ClkHigh();
	call	__tm1637ClkHigh
;	mpu6050.c: 316: delay(5);
	push	#0x05
	push	#0x00
	call	_delay
	addw	sp, #2
;	mpu6050.c: 317: _tm1637ClkLow();
	jp	__tm1637ClkLow
;	mpu6050.c: 320: void _tm1637WriteByte(unsigned char b)
;	-----------------------------------------
;	 function _tm1637WriteByte
;	-----------------------------------------
__tm1637WriteByte:
	sub	sp, #2
;	mpu6050.c: 322: for (ii = 0; ii < 8; ++ii) {
	clrw	x
	ldw	(0x01, sp), x
00105$:
;	mpu6050.c: 323: _tm1637ClkLow();
	call	__tm1637ClkLow
;	mpu6050.c: 324: if (b & 0x01) {
	ld	a, (0x05, sp)
	srl	a
	jrnc	00102$
;	mpu6050.c: 325: _tm1637DioHigh();
	call	__tm1637DioHigh
	jra	00103$
00102$:
;	mpu6050.c: 328: _tm1637DioLow();
	call	__tm1637DioLow
00103$:
;	mpu6050.c: 330: delay(15);
	push	#0x0f
	push	#0x00
	call	_delay
	addw	sp, #2
;	mpu6050.c: 331: b >>= 1;
	ld	a, (0x05, sp)
	srl	a
	ld	(0x05, sp), a
;	mpu6050.c: 332: _tm1637ClkHigh();
	call	__tm1637ClkHigh
;	mpu6050.c: 333: delay(15);
	push	#0x0f
	push	#0x00
	call	_delay
	addw	sp, #2
;	mpu6050.c: 322: for (ii = 0; ii < 8; ++ii) {
	ldw	x, (0x01, sp)
	incw	x
	ldw	(0x01, sp), x
	ldw	x, (0x01, sp)
	cpw	x, #0x0008
	jrslt	00105$
	addw	sp, #2
	ret
;	mpu6050.c: 339: void _tm1637ClkHigh(void)
;	-----------------------------------------
;	 function _tm1637ClkHigh
;	-----------------------------------------
__tm1637ClkHigh:
;	mpu6050.c: 344: PD_ODR |= 1 << 2;
	ldw	x, #0x500f
	ld	a, (x)
	or	a, #0x04
	ld	(x), a
	ret
;	mpu6050.c: 347: void _tm1637ClkLow(void)
;	-----------------------------------------
;	 function _tm1637ClkLow
;	-----------------------------------------
__tm1637ClkLow:
;	mpu6050.c: 351: PD_ODR &= ~(1 << 2);
	ldw	x, #0x500f
	ld	a, (x)
	and	a, #0xfb
	ld	(x), a
	ret
;	mpu6050.c: 357: void _tm1637DioHigh(void)
;	-----------------------------------------
;	 function _tm1637DioHigh
;	-----------------------------------------
__tm1637DioHigh:
;	mpu6050.c: 361: PD_ODR |= 1 << 3;
	ldw	x, #0x500f
	ld	a, (x)
	or	a, #0x08
	ld	(x), a
	ret
;	mpu6050.c: 365: void _tm1637DioLow(void)
;	-----------------------------------------
;	 function _tm1637DioLow
;	-----------------------------------------
__tm1637DioLow:
;	mpu6050.c: 367: PD_ODR &= ~(1 << 3);
	ldw	x, #0x500f
	ld	a, (x)
	and	a, #0xf7
	ld	(x), a
	ret
;	mpu6050.c: 378: void initMPU6050(){
;	-----------------------------------------
;	 function initMPU6050
;	-----------------------------------------
_initMPU6050:
;	mpu6050.c: 380: i2c_set_start_ack();
	call	_i2c_set_start_ack
;	mpu6050.c: 381: i2c_send_address (MPU6050_ADDR, I2C_WRITE);
	push	#0x00
	push	#0x68
	call	_i2c_send_address
	addw	sp, #2
;	mpu6050.c: 382: i2c_send_reg(0x6B);
	push	#0x6b
	call	_i2c_send_reg
	pop	a
;	mpu6050.c: 383: i2c_send_reg(0x80);
	push	#0x80
	call	_i2c_send_reg
	pop	a
;	mpu6050.c: 384: i2c_set_stop();
	call	_i2c_set_stop
;	mpu6050.c: 385: delay(100);
	push	#0x64
	push	#0x00
	call	_delay
	addw	sp, #2
;	mpu6050.c: 386: i2c_set_start_ack();
	call	_i2c_set_start_ack
;	mpu6050.c: 387: i2c_send_address (MPU6050_ADDR, I2C_WRITE);
	push	#0x00
	push	#0x68
	call	_i2c_send_address
	addw	sp, #2
;	mpu6050.c: 388: i2c_send_reg(0x6B);
	push	#0x6b
	call	_i2c_send_reg
	pop	a
;	mpu6050.c: 389: i2c_send_reg(0x00);
	push	#0x00
	call	_i2c_send_reg
	pop	a
;	mpu6050.c: 390: i2c_set_stop();
	call	_i2c_set_stop
;	mpu6050.c: 391: delay(100);
	push	#0x64
	push	#0x00
	call	_delay
	addw	sp, #2
;	mpu6050.c: 392: i2c_set_start_ack();
	call	_i2c_set_start_ack
;	mpu6050.c: 393: i2c_send_address (MPU6050_ADDR, I2C_WRITE);
	push	#0x00
	push	#0x68
	call	_i2c_send_address
	addw	sp, #2
;	mpu6050.c: 394: i2c_send_reg(0x1A);
	push	#0x1a
	call	_i2c_send_reg
	pop	a
;	mpu6050.c: 395: i2c_send_reg(0x01);
	push	#0x01
	call	_i2c_send_reg
	pop	a
;	mpu6050.c: 396: i2c_set_stop();
	call	_i2c_set_stop
;	mpu6050.c: 397: delay(100);
	push	#0x64
	push	#0x00
	call	_delay
	addw	sp, #2
;	mpu6050.c: 398: i2c_set_start_ack();
	call	_i2c_set_start_ack
;	mpu6050.c: 399: i2c_send_address (MPU6050_ADDR, I2C_WRITE);
	push	#0x00
	push	#0x68
	call	_i2c_send_address
	addw	sp, #2
;	mpu6050.c: 400: i2c_send_reg(0x1B);
	push	#0x1b
	call	_i2c_send_reg
	pop	a
;	mpu6050.c: 401: i2c_send_reg(0x01);
	push	#0x01
	call	_i2c_send_reg
	pop	a
;	mpu6050.c: 402: i2c_set_stop();
	jp	_i2c_set_stop
;	mpu6050.c: 435: unsigned int getMPU6050(){
;	-----------------------------------------
;	 function getMPU6050
;	-----------------------------------------
_getMPU6050:
	sub	sp, #6
;	mpu6050.c: 439: xh = i2c_read_register (MPU6050_ADDR, 0x43);
	push	#0x43
	push	#0x68
	call	_i2c_read_register
	addw	sp, #2
	ld	(0x02, sp), a
;	mpu6050.c: 440: UARTPrintF("read 1 \n\r");
	ldw	x, #___str_0+0
	pushw	x
	call	_UARTPrintF
	addw	sp, #2
;	mpu6050.c: 441: xl = i2c_read_register (MPU6050_ADDR, 0x44);
	push	#0x44
	push	#0x68
	call	_i2c_read_register
	addw	sp, #2
	ld	(0x01, sp), a
;	mpu6050.c: 442: UARTPrintF("read 2 \n\r");
	ldw	x, #___str_1+0
	pushw	x
	call	_UARTPrintF
	addw	sp, #2
;	mpu6050.c: 443: xx = xh << 8 | xl;
	ld	a, (0x02, sp)
	ld	(0x06, sp), a
	clr	(0x05, sp)
	ldw	x, (0x05, sp)
	sllw	x
	sllw	x
	sllw	x
	sllw	x
	sllw	x
	sllw	x
	sllw	x
	sllw	x
	ld	a, (0x01, sp)
	ld	(0x04, sp), a
	clr	(0x03, sp)
	ld	a, xl
	or	a, (0x04, sp)
	ld	xl, a
	ld	a, xh
	or	a, (0x03, sp)
	ld	xh, a
;	mpu6050.c: 444: return(xx);
	addw	sp, #6
	ret
;	mpu6050.c: 448: int main () {
;	-----------------------------------------
;	 function main
;	-----------------------------------------
_main:
	sub	sp, #22
;	mpu6050.c: 455: InitializeSystemClock();
	call	_InitializeSystemClock
;	mpu6050.c: 458: PD_DDR = (1 << 3) | (1 << 2); // output mode
	ldw	x, #0x5011
	ld	a, #0x0c
	ld	(x), a
;	mpu6050.c: 459: PB_DDR = (0 << 4);
	ldw	x, #0x5007
	clr	(x)
;	mpu6050.c: 460: PB_DDR = (0 << 5);
	ldw	x, #0x5007
	clr	(x)
;	mpu6050.c: 464: PD_CR2 = (1 << 3) | (1 << 2); // up to 10MHz speed
	ldw	x, #0x5013
	ld	a, #0x0c
	ld	(x), a
;	mpu6050.c: 465: PD_CR1 = 0; // push-pull
	ldw	x, #0x5012
	clr	(x)
;	mpu6050.c: 466: PD_CR2 = 0; // up to 10MHz speed
	ldw	x, #0x5013
	clr	(x)
;	mpu6050.c: 467: tm1637Init();
	call	_tm1637Init
;	mpu6050.c: 469: InitializeUART();
	call	_InitializeUART
;	mpu6050.c: 470: UARTPrintF("uart initialised \n\r");
	ldw	x, #___str_2+0
	pushw	x
	call	_UARTPrintF
	addw	sp, #2
;	mpu6050.c: 471: InitializeI2C();
	call	_InitializeI2C
;	mpu6050.c: 472: delay(200);
	push	#0xc8
	push	#0x00
	call	_delay
	addw	sp, #2
;	mpu6050.c: 473: UARTPrintF("testing 0 \n\r");
	ldw	x, #___str_3+0
	pushw	x
	call	_UARTPrintF
	addw	sp, #2
;	mpu6050.c: 474: initMPU6050();
	call	_initMPU6050
;	mpu6050.c: 475: delay(200);
	push	#0xc8
	push	#0x00
	call	_delay
	addw	sp, #2
;	mpu6050.c: 478: UARTPrintF("testing 1 \n\r");
	ldw	x, #___str_4+0
	pushw	x
	call	_UARTPrintF
	addw	sp, #2
;	mpu6050.c: 479: while (1) {
00114$:
;	mpu6050.c: 480: objTemp = getMPU6050();
	call	_getMPU6050
	pushw	x
	call	___uint2fs
	addw	sp, #2
	ldw	(0x05, sp), x
	ldw	(0x03, sp), y
;	mpu6050.c: 484: while (objTemp > 1000) {
	clrw	x
	ldw	(0x0b, sp), x
00101$:
	clrw	x
	pushw	x
	push	#0x7a
	push	#0x44
	ldw	x, (0x09, sp)
	pushw	x
	ldw	x, (0x09, sp)
	pushw	x
	call	___fsgt
	addw	sp, #8
	tnz	a
	jreq	00127$
;	mpu6050.c: 485: vierde+=1;
	ldw	x, (0x0b, sp)
	incw	x
	ldw	(0x0b, sp), x
;	mpu6050.c: 486: objTemp-=1000;
	clrw	x
	pushw	x
	push	#0x7a
	push	#0x44
	ldw	x, (0x09, sp)
	pushw	x
	ldw	x, (0x09, sp)
	pushw	x
	call	___fssub
	addw	sp, #8
	ldw	(0x05, sp), x
	ldw	(0x03, sp), y
	jra	00101$
;	mpu6050.c: 488: while (objTemp > 100) {
00127$:
	ldw	y, (0x0b, sp)
	ldw	(0x13, sp), y
	clrw	x
	ldw	(0x07, sp), x
00104$:
	clrw	x
	pushw	x
	push	#0xc8
	push	#0x42
	ldw	x, (0x09, sp)
	pushw	x
	ldw	x, (0x09, sp)
	pushw	x
	call	___fsgt
	addw	sp, #8
	tnz	a
	jreq	00128$
;	mpu6050.c: 489: derde+=1;
	ldw	x, (0x07, sp)
	incw	x
	ldw	(0x07, sp), x
;	mpu6050.c: 490: objTemp-=100;
	clrw	x
	pushw	x
	push	#0xc8
	push	#0x42
	ldw	x, (0x09, sp)
	pushw	x
	ldw	x, (0x09, sp)
	pushw	x
	call	___fssub
	addw	sp, #8
	ldw	(0x05, sp), x
	ldw	(0x03, sp), y
	jra	00104$
;	mpu6050.c: 492: while (objTemp > 10) {
00128$:
	ldw	y, (0x07, sp)
	ldw	(0x11, sp), y
	clrw	x
	ldw	(0x09, sp), x
00107$:
	clrw	x
	pushw	x
	push	#0x20
	push	#0x41
	ldw	x, (0x09, sp)
	pushw	x
	ldw	x, (0x09, sp)
	pushw	x
	call	___fsgt
	addw	sp, #8
	tnz	a
	jreq	00129$
;	mpu6050.c: 493: tweede+=1;
	ldw	x, (0x09, sp)
	incw	x
	ldw	(0x09, sp), x
;	mpu6050.c: 494: objTemp-=10;
	clrw	x
	pushw	x
	push	#0x20
	push	#0x41
	ldw	x, (0x09, sp)
	pushw	x
	ldw	x, (0x09, sp)
	pushw	x
	call	___fssub
	addw	sp, #8
	ldw	(0x05, sp), x
	ldw	(0x03, sp), y
	jra	00107$
;	mpu6050.c: 496: while (objTemp > 0)
00129$:
	ldw	y, (0x09, sp)
	ldw	(0x0f, sp), y
	clrw	x
	ldw	(0x01, sp), x
00110$:
	clrw	x
	pushw	x
	clrw	x
	pushw	x
	ldw	x, (0x09, sp)
	pushw	x
	ldw	x, (0x09, sp)
	pushw	x
	call	___fsgt
	addw	sp, #8
	tnz	a
	jreq	00112$
;	mpu6050.c: 498: eerste+=1;
	ldw	x, (0x01, sp)
	incw	x
	ldw	(0x01, sp), x
;	mpu6050.c: 499: objTemp-=1;
	clrw	x
	pushw	x
	push	#0x80
	push	#0x3f
	ldw	x, (0x09, sp)
	pushw	x
	ldw	x, (0x09, sp)
	pushw	x
	call	___fssub
	addw	sp, #8
	ldw	(0x05, sp), x
	ldw	(0x03, sp), y
	jra	00110$
00112$:
;	mpu6050.c: 510: utemp=vierde*1000+derde*100+tweede*10+eerste;
	ldw	x, (0x13, sp)
	pushw	x
	push	#0xe8
	push	#0x03
	call	__mulint
	addw	sp, #4
	ldw	(0x0d, sp), x
	ldw	x, (0x11, sp)
	pushw	x
	push	#0x64
	push	#0x00
	call	__mulint
	addw	sp, #4
	addw	x, (0x0d, sp)
	ldw	(0x15, sp), x
	ldw	x, (0x0f, sp)
	pushw	x
	push	#0x0a
	push	#0x00
	call	__mulint
	addw	sp, #4
	addw	x, (0x15, sp)
	addw	x, (0x01, sp)
	clrw	y
	tnzw	x
	jrpl	00162$
	decw	y
00162$:
;	mpu6050.c: 513: tm1637DisplayDecimal(utemp, 1); // eg 37:12
	push	#0x01
	push	#0x00
	pushw	x
	pushw	y
	call	_tm1637DisplayDecimal
	addw	sp, #6
;	mpu6050.c: 517: delayTenMicro();
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
	.ascii "read 1 "
	.db 0x0A
	.db 0x0D
	.db 0x00
___str_1:
	.ascii "read 2 "
	.db 0x0A
	.db 0x0D
	.db 0x00
___str_2:
	.ascii "uart initialised "
	.db 0x0A
	.db 0x0D
	.db 0x00
___str_3:
	.ascii "testing 0 "
	.db 0x0A
	.db 0x0D
	.db 0x00
___str_4:
	.ascii "testing 1 "
	.db 0x0A
	.db 0x0D
	.db 0x00
	.area INITIALIZER
	.area CABS (ABS)
