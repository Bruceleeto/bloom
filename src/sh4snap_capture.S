/* int sh4_snapshot_capture(uint32_t state[204]);   (KOS / GCC SH-4 ABI, call from privileged code)
 * Saves the CPU state into `state` (sim/snapshot/FORMAT.md) and returns 0. A run resumed from the
 * snapshot continues from the same return with 0 replaced by 1, like setjmp. Clobbers only
 * caller-saved registers (R0-R7, T); interrupts are masked while it runs. Uses only CPU
 * instructions and SH7091 control/array registers. */
.text
.balign 4
.global _sh4_snapshot_capture
_sh4_snapshot_capture:
 mov.l r0,@(0,r4)
 mov.l r1,@(4,r4)
 mov.l r2,@(8,r4)
 mov.l r3,@(12,r4)
 mov.l r4,@(16,r4)
 mov.l r5,@(20,r4)
 mov.l r6,@(24,r4)
 mov.l r7,@(28,r4)
 mov.l r8,@(32,r4)
 mov.l r9,@(36,r4)
 mov.l r10,@(40,r4)
 mov.l r11,@(44,r4)
 mov.l r12,@(48,r4)
 mov.l r13,@(52,r4)
 mov.l r14,@(56,r4)
 mov.l r15,@(60,r4)
 stc sr,r1                    /* SR at entry */
 mov r1,r0
 or #0xf0,r0
 ldc r0,sr                    /* IMASK = 15 */
 mov r4,r5
 add #64,r5
 mov.l .Lmagic,r0
 mov.l r0,@r5
 add #4,r5
 mov #1,r0
 mov.l r0,@r5
 add #4,r5
 sts pr,r0                    /* resume PC = return address */
 mov.l r0,@r5
 add #4,r5
 mov.l r1,@r5
 add #4,r5
.macro SAVE insn
 \insn
 mov.l r0,@r5
 add #4,r5
.endm
 SAVE "stc gbr,r0"
 SAVE "stc vbr,r0"
 SAVE "stc ssr,r0"
 SAVE "stc spc,r0"
 SAVE "stc sgr,r0"
 SAVE "stc dbr,r0"
 SAVE "sts pr,r0"
 SAVE "sts mach,r0"
 SAVE "sts macl,r0"
 SAVE "sts fpscr,r0"
 SAVE "sts fpul,r0"
 mov.l .Lccr,r2
 SAVE "mov.l @r2,r0"
 mov.l .Lmmucr,r2
 SAVE "mov.l @r2,r0"
 mov.l .Lpteh,r2
 SAVE "mov.l @r2,r0"
 mov.l .Lqacr0,r2
 SAVE "mov.l @r2,r0"
 mov.l .Lqacr1,r2
 SAVE "mov.l @r2,r0"
 SAVE "stc r0_bank,r0"
 SAVE "stc r1_bank,r0"
 SAVE "stc r2_bank,r0"
 SAVE "stc r3_bank,r0"
 SAVE "stc r4_bank,r0"
 SAVE "stc r5_bank,r0"
 SAVE "stc r6_bank,r0"
 SAVE "stc r7_bank,r0"
 /* FPU registers in the FR = 0 view, then XF */
 sts fpscr,r3
 mov.l .Lfpscr0,r0
 lds r0,fpscr
.irp n, 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15
 fmov.s fr\n,@r5
 add #4,r5
.endr
 frchg
.irp n, 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15
 fmov.s fr\n,@r5
 add #4,r5
.endr
 lds r3,fpscr
 /* UTLB address and data array 1, entries 0..63 */
 mov.l .Lutlba,r6
 mov.l .Lutlbd,r7
 mov #64,r2
 mov #1,r3
 shll8 r3
1:
 mov.l @r6,r0
 mov.l r0,@r5
 add #4,r5
 mov.l @r7,r0
 mov.l r0,@r5
 add #4,r5
 add r3,r6
 dt r2
 bf/s 1b
 add r3,r7
 mov #1,r0                    /* the value a resumed run returns */
 mov.l r0,@(0,r4)
 ldc r1,sr
 rts
 mov #0,r0
.balign 4
.Lmagic:  .long 0x53345353
.Lccr:    .long 0xff00001c
.Lmmucr:  .long 0xff000010
.Lpteh:   .long 0xff000000
.Lqacr0:  .long 0xff000038
.Lqacr1:  .long 0xff00003c
.Lfpscr0: .long 0x00040001
.Lutlba:  .long 0xf6000000
.Lutlbd:  .long 0xf7000000
