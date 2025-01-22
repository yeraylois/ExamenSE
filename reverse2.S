.syntax unified
.text
.cpu cortex-m0plus
.thumb_func
.global reverse_int2
.type reverse_int2, %function

reverse_int2:
    movs    r3, r0
    push    {lr}
    movs    r2, #32
    movs    r0, #0
    movs    r1, #0

.L2:
    lsls    r0, r0, #1
    lsrs    r3, r3, #1
    adcs    r0, r1
    subs    r2, #1
    bne     .L2
    pop     {pc}
