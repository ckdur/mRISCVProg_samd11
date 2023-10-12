.section .rodata
.global _binary_test5_dat_start
.global _binary_test5_dat_end
.balign 4
_binary_test5_dat_start:
.incbin "test5.dat"
_binary_test5_dat_end: