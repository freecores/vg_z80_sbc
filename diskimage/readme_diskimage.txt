To create a disk image:

1. Convert a 275-byte sector VGI image to 512-byte sectors:
vgi2flash vgboot.vgi vgboot.bin

2. Convert .bin file to Intel Hex (I used Needham's EMP Device Programming Software.)

3. Convert to MCS file, use 'packihx':
packihx vgboot.hex > vgboot.mcs

4. Program vgboot.mcs to FLASH using the s3esk_picoblaze_nor_flash_programmer from Xilinx.
This will take long time (~1 hour) and will end with address 199FF0.

NOTE: If you run into a problem where the Hyperterminal hangs during
the 'P' command, power-cycle your S3E board, re-load the flash
programmer, and try again.

exz80all.com:
adc,sbc hl,<bc,de,hl,sp> 71680 cycles, CRC expected: f39089a0, found 8e4e5173
add hl,<bc,de,hl,sp> 35840 cycles, OK
add ix,<bc,de,ix,sp> 35840 cycles, OK
add iy,<bc,de,iy,sp> 35840 cycles, OK
aluop a,nn 45056 cycles, CRC expected: 48799360 found bf127979
aluop a,<b,c,d,e,h,l,(hl),a> 753664 cycles, CRC expected: fe43b016 found e8a9643b
aluop a,<ixh,ixl,iyh,iyl> 376832 cycles 

bit n,(<ix,iy>+1) Ok
bit n,(<ix,iy>-1) Ok
inc/dec sp OK