Available Tests

Test name                       Purpose
test_single_write     Directed NONSEQ write transaction
test_single_read      Directed NONSEQ read transaction
test_idle_busy        Verifies cyc_o/stb_o de-assert on IDLE/BUSY
test_no_hsel          Verifies cyc_o=0 when hsel=0
test_wb_stallWB       slave stalls (ack_i=0) then acknowledges
test_rw_same_addr     Back-to-back write+read to same address
test_random200        transaction random sweep for coverage


Code Coverage:

Enabled by -cm line+cond+fsm+tgl+branch (VCS) or equivalent.
SVA cover property directives add targeted structural coverage for:

NONSEQ write observed
NONSEQ read observed
WB slave stall (ack_i=0 while stb=1)


SVA assert property directives catch protocol violations:

cyc_o must be 1 whenever stb_o is 1
hready == ack_i when stb_o is asserted
cyc_o deasserts cycle after hsel drops
stb_o deasserts cycle after IDLE transfer

