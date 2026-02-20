TEST=${1:-test_random}
SEED=${2:-1}

SRC_FILES="design.v ahb_wb_uvm_tb.sv"
UVM_HOME=${UVM_HOME:-/tools/uvm-1.2}

echo "================================================================"
echo " Running test: $TEST  (seed=$SEED)"
echo "================================================================"


if command -v xrun &>/dev/null; then
  xrun -sv -uvm -uvmhome CDNS-1.2                          \
       -access +rwc                                         \
       -coverage all                                        \
       -covfile coverage.ccf                                \
       +UVM_TESTNAME=$TEST                                  \
       +ntb_random_seed=$SEED                               \
       $SRC_FILES                                           \
       -log xrun_${TEST}.log
  echo "Coverage report: imc -load cov_work/scope/test"
