//=============================================================================
// AHB to Wishbone Bridge - Complete UVM Testbench
// Includes: Interface, Sequences, Driver, Monitor, Scoreboard,
//           Coverage Collector, Agent, Env, and Tests
//=============================================================================

`timescale 1ns/1ns

//=============================================================================
// PACKAGE - Imports and typedefs
//=============================================================================
package ahb2wb_pkg;
  import uvm_pkg::*;
  `include "uvm_macros.svh"

  //===========================================================================
  // Parameters
  //===========================================================================
  parameter int AWIDTH = 16;
  parameter int DWIDTH = 32;

  // AHB HTRANS encoding
  typedef enum logic [1:0] {
    IDLE   = 2'b00,
    BUSY   = 2'b01,
    NONSEQ = 2'b10,
    SEQ    = 2'b11
  } htrans_e;

  // AHB HBURST encoding
  typedef enum logic [2:0] {
    SINGLE  = 3'b000,
    INCR    = 3'b001,
    WRAP4   = 3'b010,
    INCR4   = 3'b011,
    WRAP8   = 3'b100,
    INCR8   = 3'b101,
    WRAP16  = 3'b110,
    INCR16  = 3'b111
  } hburst_e;

  // AHB HSIZE encoding
  typedef enum logic [2:0] {
    BYTE_SZ   = 3'b000,
    HALF_WORD = 3'b001,
    WORD      = 3'b010
  } hsize_e;

  // HRESP encoding
  typedef enum logic [1:0] {
    OKAY  = 2'b00,
    ERROR = 2'b01
  } hresp_e;

  //===========================================================================
  // AHB Sequence Item
  //===========================================================================
  class ahb_seq_item extends uvm_sequence_item;
    `uvm_object_utils_begin(ahb_seq_item)
      `uvm_field_int   (haddr,  UVM_ALL_ON)
      `uvm_field_int   (hwdata, UVM_ALL_ON)
      `uvm_field_int   (hwrite, UVM_ALL_ON)
      `uvm_field_enum  (htrans_e, htrans, UVM_ALL_ON)
      `uvm_field_enum  (hburst_e, hburst, UVM_ALL_ON)
      `uvm_field_enum  (hsize_e,  hsize,  UVM_ALL_ON)
      `uvm_field_int   (hsel,   UVM_ALL_ON)
      `uvm_field_int   (ack_i,  UVM_ALL_ON)
      `uvm_field_int   (dat_i,  UVM_ALL_ON)
      // Response fields (driven by monitor)
      `uvm_field_int   (hrdata, UVM_ALL_ON)
      `uvm_field_int   (hready, UVM_ALL_ON)
      `uvm_field_enum  (hresp_e, hresp, UVM_ALL_ON)
      `uvm_field_int   (cyc_o,  UVM_ALL_ON)
      `uvm_field_int   (stb_o,  UVM_ALL_ON)
      `uvm_field_int   (we_o,   UVM_ALL_ON)
      `uvm_field_int   (adr_o,  UVM_ALL_ON)
      `uvm_field_int   (dat_o,  UVM_ALL_ON)
    `uvm_object_utils_end

    // Stimulus fields
    rand logic [AWIDTH-1:0] haddr;
    rand logic [DWIDTH-1:0] hwdata;
    rand logic               hwrite;
    rand htrans_e            htrans;
    rand hburst_e            hburst;
    rand hsize_e             hsize;
    rand logic               hsel;
    rand logic               ack_i;
    rand logic [DWIDTH-1:0] dat_i;

    // Response/observation fields
    logic [DWIDTH-1:0] hrdata;
    logic               hready;
    hresp_e             hresp;
    logic               cyc_o;
    logic               stb_o;
    logic               we_o;
    logic [AWIDTH-1:0] adr_o;
    logic [DWIDTH-1:0] dat_o;

    // Constraints
    constraint c_burst_single { hburst == SINGLE; }  // DUT supports single only
    constraint c_trans_valid  { htrans inside {IDLE, BUSY, NONSEQ}; }
    constraint c_size_word    { hsize == WORD; }
    constraint c_hsel_active  { hsel == 1'b1; }
    constraint c_ack_normal   { ack_i dist {1'b1 := 80, 1'b0 := 20}; }

    function new(string name = "ahb_seq_item");
      super.new(name);
    endfunction
  endclass

  //===========================================================================
  // WB Slave Response Item (used internally by wb_slave)
  //===========================================================================
  class wb_resp_item extends uvm_sequence_item;
    `uvm_object_utils_begin(wb_resp_item)
      `uvm_field_int(ack,   UVM_ALL_ON)
      `uvm_field_int(dat_i, UVM_ALL_ON)
    `uvm_object_utils_end

    rand logic               ack;
    rand logic [DWIDTH-1:0] dat_i;

    constraint c_ack { ack dist {1'b1 := 90, 1'b0 := 10}; }

    function new(string name = "wb_resp_item");
      super.new(name);
    endfunction
  endclass

  //===========================================================================
  // BASE SEQUENCE
  //===========================================================================
  class ahb_base_seq extends uvm_sequence #(ahb_seq_item);
    `uvm_object_utils(ahb_base_seq)
    function new(string name = "ahb_base_seq");
      super.new(name);
    endfunction
  endclass

  //===========================================================================
  // SEQUENCE: Single Write
  //===========================================================================
  class ahb_single_write_seq extends ahb_base_seq;
    `uvm_object_utils(ahb_single_write_seq)

    rand logic [AWIDTH-1:0] addr;
    rand logic [DWIDTH-1:0] data;
    int unsigned num_beats = 1;

    function new(string name = "ahb_single_write_seq");
      super.new(name);
    endfunction

    task body();
      ahb_seq_item item;
      repeat(num_beats) begin
        item = ahb_seq_item::type_id::create("item");
        start_item(item);
        if (!item.randomize() with {
          htrans == NONSEQ;
          hburst == SINGLE;
          hwrite == 1'b1;
          hsel   == 1'b1;
          haddr  == addr;
          hwdata == data;
          ack_i  == 1'b1;
        }) `uvm_fatal("RAND", "Randomization failed")
        finish_item(item);
      end
    endtask
  endclass

  //===========================================================================
  // SEQUENCE: Single Read
  //===========================================================================
  class ahb_single_read_seq extends ahb_base_seq;
    `uvm_object_utils(ahb_single_read_seq)

    rand logic [AWIDTH-1:0] addr;
    rand logic [DWIDTH-1:0] slave_data;
    int unsigned num_beats = 1;

    function new(string name = "ahb_single_read_seq");
      super.new(name);
    endfunction

    task body();
      ahb_seq_item item;
      repeat(num_beats) begin
        item = ahb_seq_item::type_id::create("item");
        start_item(item);
        if (!item.randomize() with {
          htrans == NONSEQ;
          hburst == SINGLE;
          hwrite == 1'b0;
          hsel   == 1'b1;
          haddr  == addr;
          dat_i  == slave_data;
          ack_i  == 1'b1;
        }) `uvm_fatal("RAND", "Randomization failed")
        finish_item(item);
      end
    endtask
  endclass

  //===========================================================================
  // SEQUENCE: IDLE Transfer
  //===========================================================================
  class ahb_idle_seq extends ahb_base_seq;
    `uvm_object_utils(ahb_idle_seq)
    int unsigned num_beats = 4;
    function new(string name = "ahb_idle_seq");
      super.new(name);
    endfunction
    task body();
      ahb_seq_item item;
      repeat(num_beats) begin
        item = ahb_seq_item::type_id::create("item");
        start_item(item);
        if (!item.randomize() with {
          htrans == IDLE;
          hsel   == 1'b1;
          ack_i  == 1'b1;
        }) `uvm_fatal("RAND", "Randomization failed")
        finish_item(item);
      end
    endtask
  endclass

  //===========================================================================
  // SEQUENCE: BUSY Transfer
  //===========================================================================
  class ahb_busy_seq extends ahb_base_seq;
    `uvm_object_utils(ahb_busy_seq)
    int unsigned num_beats = 2;
    function new(string name = "ahb_busy_seq");
      super.new(name);
    endfunction
    task body();
      ahb_seq_item item;
      repeat(num_beats) begin
        item = ahb_seq_item::type_id::create("item");
        start_item(item);
        if (!item.randomize() with {
          htrans == BUSY;
          hsel   == 1'b1;
          ack_i  == 1'b1;
        }) `uvm_fatal("RAND", "Randomization failed")
        finish_item(item);
      end
    endtask
  endclass

  //===========================================================================
  // SEQUENCE: hsel deasserted
  //===========================================================================
  class ahb_no_sel_seq extends ahb_base_seq;
    `uvm_object_utils(ahb_no_sel_seq)
    function new(string name = "ahb_no_sel_seq");
      super.new(name);
    endfunction
    task body();
      ahb_seq_item item;
      item = ahb_seq_item::type_id::create("item");
      start_item(item);
      if (!item.randomize() with {
        hsel   == 1'b0;
        htrans == NONSEQ;
        ack_i  == 1'b1;
      }) `uvm_fatal("RAND", "Randomization failed")
      finish_item(item);
    endtask
  endclass

  //===========================================================================
  // SEQUENCE: WB slave stall (ack_i=0)
  //===========================================================================
  class ahb_wb_stall_seq extends ahb_base_seq;
    `uvm_object_utils(ahb_wb_stall_seq)
    function new(string name = "ahb_wb_stall_seq");
      super.new(name);
    endfunction
    task body();
      ahb_seq_item item;
      // Write with stall then ack
      item = ahb_seq_item::type_id::create("item");
      start_item(item);
      if (!item.randomize() with {
        htrans == NONSEQ;
        hburst == SINGLE;
        hwrite == 1'b1;
        hsel   == 1'b1;
        ack_i  == 1'b0;
      }) `uvm_fatal("RAND", "Randomization failed")
      finish_item(item);
      // Now ack
      item = ahb_seq_item::type_id::create("item");
      start_item(item);
      if (!item.randomize() with {
        htrans == NONSEQ;
        hburst == SINGLE;
        hwrite == 1'b1;
        hsel   == 1'b1;
        ack_i  == 1'b1;
      }) `uvm_fatal("RAND", "Randomization failed")
      finish_item(item);
    endtask
  endclass

  //===========================================================================
  // SEQUENCE: Random – full coverage sweep
  //===========================================================================
  class ahb_rand_seq extends ahb_base_seq;
    `uvm_object_utils(ahb_rand_seq)
    int unsigned num_txns = 50;
    function new(string name = "ahb_rand_seq");
      super.new(name);
    endfunction
    task body();
      ahb_seq_item item;
      repeat(num_txns) begin
        item = ahb_seq_item::type_id::create("item");
        start_item(item);
        // relax burst/trans constraints for coverage
        if (!item.randomize() with {
          hburst == SINGLE;
          htrans inside {IDLE, BUSY, NONSEQ};
          hsel   == 1'b1;
        }) `uvm_fatal("RAND", "Randomization failed")
        finish_item(item);
      end
    endtask
  endclass

  //===========================================================================
  // SEQUENCE: Mixed read/write to same address
  //===========================================================================
  class ahb_rw_same_addr_seq extends ahb_base_seq;
    `uvm_object_utils(ahb_rw_same_addr_seq)
    rand logic [AWIDTH-1:0] target_addr;
    function new(string name = "ahb_rw_same_addr_seq");
      super.new(name);
    endfunction
    task body();
      ahb_seq_item item;
      // Write
      item = ahb_seq_item::type_id::create("wr");
      start_item(item);
      if (!item.randomize() with {
        htrans  == NONSEQ; hburst == SINGLE;
        hwrite  == 1'b1; hsel == 1'b1;
        haddr   == target_addr; ack_i == 1'b1;
      }) `uvm_fatal("RAND","fail")
      finish_item(item);
      // Read
      item = ahb_seq_item::type_id::create("rd");
      start_item(item);
      if (!item.randomize() with {
        htrans  == NONSEQ; hburst == SINGLE;
        hwrite  == 1'b0; hsel == 1'b1;
        haddr   == target_addr; ack_i == 1'b1;
      }) `uvm_fatal("RAND","fail")
      finish_item(item);
    endtask
  endclass

  //===========================================================================
  // COVERAGE COLLECTOR
  //===========================================================================
  class ahb2wb_coverage extends uvm_subscriber #(ahb_seq_item);
    `uvm_component_utils(ahb2wb_coverage)

    ahb_seq_item item;

    // Functional coverage groups
    covergroup cg_ahb_trans;
      cp_htrans: coverpoint item.htrans {
        bins idle   = {IDLE};
        bins busy   = {BUSY};
        bins nonseq = {NONSEQ};
      }
      cp_hwrite: coverpoint item.hwrite {
        bins read  = {1'b0};
        bins write = {1'b1};
      }
      cp_hsel: coverpoint item.hsel {
        bins selected     = {1'b1};
        bins not_selected = {1'b0};
      }
      cp_hburst: coverpoint item.hburst {
        bins single = {SINGLE};
        bins other  = default;
      }
      cp_hsize: coverpoint item.hsize {
        bins byte_sz   = {BYTE_SZ};
        bins half_word = {HALF_WORD};
        bins word      = {WORD};
      }
      // Cross coverage
      cx_trans_write: cross cp_htrans, cp_hwrite;
      cx_trans_sel:   cross cp_htrans, cp_hsel;
      cx_write_sel:   cross cp_hwrite, cp_hsel;
    endgroup

    covergroup cg_wb_signals;
      cp_cyc: coverpoint item.cyc_o {
        bins active   = {1'b1};
        bins inactive = {1'b0};
      }
      cp_stb: coverpoint item.stb_o {
        bins active   = {1'b1};
        bins inactive = {1'b0};
      }
      cp_we: coverpoint item.we_o {
        bins write = {1'b1};
        bins read  = {1'b0};
      }
      cp_ack: coverpoint item.ack_i {
        bins ack_recv  = {1'b1};
        bins ack_stall = {1'b0};
      }
      cx_cyc_stb:    cross cp_cyc, cp_stb;
      cx_stb_we:     cross cp_stb, cp_we;
      cx_stb_ack:    cross cp_stb, cp_ack;
    endgroup

    covergroup cg_response;
      cp_hresp: coverpoint item.hresp {
        bins okay  = {OKAY};
        bins error = {ERROR};
      }
      cp_hready: coverpoint item.hready {
        bins ready     = {1'b1};
        bins not_ready = {1'b0};
      }
      cx_hresp_hready: cross cp_hresp, cp_hready;
    endgroup

    covergroup cg_address;
      cp_adr_lo: coverpoint item.adr_o[3:0] {
        bins lo[] = {[0:15]};
      }
      cp_adr_hi: coverpoint item.adr_o[AWIDTH-1:AWIDTH-4] {
        bins hi[] = {[0:15]};
      }
    endgroup

    covergroup cg_data_values;
      cp_dat_o_zero:    coverpoint (item.dat_o  == '0) { bins yes = {1}; bins no = {0}; }
      cp_dat_o_ones:    coverpoint (item.dat_o  == '1) { bins yes = {1}; bins no = {0}; }
      cp_hrdata_zero:   coverpoint (item.hrdata == '0) { bins yes = {1}; bins no = {0}; }
      cp_hrdata_ones:   coverpoint (item.hrdata == '1) { bins yes = {1}; bins no = {0}; }
    endgroup

    function new(string name, uvm_component parent);
      super.new(name, parent);
      cg_ahb_trans  = new();
      cg_wb_signals = new();
      cg_response   = new();
      cg_address    = new();
      cg_data_values = new();
    endfunction

    function void write(ahb_seq_item t);
      item = t;
      cg_ahb_trans.sample();
      cg_wb_signals.sample();
      cg_response.sample();
      cg_address.sample();
      cg_data_values.sample();
    endfunction

    function void report_phase(uvm_phase phase);
      `uvm_info("COV", $sformatf("AHB Trans coverage:  %.2f%%", cg_ahb_trans.get_coverage()),  UVM_LOW)
      `uvm_info("COV", $sformatf("WB Signals coverage: %.2f%%", cg_wb_signals.get_coverage()), UVM_LOW)
      `uvm_info("COV", $sformatf("Response coverage:   %.2f%%", cg_response.get_coverage()),   UVM_LOW)
      `uvm_info("COV", $sformatf("Address coverage:    %.2f%%", cg_address.get_coverage()),    UVM_LOW)
      `uvm_info("COV", $sformatf("Data Values cov:     %.2f%%", cg_data_values.get_coverage()),UVM_LOW)
    endfunction
  endclass

  //===========================================================================
  // SCOREBOARD
  //===========================================================================
  class ahb2wb_scoreboard extends uvm_scoreboard;
    `uvm_component_utils(ahb2wb_scoreboard)

    uvm_analysis_imp #(ahb_seq_item, ahb2wb_scoreboard) analysis_export;

    int unsigned pass_cnt = 0;
    int unsigned fail_cnt = 0;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      analysis_export = new("analysis_export", this);
    endfunction

    function void write(ahb_seq_item item);
      check_cyc_stb(item);
      check_write_path(item);
      check_read_path(item);
      check_no_sel(item);
      check_hresp(item);
    endfunction

    // When NONSEQ+hsel+hready → cyc_o and stb_o must be asserted
    function void check_cyc_stb(ahb_seq_item item);
      if (item.htrans == NONSEQ && item.hsel && item.cyc_o !== 1'b1) begin
        `uvm_error("SB", $sformatf("FAIL: NONSEQ but cyc_o=%0b at addr=0x%0h",
                                   item.cyc_o, item.haddr))
        fail_cnt++;
      end else if (item.htrans == NONSEQ && item.hsel && item.stb_o !== 1'b1) begin
        `uvm_error("SB", $sformatf("FAIL: NONSEQ but stb_o=%0b at addr=0x%0h",
                                   item.stb_o, item.haddr))
        fail_cnt++;
      end else if (item.htrans == NONSEQ && item.hsel) begin
        `uvm_info("SB", $sformatf("PASS: cyc_o=1 stb_o=1 for NONSEQ addr=0x%0h",
                                  item.haddr), UVM_HIGH)
        pass_cnt++;
      end
    endfunction

    // Write path: we_o must be 1, adr_o == haddr, dat_o == hwdata
    function void check_write_path(ahb_seq_item item);
      if (item.htrans == NONSEQ && item.hsel && item.hwrite) begin
        if (item.we_o !== 1'b1)
          `uvm_error("SB", $sformatf("FAIL: Write but we_o=%0b", item.we_o))
        if (item.adr_o !== item.haddr)
          `uvm_error("SB", $sformatf("FAIL: adr_o=0x%0h != haddr=0x%0h",
                                     item.adr_o, item.haddr))
        if (item.stb_o && item.ack_i && item.dat_o !== item.hwdata)
          `uvm_error("SB", $sformatf("FAIL: dat_o=0x%0h != hwdata=0x%0h",
                                     item.dat_o, item.hwdata))
        else begin
          pass_cnt++;
          `uvm_info("SB", "PASS: Write path check", UVM_HIGH)
        end
      end
    endfunction

    // Read path: we_o must be 0, hrdata == dat_i
    function void check_read_path(ahb_seq_item item);
      if (item.htrans == NONSEQ && item.hsel && !item.hwrite) begin
        if (item.we_o !== 1'b0)
          `uvm_error("SB", $sformatf("FAIL: Read but we_o=%0b", item.we_o))
        if (item.stb_o && item.ack_i && item.hrdata !== item.dat_i)
          `uvm_error("SB", $sformatf("FAIL: hrdata=0x%0h != dat_i=0x%0h",
                                     item.hrdata, item.dat_i))
        else begin
          pass_cnt++;
          `uvm_info("SB", "PASS: Read path check", UVM_HIGH)
        end
      end
    endfunction

    // When hsel=0 → cyc_o must be 0
    function void check_no_sel(ahb_seq_item item);
      if (!item.hsel && item.cyc_o !== 1'b0) begin
        `uvm_error("SB", $sformatf("FAIL: hsel=0 but cyc_o=%0b", item.cyc_o))
        fail_cnt++;
      end
    endfunction

    // IDLE transfer → hresp should be OKAY
    function void check_hresp(ahb_seq_item item);
      if (item.htrans == IDLE && item.hresp !== OKAY) begin
        `uvm_error("SB", "FAIL: IDLE transfer but hresp != OKAY")
        fail_cnt++;
      end
    endfunction

    function void report_phase(uvm_phase phase);
      `uvm_info("SB", $sformatf("Scoreboard: PASS=%0d  FAIL=%0d", pass_cnt, fail_cnt), UVM_LOW)
      if (fail_cnt > 0)
        `uvm_error("SB", "*** TESTBENCH: SOME CHECKS FAILED ***")
      else
        `uvm_info("SB",  "*** TESTBENCH: ALL CHECKS PASSED ***", UVM_LOW)
    endfunction
  endclass

  //===========================================================================
  // DRIVER
  //===========================================================================
  class ahb2wb_driver extends uvm_driver #(ahb_seq_item);
    `uvm_component_utils(ahb2wb_driver)

    virtual ahb2wb_if vif;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      if (!uvm_config_db #(virtual ahb2wb_if)::get(this, "", "vif", vif))
        `uvm_fatal("CFG", "Virtual interface not found in config_db")
    endfunction

    task run_phase(uvm_phase phase);
      ahb_seq_item item;
      // Initialise signals
      vif.hsel    <= 0; vif.htrans <= IDLE; vif.hburst <= SINGLE;
      vif.hsize   <= WORD; vif.hwrite <= 0;
      vif.haddr   <= 0; vif.hwdata  <= 0;
      vif.ack_i   <= 0; vif.dat_i   <= 0;
      @(posedge vif.hclk);
      forever begin
        seq_item_port.get_next_item(item);
        drive_item(item);
        seq_item_port.item_done();
      end
    endtask

    task drive_item(ahb_seq_item item);
      @(posedge vif.hclk);
      // AHB address phase
      vif.hsel    <= item.hsel;
      vif.htrans  <= item.htrans;
      vif.hburst  <= item.hburst;
      vif.hsize   <= item.hsize;
      vif.hwrite  <= item.hwrite;
      vif.haddr   <= item.haddr;
      // Wishbone slave response
      vif.ack_i   <= item.ack_i;
      vif.dat_i   <= item.dat_i;
      // AHB data phase (one cycle later)
      @(posedge vif.hclk);
      vif.hwdata  <= item.hwdata;
      // Wait for hready
      @(posedge vif.hclk);
    endtask
  endclass

  //===========================================================================
  // MONITOR
  //===========================================================================
  class ahb2wb_monitor extends uvm_monitor;
    `uvm_component_utils(ahb2wb_monitor)

    virtual ahb2wb_if vif;
    uvm_analysis_port #(ahb_seq_item) ap;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      ap = new("ap", this);
      if (!uvm_config_db #(virtual ahb2wb_if)::get(this, "", "vif", vif))
        `uvm_fatal("CFG", "Virtual interface not found in config_db")
    endfunction

    task run_phase(uvm_phase phase);
      ahb_seq_item item;
      forever begin
        @(posedge vif.hclk);
        // Sample after combinational settling
        #1;
        item = ahb_seq_item::type_id::create("mon_item");
        // AHB inputs
        item.htrans  = htrans_e'(vif.htrans);
        item.hburst  = hburst_e'(vif.hburst);
        item.hsize   = hsize_e'(vif.hsize);
        item.hwrite  = vif.hwrite;
        item.hsel    = vif.hsel;
        item.haddr   = vif.haddr;
        item.hwdata  = vif.hwdata;
        item.ack_i   = vif.ack_i;
        item.dat_i   = vif.dat_i;
        // DUT outputs
        item.hready  = vif.hready;
        item.hresp   = hresp_e'(vif.hresp);
        item.hrdata  = vif.hrdata;
        item.cyc_o   = vif.cyc_o;
        item.stb_o   = vif.stb_o;
        item.we_o    = vif.we_o;
        item.adr_o   = vif.adr_o;
        item.dat_o   = vif.dat_o;
        ap.write(item);
      end
    endtask
  endclass

  //===========================================================================
  // AGENT
  //===========================================================================
  class ahb2wb_agent extends uvm_agent;
    `uvm_component_utils(ahb2wb_agent)

    ahb2wb_driver  driver;
    ahb2wb_monitor monitor;
    uvm_sequencer #(ahb_seq_item) sequencer;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      monitor   = ahb2wb_monitor::type_id::create("monitor", this);
      if (get_is_active() == UVM_ACTIVE) begin
        driver    = ahb2wb_driver::type_id::create("driver", this);
        sequencer = uvm_sequencer #(ahb_seq_item)::type_id::create("sequencer", this);
      end
    endfunction

    function void connect_phase(uvm_phase phase);
      if (get_is_active() == UVM_ACTIVE)
        driver.seq_item_port.connect(sequencer.seq_item_export);
    endfunction
  endclass

  //===========================================================================
  // ENVIRONMENT
  //===========================================================================
  class ahb2wb_env extends uvm_env;
    `uvm_component_utils(ahb2wb_env)

    ahb2wb_agent      agent;
    ahb2wb_scoreboard scoreboard;
    ahb2wb_coverage   coverage;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      agent      = ahb2wb_agent::type_id::create("agent", this);
      scoreboard = ahb2wb_scoreboard::type_id::create("scoreboard", this);
      coverage   = ahb2wb_coverage::type_id::create("coverage", this);
    endfunction

    function void connect_phase(uvm_phase phase);
      agent.monitor.ap.connect(scoreboard.analysis_export);
      agent.monitor.ap.connect(coverage.analysis_export);
    endfunction
  endclass

  //===========================================================================
  // BASE TEST
  //===========================================================================
  class ahb2wb_base_test extends uvm_test;
    `uvm_component_utils(ahb2wb_base_test)

    ahb2wb_env env;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      env = ahb2wb_env::type_id::create("env", this);
    endfunction

    task run_phase(uvm_phase phase);
      phase.raise_objection(this);
      apply_reset();
      phase.drop_objection(this);
    endtask

    virtual task apply_reset();
      // Implemented by the top-level module
    endtask
  endclass

  //===========================================================================
  // TEST: Single Write
  //===========================================================================
  class test_single_write extends ahb2wb_base_test;
    `uvm_component_utils(test_single_write)
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    task run_phase(uvm_phase phase);
      ahb_single_write_seq seq;
      phase.raise_objection(this);
      seq = ahb_single_write_seq::type_id::create("seq");
      if (!seq.randomize() with { addr == 16'hABCD; data == 32'hDEAD_BEEF; })
        `uvm_fatal("RAND","fail")
      seq.start(env.agent.sequencer);
      #100;
      phase.drop_objection(this);
    endtask
  endclass

  //===========================================================================
  // TEST: Single Read
  //===========================================================================
  class test_single_read extends ahb2wb_base_test;
    `uvm_component_utils(test_single_read)
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    task run_phase(uvm_phase phase);
      ahb_single_read_seq seq;
      phase.raise_objection(this);
      seq = ahb_single_read_seq::type_id::create("seq");
      if (!seq.randomize() with { addr == 16'h1234; slave_data == 32'hCAFE_BABE; })
        `uvm_fatal("RAND","fail")
      seq.start(env.agent.sequencer);
      #100;
      phase.drop_objection(this);
    endtask
  endclass

  //===========================================================================
  // TEST: IDLE and BUSY transfer types
  //===========================================================================
  class test_idle_busy extends ahb2wb_base_test;
    `uvm_component_utils(test_idle_busy)
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    task run_phase(uvm_phase phase);
      ahb_idle_seq idle_seq;
      ahb_busy_seq busy_seq;
      phase.raise_objection(this);
      idle_seq = ahb_idle_seq::type_id::create("idle_seq");
      idle_seq.num_beats = 4;
      idle_seq.start(env.agent.sequencer);
      busy_seq = ahb_busy_seq::type_id::create("busy_seq");
      busy_seq.num_beats = 3;
      busy_seq.start(env.agent.sequencer);
      #100;
      phase.drop_objection(this);
    endtask
  endclass

  //===========================================================================
  // TEST: hsel deasserted
  //===========================================================================
  class test_no_hsel extends ahb2wb_base_test;
    `uvm_component_utils(test_no_hsel)
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    task run_phase(uvm_phase phase);
      ahb_no_sel_seq seq;
      phase.raise_objection(this);
      seq = ahb_no_sel_seq::type_id::create("seq");
      seq.start(env.agent.sequencer);
      #100;
      phase.drop_objection(this);
    endtask
  endclass

  //===========================================================================
  // TEST: WB slave stall
  //===========================================================================
  class test_wb_stall extends ahb2wb_base_test;
    `uvm_component_utils(test_wb_stall)
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    task run_phase(uvm_phase phase);
      ahb_wb_stall_seq seq;
      phase.raise_objection(this);
      seq = ahb_wb_stall_seq::type_id::create("seq");
      seq.start(env.agent.sequencer);
      #100;
      phase.drop_objection(this);
    endtask
  endclass

  //===========================================================================
  // TEST: Random – main coverage-closure test
  //===========================================================================
  class test_random extends ahb2wb_base_test;
    `uvm_component_utils(test_random)
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    task run_phase(uvm_phase phase);
      ahb_rand_seq seq;
      phase.raise_objection(this);
      seq = ahb_rand_seq::type_id::create("seq");
      seq.num_txns = 200;
      seq.start(env.agent.sequencer);
      #200;
      phase.drop_objection(this);
    endtask
  endclass

  //===========================================================================
  // TEST: Read/Write to same address
  //===========================================================================
  class test_rw_same_addr extends ahb2wb_base_test;
    `uvm_component_utils(test_rw_same_addr)
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    task run_phase(uvm_phase phase);
      ahb_rw_same_addr_seq seq;
      phase.raise_objection(this);
      repeat(10) begin
        seq = ahb_rw_same_addr_seq::type_id::create("seq");
        if (!seq.randomize()) `uvm_fatal("RAND","fail")
        seq.start(env.agent.sequencer);
      end
      #100;
      phase.drop_objection(this);
    endtask
  endclass

endpackage // ahb2wb_pkg

//=============================================================================
// INTERFACE
//=============================================================================
interface ahb2wb_if #(
  parameter AWIDTH = 16,
  parameter DWIDTH = 32
) (input logic hclk, input logic clk_i);

  // AHB signals
  logic               hresetn;
  logic [AWIDTH-1:0]  haddr;
  logic [1:0]         htrans;
  logic               hwrite;
  logic [2:0]         hsize;
  logic [2:0]         hburst;
  logic               hsel;
  logic [DWIDTH-1:0]  hwdata;
  logic [DWIDTH-1:0]  hrdata;
  logic [1:0]         hresp;
  logic               hready;

  // Wishbone signals
  logic [DWIDTH-1:0]  dat_i;
  logic               ack_i;
  logic               rst_i;
  logic [AWIDTH-1:0]  adr_o;
  logic [DWIDTH-1:0]  dat_o;
  logic               cyc_o;
  logic               we_o;
  logic               stb_o;

  // Clocking block for driver (AHB master side)
  clocking drv_cb @(posedge hclk);
    default input #1 output #1;
    output hresetn, haddr, htrans, hwrite, hsize, hburst, hsel, hwdata;
    output dat_i, ack_i, rst_i;
    input  hrdata, hresp, hready;
    input  adr_o, dat_o, cyc_o, we_o, stb_o;
  endclocking

  // Clocking block for monitor
  clocking mon_cb @(posedge hclk);
    default input #1;
    input hresetn, haddr, htrans, hwrite, hsize, hburst, hsel, hwdata;
    input dat_i, ack_i;
    input hrdata, hresp, hready;
    input adr_o, dat_o, cyc_o, we_o, stb_o;
  endclocking

  // Modport for driver
  modport drv_mp  (clocking drv_cb, input hclk);
  // Modport for monitor
  modport mon_mp  (clocking mon_cb, input hclk);

endinterface

//=============================================================================
// TOP-LEVEL TESTBENCH MODULE
//=============================================================================
module tb_top;
  import uvm_pkg::*;
  `include "uvm_macros.svh"
  import ahb2wb_pkg::*;

  // Clock generation
  logic hclk;
  logic clk_i;

  initial hclk  = 0;
  always  #5 hclk  = ~hclk;   // 100 MHz

  initial clk_i = 0;
  always  #5 clk_i = ~clk_i;  // Same frequency; can be offset if needed

  // Interface instantiation
  ahb2wb_if #(.AWIDTH(16), .DWIDTH(32)) dut_if (.hclk(hclk), .clk_i(clk_i));

  // DUT instantiation
  ahb2wb #(.AWIDTH(16), .DWIDTH(32)) dut (
    // Wishbone outputs
    .adr_o    (dut_if.adr_o),
    .dat_o    (dut_if.dat_o),
    .cyc_o    (dut_if.cyc_o),
    .we_o     (dut_if.we_o),
    .stb_o    (dut_if.stb_o),
    // Wishbone inputs
    .dat_i    (dut_if.dat_i),
    .ack_i    (dut_if.ack_i),
    .clk_i    (clk_i),
    .rst_i    (dut_if.rst_i),
    // AHB inputs
    .hclk     (hclk),
    .hresetn  (dut_if.hresetn),
    .haddr    (dut_if.haddr),
    .htrans   (dut_if.htrans),
    .hwrite   (dut_if.hwrite),
    .hsize    (dut_if.hsize),
    .hburst   (dut_if.hburst),
    .hsel     (dut_if.hsel),
    .hwdata   (dut_if.hwdata),
    // AHB outputs
    .hrdata   (dut_if.hrdata),
    .hresp    (dut_if.hresp),
    .hready   (dut_if.hready)
  );

  //===========================================================================
  // CODE COVERAGE: Bind assertions and toggle coverage hints
  //===========================================================================
  // Toggle coverage is automatically collected by simulators on all DUT signals.
  // The following SVA properties augment structural/functional coverage.
  //===========================================================================

  // Property: cyc_o must be 1 whenever stb_o is 1
  property p_cyc_when_stb;
    @(posedge hclk) disable iff (!dut_if.hresetn)
    dut_if.stb_o |-> dut_if.cyc_o;
  endproperty
  ap_cyc_when_stb: assert property (p_cyc_when_stb)
    else `uvm_error("SVA", "cyc_o=0 while stb_o=1")

  // Property: hready == ack_i when stb_o is high
  property p_hready_equals_ack;
    @(posedge hclk) disable iff (!dut_if.hresetn)
    dut_if.stb_o |-> (dut_if.hready == dut_if.ack_i);
  endproperty
  ap_hready_ack: assert property (p_hready_equals_ack)
    else `uvm_error("SVA", "hready != ack_i when stb_o=1")

  // Property: cyc_o deasserts when hsel is low and hready is high
  property p_cyc_deassert_no_sel;
    @(posedge hclk) disable iff (!dut_if.hresetn)
    (!dut_if.hsel && dut_if.hready) |=> !dut_if.cyc_o;
  endproperty
  ap_cyc_deassert: assert property (p_cyc_deassert_no_sel)
    else `uvm_error("SVA", "cyc_o not deasserted after hsel low")

  // Property: stb_o deasserts on IDLE transfer
  property p_stb_idle;
    @(posedge hclk) disable iff (!dut_if.hresetn)
    (dut_if.htrans == 2'b00 && dut_if.hsel) |=>
      (dut_if.stb_o == 1'b0);
  endproperty
  ap_stb_idle: assert property (p_stb_idle)
    else `uvm_error("SVA", "stb_o not deasserted after IDLE htrans")

  // Cover: NONSEQ write was observed
  cover property (@(posedge hclk) disable iff (!dut_if.hresetn)
    (dut_if.htrans == 2'b10 && dut_if.hsel && dut_if.hwrite))
  ;  // cp_nonseq_write

  // Cover: NONSEQ read was observed
  cover property (@(posedge hclk) disable iff (!dut_if.hresetn)
    (dut_if.htrans == 2'b10 && dut_if.hsel && !dut_if.hwrite))
  ;  // cp_nonseq_read

  // Cover: WB ack stall (ack_i=0 while stb=1)
  cover property (@(posedge hclk) disable iff (!dut_if.hresetn)
    (dut_if.stb_o && !dut_if.ack_i))
  ;  // cp_wb_stall

  //===========================================================================
  // Reset task
  //===========================================================================
  initial begin
    // Assert resets
    dut_if.hresetn <= 0;
    dut_if.rst_i   <= 1;
    dut_if.hsel    <= 0;
    dut_if.htrans  <= 2'b00;
    dut_if.hburst  <= 3'b000;
    dut_if.hsize   <= 3'b010;
    dut_if.hwrite  <= 0;
    dut_if.haddr   <= 0;
    dut_if.hwdata  <= 0;
    dut_if.dat_i   <= 0;
    dut_if.ack_i   <= 0;
    repeat(5) @(posedge hclk);
    dut_if.hresetn <= 1;
    dut_if.rst_i   <= 0;
    @(posedge hclk);

    // Register virtual interface
    uvm_config_db #(virtual ahb2wb_if)::set(null, "uvm_test_top.*", "vif", dut_if);

    // Run the test specified via +UVM_TESTNAME
    run_test();
  end

  // Simulation timeout
  initial begin
    #500_000;
    `uvm_fatal("TIMEOUT", "Simulation exceeded 500us – possible hang")
  end

  //===========================================================================
  // Waveform dump (optional – comment out if not needed)
  //===========================================================================
  initial begin
    $dumpfile("ahb2wb_tb.vcd");
    $dumpvars(0, tb_top);
  end

endmodule
