`timescale 1ns/1ps

module rv32i_tb;

  // Testbench signals
  reg clk;
  reg reset;

  wire [31:0] pc_out;
  wire [31:0] instr_out;
  wire [31:0] alu_result_out;
  wire [31:0] reg_data_out;

  // Instantiate the DUT (Device Under Test)
  rv32i dut (
    .clk(clk),
    .reset(reset),
    .pc_out(pc_out),
    .instr_out(instr_out),
    .alu_result_out(alu_result_out),
    .reg_data_out(reg_data_out)
  );

  // Clock generation: 10 ns period
  initial clk = 0;
  always #5 clk = ~clk;

  // Reset task
  task reset_dut;
    begin
      reset = 1;
      @(posedge clk);
      @(posedge clk);
      reset = 0;
    end
  endtask

  // Display header for waveform viewing/logging
  initial begin
    $display("Time(ns) | PC      | Instr    | ALU_Result | Reg[1] ");
    $display("-----------------------------------------------------");
  end

  // Monitor signals at every clock positive edge
  always @(posedge clk) begin
    $display("%8t | %h | %h | %h | %h", $time, pc_out, instr_out, alu_result_out, reg_data_out);
  end

  // Main test sequence
  initial begin
    // Initialize signals
    reset = 0;

    // Apply reset
    reset_dut();

    // Run simulation for a sufficient number of clock cycles to see program execution
    repeat (20) @(posedge clk);

    // You can add checks here (assertions or comparisons) to verify correctness if desired
    // Example simple assertion: check if regfile[1] (x1) contains expected value after additions
    // (You need to extend modification of the DUT to expose regfile values for detailed checking)

    // Finish simulation
    $display("\nSimulation complete.");
    $stop;
  end

endmodule
