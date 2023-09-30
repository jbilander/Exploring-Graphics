`timescale 1ns / 1ps

module clock_480p(
    input C27M,
    input RESET_n,
    output PCLK
);

wire C302M;     // The CLKOUT_FREQ=302.4 MHz gets divided by 12 to generate 25.2 MHz above

Gowin_rPLL_25M gen_C27M(
    .clkout(C302M),     //output clkout
    .clkoutd(PCLK),     //output clkoutd
    .reset(!RESET_n),   //input reset
    .clkin(C27M)        //input clkin
);

endmodule
