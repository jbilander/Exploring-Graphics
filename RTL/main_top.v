`timescale 1ns / 1ps

module main_top(
    input C27M,     // Input XTAL clock (27 MHz)
    input RESET_n,  // Push-button S2 on Tang9k, press down to reset
    output HSYNC,   // A screen begins a new line when it receives a horizontal sync,
    output VSYNC,   // and a new frame on a vertical sync
    output reg LED

);

wire C25M;      // The 25.2 MHz pixel clock for 480p@60 Hz
wire C302M;     // The CLKOUT_FREQ=302.4 MHz gets divided by 12 to generate 25.2 MHz above

Gowin_rPLL_25M gen_C27M(
    .clkout(C302M), //output clkout
    .clkoutd(C25M), //output clkoutd
    .reset(!RESET_n), //input reset
    .clkin(C27M) //input clkin
);

reg [31:0] counter;

always @ (negedge RESET_n or posedge C25M) begin

    if (!RESET_n) begin

        LED <= 1'b0;
        counter <= 32'b0;

    end else begin

        counter <= counter + 1'b1;

        if (counter > 10000000) begin
            LED <= ~LED;
            counter <= 32'b0;
        end
    end

end

endmodule
