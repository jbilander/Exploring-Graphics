// Project F: FPGA Graphics - Simple 640x480p60 Display
// (C)2023 Will Green, open source hardware released under the MIT License
// Learn more at https://projectf.io/posts/fpga-graphics/
//
// Modified by J.Bilander for Tang Nano9K 2023-09-30

`timescale 1ns / 1ps

module main_top(
    input C27M,         // Input XTAL clock (27 MHz)
    input RESET_n,      // Push-button S2 on Tang9k, press down to reset
    output PCLK,        // Pixel-clock output
    output PLOCK,       // Pixel-clock lock, indicates when Gowin rPLL generates a stable pixel-clock
    output HSYNC,       // A screen begins a new line when it receives a horizontal sync,
    output VSYNC,       //  and a new frame on a vertical sync
    output DE,          // Data enable. This signal is high when input pixel data is valid to the transmitter 
                        //  and low otherwise. It is critical that this signal have the same
                        //  setup/hold timing as the data bus.
    output reg LED
);

// generate pixel clock
clock_480p pclk_480p(
    .C27M(C27M),
    .RESET_n(RESET_n),
    .PCLK(PCLK),        // The rPLL generated 25.2 MHz pixel clock for 480p@60 Hz
    .PLOCK(PLOCK)
);

localparam CORDW = 10;      // screen coordinate width in bits
wire [CORDW-1:0] sx, sy;    // screen coordinates

simple_480p display_inst (
    .PCLK(PCLK),
    .RST_PCLK(!PLOCK),   // wait for clock lock
    .SX(sx),
    .SY(sy),
    .HSYNC(HSYNC),
    .VSYNC(VSYNC),
    .DE(DE)
);


reg [31:0] counter;

always @ (negedge RESET_n or posedge PCLK) begin

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
