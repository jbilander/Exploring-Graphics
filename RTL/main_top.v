// Project F: FPGA Graphics - Simple 640x480p60 Display
// (C)2023 Will Green, open source hardware released under the MIT License
// Learn more at https://projectf.io/posts/fpga-graphics/
//
// Modified by J.Bilander for Tang Nano9K 2023-09-30

`timescale 1ns / 1ps

module main_top(
    input C27M,             // Input XTAL clock (27 MHz)
    input RESET_n,          // Push-button S2 on Tang9k, press down to reset
    output PCLK,            // Pixel-clock output
    output HSYNC,           // A screen begins a new line when it receives a horizontal sync,
    output VSYNC,           //  and a new frame on a vertical sync
    output DE,              // Data enable. This signal is high when input pixel data is valid to the transmitter and low otherwise.
    output reg [4:0] RED,   // 5-bit DVI red
    output reg [5:0] GREEN, // 6-bit DVI green
    output reg [4:0] BLUE   // 5-bit DVI blue
);

wire plock;             // Pixel-clock lock, indicates when Gowin rPLL generates a stable pixel-clock

// generate pixel clock
clock_480p pclk_480p(
    .C27M(C27M),
    .RESET_n(RESET_n),
    .PCLK(PCLK),        // The rPLL generated 25.2 MHz pixel clock for 480p@60 Hz
    .PLOCK(plock)
);

localparam CORDW = 10;      // screen coordinate width in bits
wire [CORDW-1:0] sx, sy;    // screen coordinates

simple_480p display_inst (
    .PCLK(PCLK),
    .RST_PCLK(!plock),   // wait for clock lock
    .SX(sx),
    .SY(sy),
    .HSYNC(HSYNC),
    .VSYNC(VSYNC),
    .DE(DE)
);

assign square = (sx > 220 && sx < 420) && (sy > 140 && sy < 340);

// paint colour: white inside square, blue outside
wire [4:0] paint_r = square ? 5'hFF : 5'h1;
wire [5:0] paint_g = square ? 6'hFF : 6'h3;
wire [4:0] paint_b = square ? 5'hFF : 5'h5;

// display colour: paint colour but black in blanking interval
wire [4:0] display_r = DE ? paint_r : 5'h0;
wire [5:0] display_g = DE ? paint_g : 6'h0;
wire [4:0] display_b = DE ? paint_b : 5'h0;

//Pmod output
always @(posedge PCLK) begin
    RED <= display_r;
    GREEN <= display_g;
    BLUE <= display_b;
end

endmodule
