`timescale 1ns / 1ps

module main_top(
    input C27M,         // Input XTAL clock (27 MHz)
    input RESET_n,      // Push-button S2 on Tang9k, press down to reset
    output PCLK,        // Pixel-clock output
    output HSYNC,       // A screen begins a new line when it receives a horizontal sync,
    output VSYNC,       // and a new frame on a vertical sync
    output reg LED
);

clock_480p pclk_480p(
    .C27M(C27M),
    .RESET_n(RESET_n),
    .PCLK(PCLK)         // The rPLL generated 25.2 MHz pixel clock for 480p@60 Hz
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
