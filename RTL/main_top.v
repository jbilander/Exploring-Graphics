`timescale 1ns / 1ps

module main_top(
    output HSYNC,    // A screen begins a new line when it receives a horizontal sync
    output VSYNC     // and a new frame on a vertical sync
);
