/***************************************************************
 * FPGA Revolution Open Bootcamp
 * Episode 34 - Flying Cubes on FPGA
 *
 * Rasterize a flat-top triangle
 **************************************************************/
 
module triangle_t_clb #(
    parameter WIDTH = 1,                               // line width
    parameter COLOR = 24'h FFFFFF,                     // line color
    parameter SLOPE_RES = 28,                          // bits for slope
    parameter FRACT_RES = 16                           // bits for fractional resolution
) (
    // Video timing
    input pixel_clk,                                   // 75 MHz pixel clock
    input rst,                                         // system reset
    input fsync,                                       // frame sync
    input active,                                      // line active
    input signed [11:0] hpos,                          // horizontal pixel position
    input signed [11:0] vpos,                          // vertical pixel position

    // Positions of vertices
    input signed [SLOPE_RES-1:0] x_p1, y_p1,
    input signed [SLOPE_RES-1:0] x_p2, y_p2,         
    input signed [SLOPE_RES-1:0] x_p3, y_p3,
    input signed [SLOPE_RES-1:0] dx_p1p2,                  
    input signed [SLOPE_RES-1:0] dx_p1p3,             
    input [23:0] color,             
    
    // RGB pixels and active indicator
    output [7:0] pixel_tri_t [0:2],                   // RGB pixels
    output active_tri_t                               // active pixel indicator of flat-bottom triangle
);

reg signed [SLOPE_RES-1:0] x_l1;
wire signed [SLOPE_RES-1:0] x_l1_dsp;
reg signed [SLOPE_RES-1:0] x_l2;
wire signed [SLOPE_RES-1:0] x_l2_dsp;
reg active_ff;

reg signed [SLOPE_RES-1:0] x_p1_fsync, y_p1_fsync;
reg signed [SLOPE_RES-1:0] x_p2_fsync, y_p2_fsync;      
reg signed [SLOPE_RES-1:0] x_p3_fsync, y_p3_fsync;
reg signed [SLOPE_RES-1:0] dx_p1p2_fsync;                 
reg signed [SLOPE_RES-1:0] dx_p1p3_fsync;             


// Rasterization
always @(posedge pixel_clk)
begin
    active_ff <= active;

    if (fsync) begin
        x_l1 <= x_p2;
        x_l2 <= x_p3;

        x_p1_fsync <= x_p1; y_p1_fsync <= y_p1;
        x_p2_fsync <= x_p2; y_p2_fsync <= y_p2;
        x_p3_fsync <= x_p3; y_p3_fsync <= y_p3;
        dx_p1p2_fsync <= dx_p1p2;
        dx_p1p3_fsync <= dx_p1p3;
    end else if (~active && active_ff && vpos >= y_p2_fsync[SLOPE_RES-1:FRACT_RES] && vpos < y_p1_fsync[SLOPE_RES-1:FRACT_RES]) begin
        x_l1 <= x_l1 + dx_p1p2_fsync;
        x_l2 <= x_l2 + dx_p1p3_fsync;
    end
end

assign active_tri_t = (hpos >= x_l1[SLOPE_RES-1:FRACT_RES] && hpos <= x_l2[SLOPE_RES-1:FRACT_RES] && vpos >= y_p2_fsync[SLOPE_RES-1:FRACT_RES] && vpos < y_p1_fsync[SLOPE_RES-1:FRACT_RES])? 1'b1 : 1'b0;

assign pixel_tri_t[2] = active_tri_t? color[23:16] : 8'h0;
assign pixel_tri_t[1] = active_tri_t? color[15:8] : 8'h0;
assign pixel_tri_t[0] = active_tri_t? color[7:0]  : 8'h0;


endmodule 
