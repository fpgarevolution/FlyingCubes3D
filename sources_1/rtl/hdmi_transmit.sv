/***************************************************************
 * FPGA Revolution Open Bootcamp
 * Episode 34 - Flying Cubes on FPGA
 *
 * HDMI driver with TMDS encoding + video timing
 **************************************************************/
 
module hdmi_transmit (
    input clk125,                                         // 125 MHz input clock
    input rst_n,                                          // Reset input                                    
    
    // RGB pixel data to be encoded and transmitted
    input [7:0] pixel [0:2],
    
    // Shared video timing to the rest of the system
    output pixel_clk,                                     // 75 MHz pixel clock
    output rst,                                           // system reset
    output active,                                        // active pixel indicator
    output fsync,                                         // frame sync
    output vsync_out,
    output hsync_out,
    output reg [10:0] hpos,                               // horizontal position
    output reg [9:0] vpos,                                // vertical position
    
    // HDMI output
    output tmds_tx_clk_p,                                 // TMDS clock p channel
    output tmds_tx_clk_n,                                 // TMDS clock n channel
    output [2:0] tmds_tx_data_p,                          // TMDS data p channels
    output [2:0] tmds_tx_data_n                           // TMDS data n channels
);

wire serdes_clk;                                          // 375 MHz serdes clock
reg [7:0] rstcnt;
wire locked;                                              // MMCM locked indicator
reg active_ff;                                            // active reclocked
wire pixel_clk_mmcm;

wire hblank, vblank;                                      // video timing
wire vsync, hsync;
reg vsync_ff, hsync_ff;

wire [1:0] ctl [0:2];                                     // control bits for TMDS data channels
wire [9:0] tmds_data [0:2];                               // encoded TMDS data

// Utilize MMCM to generate 75 MHz pixel clock and 375 MHz TMDS serdes clock
mmcm_0 mmcm_0_inst (
    .clk_in1  (clk125),                                   // 125 MHz input clock
    .reset    (~rst_n),                                   // reset from Zynq, since 125MHz is now sourced from Zynq interface
    .clk_out1 (pixel_clk_mmcm),                           // 75 MHz pixel clock
    .clk_out2 (serdes_clk),                               // 375 MHz TMDS serdes clock
    .locked   (locked)                                    // locked indicator
);

BUFG bufg_pixel_clk (
    .I  (pixel_clk_mmcm),
    .O  (pixel_clk)
);

// Transition into system reset following MMCM locked indicator
always @(posedge pixel_clk or negedge locked)
begin
    if (~locked) begin
        rstcnt <= 0;
    end else begin
        if (rstcnt != 8'hff) begin
            rstcnt <= rstcnt + 1;
        end
    end
end

assign rst = (rstcnt == 8'hff)? 1'b0 : 1'b1;

// Video timing generator
video_timing video_timing_inst (
    .clk        (pixel_clk),
    .clken      (1'b1),
    .gen_clken  (1'b1),
    .sof_state  (1'b0),
    .hsync_out  (hsync),
    .hblank_out (hblank),
    .vsync_out  (vsync),
    .vblank_out (vblank),
    .active_video_out (active),
    .resetn     (~rst),
    .fsync_out  (fsync)
);

assign vsync_out = vsync && ~vsync_ff;
assign hsync_out = hsync && ~hsync_ff;
// Track pixel position
always @(posedge pixel_clk)
begin
    active_ff <= active;
    vsync_ff <= vsync;
    hsync_ff <= hsync;
    // Horizontal position
    if (rst || hsync_out) begin
        hpos <= 0;
    end else if (active) begin
        hpos <= hpos + 1;
    end
    // Vertical position
    if (rst || vsync_out) begin
        vpos <= 0;
    end else if (hsync_out) begin
        vpos <= vpos + 1;
    end
end

assign ctl[0] = {vsync, hsync};                    // vsync and hsync go onto TMDS channel 0
assign ctl[1] = 2'b00;
assign ctl[2] = 2'b00;

// Encode video data onto three TMDS data channels
generate
    genvar i;
    
    for (i=0; i<3; i=i+1) begin
        // TMDS data encoder
        tmds_encode tmds_encode_inst (
            .pixel_clk  (pixel_clk),               // pixel clock
            .rst        (rst),                     // reset
            .ctl        (ctl[i]),                  // control bits
            .active     (active),                  // active pixel indicator
            .pdata      (pixel[i]),                // 8-bit pixel data
            .tmds_data  (tmds_data[i])             // encoded 10-bit TMDS data
        );
        // TMDS data output serdes
        tmds_oserdes tmds_oserdes_inst (
            .pixel_clk  (pixel_clk),               // pixel clock
            .serdes_clk (serdes_clk),              // serdes clock
            .rst        (rst),                     // reset
            .tmds_data  (tmds_data[i]),            // encoded 10-bit TMDS data
            .tmds_serdes_p (tmds_tx_data_p[i]),    // TMDS data p channel
            .tmds_serdes_n (tmds_tx_data_n[i])     // TMDS data n channel
        );
    end
endgenerate

// TMDS clock output serdes
tmds_oserdes tmds_oserdes_clock (
    .pixel_clk   (pixel_clk),                      // pixel clock
    .serdes_clk  (serdes_clk),                     // serdes clock
    .rst         (rst),                            // reset
    .tmds_data   (10'b 0000011111),                // pixel clock pattern
    .tmds_serdes_p (tmds_tx_clk_p),                // TMDS clock p channel
    .tmds_serdes_n (tmds_tx_clk_n)                 // TMDS clock n channel
);

endmodule
