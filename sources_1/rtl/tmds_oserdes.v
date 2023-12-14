/*****************************************************************************************
 * FPGA Revolution Open Bootcamp
 * Episode 34 - Flying Cubes on FPGA
 *
 * Serialization of 10-bit encoded transition-minimized differential signaling (TMDS) data
 ****************************************************************************************/
 
module tmds_oserdes (
    input pixel_clk,                                  // pixel clock
    input serdes_clk,                                 // serdes clock
    input rst,                                        // reset
    input [9:0] tmds_data,                            // encoded 10-bit TMDS data
    
    output tmds_serdes_p,                             // TMDS p channel
    output tmds_serdes_n                              // TMDS n channel
);

wire tmds_serdes_data;                                // serialized TMDS data
wire shiftout_0, shiftout_1;                          // used for 10-bit expansion mode

// TMDS differential output buffer
OBUFDS #(
    .IOSTANDARD ("TMDS_33")
) obufds_tmds (
    .I  (tmds_serdes_data),
    .O  (tmds_serdes_p),
    .OB (tmds_serdes_n)
);

// Utilize OSERDESE2 in expansion mode to serialize 10-bit of TMDS data
// Primary oserdes
OSERDESE2 #(
    .DATA_RATE_OQ    ("DDR"),
    .DATA_RATE_TQ    ("SDR"),
    .DATA_WIDTH      (10),
    .INIT_OQ         (1'b0),
    .INIT_TQ         (1'b0),
    .SERDES_MODE     ("MASTER"),
    .SRVAL_OQ        (1'b0),
    .SRVAL_TQ        (1'b0),
    .TBYTE_CTL       ("FALSE"),
    .TBYTE_SRC       ("FALSE"),
    .TRISTATE_WIDTH  (1)
) oserdes_primary (
    .OFB       (),
    .OQ        (tmds_serdes_data),                      // serialized data output
    .SHIFTOUT1 (),
    .SHIFTOUT2 (),
    .TBYTEOUT  (),
    .TFB       (),
    .TQ        (),
    .CLK       (serdes_clk),                            // serdes clock
    .CLKDIV    (pixel_clk),                             // pixel clock
    .D1        (tmds_data[0]),                          // encoded TMDS data bit 0
    .D2        (tmds_data[1]),                          // encoded TMDS data bit 1
    .D3        (tmds_data[2]),                          // encoded TMDS data bit 2
    .D4        (tmds_data[3]),                          // encoded TMDS data bit 3
    .D5        (tmds_data[4]),                          // encoded TMDS data bit 4
    .D6        (tmds_data[5]),                          // encoded TMDS data bit 5
    .D7        (tmds_data[6]),                          // encoded TMDS data bit 6
    .D8        (tmds_data[7]),                          // encoded TMDS data bit 7
    .OCE       (1'b1),
    .RST       (rst),
    .SHIFTIN1  (shiftout_0),                            // from secondary oserdes, 10-bit expansion mode
    .SHIFTIN2  (shiftout_1),                            // from secondary oserdes, 10-bit expansion mode
    .T1        (1'b0),
    .T2        (1'b0),
    .T3        (1'b0),
    .T4        (1'b0),
    .TBYTEIN   (1'b0),
    .TCE       (1'b0)
);

// Utilize OSERDESE2 in expansion mode to serialize 10-bit of data
// Secondary oserdes
OSERDESE2 #(
    .DATA_RATE_OQ  ("DDR"),
    .DATA_RATE_TQ  ("SDR"),
    .DATA_WIDTH    (10),
    .INIT_OQ       (1'b0),
    .INIT_TQ       (1'b0),
    .SERDES_MODE   ("SLAVE"),
    .SRVAL_OQ      (1'b0),
    .SRVAL_TQ      (1'b0),
    .TBYTE_CTL     ("FALSE"),
    .TBYTE_SRC     ("FALSE"),
    .TRISTATE_WIDTH(1)
) oserdes_secondary (
    .OFB       (),
    .OQ        (),
    .SHIFTOUT1 (shiftout_0),                            // to primary OSERDES, 10-bit expansion mode
    .SHIFTOUT2 (shiftout_1),                            // to primary OSERDES, 10-bit expansion mode
    .TBYTEOUT  (),
    .TFB       (),
    .TQ        (),
    .CLK       (serdes_clk),                            // serdes clock
    .CLKDIV    (pixel_clk),                             // pixel clock
    .D1        (1'b0),
    .D2        (1'b0),
    .D3        (tmds_data[8]),                          // encoded TMDS data bit 8
    .D4        (tmds_data[9]),                          // encoded TMDS data bit 9
    .D5        (1'b0),
    .D6        (1'b0),
    .D7        (1'b0),
    .D8        (1'b0),
    .OCE       (1'b1),
    .RST       (rst),
    .SHIFTIN1  (1'b0),
    .SHIFTIN2  (1'b0),
    .T1        (1'b0),
    .T2        (1'b0),
    .T3        (1'b0),
    .T4        (1'b0),
    .TBYTEIN   (1'b0),
    .TCE       (1'b0)
);

endmodule
