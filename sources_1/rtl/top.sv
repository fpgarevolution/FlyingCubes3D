/***************************************************************
 * FPGA Revolution Open Bootcamp
 * Episode 34 - Flying Cubes on FPGA
 *
 * Top-level
 **************************************************************/
 
module top (
    // 125 MHz input clock
    input clk125,

    // Zynq IO
    inout DDR_addr,
    inout DDR_ba,
    inout DDR_cas_n,
    inout DDR_ck_n,
    inout DDR_ck_p,
    inout DDR_cke,
    inout DDR_cs_n,
    inout DDR_dm,
    inout DDR_dq,
    inout DDR_dqs_n,
    inout DDR_dqs_p,
    inout DDR_odt,
    inout DDR_ras_n,
    inout DDR_reset_n,
    inout DDR_we_n,
    inout FIXED_IO_ddr_vrn,
    inout FIXED_IO_ddr_vrp,
    inout FIXED_IO_mio,
    inout FIXED_IO_ps_clk,
    inout FIXED_IO_ps_porb,
    inout FIXED_IO_ps_srstb,

    // Speed control for flying cubes
    input right,                                    // Increase
    input left,                                     // Decrease
    
    // HDMI output
    output tmds_tx_clk_p,                           // TMDS clock p channel
    output tmds_tx_clk_n,                           // TMDS clock n channel
    output [2:0] tmds_tx_data_p,                    // TMDS data p channels
    output [2:0] tmds_tx_data_n                     // TMDS data n channels
);


localparam SLOPE_RES = 28;                          // bits resolution for position and slopes

// Number of triangle pair supported (1 pair = 1 flat-bottom + 1 flat-top)
localparam TRI_MAX = 64;
localparam DMA_WORD_PER_TRI = 18;                   // half for flat-bottom and half for flat-top
localparam DMA_LENGTH = 256*5;                      // number of 32-bit words through DMA needed to support the system, minimum that covers TRI_MAX*DMA_WORD_PER_TRI

wire pixel_clk;                                     // 75 MHz pixel clock
wire rst;                                           // active-high system reset

wire active;                                        // active pixel indicator
wire fsync;                                         // frame sync
reg fsync_axis_clk [0:1];
wire signed [11:0] hpos;                            // horizontal position
wire signed [11:0] vpos;                            // vertical position
wire [7:0] pixel [0:2];                             // RGB pixels for display


reg [10:0] dma_cnt;                                 // Need enough bits to support DMA_LENGTH
wire [7:0] pixel_combine[0:TRI_MAX-1][0:2];
// Flat-bottom triangle
reg signed [SLOPE_RES-1:0] x_p1_b[0:TRI_MAX-1], y_p1_b[0:TRI_MAX-1];
reg signed [SLOPE_RES-1:0] x_p2_b[0:TRI_MAX-1], y_p2_b[0:TRI_MAX-1];
reg signed [SLOPE_RES-1:0] x_p3_b[0:TRI_MAX-1], y_p3_b[0:TRI_MAX-1];
reg signed [SLOPE_RES-1:0] dx_p1p2_b[0:TRI_MAX-1];                         // 1.11.12
reg signed [SLOPE_RES-1:0] dx_p1p3_b[0:TRI_MAX-1];
reg [23:0] color_b[0:TRI_MAX-1];
wire active_tri_b[0:TRI_MAX-1];
wire [7:0] pixel_tri_b [0:TRI_MAX-1][0:2];                       // RGB pixels for display
reg tri_b_activate[0:TRI_MAX-1];
// Flat-top triangle
reg signed [SLOPE_RES-1:0] x_p1_t[0:TRI_MAX-1], y_p1_t[0:TRI_MAX-1];
reg signed [SLOPE_RES-1:0] x_p2_t[0:TRI_MAX-1], y_p2_t[0:TRI_MAX-1];
reg signed [SLOPE_RES-1:0] x_p3_t[0:TRI_MAX-1], y_p3_t[0:TRI_MAX-1];
reg signed [SLOPE_RES-1:0] dx_p1p2_t[0:TRI_MAX-1];                         // 1.11.12
reg signed [SLOPE_RES-1:0] dx_p1p3_t[0:TRI_MAX-1];
reg [23:0] color_t[0:TRI_MAX-1];
wire active_tri_t[0:TRI_MAX-1];
wire [7:0] pixel_tri_t [0:TRI_MAX-1][0:2];                       // RGB pixels for display
reg tri_t_activate[0:TRI_MAX-1];


// Buttons sync
reg right_ff[0:2];
reg left_ff[0:2];
reg right_latched, left_latched;
reg right_latched_fsync, left_latched_fsync;

// gpio
wire gpio;
reg [31:0] duration;

// For Zynq interface
wire [31:0] axis_tdata;
wire [3:0] axis_tkeep;
wire axis_tlast;
reg axis_tready;
wire axis_tvalid;
wire axis_clk;
wire rst_n;                         
reg [0:2] axis_clk_rst;                  
wire axis_rst;          


// Manage 3D polygon data coming in through DMA
always @(posedge axis_clk)
begin
    fsync_axis_clk <= {fsync, fsync_axis_clk[0]};

    if (axis_rst) begin
        dma_cnt <= 0;
    end else if (axis_tready && axis_tvalid) begin
        if (dma_cnt == DMA_LENGTH-1) begin
            dma_cnt <= 0;
        end else begin
            dma_cnt <= dma_cnt + 1;
        end
    end

    // Speed control sync
    right_ff <= {right, right_ff[0:1]};
    left_ff <= {left, left_ff[0:1]};
    if (axis_rst) begin
        right_latched <= 1'b0;
        left_latched <= 1'b0;
        right_latched_fsync <= 1'b0;
        left_latched_fsync <= 1'b0;
    end else if (~fsync_axis_clk[0] && fsync_axis_clk[1]) begin
        right_latched_fsync <= right_latched;
        left_latched_fsync  <= left_latched;
    end else if (axis_tready && axis_tvalid) begin
        right_latched <= 1'b0;                                       // DMA happening, can reset
        left_latched <= 1'b0;                                     
    end else if (right_ff[1] && ~right_ff[2]) begin
        right_latched <= 1'b1;
    end else if (left_ff[1] && ~left_ff[2]) begin
        left_latched <= 1'b1;
    end
end

generate
    genvar i;
    for (i=0; i<TRI_MAX; i=i+1) begin
        always @(posedge axis_clk)
        begin
            if (axis_tready && axis_tvalid) begin
                // Flat-bottom triangle 0 vertices and slopes
                if (dma_cnt == DMA_WORD_PER_TRI*i) begin
                    x_p1_b[i] <= $signed (axis_tdata[SLOPE_RES-1:0]);
                end else if (dma_cnt == DMA_WORD_PER_TRI*i + 1) begin
                    y_p1_b[i] <= $signed (axis_tdata[SLOPE_RES-1:0]);
                end else if (dma_cnt == DMA_WORD_PER_TRI*i + 2) begin
                    x_p2_b[i] <= $signed (axis_tdata[SLOPE_RES-1:0]);
                end else if (dma_cnt == DMA_WORD_PER_TRI*i + 3) begin
                    y_p2_b[i] <= $signed (axis_tdata[SLOPE_RES-1:0]);
                end else if (dma_cnt == DMA_WORD_PER_TRI*i + 4) begin
                    x_p3_b[i] <= $signed (axis_tdata[SLOPE_RES-1:0]);
                end else if (dma_cnt == DMA_WORD_PER_TRI*i + 5) begin
                    y_p3_b[i] <= $signed (axis_tdata[SLOPE_RES-1:0]);
                end else if (dma_cnt == DMA_WORD_PER_TRI*i + 6) begin
                    dx_p1p2_b[i] <= $signed (axis_tdata[SLOPE_RES-1:0]);
    
                    tri_b_activate[i] <= axis_tdata[31];
                end else if (dma_cnt == DMA_WORD_PER_TRI*i + 7) begin
                    dx_p1p3_b[i] <= $signed (axis_tdata[SLOPE_RES-1:0]);
                end else if (dma_cnt == DMA_WORD_PER_TRI*i + 8) begin
                    color_b[i] <= axis_tdata[23:0];
                end 
                // Flat-top triangle 0 vertices and slopes
                if (dma_cnt == DMA_WORD_PER_TRI*i + 9) begin
                    x_p1_t[i] <= $signed (axis_tdata[SLOPE_RES-1:0]);
                end else if (dma_cnt == DMA_WORD_PER_TRI*i + 10) begin
                    y_p1_t[i] <= $signed (axis_tdata[SLOPE_RES-1:0]);
                end else if (dma_cnt == DMA_WORD_PER_TRI*i + 11) begin
                    x_p2_t[i] <= $signed (axis_tdata[SLOPE_RES-1:0]);
                end else if (dma_cnt == DMA_WORD_PER_TRI*i + 12) begin
                    y_p2_t[i] <= $signed (axis_tdata[SLOPE_RES-1:0]);
                end else if (dma_cnt == DMA_WORD_PER_TRI*i + 13) begin
                    x_p3_t[i] <= $signed (axis_tdata[SLOPE_RES-1:0]);
                end else if (dma_cnt == DMA_WORD_PER_TRI*i + 14) begin
                    y_p3_t[i] <= $signed (axis_tdata[SLOPE_RES-1:0]);
                end else if (dma_cnt == DMA_WORD_PER_TRI*i + 15) begin
                    dx_p1p2_t[i] <= $signed (axis_tdata[SLOPE_RES-1:0]);
    
                    tri_t_activate[i] <= axis_tdata[31];
                end else if (dma_cnt == DMA_WORD_PER_TRI*i + 16) begin
                    dx_p1p3_t[i] <= $signed (axis_tdata[SLOPE_RES-1:0]);
                end else if (dma_cnt == DMA_WORD_PER_TRI*i + 17) begin
                    color_t[i] <= axis_tdata[23:0];
                end 

            end
        end
    end
endgenerate
 

localparam DSP_BREAK = TRI_MAX - 12;                           // Control usage of DSP resources

// Build up army of pairs of flat-bottom and flat-top triangles
// Using DSP resources
generate
    genvar j, k;
    for (j=0; j<DSP_BREAK; j=j+1) begin
        // Flat-bottom triangle
        triangle_b #(
            .WIDTH (1),                                        // line width
            .COLOR (24'h FFFFFF)                               // line color
        ) triangle_b_0 (
            // Video timing
            .pixel_clk  (pixel_clk),                           // 75 MHz pixel clock
            .rst        (rst),                                 // system reset
            .fsync      (fsync),                               // frame sync
            .active     (active),                              // line active
            .hpos       (hpos),                                // horizontal pixel position
            .vpos       (vpos),                                // vertical pixel position
        
            // Positions of vertices
            .x_p1 (x_p1_b[j]), .y_p1 (y_p1_b[j]),
            .x_p2 (x_p2_b[j]), .y_p2 (y_p2_b[j]),
            .x_p3 (x_p3_b[j]), .y_p3 (y_p3_b[j]),
            .dx_p1p2 (dx_p1p2_b[j]),                  
            .dx_p1p3 (dx_p1p3_b[j]),             
            .color   (color_b[j]),
            
            // RGB pixels and active indicator
            .pixel_tri_b  (pixel_tri_b[j]),                    // RGB pixels
            .active_tri_b (active_tri_b[j])                    // active pixel indicator of flat-bottom triangle
        );
        
        // Flat-top triangle
        triangle_t #(
            .WIDTH (1),                                        // line width
            .COLOR (24'h FFFFFF)                               // line color
        ) triangle_t_0 (
            // Video timing
            .pixel_clk  (pixel_clk),                           // 75 MHz pixel clock
            .rst        (rst),                                 // system reset
            .fsync      (fsync),                               // frame sync
            .active     (active),                              // line active
            .hpos       (hpos),                                // horizontal pixel position
            .vpos       (vpos),                                // vertical pixel position
        
            // Positions of vertices
            .x_p1 (x_p1_t[j]), .y_p1 (y_p1_t[j]),
            .x_p2 (x_p2_t[j]), .y_p2 (y_p2_t[j]),
            .x_p3 (x_p3_t[j]), .y_p3 (y_p3_t[j]),
            .dx_p1p2 (dx_p1p2_t[j]),                  
            .dx_p1p3 (dx_p1p3_t[j]),             
            .color   (color_t[j]),
            
            // RGB pixels and active indicator
            .pixel_tri_t  (pixel_tri_t[j]),                      // RGB pixels
            .active_tri_t (active_tri_t[j])                      // active pixel indicator of flat-bottom triangle
        );

        // Loop through RGB components
        for (k=0; k<3; k=k+1) begin
            assign pixel_combine[j][k] = (pixel_tri_b[j][k] & {8{active_tri_b[j]}}) | (pixel_tri_t[j][k] & {8{active_tri_t[j]}});
        end

    end
endgenerate
// Without using DSP resources
generate
    genvar jj, kk;
    for (jj=DSP_BREAK; jj<TRI_MAX; jj=jj+1) begin
        // Flat-bottom triangle
        triangle_b_clb #(
            .WIDTH (1),                                        // line width
            .COLOR (24'h FFFFFF)                               // line color
        ) triangle_b_0 (
            // Video timing
            .pixel_clk  (pixel_clk),                           // 75 MHz pixel clock
            .rst        (rst),                                 // system reset
            .fsync      (fsync),                               // frame sync
            .active     (active),                              // line active
            .hpos       (hpos),                                // horizontal pixel position
            .vpos       (vpos),                                // vertical pixel position
        
            // Positions of vertices
            .x_p1 (x_p1_b[jj]), .y_p1 (y_p1_b[jj]),
            .x_p2 (x_p2_b[jj]), .y_p2 (y_p2_b[jj]),
            .x_p3 (x_p3_b[jj]), .y_p3 (y_p3_b[jj]),
            .dx_p1p2 (dx_p1p2_b[jj]),                  
            .dx_p1p3 (dx_p1p3_b[jj]),             
            .color   (color_b[jj]),
            
            // RGB pixels and active indicator
            .pixel_tri_b  (pixel_tri_b[jj]),                    // RGB pixels
            .active_tri_b (active_tri_b[jj])                    // active pixel indicator of flat-bottom triangle
        );
        
        // Flat-top triangle
        triangle_t_clb #(
            .WIDTH (1),                                        // line width
            .COLOR (24'h FFFFFF)                               // line color
        ) triangle_t_0 (
            // Video timing
            .pixel_clk  (pixel_clk),                           // 75 MHz pixel clock
            .rst        (rst),                                 // system reset
            .fsync      (fsync),                               // frame sync
            .active     (active),                              // line active
            .hpos       (hpos),                                // horizontal pixel position
            .vpos       (vpos),                                // vertical pixel position
        
            // Positions of vertices
            .x_p1 (x_p1_t[jj]), .y_p1 (y_p1_t[jj]),
            .x_p2 (x_p2_t[jj]), .y_p2 (y_p2_t[jj]),
            .x_p3 (x_p3_t[jj]), .y_p3 (y_p3_t[jj]),
            .dx_p1p2 (dx_p1p2_t[jj]),                  
            .dx_p1p3 (dx_p1p3_t[jj]),             
            .color   (color_t[jj]),
            
            // RGB pixels and active indicator
            .pixel_tri_t  (pixel_tri_t[jj]),                      // RGB pixels
            .active_tri_t (active_tri_t[jj])                      // active pixel indicator of flat-bottom triangle
        );

        // Loop through RGB components
        for (kk=0; kk<3; kk=kk+1) begin
            assign pixel_combine[jj][kk] = (pixel_tri_b[jj][kk] & {8{active_tri_b[jj]}}) | (pixel_tri_t[jj][kk] & {8{active_tri_t[jj]}});
        end

    end
endgenerate


// Combine pixels
localparam GROUP_SIZE = 16;
localparam GROUP = TRI_MAX / GROUP_SIZE;
reg [7:0] temp[0:2][0:GROUP-1];

generate
    genvar l;

    for (l=0; l<GROUP; l=l+1) begin
        always @(*)
        begin
            temp[2][l] = pixel_combine[l*GROUP_SIZE][2]    | pixel_combine[l*GROUP_SIZE+1][2]  | pixel_combine[l*GROUP_SIZE+2][2]  | pixel_combine[l*GROUP_SIZE+3][2]  |
                          pixel_combine[l*GROUP_SIZE+4][2]  | pixel_combine[l*GROUP_SIZE+5][2]  | pixel_combine[l*GROUP_SIZE+6][2]  | pixel_combine[l*GROUP_SIZE+7][2]  |
                          pixel_combine[l*GROUP_SIZE+8][2]  | pixel_combine[l*GROUP_SIZE+9][2]  | pixel_combine[l*GROUP_SIZE+10][2] | pixel_combine[l*GROUP_SIZE+11][2] |
                          pixel_combine[l*GROUP_SIZE+12][2] | pixel_combine[l*GROUP_SIZE+13][2] | pixel_combine[l*GROUP_SIZE+14][2] | pixel_combine[l*GROUP_SIZE+15][2] ; 
        
            temp[1][l] = pixel_combine[l*GROUP_SIZE][1]    | pixel_combine[l*GROUP_SIZE+1][1]  | pixel_combine[l*GROUP_SIZE+2][1]  | pixel_combine[l*GROUP_SIZE+3][1]  |
                          pixel_combine[l*GROUP_SIZE+4][1]  | pixel_combine[l*GROUP_SIZE+5][1]  | pixel_combine[l*GROUP_SIZE+6][1]  | pixel_combine[l*GROUP_SIZE+7][1]  |
                          pixel_combine[l*GROUP_SIZE+8][1]  | pixel_combine[l*GROUP_SIZE+9][1]  | pixel_combine[l*GROUP_SIZE+10][1] | pixel_combine[l*GROUP_SIZE+11][1] |
                          pixel_combine[l*GROUP_SIZE+12][1] | pixel_combine[l*GROUP_SIZE+13][1] | pixel_combine[l*GROUP_SIZE+14][1] | pixel_combine[l*GROUP_SIZE+15][1] ; 
        
            temp[0][l] = pixel_combine[l*GROUP_SIZE][0]    | pixel_combine[l*GROUP_SIZE+1][0]  | pixel_combine[l*GROUP_SIZE+2][0]  | pixel_combine[l*GROUP_SIZE+3][0]  |
                          pixel_combine[l*GROUP_SIZE+4][0]  | pixel_combine[l*GROUP_SIZE+5][0]  | pixel_combine[l*GROUP_SIZE+6][0]  | pixel_combine[l*GROUP_SIZE+7][0]  |
                          pixel_combine[l*GROUP_SIZE+8][0]  | pixel_combine[l*GROUP_SIZE+9][0]  | pixel_combine[l*GROUP_SIZE+10][0] | pixel_combine[l*GROUP_SIZE+11][0] |
                          pixel_combine[l*GROUP_SIZE+12][0] | pixel_combine[l*GROUP_SIZE+13][0] | pixel_combine[l*GROUP_SIZE+14][0] | pixel_combine[l*GROUP_SIZE+15][0] ; 
        end
    end
endgenerate

// RGB pixels going to display, take into account TRI_MAX and GROUP/GROUP_SIZE
generate
    genvar m;
    for (m=0; m<3; m=m+1) begin
        assign pixel[m] = temp[m][0] | temp[m][1] | temp[m][2] | temp[m][3] | temp[m][4] | temp[m][5] | temp[m][6] | temp[m][7];
    end
endgenerate


// HDMI transmit + clock/video timing
hdmi_transmit hdmi_transmit_inst (
    .rst_n       (1'b1),                  
    .clk125      (clk125),                          // 125 MHz input clock from Zynq
    .pixel       (pixel),                           // RGB pixel data to be encoded and transmitted
    
    // Shared video interface to the rest of the system
    .pixel_clk   (pixel_clk),                       // 75 MHz pixel clock
    .rst         (rst),                             // system reset
    .active      (active),                          // active pixel indicator
    .fsync       (fsync),                           // frame sync
    .hpos        (hpos),                            // horizontal position
    .vpos        (vpos),                            // vertical position
    
    // HDMI output
    .tmds_tx_clk_p  (tmds_tx_clk_p),
    .tmds_tx_clk_n  (tmds_tx_clk_n),
    .tmds_tx_data_p (tmds_tx_data_p),
    .tmds_tx_data_n (tmds_tx_data_n)
);

// Zynq interface
design_1_wrapper Zynq_intfc (
    .DDR_addr             (DDR_addr),
    .DDR_ba               (DDR_ba),
    .DDR_cas_n            (DDR_cas_n),
    .DDR_ck_n             (DDR_ck_n),
    .DDR_ck_p             (DDR_ck_p),
    .DDR_cke              (DDR_cke),
    .DDR_cs_n             (DDR_cs_n),
    .DDR_dm               (DDR_dm),
    .DDR_dq               (DDR_dq),
    .DDR_dqs_n            (DDR_dqs_n),
    .DDR_dqs_p            (DDR_dqs_p),
    .DDR_odt              (DDR_odt),
    .DDR_ras_n            (DDR_ras_n),
    .DDR_reset_n          (DDR_reset_n),
    .DDR_we_n             (DDR_we_n),
    .FIXED_IO_ddr_vrn     (FIXED_IO_ddr_vrn),
    .FIXED_IO_ddr_vrp     (FIXED_IO_ddr_vrp),
    .FIXED_IO_mio         (FIXED_IO_mio),
    .FIXED_IO_ps_clk      (FIXED_IO_ps_clk),
    .FIXED_IO_ps_porb     (FIXED_IO_ps_porb),
    .FIXED_IO_ps_srstb    (FIXED_IO_ps_srstb),
    .M_AXIS_0_tdata       (axis_tdata),
    .M_AXIS_0_tkeep       (axis_tkeep),
    .M_AXIS_0_tlast       (axis_tlast),
    .M_AXIS_0_tready      (axis_tready),
    .M_AXIS_0_tvalid      (axis_tvalid),
    .M_AXIS_CLK           (axis_clk),
    .RST_N                (rst_n),
    .interrupt            (fsync),
    .gpio                 (gpio),
    .duration             ({right_latched_fsync, left_latched_fsync, duration[29:0]})
);

assign axis_rst = axis_clk_rst[2];

always @(posedge axis_clk)
begin
    axis_clk_rst <= {rst, axis_clk_rst[0:1]};
    if (axis_rst) begin
        axis_tready <= 1'b0;
    end else begin
        axis_tready <= 1'b1;                    
    end
end

// Measure PS application execution time
always @(posedge axis_clk)
begin
    if (~gpio) begin
        duration <= 0;
    end else begin
        duration <= duration + 1;
    end
end


endmodule
