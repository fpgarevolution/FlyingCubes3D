/***************************************************************
 * FPGA Revolution Open Bootcamp
 * Episode 34 - Flying Cubes on FPGA
 *
 * Transition-minimized differential signaling (TMDS) encoder
 **************************************************************/
 
module tmds_encode (
    input pixel_clk,                                 // pixel clock
    input rst,                                       // reset
    
    input [1:0] ctl,                                 // control bits
    input active,                                    // active pixel indicator
    input [7:0] pdata,                               // 8-bit pixel data
    output [9:0] tmds_data                           // encoded 10-bit TMDS data
);

integer i, j;
reg [8:0] p_xor, p_xor_1;                            // XOR op result
reg [8:0] p_xnor, p_xnor_1;                          // XNOR op result

// Stage 1, pipeline 1
reg active_1;
reg [1:0] ctl_1;
reg [7:0] pdata_1;
reg [3:0] ones_1;                                    // number of ones in pixel data
reg [3:0] ones_xor;                                  // number of ones in XORed data
reg [3:0] ones_xnor;                                 // number of ones in XNORed data

// Stage 1, pipeline 2
reg active_2;
reg [1:0] ctl_2;
reg [8:0] pdata_2;
reg [3:0] ones_2;                                    // number of ones in chosen XORed or XNORed data
reg [3:0] zeros_2;                                   // number of zeros in chosen XORed or XNORed data
reg signed [5:0] diff_2;                             // difference between number of ones and zeros in chosen XORed or XNORed data

// Stage 2
reg signed [5:0] disparity;                          // running disparity
reg [9:0] tdata;                                     // encoded TMDS data

// XOR op
always @(*)
begin
    p_xor[0] = pdata[0];
    for (i=1; i<8; i=i+1) begin
        p_xor[i] = p_xor[i-1] ^ pdata[i];
    end
    p_xor[8] = 1'b1;
end
// XNOR op
always @(*)
begin
    p_xnor[0] = pdata[0];
    for (j=1; j<8; j=j+1) begin
        p_xnor[j] = p_xnor[j-1] ~^ pdata[j];
    end
    p_xnor[8] = 1'b0;
end

// Stage 1, pipeline 1
always @(posedge pixel_clk)
begin
    active_1  <= active;
    ctl_1     <= ctl;
    pdata_1   <= pdata;
    
    ones_1    <= pdata[0] + pdata[1] + pdata[2] + pdata[3] + pdata[4] + pdata[5] + pdata[6] + pdata[7];
    ones_xor  <= p_xor[0] + p_xor[1] + p_xor[2] + p_xor[3] + p_xor[4] + p_xor[5] + p_xor[6] + p_xor[7];
    ones_xnor <= p_xnor[0] + p_xnor[1] + p_xnor[2] + p_xnor[3] + p_xnor[4] + p_xnor[5] + p_xnor[6] + p_xnor[7];

    p_xor_1  <= p_xor;
    p_xnor_1 <= p_xnor;
end

// Stage 1, pipeline 2
always @(posedge pixel_clk)
begin
    active_2  <= active_1;
    ctl_2     <= ctl_1;
    
    if ((ones_1 >4) || (ones_1 == 4 && pdata_1[0] == 1'b0)) begin
        pdata_2  <= p_xnor_1;
        ones_2   <= ones_xnor;
        zeros_2  <= 8 - ones_xnor;
        diff_2   <= $signed(ones_xnor) + $signed(ones_xnor) - 8;
    end else begin
        pdata_2  <= p_xor_1;
        ones_2   <= ones_xor;
        zeros_2  <= 8 - ones_xor;
        diff_2   <= $signed(ones_xor) + $signed(ones_xor) - 8;
    end
end

// Stage 2
always @(posedge pixel_clk)
begin
    if (rst) begin
        tdata <= 10'b 11_0101_0100;
        disparity <= 0;
    end else if (~active_2) begin                            // pixels not active
        // Control data encoding
        if (ctl == 2'b00) begin
            tdata <= 10'b 11_0101_0100;
        end else if (ctl == 2'b01) begin
            tdata <= 10'b 00_1010_1011;
        end else if (ctl == 2'b10) begin
            tdata <= 10'b 01_0101_0100;
        end else begin
            tdata <= 10'b 10_1010_1011;
        end
        
        disparity <= 0;                                      // reset running disparity
        
    end else begin                                           // pixels active
        // DC balance
        if ((disparity == 0) || (ones_2 == 4)) begin
            if (pdata_2[8]) begin
                tdata <= {1'b0, 1'b1, pdata_2[7:0]};
                disparity <= disparity + diff_2;
            end else begin
                tdata <= {1'b1, 1'b0, ~pdata_2[7:0]};
                disparity <= disparity - diff_2;
            end
        end else begin
            if ((disparity > 0 && ones_2 > 4) || (disparity < 0 && ones_2 < 4)) begin
                if (pdata_2[8]) begin
                    tdata <= {1'b1, 1'b1, ~pdata_2[7:0]};
                    disparity <= disparity - diff_2 + 2;
                end else begin
                    tdata <= {1'b1, 1'b0, ~pdata_2[7:0]};
                    disparity <= disparity - diff_2;
                end
            end else begin
                if (pdata_2[8]) begin
                    tdata <= {1'b0, 1'b1, pdata_2[7:0]};
                    disparity <= disparity + diff_2;
                end else begin
                    tdata <= {1'b0, 1'b0, pdata_2[7:0]};
                    disparity <= disparity + diff_2 - 2;
                end
            end
        end
    end
end

assign tmds_data = tdata;

endmodule
