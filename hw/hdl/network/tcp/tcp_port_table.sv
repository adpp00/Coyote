/**
  * Copyright (c) 2021, Systems Group, ETH Zurich
  * All rights reserved.
  *
  * (BSD 3-Clause License text omitted for brevity)
  */

`timescale 1ns / 1ps
import lynxTypes::*;

`timescale 1ns / 1ps
import lynxTypes::*;

/**
 * @brief TCP port table (no hash, tag for future use)
 *
 * Data word layout in RAM:
 *   [MSB:LSB] = {VALID(1), TAG(7), VFPGA(8)}
 */

module tcp_port_table (
    input  logic                                   aclk,
    input  logic                                   aresetn,

    // Listen requests from regions -> arbitrated to a single downstream req
    metaIntf.s                                     s_listen_req [N_REGIONS],
    metaIntf.m                                     m_listen_req,

    // Listen response from network -> demux to the requesting region
    metaIntf.s                                     s_listen_rsp,
    metaIntf.m                                     m_listen_rsp [N_REGIONS],

    // Side-B lookup used by conn_table notify fallback (read-only)
    input  logic [TCP_IP_PORT_BITS-1:0]            port_addr,   // full ip_port
    output logic [TCP_PORT_TABLE_DATA_BITS-1:0]    rsid_out     // {VALID,TAG,VFPGA}
);

    // -- Constants --------------------------------------------------------------
    localparam int DATA_BITS   = TCP_PORT_TABLE_DATA_BITS;        // e.g., 16 = {VALID(1), TAG(7), VFPGA(8)}
    localparam int PORT_BITS   = TCP_IP_PORT_BITS;                // ip_port width
    localparam int INDEX_BITS  = TCP_PORT_ORDER;                  // e.g., 9
    localparam int TAG_BITS    = PORT_BITS - INDEX_BITS;          // e.g., 7
    localparam int VFPGA_BITS  = 8;
    localparam int WSTRB_BITS  = (DATA_BITS+7)/8;

    // Data word layout: [MSB:LSB] = {VALID(1), TAG(7), VFPGA(8)}
    localparam int VFPGA_LSB   = 0;
    localparam int VFPGA_MSB   = VFPGA_LSB + VFPGA_BITS - 1;      // 7
    localparam int TAG_LSB     = VFPGA_MSB + 1;                   // 8
    localparam int TAG_MSB     = TAG_LSB + TAG_BITS - 1;          // 14
    localparam int VALID_BIT   = TAG_MSB + 1;                     // 15

    // -- Regs and signals -------------------------------------------------------
    typedef enum logic[2:0] {ST_IDLE, ST_LUP, ST_CHECK, ST_RSP_COL, ST_SEND, ST_RSP_WAIT} state_t;
    logic [2:0]                     state_C, state_N;

    logic [PORT_BITS-1:0]           port_C,  port_N;     
    logic [N_REGIONS_BITS-1:0]      vfid_C,  vfid_N;

    logic [INDEX_BITS-1:0]          index_C;
    logic [TAG_BITS-1:0]            tag_C;
    assign index_C = port_C[INDEX_BITS-1:0];                      
    assign tag_C   = port_C[INDEX_BITS+TAG_BITS-1:INDEX_BITS];    

    logic [WSTRB_BITS-1:0]          a_we;
    logic [DATA_BITS-1:0]           a_data_in;
    logic [DATA_BITS-1:0]           a_data_out;

    logic [DATA_BITS-1:0]           b_data_out;
    assign rsid_out = b_data_out;

    logic                           hit;
    assign hit = (a_data_out[VALID_BIT] == 1'b1);

    // -- Arbitration ------------------------------------------------------------
    metaIntf #(.STYPE(tcp_listen_req_t)) listen_req_arb ();
    logic [N_REGIONS_BITS-1:0] vfid_arb;

    meta_arbiter #(
        .DATA_BITS($bits(tcp_listen_req_t))
    ) i_tcp_port_arb_in (
        .aclk   (aclk),
        .aresetn(aresetn),
        .s_meta (s_listen_req),
        .m_meta (listen_req_arb),
        .id_out (vfid_arb)
    );

    // -- REG --------------------------------------------------------------------
    always_ff @(posedge aclk) begin : REG_LISTEN
        if (!aresetn) begin
            state_C <= ST_IDLE;
            port_C  <= '0;
            vfid_C  <= '0;
        end
        else begin
            state_C <= state_N;
            port_C  <= port_N;
            vfid_C  <= vfid_N;
        end
    end

    // -- NSL --------------------------------------------------------------------
    always_comb begin : NSL_LISTEN
        state_N = state_C;
        case (state_C)
            ST_IDLE:     state_N = listen_req_arb.valid ? ST_LUP : ST_IDLE;
            ST_LUP:      state_N = ST_CHECK;                       // 1-cycle read latency
            ST_CHECK:    state_N = hit ? ST_RSP_COL : ST_SEND;     // hit: already taken
            ST_RSP_COL:  state_N = m_listen_rsp[vfid_C].ready ? ST_IDLE : ST_RSP_COL;
            ST_SEND:     state_N = m_listen_req.ready ? ST_RSP_WAIT : ST_SEND;
            ST_RSP_WAIT: state_N = (s_listen_rsp.valid && s_listen_rsp.ready) ? ST_IDLE : ST_RSP_WAIT;
            default:     state_N = ST_IDLE;
        endcase
    end

    // -- DP ---------------------------------------------------------------------
    always_comb begin : DP_LISTEN
        port_N = port_C;
        vfid_N = vfid_C;

        listen_req_arb.ready = 1'b0;

        m_listen_req.valid   = 1'b0;
        m_listen_req.data    = port_C;

        s_listen_rsp.ready   = 1'b0;
        
        for (int i = 0; i < N_REGIONS; i++) begin
            m_listen_rsp[i].valid = 1'b0;
            m_listen_rsp[i].data  = '0;
        end

        a_we      = '0;
        a_data_in = '0;

        case (state_C)
            ST_IDLE: begin
                if (listen_req_arb.valid) begin
                    listen_req_arb.ready = 1'b1;
                    port_N = listen_req_arb.data.ip_port; // pick request
                    vfid_N = vfid_arb;
                end
            end

            ST_LUP: begin
                // wait 1 cycle for a_data_out
            end

            ST_CHECK: begin
                // if hit -> respond collision, else proceed to send
            end

            ST_RSP_COL: begin
                // collision: reply immediately to requester (failure)
                m_listen_rsp[vfid_C].valid = 1'b1;
                m_listen_rsp[vfid_C].data  = '0; 
            end

            ST_SEND: begin
                // forward listen req downstream
                m_listen_req.valid = 1'b1;
            end

            ST_RSP_WAIT: begin
                // forward response back only to requester
                s_listen_rsp.ready         = m_listen_rsp[vfid_C].ready;
                m_listen_rsp[vfid_C].valid = s_listen_rsp.valid;
                m_listen_rsp[vfid_C].data  = s_listen_rsp.data;

                if (s_listen_rsp.valid && s_listen_rsp.ready) begin
                    // On success, allocate {VALID,TAG,VFPGA} entry
                    if (s_listen_rsp.data[0]) begin
                        a_we = {WSTRB_BITS{1'b1}};              
                        a_data_in                      = '0;
                        a_data_in[VALID_BIT]           = 1'b1;
                        a_data_in[TAG_MSB:TAG_LSB]     = tag_C; 
                        a_data_in[VFPGA_MSB:VFPGA_LSB] = vfid_C[VFPGA_BITS-1:0];
                    end
                end
            end

            default: ;
        endcase
    end

    // -- RAM: WRITE_FIRST -------------------------------------------------------
    ram_tp_c #(
        .ADDR_BITS (INDEX_BITS),   // 9
        .DATA_BITS (DATA_BITS)     // 16: {VALID,TAG,VFPGA}
    ) inst_tcp_port_table (
        .clk        (aclk),

        .a_en       (1'b1),
        .a_we       (a_we),
        .a_addr     (index_C),                     
        .a_data_in  (a_data_in),
        .a_data_out (a_data_out),

        .b_en       (1'b1),
        .b_addr     (port_addr[INDEX_BITS-1:0]),    
        .b_data_out (b_data_out)
    );

endmodule