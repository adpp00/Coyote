/**
 * This file is part of the Coyote <https://github.com/fpgasystems/Coyote>
 *
 * MIT Licence
 * Copyright (c) 2021-2025, Systems Group, ETH Zurich
 * All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

`timescale 1ns / 1ps
import lynxTypes::*;

/**
 * @brief TCP connection table
 *
 * - Arbitrates open/close requests from vFPGAs.
 * - On successful open, records SID->VFID mapping.
 * - Routes notifications by SID if present; otherwise by dst_port via tcp_port_table.
 * - TAG is unused; port table checks only the VALID bit.
 * - No invalidation on close (avoids write contention); map is updated on open-success only.
 */

module tcp_conn_table (
    input  logic                                   aclk,
    input  logic                                   aresetn,

    // Open / Close
    metaIntf.s                                     s_open_req [N_REGIONS],
    metaIntf.m                                     m_open_req,

    metaIntf.s                                     s_close_req [N_REGIONS],
    metaIntf.m                                     m_close_req,

    metaIntf.s                                     s_open_rsp,
    metaIntf.m                                     m_open_rsp [N_REGIONS],

    // Notify
    metaIntf.s                                     s_notify,
    metaIntf.m                                     m_notify [N_REGIONS],

    // Port-table B-port proxy (lookup only)
    output logic [TCP_IP_PORT_BITS-1:0]            port_addr,   // forward notify.dst_port
    input  logic [TCP_PORT_TABLE_DATA_BITS-1:0]    rsid_in      // [VALID ... VFID(7:0)]
);

    // -- Constants --------------------------------------------------------------
    localparam int PORT_BITS      = TCP_IP_PORT_BITS;
    localparam int ADDR_BITS      = TCP_IP_ADDRESS_BITS;
    localparam int SID_BITS       = TCP_SESSION_BITS;

    localparam int VFPGA_BITS     = 8;                              // VFID width used in port table
    localparam int SID_ADDR_BITS  = 8;                              // max 256 SIDs -> use lower 8 bits
    localparam int SID_DATA_BITS  = 1 + VFPGA_BITS;                 // {VALID(1), VFID(8)}
    localparam int SID_WSTRB_BITS = (SID_DATA_BITS+7)/8;

    localparam int PT_DATA_BITS   = TCP_PORT_TABLE_DATA_BITS;       // e.g., 16 = {VALID, TAG?, VFID}
    localparam int PT_VALID_BIT   = PT_DATA_BITS-1;                 // MSB = VALID

    // -- Regs and signals -------------------------------------------------------
    // Open FSM
    typedef enum logic [1:0] { ST_O_IDLE, ST_O_SEND, ST_O_RSP_WAIT } o_state_t;
    o_state_t                         o_state_C, o_state_N;

    logic [PORT_BITS-1:0]             o_port_C,  o_port_N;
    logic [ADDR_BITS-1:0]             o_addr_C,  o_addr_N;
    logic [N_REGIONS_BITS-1:0]        o_vfid_C,  o_vfid_N;

    // Open RSP capture
    logic                             open_rsp_fire;
    logic                             open_rsp_success;                 // s_open_rsp.data[0]
    logic [SID_BITS-1:0]              open_rsp_sid;

    // SID map (A=write, B=read)
    logic [SID_ADDR_BITS-1:0]         sid_waddr, sid_raddr;
    logic [SID_DATA_BITS-1:0]         sid_wdata, sid_rdata;
    logic [SID_WSTRB_BITS-1:0]        sid_we;

    // Notify FSM
    typedef enum logic [2:0] { N_IDLE, N_SID_LUP, N_PICK, N_PT_LOOKUP, N_ROUTE } n_state_t;
    n_state_t                         n_state_C, n_state_N;

    tcp_notify_t                      not_C, not_N;

    // SID hit / port-table result
    logic                             sid_hit;
    logic [7:0]                       vfid_from_sid_8;
    logic [7:0]                       vfid_from_port_8;
    logic                             pt_valid_now;

    // Route latches (glitch-free)
    logic                             route_sel_sid_C,  route_sel_sid_N;  // 1: SID route, 0: port route
    logic                             route_pt_valid_C, route_pt_valid_N; // latched port-table VALID
    logic [N_REGIONS_BITS-1:0]        route_vfid_C,     route_vfid_N;     // final VFID

    // -- Arbitration ------------------------------------------------------------
    // Open
    metaIntf #(.STYPE(tcp_open_req_t)) open_req_arb ();
    logic [N_REGIONS_BITS-1:0] vfid_open_arb;

    meta_arbiter #(
        .DATA_BITS($bits(tcp_open_req_t))
    ) i_tcp_open_req_arb_in (
        .aclk    (aclk),
        .aresetn (aresetn),
        .s_meta  (s_open_req),
        .m_meta  (open_req_arb),
        .id_out  (vfid_open_arb)
    );

    // Close (pass-through, no map invalidation)
    meta_arbiter #(
        .DATA_BITS($bits(tcp_close_req_t))
    ) i_tcp_close_req_arb_in (
        .aclk    (aclk),
        .aresetn (aresetn),
        .s_meta  (s_close_req),
        .m_meta  (m_close_req),
        .id_out  (/*unused*/)
    );

    // -- Decodes ----------------------------------------------------------------
    assign sid_hit          = sid_rdata[SID_DATA_BITS-1];     // VALID
    assign vfid_from_sid_8  = sid_rdata[VFPGA_BITS-1:0];
    assign vfid_from_port_8 = rsid_in[VFPGA_BITS-1:0];
    assign pt_valid_now     = rsid_in[PT_VALID_BIT];

    function automatic [N_REGIONS_BITS-1:0] vf_cast8 (input [7:0] v);
        vf_cast8 = v[N_REGIONS_BITS-1:0]; // N_REGIONS ≤ 256
    endfunction

    // -- REG (OPEN) -------------------------------------------------------------
    always_ff @(posedge aclk) begin : REG_OPEN
        if (!aresetn) begin
            o_state_C <= ST_O_IDLE;
            o_port_C  <= '0;
            o_addr_C  <= '0;
            o_vfid_C  <= '0;
        end
        else begin
            o_state_C <= o_state_N;
            o_port_C  <= o_port_N;
            o_addr_C  <= o_addr_N;
            o_vfid_C  <= o_vfid_N;
        end
    end

    // -- REG (NOTIFY) -----------------------------------------------------------
    always_ff @(posedge aclk) begin : REG_NOTIFY
        if (!aresetn) begin
            n_state_C        <= N_IDLE;
            not_C            <= '0;
            route_vfid_C     <= '0;
            route_sel_sid_C  <= 1'b0;
            route_pt_valid_C <= 1'b0;
        end
        else begin
            n_state_C        <= n_state_N;
            not_C            <= not_N;
            route_vfid_C     <= route_vfid_N;
            route_sel_sid_C  <= route_sel_sid_N;
            route_pt_valid_C <= route_pt_valid_N;
        end
    end

    // -- NSL (OPEN) -------------------------------------------------------------
    always_comb begin : NSL_OPEN
        o_state_N = o_state_C;
        case (o_state_C)
            ST_O_IDLE:     o_state_N = open_req_arb.valid ? ST_O_SEND : ST_O_IDLE;
            ST_O_SEND:     o_state_N = m_open_req.ready ? ST_O_RSP_WAIT : ST_O_SEND;
            ST_O_RSP_WAIT: o_state_N = (s_open_rsp.valid && m_open_rsp[o_vfid_C].ready) ? ST_O_IDLE : ST_O_RSP_WAIT;
            default:       o_state_N = ST_O_IDLE;
        endcase
    end

    // -- DP (OPEN) --------------------------------------------------------------
    always_comb begin : DP_OPEN
        // Defaults
        o_port_N = o_port_C;
        o_addr_N = o_addr_C;
        o_vfid_N = o_vfid_C;

        open_req_arb.ready         = 1'b0;
        m_open_req.valid           = 1'b0;
        m_open_req.data.ip_port    = o_port_C;
        m_open_req.data.ip_address = o_addr_C;

        for (int i = 0; i < N_REGIONS; i++) begin
            m_open_rsp[i].valid = 1'b0;
            m_open_rsp[i].data  = '0;
        end

        // SID map write defaults
        sid_we    = '0;
        sid_waddr = '0;
        sid_wdata = '0;

        // RSP capture defaults
        open_rsp_fire    = 1'b0;
        open_rsp_success = 1'b0;
        open_rsp_sid     = '0;

        // Open FSM datapath
        case (o_state_C)
            ST_O_IDLE: begin
                if (open_req_arb.valid) begin
                    open_req_arb.ready = 1'b1;
                    o_port_N = open_req_arb.data.ip_port;
                    o_addr_N = open_req_arb.data.ip_address;
                    o_vfid_N = vfid_open_arb;
                end
            end

            ST_O_SEND: begin
                m_open_req.valid           = 1'b1;
                m_open_req.data.ip_port    = o_port_C;
                m_open_req.data.ip_address = o_addr_C;
            end

            ST_O_RSP_WAIT: begin
                // Forward response only to the requester
                m_open_rsp[o_vfid_C].valid = s_open_rsp.valid;
                m_open_rsp[o_vfid_C].data  = s_open_rsp.data;

                if (s_open_rsp.valid && m_open_rsp[o_vfid_C].ready) begin
                    open_rsp_fire    = 1'b1;
                    open_rsp_success = s_open_rsp.data[0];    // success bit (LSB)
                    open_rsp_sid     = s_open_rsp.data.sid;   // SID field
                end
            end

            default: ;
        endcase

        // Update map on successful open (no close invalidation)
        if (open_rsp_fire && open_rsp_success) begin
            sid_we                     = {SID_WSTRB_BITS{1'b1}};
            sid_waddr                  = open_rsp_sid[SID_ADDR_BITS-1:0];
            sid_wdata[SID_DATA_BITS-1] = 1'b1;                           // VALID
            sid_wdata[VFPGA_BITS-1:0]  = o_vfid_C[VFPGA_BITS-1:0];       // owner VFID (8b)
        end
    end

    // -- READY (OPEN RSP) -------------------------------------------------------
    always_comb begin : READY_OPEN_RSP
        s_open_rsp.ready = (o_state_C == ST_O_RSP_WAIT) ? m_open_rsp[o_vfid_C].ready : 1'b0;
    end

    // -- NSL (NOTIFY) -----------------------------------------------------------
    always_comb begin : NSL_NOTIFY
        n_state_N = n_state_C;
        case (n_state_C)
            N_IDLE:      n_state_N = s_notify.valid ? N_SID_LUP : N_IDLE;
            N_SID_LUP:   n_state_N = N_PICK;                 // 1-cycle SID-map read latency
            N_PICK:      n_state_N = sid_hit ? N_ROUTE : N_PT_LOOKUP;
            N_PT_LOOKUP: n_state_N = N_ROUTE;                // 1-cycle wait for port table output
            N_ROUTE:     n_state_N = N_ROUTE;                // DP will advance to IDLE on ready
            default:     n_state_N = N_IDLE;
        endcase
    end

    // -- DP (NOTIFY) ------------------------------------------------------------
    always_comb begin : DP_NOTIFY
        // Defaults
        not_N = not_C;

        // 1-entry input buffer behavior
        s_notify.ready = (n_state_C == N_IDLE);

        // SID-map read address (truncate)
        sid_raddr = not_C.sid[SID_ADDR_BITS-1:0];

        // Output defaults
        for (int i = 0; i < N_REGIONS; i++) begin
            m_notify[i].valid = 1'b0;
            m_notify[i].data  = not_C;
        end
        port_addr = '0;

        // Route latches
        route_vfid_N     = route_vfid_C;
        route_sel_sid_N  = route_sel_sid_C;
        route_pt_valid_N = route_pt_valid_C;

        case (n_state_C)
            N_IDLE: begin
                if (s_notify.valid) begin
                    not_N = s_notify.data; // capture payload
                end
            end

            N_SID_LUP: begin
                // sid_rdata valid next cycle
            end

            N_PICK: begin
                if (sid_hit) begin
                    route_vfid_N     = vf_cast8(vfid_from_sid_8);
                    route_sel_sid_N  = 1'b1;      // choose SID route
                    route_pt_valid_N = 1'b0;
                end
                else begin
                    // trigger port-table lookup by sending full port
                    port_addr        = not_C.dst_port;
                    route_sel_sid_N  = 1'b0;      // choose port route
                    route_pt_valid_N = 1'b0;      // will latch in next state
                end
            end

            N_PT_LOOKUP: begin
                // latch rsid_in after 1-cycle wait
                if (!sid_hit) begin
                    route_pt_valid_N = pt_valid_now;
                    route_vfid_N     = vf_cast8(vfid_from_port_8);
                end
            end

            N_ROUTE: begin
                // use only latched values
                logic route_has_dst;
                route_has_dst = route_sel_sid_C || route_pt_valid_C;

                if (route_has_dst) begin
                    m_notify[route_vfid_C].valid = 1'b1;
                    m_notify[route_vfid_C].data  = not_C;

                    if (m_notify[route_vfid_C].ready) begin
                        route_sel_sid_N  = 1'b0;
                        route_pt_valid_N = 1'b0;
                        n_state_N        = N_IDLE;
                    end
                end
                else begin
                    // drop on no destination, then return to IDLE
                    n_state_N = N_IDLE;
                end
            end

            default: ;
        endcase
    end

    ram_tp_c #(
        .ADDR_BITS (SID_ADDR_BITS),   // 8
        .DATA_BITS (SID_DATA_BITS)    // 9: {VALID, VFID[7:0]}
    ) i_sid2vfid (
        .clk        (aclk),

        .a_en       (1'b1),
        .a_we       (sid_we),
        .a_addr     (sid_waddr),
        .a_data_in  (sid_wdata),
        .a_data_out (/*unused*/),

        .b_en       (1'b1),
        .b_addr     (sid_raddr),
        .b_data_out (sid_rdata)
    );

endmodule
