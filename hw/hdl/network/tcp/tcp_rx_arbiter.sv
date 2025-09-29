`timescale 1ns / 1ps
import lynxTypes::*;

/**
 * RX arbitration 
 *
 */
module tcp_rx_arbiter (
    input  logic                                  aclk,
    input  logic                                  aresetn,

    // Read descriptors from regions (slave side), arbitrated to m_rd_pkg
    metaIntf.s                                    s_rd_pkg [N_REGIONS],  
    metaIntf.m                                    m_rd_pkg,              

    // RX meta in, broadcast-muxed to one region (exactly once per transfer)
    metaIntf.s                                    s_rx_meta,              
    metaIntf.m                                    m_rx_meta [N_REGIONS],  

    // AXI4S data stream in, routed to one region
    AXI4S.s                                       s_axis_rx,              
    AXI4S.m                                       m_axis_rx [N_REGIONS]   
);

  // ---------------------------------------------------------------------------
  // Params
  // ---------------------------------------------------------------------------
  localparam int BEAT_LOG_BITS = $clog2(AXI_NET_BITS/8);  // 512b → 64B → 6
  localparam int SEQ_W         = N_REGIONS_BITS + TCP_LEN_BITS;

  // ---------------------------------------------------------------------------
  // Arbiter: pick a region descriptor and output payload + selected VFID
  // ---------------------------------------------------------------------------
  logic [N_REGIONS_BITS-1:0] vfid_pick;
  metaIntf #(.STYPE(tcp_rd_pkg_t)) rd_pkg ();

  meta_arbiter #(.DATA_BITS($bits(tcp_rd_pkg_t))) i_meta_arb (
    .aclk    (aclk),
    .aresetn (aresetn),
    .s_meta  (s_rd_pkg),
    .m_meta  (rd_pkg),
    .id_out  (vfid_pick)
  );

  // ---------------------------------------------------------------------------
  // Sequencing queue: {vfid,len}
  // - Enqueue only when both queue and downstream m_rd_pkg accept (same cycle)
  // - m_rd_pkg.valid pulses exactly when item is actually enqueued (1:1 매칭)
  // ---------------------------------------------------------------------------
  metaIntf #(.STYPE(logic[SEQ_W-1:0])) seq_snk ();
  metaIntf #(.STYPE(logic[SEQ_W-1:0])) seq_src ();

  assign seq_snk.data = {vfid_pick, rd_pkg.data.len};

  always_comb begin
    seq_snk.valid  = 1'b0;
    rd_pkg.ready   = 1'b0;
    m_rd_pkg.valid = 1'b0;
    m_rd_pkg.data  = rd_pkg.data;

    if (rd_pkg.valid) begin
      seq_snk.valid  = m_rd_pkg.ready;
      rd_pkg.ready   = seq_snk.ready & m_rd_pkg.ready;
      m_rd_pkg.valid = seq_snk.ready;        // 실제 enq 시점 펄스
    end
  end

  queue #(
    .QTYPE (logic [SEQ_W-1:0]),
    .QDEPTH(N_OUTSTANDING)
  ) i_seq_q (
    .aclk     (aclk),
    .aresetn  (aresetn),
    .val_snk  (seq_snk.valid),
    .rdy_snk  (seq_snk.ready),
    .data_snk (seq_snk.data),
    .val_src  (seq_src.valid),
    .rdy_src  (seq_src.ready),
    .data_src (seq_src.data)
  );

  // ---------------------------------------------------------------------------
  // FSM (+ meta token)
  // - Latch VFID/LEN per transfer
  // - Beats = ceil(len/64B); counter runs 0..n_beats-1 (last_idx 저장)
  // ---------------------------------------------------------------------------
  typedef enum logic [0:0] { ST_IDLE, ST_MUX } state_t;

  state_t                              state_C, state_N;
  logic [N_REGIONS_BITS-1:0]           vfid_C, vfid_N;
  logic [TCP_LEN_BITS-BEAT_LOG_BITS:0] cnt_C,  cnt_N;
  logic [TCP_LEN_BITS-BEAT_LOG_BITS:0] n_last_C, n_last_N;  // last beat index
  logic                                zero_len_C, zero_len_N;
  logic                                meta_pending_C, meta_pending_N;    // 1 meta/transfer

  // ---- Registers ----
  always_ff @(posedge aclk) begin
    if (!aresetn) begin
      state_C        <= ST_IDLE;
      vfid_C         <= '0;
      cnt_C          <= '0;
      n_last_C       <= '0;
      zero_len_C     <= 1'b0;
      meta_pending_C <= 1'b0;
    end else begin
      state_C        <= state_N;
      vfid_C         <= vfid_N;
      cnt_C          <= cnt_N;
      n_last_C       <= n_last_N;
      zero_len_C     <= zero_len_N;
      meta_pending_C <= meta_pending_N;
    end
  end

  // ---------------------------------------------------------------------------
  // 1-beat registered slice between s_axis_rx and selected region
  // ---------------------------------------------------------------------------
  logic                                  pipe_v_C, pipe_v_N;
  logic [AXI_NET_BITS-1:0]               pipe_d_C, pipe_d_N;
  logic [AXI_NET_BITS/8-1:0]             pipe_k_C, pipe_k_N;
  logic                                  pipe_l_C, pipe_l_N;

  wire sel_ready      = (state_C == ST_MUX && !zero_len_C) ? m_axis_rx[vfid_C].tready : 1'b0;
  wire down_fire      = pipe_v_C & sel_ready;                                  
  wire up_ready_raw   = (!pipe_v_C) | down_fire;                               
  wire up_ready_gate  = (state_C == ST_MUX && !zero_len_C) ? up_ready_raw : 1'b0;
  wire up_fire        = s_axis_rx.tvalid & up_ready_gate;                      

  always_ff @(posedge aclk) begin
    if (!aresetn) begin
      pipe_v_C <= 1'b0;
      pipe_d_C <= '0;
      pipe_k_C <= '0;
      pipe_l_C <= 1'b0;
    end else begin
      pipe_v_C <= pipe_v_N;
      pipe_d_C <= pipe_d_N;
      pipe_k_C <= pipe_k_N;
      pipe_l_C <= pipe_l_N;
    end
  end

  always_comb begin
    pipe_v_N = pipe_v_C;
    pipe_d_N = pipe_d_C;
    pipe_k_N = pipe_k_C;
    pipe_l_N = pipe_l_C;

    if (down_fire) begin
      if (up_fire) begin
        pipe_v_N = 1'b1;
        pipe_d_N = s_axis_rx.tdata;
        pipe_k_N = s_axis_rx.tkeep;
        pipe_l_N = s_axis_rx.tlast;
      end else begin
        pipe_v_N = 1'b0;
      end
    end
    else if (up_fire) begin
      pipe_v_N = 1'b1;
      pipe_d_N = s_axis_rx.tdata;
      pipe_k_N = s_axis_rx.tkeep;
      pipe_l_N = s_axis_rx.tlast;
    end
  end

  // ---------------------------------------------------------------------------
  // Next-state & datapath (FSM + meta token + 카운터)
  // ---------------------------------------------------------------------------
  always_comb begin
    state_N        = state_C;
    vfid_N         = vfid_C;
    cnt_N          = cnt_C;
    n_last_N       = n_last_C;
    zero_len_N     = zero_len_C;
    meta_pending_N = meta_pending_C;

    logic tr_fire = down_fire;
    logic tr_last = tr_fire & (cnt_C == n_last_C);
    logic tr_done = (zero_len_C ? (~meta_pending_C) : tr_last);

    unique case (state_C)
      // -----------------------------
      ST_IDLE: begin
        cnt_N = '0;

        if (seq_src.valid) begin
          // 새 전송 래치
          pop_seq = 1'b1;

          logic [SEQ_W-1:0]         raw;
          logic [TCP_LEN_BITS-1:0]  len;
          raw     = seq_src.data;
          len     = raw[TCP_LEN_BITS-1:0];
          vfid_N  = raw[TCP_LEN_BITS +: N_REGIONS_BITS];

          zero_len_N     = (len == '0);
          meta_pending_N = 1'b1;  

          if (len == '0) begin
            n_last_N = '0;        
          end else begin
            // last_idx = ceil(len/64B) - 1
            n_last_N = (len[BEAT_LOG_BITS-1:0] != 0)
                     ?  len[TCP_LEN_BITS-1:BEAT_LOG_BITS]
                     : (len[TCP_LEN_BITS-1:BEAT_LOG_BITS] - 1);
          end

          state_N = ST_MUX;
        end
      end

      // -----------------------------
      ST_MUX: begin
        if (tr_fire) begin
          cnt_N = cnt_C + 1;
        end

        if (s_rx_meta.valid && m_rx_meta[vfid_C].ready && meta_pending_C) begin
          meta_pending_N = 1'b0;
        end

        if (tr_done) begin
          if (seq_src.valid) begin
            pop_seq = 1'b1;

            logic [SEQ_W-1:0]         raw;
            logic [TCP_LEN_BITS-1:0]  len;
            raw     = seq_src.data;
            len     = raw[TCP_LEN_BITS-1:0];
            vfid_N  = raw[TCP_LEN_BITS +: N_REGIONS_BITS];

            zero_len_N     = (len == '0);
            meta_pending_N = 1'b1;

            if (len == '0) begin
              n_last_N = '0;
            end else begin
              n_last_N = (len[BEAT_LOG_BITS-1:0] != 0)
                       ?  len[TCP_LEN_BITS-1:BEAT_LOG_BITS]
                       : (len[TCP_LEN_BITS-1:BEAT_LOG_BITS] - 1);
            end

            cnt_N   = '0;
            state_N = ST_MUX;
          end else begin
            state_N = ST_IDLE;
          end
        end
      end
    endcase
  end

  logic pop_seq;
  assign seq_src.ready = pop_seq;

  // ---------------------------------------------------------------------------
  // AXI4S data mux (registered)
  // ---------------------------------------------------------------------------
  for (genvar i = 0; i < N_REGIONS; i++) begin : G_DATA_MUX
    assign m_axis_rx[i].tvalid = (state_C == ST_MUX && !zero_len_C && i == vfid_C) ? pipe_v_C : 1'b0;
    assign m_axis_rx[i].tdata  = pipe_d_C;
    assign m_axis_rx[i].tkeep  = pipe_k_C;
    assign m_axis_rx[i].tlast  = pipe_l_C;
  end

  assign s_axis_rx.tready = up_ready_gate;

  // ---------------------------------------------------------------------------
  // RX meta mux (once per transfer)
  // ---------------------------------------------------------------------------
  for (genvar i = 0; i < N_REGIONS; i++) begin : G_META_MUX
    assign m_rx_meta[i].valid = (state_C == ST_MUX && meta_pending_C && (i == vfid_C)) ? s_rx_meta.valid : 1'b0;
    assign m_rx_meta[i].data  = s_rx_meta.data;
  end
  assign s_rx_meta.ready = (state_C == ST_MUX && meta_pending_C) ? m_rx_meta[vfid_C].ready : 1'b0;

endmodule
