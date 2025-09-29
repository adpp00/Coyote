`timescale 1ns / 1ps
import lynxTypes::*;

/**
 * @brief   TX arbitration (512b AXI4S)
 *
 * - Regions -> (arb) -> m_tx_meta 로 메타 전달. 실제 enq 시점에 {vfid,len}를 시퀀싱 큐에 저장.
 * - 데이터는 선택된 VF에서만 ceil(len/64B) 비트 만큼 m_axis_tx로 라우팅.
 * - len==0 은 데이터 비트 없이 즉시 종료(상태는 별도 큐로 라우팅되므로 문제 없음).
 * - s_tx_stat 는 "수락된 전송의 순서"대로 준비된 VF 에 단 한 번 전달(별도 완료 큐 사용).
 */
module tcp_tx_arbiter (
    input  logic                                  aclk,
    input  logic                                  aresetn,

    // TX meta in from regions (arbitrated)
    metaIntf.s                                    s_tx_meta [N_REGIONS],    // STYPE: tcp_tx_meta_t
    metaIntf.m                                    m_tx_meta,                // STYPE: tcp_tx_meta_t

    // TX status from core, demuxed to regions (exactly once per transfer)
    metaIntf.s                                    s_tx_stat,                // STYPE: tcp_tx_stat_t
    metaIntf.m                                    m_tx_stat [N_REGIONS],    // STYPE: tcp_tx_stat_t

    // AXI4S data from regions (one selected at a time) -> single AXI4S out
    AXI4S.s                                       s_axis_tx [N_REGIONS],    // 512b
    AXI4S.m                                       m_axis_tx                 // 512b
);

  // ---------------------------------------------------------------------------
  // 0) Params
  // ---------------------------------------------------------------------------
  localparam int BEAT_LOG_BITS = $clog2(AXI_NET_BITS/8);  // 512b -> 64B -> 6
  localparam int SEQ_W         = N_REGIONS_BITS + TCP_LEN_BITS;

  // ---------------------------------------------------------------------------
  // 1) Meta arbitration: pick region + payload
  // ---------------------------------------------------------------------------
  logic [N_REGIONS_BITS-1:0] vfid_pick;
  metaIntf #(.STYPE(tcp_tx_meta_t)) tx_meta ();

  meta_arbiter #(.DATA_BITS($bits(tcp_tx_meta_t))) i_meta_arbiter (
    .aclk    (aclk),
    .aresetn (aresetn),
    .s_meta  (s_tx_meta),
    .m_meta  (tx_meta),
    .id_out  (vfid_pick)
  );

  // ---------------------------------------------------------------------------
  // 2) Single sequencing queue: {vfid, len}
  // ---------------------------------------------------------------------------
  metaIntf #(.STYPE(logic[SEQ_W-1:0])) seq_snk ();
  metaIntf #(.STYPE(logic[SEQ_W-1:0])) seq_src ();

  assign seq_snk.data = {vfid_pick, tx_meta.data.len};

  // enq 파이어 검출을 위해 snk handshake를 사용
  wire enq_fire;

  always_comb begin
    // 기본값
    seq_snk.valid   = 1'b0;
    tx_meta.ready   = 1'b0;
    m_tx_meta.valid = 1'b0;
    m_tx_meta.data  = tx_meta.data;

    if (tx_meta.valid) begin
      // m_tx_meta 가 지금 수락할 때만 큐 enq
      seq_snk.valid   = m_tx_meta.ready;
      tx_meta.ready   = seq_snk.ready & m_tx_meta.ready;
      m_tx_meta.valid = seq_snk.ready;     // 실제 enq 되는 사이클에 pulse
    end
  end

  assign enq_fire = seq_snk.valid & seq_snk.ready;

  // 시퀀싱 큐
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
  // 2.5) Completion queue for status routing (order-based)
  // ---------------------------------------------------------------------------
  metaIntf #(.STYPE(logic[N_REGIONS_BITS-1:0])) cmp_snk ();
  metaIntf #(.STYPE(logic[N_REGIONS_BITS-1:0])) cmp_src ();

  assign cmp_snk.data  = vfid_pick;
  assign cmp_snk.valid = enq_fire;  // 메타 enq 와 동일 사이클에 완료 큐 enq

  queue #(
    .QTYPE (logic [N_REGIONS_BITS-1:0]),
    .QDEPTH(N_OUTSTANDING)
  ) i_cmp_q (
    .aclk     (aclk),
    .aresetn  (aresetn),
    .val_snk  (cmp_snk.valid),
    .rdy_snk  (/*unused*/),         // 작은 큐 가정: overflow 없도록 N_OUTSTANDING 충분
    .data_snk (cmp_snk.data),
    .val_src  (cmp_src.valid),
    .rdy_src  (cmp_src.ready),
    .data_src (cmp_src.data)
  );

  // ---------------------------------------------------------------------------
  // 3) Data path FSM
  //    - Latch VFID/LEN at start of a transfer
  //    - Beats = ceil(len/64B); counter 0..n_beats
  // ---------------------------------------------------------------------------
  typedef enum logic [0:0] { ST_IDLE, ST_MUX } state_t;

  state_t                              state_C, state_N;
  logic [N_REGIONS_BITS-1:0]           vfid_C, vfid_N;
  logic [TCP_LEN_BITS-BEAT_LOG_BITS:0] cnt_C,  cnt_N;
  logic [TCP_LEN_BITS-BEAT_LOG_BITS:0] n_beats_C, n_beats_N;
  logic                                zero_len_C, zero_len_N;

  logic tr_fire;         // 데이터 비트 전달됨
  logic tr_done_data;    // 마지막 데이터 비트
  logic tr_done;         // 전체 전송 완료 (zero-len 포함)
  logic pop_seq;         // 시퀀싱 큐 pop

  // 레지스터
  always_ff @(posedge aclk) begin
    if (!aresetn) begin
      state_C     <= ST_IDLE;
      vfid_C      <= '0;
      cnt_C       <= '0;
      n_beats_C   <= '0;
      zero_len_C  <= 1'b0;
    end else begin
      state_C     <= state_N;
      vfid_C      <= vfid_N;
      cnt_C       <= cnt_N;
      n_beats_C   <= n_beats_N;
      zero_len_C  <= zero_len_N;
    end
  end

  // NSL
  always_comb begin
    state_N = state_C;
    unique case (state_C)
      ST_IDLE: begin
        if (seq_src.valid) state_N = ST_MUX;
      end
      ST_MUX: begin
        if (tr_done) state_N = (seq_src.valid) ? ST_MUX : ST_IDLE;
      end
    endcase
  end

  // Datapath
  always_comb begin
    // 기본값
    vfid_N     = vfid_C;
    cnt_N      = cnt_C;
    n_beats_N  = n_beats_C;
    zero_len_N = zero_len_C;
    pop_seq    = 1'b0;

    tr_fire      = m_axis_tx.tvalid & m_axis_tx.tready;
    tr_done_data = (cnt_C == n_beats_C) & tr_fire;
    tr_done      = tr_done_data | zero_len_C; 

    unique case (state_C)
      ST_IDLE: begin
        cnt_N = '0;

        if (seq_src.valid) begin
          pop_seq = 1'b1;

          logic [SEQ_W-1:0]       raw;
          logic [TCP_LEN_BITS-1:0] len;
          raw     = seq_src.data;
          len     = raw[TCP_LEN_BITS-1:0];
          vfid_N  = raw[TCP_LEN_BITS +: N_REGIONS_BITS];

          zero_len_N = (len == '0);

          if (len == '0) begin
            n_beats_N = '0;
          end else begin
            n_beats_N =
              (len[BEAT_LOG_BITS-1:0] != 0)
              ? len[TCP_LEN_BITS-1:BEAT_LOG_BITS]
              : (len[TCP_LEN_BITS-1:BEAT_LOG_BITS] - 1);
          end
        end
      end

      ST_MUX: begin
        if (tr_fire) begin
          cnt_N = cnt_C + 1;
        end

        if (tr_done) begin
          if (seq_src.valid) begin
            pop_seq = 1'b1;

            logic [SEQ_W-1:0]       raw;
            logic [TCP_LEN_BITS-1:0] len;
            raw     = seq_src.data;
            len     = raw[TCP_LEN_BITS-1:0];
            vfid_N  = raw[TCP_LEN_BITS +: N_REGIONS_BITS];

            zero_len_N = (len == '0);

            if (len == '0) begin
              n_beats_N = '0;
            end else begin
              n_beats_N =
                (len[BEAT_LOG_BITS-1:0] != 0)
                ? len[TCP_LEN_BITS-1:BEAT_LOG_BITS]
                : (len[TCP_LEN_BITS-1:BEAT_LOG_BITS] - 1);
            end

            cnt_N = '0;
          end
        end
      end
    endcase
  end

  assign seq_src.ready = pop_seq;

  // ---------------------------------------------------------------------------
  // 4) AXI4S data mux
  // ---------------------------------------------------------------------------
  assign m_axis_tx.tvalid = (state_C == ST_MUX && !zero_len_C) ? s_axis_tx[vfid_C].tvalid : 1'b0;
  assign m_axis_tx.tdata  = s_axis_tx[vfid_C].tdata;
  assign m_axis_tx.tkeep  = s_axis_tx[vfid_C].tkeep;
  assign m_axis_tx.tlast  = s_axis_tx[vfid_C].tlast;

  for (genvar i = 0; i < N_REGIONS; i++) begin : G_READY_FANOUT
    assign s_axis_tx[i].tready = (state_C == ST_MUX && !zero_len_C && i == vfid_C) ? m_axis_tx.tready : 1'b0;
  end

  // ---------------------------------------------------------------------------
  // 5) TX status mux (order-based)
  // ---------------------------------------------------------------------------
  for (genvar i = 0; i < N_REGIONS; i++) begin : G_STAT_MUX
    assign m_tx_stat[i].valid = (cmp_src.valid && (i == cmp_src.data)) ? s_tx_stat.valid : 1'b0;
    assign m_tx_stat[i].data  = s_tx_stat.data;
  end

  assign s_tx_stat.ready = (cmp_src.valid) ? m_tx_stat[cmp_src.data].ready : 1'b0;
  assign cmp_src.ready   = s_tx_stat.valid & s_tx_stat.ready;  // status handshake → pop

endmodule
