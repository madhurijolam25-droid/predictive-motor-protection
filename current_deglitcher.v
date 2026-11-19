// Reject a sudden current jump for the configured number of samples.
// The fast-trip detector remains connected directly to the raw sampled path.
module current_deglitcher #(
    parameter [11:0] JUMP_LIMIT = 12'd1000,
    parameter [7:0]  GLITCH_SAMPLES = 8'd2
) (
    input             clk,
    input             reset,
    input      [11:0] current_in,
    output reg [11:0] deglitched_current,
    output reg        deglitch_fault
);

    reg  [7:0] spike_count;
    wire [12:0] abs_jump;

    // Compare against the last accepted value, not the previous raw sample.
    assign abs_jump = (current_in >= deglitched_current)
                    ? (current_in - deglitched_current)
                    : (deglitched_current - current_in);

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            deglitched_current <= 12'd0;
            deglitch_fault     <= 1'b0;
            spike_count        <= 8'd0;
        end else if (abs_jump > JUMP_LIMIT) begin
            if ((GLITCH_SAMPLES <= 8'd1) ||
                (spike_count >= GLITCH_SAMPLES - 1'b1)) begin
                // The jump persisted long enough; accept the new value.
                deglitched_current <= current_in;
                deglitch_fault     <= 1'b0;
                spike_count        <= 8'd0;
            end else begin
                // Hold the last accepted value while qualifying the jump.
                deglitch_fault <= 1'b1;
                spike_count    <= spike_count + 8'd1;
            end
        end else begin
            // Normal-sized changes are accepted immediately.
            deglitched_current <= current_in;
            deglitch_fault     <= 1'b0;
            spike_count        <= 8'd0;
        end
    end
endmodule
