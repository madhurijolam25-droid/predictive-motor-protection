`timescale 1ns/1ps

// End-to-end behavioral checks for startup, faults, recovery, and lockout.
module tb_motor_protection_top;

    localparam [3:0] ST_NORMAL     = 4'd2;
    localparam [3:0] ST_SELF_CHECK = 4'd9;
    localparam [3:0] ST_LOCKOUT    = 4'd11;
    localparam [3:0] ST_RECOVERY   = 4'd10;
    localparam [2:0] SAFE          = 3'd0;

    reg clk;
    reg reset;
    reg [11:0] current_raw;
    reg [11:0] voltage_raw;
    reg [11:0] temperature_raw;
    reg sensor_valid_raw;

    wire pwm_out;
    wire [9:0] pwm_duty;
    wire [2:0] risk_level;
    wire fault;
    wire [3:0] fault_code;
    wire [3:0] fsm_state;
    wire [11:0] filtered_current;
    wire signed [12:0] delta_i;
    wire [11:0] predicted_current;
    wire [11:0] thermal_state;
    wire [23:0] power_estimate;
    wire [3:0] last_fault_code;
    wire [11:0] peak_current;
    wire [15:0] fault_duration;
    wire stall_fault;
    wire sensor_fault;
    wire persistent_fault;

    motor_protection_top dut (
        .clk(clk),
        .reset(reset),
        .current_raw(current_raw),
        .voltage_raw(voltage_raw),
        .temperature_raw(temperature_raw),
        .sensor_valid_raw(sensor_valid_raw),
        .pwm_out(pwm_out),
        .pwm_duty(pwm_duty),
        .risk_level(risk_level),
        .fault(fault),
        .fault_code(fault_code),
        .fsm_state(fsm_state),
        .filtered_current(filtered_current),
        .delta_i(delta_i),
        .predicted_current(predicted_current),
        .thermal_state(thermal_state),
        .power_estimate(power_estimate),
        .last_fault_code(last_fault_code),
        .peak_current(peak_current),
        .fault_duration(fault_duration),
        .stall_fault(stall_fault),
        .sensor_fault(sensor_fault),
        .persistent_fault(persistent_fault)
    );

    always #5 clk = ~clk;

    // Log meaningful status changes without printing on every clock edge.
    always @(fsm_state or fault or fault_code or pwm_duty or
             sensor_fault or persistent_fault) begin
        $display(
            "[%0t ns] state=%0d fault=%b code=%0d pwm_duty=%0d sensor_fault=%b persistent_fault=%b",
            $time, fsm_state, fault, fault_code, pwm_duty,
            sensor_fault, persistent_fault
        );
    end

    // Apply reset long enough to initialize synchronous and asynchronous logic.
    task apply_reset;
        begin
            reset = 1'b1;
            repeat (2) @(posedge clk);
            @(negedge clk);
            reset = 1'b0;
        end
    endtask

    // Fail with a useful message instead of waiting indefinitely for a state.
    task wait_for_state;
        input [3:0] expected_state;
        input integer max_cycles;
        integer cycles;
        begin
            cycles = 0;
            while ((fsm_state !== expected_state) &&
                   (cycles < max_cycles)) begin
                @(negedge clk);
                cycles = cycles + 1;
            end

            if (fsm_state !== expected_state) begin
                $fatal(
                    1,
                    "Timed out waiting for FSM state %0d; got %0d after %0d cycles",
                    expected_state, fsm_state, cycles
                );
            end
        end
    endtask

    task wait_for_emergency_overcurrent;
        input integer max_cycles;
        integer cycles;
        begin
            cycles = 0;
            while ((dut.emergency_current_fault !== 1'b1) &&
                   (cycles < max_cycles)) begin
                @(negedge clk);
                cycles = cycles + 1;
            end

            if (dut.emergency_current_fault !== 1'b1) begin
                $fatal(
                    1,
                    "Timed out waiting for emergency current threshold: raw=%0d sampled=%0d filtered=%0d persistent=%b sensor_fault=%b",
                    current_raw, dut.current_sample, filtered_current,
                    persistent_fault, sensor_fault
                );
            end

            #1;
            if (fault !== 1'b1 || fault_code !== 4'd2) begin
                $fatal(
                    1,
                    "Emergency current reached threshold but was not classified correctly: filtered=%0d emergency=%b fault=%b code=%0d",
                    filtered_current, dut.emergency_current_fault,
                    fault, fault_code
                );
            end

            if (pwm_out !== 1'b0) begin
                $fatal(1, "PWM output did not shut off when fault asserted");
            end
        end
    endtask

    task set_current_and_wait;
        input [11:0] value;
        input integer cycles;
        begin
            current_raw = value;
            repeat (cycles) @(negedge clk);
        end
    endtask

    // Reusable sensor ramp that drives the complete filtering/fault path.
    task trigger_overcurrent;
        begin
            $display("[%0t ns] Applying current ramp for overcurrent test", $time);
            set_current_and_wait(12'd1000, 12);
            set_current_and_wait(12'd2000, 12);
            set_current_and_wait(12'd3000, 12);
            current_raw = 12'd4000;

            wait_for_emergency_overcurrent(32);

            if (fault_code !== 4'd2) begin
                $fatal(1, "Expected emergency-current code 2, got %0d",
                       fault_code);
            end
        end
    endtask

    // Confirm recovery is gradual and ends at the SAFE duty target.
    task recover_to_normal;
        begin
            $display("[%0t ns] Waiting for recovery", $time);
            current_raw = 12'd100;

            wait_for_state(ST_RECOVERY, 80);

            if (fault !== 1'b0) begin
                $fatal(1, "Recovery started while a fault was still active");
            end

            repeat (10) @(negedge clk);

            if (pwm_duty == 10'd0 || pwm_duty >= 10'd800) begin
                $fatal(1, "Expected intermediate recovery duty, got %0d",
                       pwm_duty);
            end

            wait_for_state(ST_NORMAL, 100);

            if (risk_level !== SAFE) begin
                $fatal(1, "Expected SAFE risk after current cleared, got %0d",
                       risk_level);
            end

            if (pwm_duty !== 10'd800) begin
                $fatal(1, "Expected safe duty 800 after recovery, got %0d",
                       pwm_duty);
            end
        end
    endtask

    initial begin
        clk = 1'b0;
        reset = 1'b0;
        current_raw = 12'd100;
        voltage_raw = 12'd2500;
        temperature_raw = 12'd500;
        sensor_valid_raw = 1'b1;

        // Test 1: startup and safe operating point, including thermal input.
        $display("Test 1: startup, successful BIST, and safe motor operation");
        apply_reset();
        wait_for_state(ST_NORMAL, 20);
        repeat (3) @(negedge clk);

        if (risk_level !== SAFE) begin
            $fatal(1, "Expected SAFE risk level, got %0d", risk_level);
        end

        if (pwm_duty !== 10'd800) begin
            $fatal(1, "Expected safe PWM duty 800, got %0d", pwm_duty);
        end

        if (thermal_state !== 12'd500) begin
            $fatal(
                1,
                "Expected thermal state to include measured temperature 500, got %0d",
                thermal_state
            );
        end

        // Test 2: emergency trip, persistent-fault lockout, and fault logging.
        $display("Test 2: rising current triggers an overcurrent fault");
        trigger_overcurrent();

        if (filtered_current < 12'd3600) begin
            $fatal(
                1,
                "Current filter did not reach overcurrent threshold: raw=%0d sampled=%0d filtered=%0d",
                current_raw, dut.current_sample, filtered_current
            );
        end

        if (fault !== 1'b1) begin
            $fatal(
                1,
                "Expected overcurrent fault: raw=%0d sampled=%0d filtered=%0d emergency=%b risk=%0d state=%0d fault_code=%0d",
                current_raw, dut.current_sample, filtered_current,
                dut.emergency_current_fault, risk_level, fsm_state, fault_code
            );
        end

        if (fault_code !== 4'd2) begin
            $fatal(1, "Expected emergency-current code 2, got %0d",
                   fault_code);
        end

        repeat (30) @(negedge clk);

        if (pwm_out !== 1'b0 || pwm_duty !== 10'd0) begin
            $fatal(1, "PWM did not remain shut down while overcurrent stayed active");
        end

        if (fsm_state !== ST_LOCKOUT) begin
            $fatal(1, "Expected persistent overcurrent to reach lockout, got state %0d",
                   fsm_state);
        end

        if (last_fault_code !== 4'd2) begin
            $fatal(1, "Expected fault logger code 2, got %0d", last_fault_code);
        end

        // Test 3: a cleared fault qualifies before PWM recovery begins.
        $display("Test 3: cleared overcurrent completes cooldown and recovery ramp");
        current_raw = 12'd100;
        sensor_valid_raw = 1'b1;
        apply_reset();
        wait_for_state(ST_NORMAL, 20);

        trigger_overcurrent();
        current_raw = 12'd100;
        wait_for_state(ST_SELF_CHECK, 50);

        if (fault !== 1'b0) begin
            $fatal(1, "Self-check began while fault remained active");
        end

        repeat (7) @(negedge clk);

        if (fsm_state !== ST_SELF_CHECK) begin
            $fatal(1, "Recovery started before the 8-cycle clear qualification completed");
        end

        recover_to_normal();

        // Test 4: enforce the configured three-attempt recovery limit.
        $display("Test 4: repeated trips are limited to three recovery attempts");
        repeat (2) begin
            trigger_overcurrent();
            recover_to_normal();
        end

        trigger_overcurrent();
        current_raw = 12'd100;
        wait_for_state(ST_LOCKOUT, 80);

        if (pwm_out !== 1'b0 || pwm_duty !== 10'd0) begin
            $fatal(1, "Expected PWM shutdown after recovery-attempt limit");
        end

        // Test 5: sensor loss during operation shuts down and locks out.
        $display("Test 5: invalid sensor causes fault, shutdown, and lockout");
        current_raw = 12'd100;
        sensor_valid_raw = 1'b1;
        apply_reset();
        wait_for_state(ST_NORMAL, 20);

        sensor_valid_raw = 1'b0;
        repeat (4) @(negedge clk);

        if (sensor_fault !== 1'b1) begin
            $fatal(1, "Expected sensor diagnostics to assert sensor_fault");
        end

        if (fault_code !== 4'd1) begin
            $fatal(1, "Expected sensor fault code 1, got %0d", fault_code);
        end

        wait_for_state(ST_LOCKOUT, 30);

        if (pwm_duty !== 10'd0) begin
            $fatal(1, "Expected PWM duty 0 in lockout, got %0d", pwm_duty);
        end

        if (last_fault_code !== 4'd1) begin
            $fatal(1, "Expected fault logger code 1, got %0d", last_fault_code);
        end

        // Test 6: invalid sensor during startup fails BIST.
        $display("Test 6: failed startup BIST enters lockout");
        current_raw = 12'd100;
        sensor_valid_raw = 1'b0;
        apply_reset();
        wait_for_state(ST_LOCKOUT, 15);

        $display("PASS: all motor protection testbench scenarios completed");
        $finish;
    end

    // Stop the simulation with a diagnostic if it exceeds 10 us.
    initial begin
        #10000;
        $fatal(1, "Global testbench timeout at %0t ns", $time);
    end

endmodule