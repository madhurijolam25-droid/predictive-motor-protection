// Top-level wiring for the simulated motor-protection controller.
module motor_protection_top (
    input clk,
    input reset,

    input [11:0] current_raw,
    input [11:0] voltage_raw,
    input [11:0] temperature_raw,
    input sensor_valid_raw,

    output pwm_out,
    output [9:0] pwm_duty,
    output [2:0] risk_level,
    output fault,
    output [3:0] fault_code,
    output [3:0] fsm_state,
    output [11:0] filtered_current,
    output signed [12:0] delta_i,
    output [11:0] predicted_current,
    output [11:0] thermal_state,
    output [23:0] power_estimate,
    output [3:0] last_fault_code,
    output [11:0] peak_current,
    output [15:0] fault_duration,
    output stall_fault,
    output sensor_fault,
    output persistent_fault
);

    // Registered sensor values and internal control signals.
    wire [11:0] current_sample;
    wire [11:0] voltage_sample;
    wire [11:0] temperature_sample;
    wire sensor_valid;

    wire warning_current;
    wire danger_current;
    wire critical_current;
    wire emergency_current_fault;

    wire voltage_fault;
    wire thermal_fault;

    wire [7:0] persistence_count;
    wire [7:0] stall_count;

    wire bist_done;
    wire bist_fail;
    wire bist_active;

    wire motor_enable;
    wire recovery_active;
    wire recovery_complete;
    wire lockout;
    wire pwm_generated;

    // -----------------------------------------
    // ADC/input sampling
    // -----------------------------------------
    adc_interface U_ADC (
        .clk(clk),
        .reset(reset),
        .current_raw(current_raw),
        .voltage_raw(voltage_raw),
        .temperature_raw(temperature_raw),
        .sensor_valid_raw(sensor_valid_raw),
        .current_sample(current_sample),
        .voltage_sample(voltage_sample),
        .temperature_sample(temperature_sample),
        .sensor_valid(sensor_valid)
    );

    // -----------------------------------------
    // Current filter
    // -----------------------------------------
    current_filter U_FILTER (
        .clk(clk),
        .reset(reset),
        .current_in(current_sample),
        .filtered_current(filtered_current)
    );

    // -----------------------------------------
    // Current threshold comparator
    // -----------------------------------------
    current_comparator U_COMPARATOR (
        .current_in(filtered_current),
        .warning(warning_current),
        .danger(danger_current),
        .critical(critical_current),
        .emergency_fault(emergency_current_fault)
    );

    // -----------------------------------------
    // Trend
    // -----------------------------------------
    trend_detector U_TREND (
        .clk(clk),
        .reset(reset),
        .current_in(filtered_current),
        .delta_i(delta_i),
        .previous_current()
    );

    // -----------------------------------------
    // Prediction
    // -----------------------------------------
    risk_predictor #(
        .PREDICTION_HORIZON(3)
    ) U_PREDICTOR (
        .clk(clk),
        .reset(reset),
        .current_in(filtered_current),
        .delta_i(delta_i),
        .predicted_current(predicted_current)
    );

    // -----------------------------------------
    // Voltage and power
    // -----------------------------------------
    voltage_power_monitor U_POWER (
        .voltage_in(voltage_sample),
        .current_in(filtered_current),
        .voltage_fault(voltage_fault),
        .power_estimate(power_estimate)
    );

    // -----------------------------------------
    // Thermal estimator
    // -----------------------------------------
    thermal_estimator U_THERMAL (
        .clk(clk),
        .reset(reset),
        .current_in(filtered_current),
        .temperature_in(temperature_sample),
        .thermal_state(thermal_state),
        .thermal_fault(thermal_fault)
    );

    // -----------------------------------------
    // Persistence
    // -----------------------------------------
    persistence_detector U_PERSISTENCE (
        .clk(clk),
        .reset(reset),
        .high_risk(critical_current || danger_current),
        .persistent_fault(persistent_fault),
        .count(persistence_count)
    );

    // -----------------------------------------
    // Stall
    // -----------------------------------------
    stall_detector U_STALL (
        .clk(clk),
        .reset(reset),
        .current_in(filtered_current),
        .delta_i(delta_i),
        .pwm_duty(pwm_duty),
        .stall_fault(stall_fault),
        .stall_count(stall_count)
    );

    // -----------------------------------------
    // Sensor diagnostics
    // -----------------------------------------
    sensor_diagnostics U_DIAG (
        .clk(clk),
        .reset(reset),
        .current_in(current_sample),
        .sensor_valid(sensor_valid),
        .sensor_fault(sensor_fault),
        .invalid_count()
    );

    // -----------------------------------------
    // Risk classifier
    // -----------------------------------------
    risk_classifier U_RISK (
        .current_in(filtered_current),
        .predicted_current(predicted_current),
        .delta_i(delta_i),
        .risk_level(risk_level)
    );

    // -----------------------------------------
    // Fault classifier
    // -----------------------------------------
    fault_classifier U_FAULT (
        .emergency_current_fault(emergency_current_fault),
        .thermal_fault(thermal_fault),
        .voltage_fault(voltage_fault),
        .sensor_fault(sensor_fault),
        .stall_fault(stall_fault),
        .persistent_fault(persistent_fault),
        .fault(fault),
        .fault_code(fault_code)
    );

    // -----------------------------------------
    // BIST
    // -----------------------------------------
    bist_controller U_BIST (
        .clk(clk),
        .reset(reset),
        .sensor_valid(sensor_valid),
        .bist_done(bist_done),
        .bist_fail(bist_fail),
        .bist_active(bist_active)
    );

    // -----------------------------------------
    // Protection FSM
    // -----------------------------------------
    protection_fsm U_FSM (
        .clk(clk),
        .reset(reset),
        .risk_level(risk_level),
        .fault(fault),
        .bist_done(bist_done),
        .bist_fail(bist_fail),
        .recovery_complete(recovery_complete),
        .state(fsm_state),
        .motor_enable(motor_enable),
        .recovery_active(recovery_active),
        .lockout(lockout)
    );

    // -----------------------------------------
    // Adaptive PWM
    // -----------------------------------------
    adaptive_pwm U_ADAPTIVE_PWM (
        .clk(clk),
        .reset(reset),
        .risk_level(risk_level),
        .motor_enable(motor_enable),
        .recovery_active(recovery_active),
        .pwm_duty(pwm_duty)
    );

    // -----------------------------------------
    // PWM generator
    // -----------------------------------------
    pwm_generator U_PWM (
        .clk(clk),
        .reset(reset),
        .duty(pwm_duty),
        .pwm_out(pwm_generated)
    );

    // Gate the PWM command off immediately for a fault or disabled FSM state.
    assign pwm_out = pwm_generated && motor_enable && !fault;

    // -----------------------------------------
    // Fault logger
    // -----------------------------------------
    fault_logger U_LOGGER (
        .clk(clk),
        .reset(reset),
        .fault(fault),
        .fault_code(fault_code),
        .current_in(filtered_current),
        .last_fault_code(last_fault_code),
        .peak_current(peak_current),
        .fault_duration(fault_duration)
    );

endmodule
