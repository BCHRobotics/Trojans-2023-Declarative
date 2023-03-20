package frc.robot.util.control;

import frc.robot.Constants.MISC;

public class PID {

    // Time Variables
    private long previousTime = 0;
    private long currentTime = 0;
    private double deltaTime = 0;
    private double previousError = 0;
    private double integral = 0;
    private double derivative = 0;

    // Input and Outpus
    private double input = 0;
    private double setpoint = 0;
    private double error = 0;
    private double output = 0;

    // PID coefficients
    private PIDConstants constants;

    public PID(PIDConstants inpuTerms) {
        this.constants = inpuTerms;
    }

    public void referenceTimer() {
        // Calculate delta t
        this.currentTime = System.nanoTime();
        this.deltaTime = ((double) (this.currentTime - this.previousTime) / 1e18);
        this.previousTime = this.currentTime;
    }

    public void resetTimer() {
        this.previousTime = 0;
        this.currentTime = 0;
        this.deltaTime = 0;
    }

    public void resetError() {
        this.error = 0;
        this.previousError = 0;
    }

    public void clearPID() {
        this.integral = 0;
        this.derivative = 0;
    }

    public void setTarget(double target) {
        this.setpoint = target;
    }

    public void setInput(double input) {
        this.input = input;
    }

    public boolean atSetpoint() {
        return MISC.WITHIN_TOLERANCE(this.input, this.setpoint, 1);
    }

    public void pushConstantsToDashboard(String label) {
        this.constants.pushToDashboard(label);
    }

    public void retrieveDashboardConstants() {
        if (this.constants.valuesChanged()) {
            this.constants.getFromDashboard();
        }
    }

    public boolean constantsChanged() {
        return this.constants.valuesChanged();
    }

    public void calculate() {
        // Calculate error, differnce between setpoint and measured value
        this.error = this.setpoint - this.input;

        // Calculate Integral and Derivative terms
        this.integral += this.error * this.deltaTime;
        this.derivative += (this.error - this.previousError) / (this.deltaTime);

        // Generate control signal
        this.output = ((this.constants.kP * this.error) + (this.constants.kI * this.integral)
                + (this.constants.kD * this.derivative)) + this.constants.kFF;
    }

    public double getOutput() {
        return this.output;
    }

}