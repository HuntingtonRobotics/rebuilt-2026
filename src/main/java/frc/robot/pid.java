package frc.robot;

public class pid {
    private double totalError = 0.0;
    private double integralLimit = 1.0;
    private double lastError = 0.0;

    public double calculate(double setpoint, double measurement, double deltaTime, double kP, double kI, double kD, double kF) {
        if (deltaTime <= 0) {
            return 0.0;
        }

        double error = setpoint - measurement;

        // Integral with clamping
        totalError += error * deltaTime;
        totalError = Math.max(-integralLimit, Math.min(integralLimit, totalError));

        // Derivative
        double derivative = (error - lastError) / deltaTime;
        lastError = error;

        // Feedforward
        double feedforward = kF * setpoint;

        return (kP * error) + (kI * totalError) + (kD * derivative) + feedforward;
    }

    public void reset() {
        totalError = 0.0;
        lastError = 0.0;
    }
}