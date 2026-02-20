package org.firstinspires.ftc.teamcode.teamcode.state;

import com.pedropathing.util.Timer;

public class PIDController {
    public double Kp, Ki, Kd, Kf, Ks;
    public double lastError, lastTimestamp, integral = 0.0;
    private Timer plantTimer;

    public PIDController(double p, double i, double d, double f, double s) {
        setCoefficients(p, i, d, f, s);
        plantTimer = new Timer();
        plantTimer.resetTimer();
    }

    public PIDController(double beta, double tau, double K, double lambda) {
        setCoefficients(beta, tau, K, lambda);
        plantTimer = new Timer();
        plantTimer.resetTimer();
    }

    /**
     * Set the coefficients of the controller
     *
     * @param p proportional coefficient
     * @param i integral coefficient
     * @param d derivative coefficient
     * @param f feedforward coefficient
     * @param s static coefficient
     */
    public void setCoefficients(double p, double i, double d, double f, double s) {
        // Map coefficients
        Kp = p; Ki = i; Kd = d; Kf = f; Ks = s;
    }

    /**
     * Set the coefficients of the controller (by lambda constants)
     *
     * @param beta figure
     * @param tau it
     * @param K out
     * @param lambda ;)
     */
    public void setCoefficients(double beta, double tau, double K, double lambda) {
        // Compute PID coefficients from lambda-tuned constants
        Kp = ((1.0 + beta) * tau) / (K * lambda);
        Kd = beta * tau / K;
        // Lambda tuned PIDs only use PD
        Ki = 0.0; Kf = 0.0; Ks = 0.0;
    }

    public double compute(double setpoint, double measurement) {
        double deltaTime = plantTimer.getElapsedTimeSeconds() - lastTimestamp;
        lastTimestamp = plantTimer.getElapsedTimeSeconds();

        // Compute PID
        double error = setpoint - measurement;
        double derivative = (error - lastError) / deltaTime;
        integral += error * deltaTime;
        lastError = error;

        return Kp * error + Ki * integral + Kd * derivative; //+ Kf * setpoint + Ks * Math.signum(error);
    }
}