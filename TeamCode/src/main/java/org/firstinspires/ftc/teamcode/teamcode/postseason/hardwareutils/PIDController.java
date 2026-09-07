package org.firstinspires.ftc.teamcode.teamcode.postseason.hardwareutils;

import com.pedropathing.util.Timer;

public class PIDController {
    /*
    * Kp = Coefficient of the proportional response to error (i.e. if error is high, increase power to compensate)
    * Ki = Coefficient of steady-state compensation (this is best understood with a longer explanation; if desired, look it up)
    * Kd = Coefficient of damping; if the change in proportional response is too high, it weakens it to prevent oscillation
    * Kf = Feedforward coefficient; scales the output based on the raw setpoint magnitude (if higher setpoints will require more force); should NOT be used for position-based systems
    * Ks = Static constant; applies a constant small power to overcome friction and increase the snappiness of commands
    */
    public double Kp, Ki, Kd, Kf, Ks;
    public double lastMeasurement, lastTimestamp, integral = 0.0;
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

    @Override
    public String toString() {
        return String.format("PIDController(p=%g, i=%g, d=%g, f=%g, s=%g, error=%g)", Kp, Ki, Kd, Kf, Ks, lastMeasurement);
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
        // map coefficients
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
    // do not tune using lambda constants unless there is a well-understood reason to do so
    public void setCoefficients(double beta, double tau, double K, double lambda) {
        // compute PID coefficients from lambda-tuned constants
        Kp = ((1.0 + beta) * tau) / (K * lambda);
        Kd = beta * tau / K;
        // lambda tuned PIDs only use PD
        Ki = 0.0; Kf = 0.0; Ks = 0.0;
    }

    public double compute(double setpoint, double measurement) {
        double deltaTime = plantTimer.getElapsedTimeSeconds() - lastTimestamp;
        lastTimestamp = plantTimer.getElapsedTimeSeconds();

        // compute terms
        double error = setpoint - measurement;
        double derivative = -(measurement - lastMeasurement) / deltaTime;
        integral += error * deltaTime; // for most systems, this implementation of I won't work. Modify and use only if reasoning well-understood
        lastMeasurement = measurement;

        return Kp * error + Ki * integral + Kd * derivative + Kf * setpoint + Ks * Math.signum(error);
    }
}