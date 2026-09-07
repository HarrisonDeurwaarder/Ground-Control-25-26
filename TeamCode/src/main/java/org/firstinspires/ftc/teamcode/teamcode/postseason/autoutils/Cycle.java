package org.firstinspires.ftc.teamcode.teamcode.postseason.autoutils;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teamcode.postseason.hardwareutils.PIDController;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

public class Cycle {
    private final List<Runnable> paths = new ArrayList<>();
    private final List<BooleanSupplier> conditions = new ArrayList<>();
    private int pathState;
    private Timer pathTimer;
    private boolean hasCurrentPathBeenRun = false;

    public Cycle() {
        // initialize timer
        pathTimer = new Timer();
        reset();
    }

    public double getPathTime() { return pathTimer.getElapsedTimeSeconds(); }

    public double getPathState() { return pathState; }

    public int getCycleLength(){ return paths.size(); }

    public void addPath(Runnable path) { addPath(path, () -> true); };

    public void addPath(Runnable path, BooleanSupplier continueCondition) {
        // add to collectables
        paths.add(path);
        conditions.add(continueCondition);
    }

    /* DEFAULT PATHS */

    /**
     * Add a path: "delay"
     *
     * @param delaySeconds seconds to delay
     */
    public void addDelay(double delaySeconds) {
        addPath(
                () -> {}, // no action is taken
                () -> pathTimer.getElapsedTimeSeconds() >= delaySeconds
        );
    }

    /**
     * Add a path: "send power to a motor"
     *
     * @param motor motor
     * @param power power
     */
    public void addSetMotorPower(DcMotorEx motor, double power) {
        addPath(
                () -> motor.setPower(power),
                () -> true // no condition is needed
        );
    }

    /**
     * Add a path: "set motor to velocity via custom PID"
     *
     * @param motor motor
     * @param pidController custom pid controller
     * @param velocity target velocity (deg/sec)
     * @param allowedError double-sided error range to proceed (deg/sec)
     */
    public void addSetMotorVelocity(DcMotorEx motor, PIDController pidController, double velocity, double allowedError) {
        addPath(
                () -> motor.setPower(pidController.compute(velocity, motor.getVelocity(AngleUnit.DEGREES))), // compute power by PID
                () -> Math.abs(velocity - motor.getVelocity(AngleUnit.DEGREES)) <= allowedError // require the velocity to be in a range
        );
    }

    /**
     * Add a path: "set motor to velocity via default PID"
     *
     * @param motor motor
     * @param velocity target velocity (deg/sec)
     * @param allowedError double-sided error range to proceed (deg/sec)
     */
    public void addSetMotorVelocity(DcMotorEx motor, double velocity, double allowedError) {
        addPath(
                () -> motor.setVelocity(velocity, AngleUnit.DEGREES),
                () -> Math.abs(velocity - motor.getVelocity(AngleUnit.DEGREES)) <= allowedError // require the velocity to be in a range
        );
    }

    /**
     * Add a path: "follow pedro pathchain"
     *
     * @param follower follower object
     * @param pathChain pathchain to follow
     * @param maxPower maximum power to set
     */
    public void addFollowPedroPath(Follower follower, PathChain pathChain, double maxPower) {
        addPath(
                () -> follower.followPath(pathChain, maxPower, true),
                () -> !follower.isBusy()
        );
    }

    /**
     * Add a path: "follow pedro pathchain"
     *
     * @param follower follower object
     * @param pathChain pathchain to follow
     */
    public void addFollowPedroPath(Follower follower, PathChain pathChain) {
        addPath(
                () -> follower.followPath(pathChain, true),
                () -> !follower.isBusy()
        );
    }

    /**
     * Add a path: "set servo position"
     *
     * @param servo servo
     * @param position position
     */
    public void addSetServoPosition(Servo servo, double position) {
        addPath(
                () -> servo.setPosition(position),
                () -> true // no condition needed
        );
    }

    /**
     * Add a path: "set servo position with continue condition"
     *
     * @param servo servo
     * @param position position
     */
    public void addSetServoPosition(Servo servo, double position, BooleanSupplier continueCondition) {
        addPath(
                () -> servo.setPosition(position),
                continueCondition
        );
    }

    /**
     * Follow the next path and increment given the condition
     * Each path is only executed one upon being reached
     *
     * @return whether the paths of this cycle have been exhausted
     */
    public boolean nextPath() {
        // exit if invalid pathState at entry
        if (pathState == -1) return true;

        // execute current path
        if (!hasCurrentPathBeenRun) { // do nothing if the path has already been run
            paths.get(pathState).run();
            hasCurrentPathBeenRun = true;
        }
        // increment path state if continue condition is met
        if (conditions.get(pathState).getAsBoolean()) incrementPathState();
        return pathState == -1;
    }

    /**
     * Reset this cycle for the next iteration
     */
    public void reset() {
        // reset pathState
        pathState = 0;
        hasCurrentPathBeenRun = false;
        // reset timer
        pathTimer.resetTimer();
    }

    private void incrementPathState() {
        // increment counter or set to -1 on > n_paths case
        pathState++;
        if (pathState >= getCycleLength()) pathState = -1;
        hasCurrentPathBeenRun = false;
        // reset timer
        pathTimer.resetTimer();
    }
}
