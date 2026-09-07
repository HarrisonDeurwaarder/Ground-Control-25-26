package org.firstinspires.ftc.teamcode.teamcode.postseason.autoutils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.epsilon.ConstantsEpsilon;
import org.firstinspires.ftc.teamcode.teamcode.state.HardwareController;


abstract class AutoTemplate extends AutoBase {

    protected static Pose startingPose = new Pose(0.0, 0.0, Math.toRadians(90.0));
    public static Pose pose1 = new Pose(0.0, 0.0, Math.toRadians(45.0));
    public static Pose pose2 = new Pose(0.0, 0.0, Math.toRadians(-45.0));
    public static Pose defaultEndPose = new Pose(10.0, 0.0, Math.toRadians(0.0));

    protected PathChain toPose1, toPose2then1, toPose2, toEnd;
    protected Cycle firstDelayCycle, secondDelayCycle, finalCycle;

    /*
    Unlike TeleOpTemplate, common OpMode inherited methods (init, start, loop, etc) should likely not be overridden.
    As such, they can likely removed from this class.
     */
    @Override
    public void init() {
        super.init();
        // add extra init functionality below, or delete this method signature
        // ...
    }

    @Override
    public void init_loop() {
        super.init_loop();
        // add extra init_loop functionality below, or delete this method signature
        // ...
    }

    @Override
    public void start() {
        super.start();
        // add extra start functionality below, or delete this method signature
        // ...
    }

    @Override
    public void loop() {
        super.init();
        // add extra loop functionality below, or delete this method signature
        // ...
    }

    @Override
    public void stop() {
        super.init();
        // add extra stop functionality below, or delete this method signature
        // ...
    }

    protected void buildPaths() {
        // instantiate Path and PathChain objects here, using static poses defined in the class definition
        // these will be subsequently registered as cycles in registerPaths()
        toPose1 = follower.pathBuilder()
                .addPath(new BezierLine(startingPose, pose1)) // Bezier curves can be used for curved paths, which are often more efficient than lines if possible (retains velocity along curve)
                .setLinearHeadingInterpolation(startingPose.getHeading(), pose1.getHeading())
                .build();

        toPose2then1 = follower.pathBuilder()
                .addPath(new BezierLine(pose1, pose2))
                .addPath(new BezierLine(pose2, pose1)) // example of multiple paths
                .setTangentHeadingInterpolation() // tangential heading interpolation is more efficient
                .build();

        toPose2 = follower.pathBuilder()
                .addPath(new BezierLine(pose1, pose2))
                .setTangentHeadingInterpolation()
                // many "callbacks" exist for executing runnables mid-path
                .addParametricCallback(0.5, () -> {}) // executes the second argument (an empty runnable here) once the robot reaches the parametric halfway point
                .addTemporalCallback(1.5, () -> {}) // executes the runnable after 1.5 seconds on the current path
                .addCallback(() -> true, () -> {}) // executes the runnable after the boolean supplier returns true (which it always will, in this case)
                .build();

        toEnd = follower.pathBuilder()
                .addPath(new BezierCurve(pose2, pose1, defaultEndPose))
                .setTangentHeadingInterpolation()
                .build();
    }

    protected void registerPaths() {
        // register paths with AutoBuilder here
        // use Callbacks for executing runnables mid-path, but use AutoBuilder cycles for sequential auto building
        firstDelayCycle = new Cycle();
        firstDelayCycle.addDelay(3.0); // delay before following path
        firstDelayCycle.addFollowPedroPath(follower, toPose1);

        secondDelayCycle = new Cycle();
        secondDelayCycle.addDelay(2.0);
        secondDelayCycle.addPath(() -> {}, () -> true); // add an empty custom path that does nothing and continues immediately
        secondDelayCycle.addFollowPedroPath(follower, toPose2then1);
        secondDelayCycle.addFollowPedroPath(follower, toPose2);

        finalCycle = new Cycle();
        finalCycle.addFollowPedroPath(follower, toEnd);

        // add cycles to autobuilder
        autoBuilder.addCycle(firstDelayCycle);
        autoBuilder.addCycle(secondDelayCycle);
        autoBuilder.addCycle(firstDelayCycle); // the same cycle can be added twice (e.g. make one shoot cycle and add it several times)
        autoBuilder.addCycle(finalCycle);
    }
}