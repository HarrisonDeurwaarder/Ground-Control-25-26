package org.firstinspires.ftc.teamcode.teamcode.postseason.autoutils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.epsilon.ConstantsEpsilon;
import org.firstinspires.ftc.teamcode.teamcode.state.HardwareController;


abstract class AutoBase extends OpMode {
    // auto timer
    protected Timer opmodeTimer;
    // pathing objects
    protected Follower follower;
    protected HardwareController hardwareController;
    protected AutoBuilder autoBuilder;

    // telemetry objects
    protected TelemetryPacket packet;
    protected FtcDashboard dashboard;

    // these poses will always be present
    protected static Pose startingPose;
    protected static Pose defaultEndPose; // auto intends to end here, but pose will be tracked regardless and saved to teleop

    @Override
    public void init() {
        // initialize timer
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        // initialize telemetry
        packet = new TelemetryPacket();
        dashboard = FtcDashboard.getInstance();

        // initialize and set up follower
        follower = ConstantsEpsilon.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();
        // initialize custom auto objects
        hardwareController = new HardwareController(hardwareMap);
        autoBuilder = new AutoBuilder(follower);

        // construct paths to register in AutoBuilder
        buildPaths();
        // add cycles to AutoBuilder
        registerPaths();
    }

    @Override
    public void init_loop() {
        // display telemetry
        addDefaultTelemetry();
        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void start() {
        // time from start not init
        opmodeTimer.resetTimer();
        // reset AutoBuilder
        autoBuilder.reset();
    }

    @Override
    public void loop() {
        // update auto objects
        follower.update();
        autoBuilder.update();

        // save position to blackboard in case of issues
        blackboard.put("Auto Pose", follower.getPose());

        // display telemetry
        addDefaultTelemetry();
        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {
        // save ending position to blackboard
        blackboard.put("Auto Pose", follower.getPose());
        // disable all motors
        follower.breakFollowing();
    }

    protected abstract void buildPaths();

    protected abstract void registerPaths();

    protected void displayTelemetryMessage() {
        telemetry.addLine("An AUTONOMOUS has been selected. Press start to begin.");
        telemetry.update();
    }

    /**
     * Add and publish default telemetry for auto
     */
    protected void addDefaultTelemetry() {
        // this method should be modified for game-specific data
        // pathing variables
        packet.put("Auto Length", autoBuilder.getAutoLength());
        packet.put("Cycle State", autoBuilder.getCycleState());
        packet.put("Cycle Timer", autoBuilder.getCycleTime());
        packet.put("Cycle Length", autoBuilder.getCurrentCycle().getCycleLength());
        packet.put("Path State", autoBuilder.getCurrentCycle().getPathState());
        packet.put("Path Timer", autoBuilder.getCurrentCycle().getPathTime());
        // localization variables
        packet.put("Current Target Pose", follower.getCurrentPath() != null ? follower.getCurrentPath().getPose(1.0) == null : null); // find the pose the at end (t=1.0) of the parametric curve
        packet.put("Position", follower.getPose());
        packet.put("Velocity", follower.getVelocity());
        packet.put("Distance Remaining", follower.getDistanceRemaining());
    }
}


