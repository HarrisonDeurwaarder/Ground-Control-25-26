package org.firstinspires.ftc.teamcode.teamcode.postseason.teleoputils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.epsilon.ConstantsEpsilon;
import org.firstinspires.ftc.teamcode.teamcode.state.HardwareController;

abstract class TeleOpTemplate extends TeleOpBase {

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
        // TeleOpBase doesn't implement start(), so no super method must be called
        // add start functionality below, or delete this method signature
        // ...
    }

    @Override
    public void loop() {
        super.loop();
        // add extra loop functionality below, or delete this method signature
        // ...
    }

    @Override
    public void stop() {
        super.stop();
        // add extra stop functionality below, or delete this method signature
        // ...
    }

    @Override
    protected double[] getDriveControls() {
        // modify these controls as necessary for the desired movement frame
        // note that these only apply to field-oriented controls, which are discouraged for most games
        double[] controls = {
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
        };
        return controls;
    }

    protected void updateControls() {
        // add all non-drivetrain controls in here
        // i.e. intake, macros, etc

        // make sure to use edge detection methods for toggles. examples:
        gamepad1.xWasPressed();
        gamepad2.crossWasReleased(); // these are both boolean suppliers (they will not execute functionality on their own)

        // check out methods and attributes for gamepad objects

        // example toggle for orientation
        if (gamepad1.aWasPressed()) robotCentric = !robotCentric; // toggle control orientation (this attribute is inherited)
        // example control for resetting odometry position to counteract drift
        // for something of this nature (high impact, low reversibility), ensure controls can not be accidentally triggered
        if (gamepad1.left_bumper && gamepad1.right_bumper) follower.setPose(recalibratedPose); // tell pedro that it is at the known recalibration pose

        // ...
    }

    protected void displayControls() {
        // fill with control information to be displayed on the driver hub. example:
        telemetry.addLine("Toggle Robot Centric - A");
        telemetry.addLine("Recalibrate Odometry - LB & RB");
        // if multiple drivers are used, consider adding a section header to differentiate
        // ...
    }
}


