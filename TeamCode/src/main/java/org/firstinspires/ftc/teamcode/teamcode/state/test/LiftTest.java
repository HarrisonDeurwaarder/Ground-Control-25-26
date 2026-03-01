package org.firstinspires.ftc.teamcode.teamcode.state.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.epsilon.ConstantsEpsilon;
import org.firstinspires.ftc.teamcode.teamcode.state.HardwareController;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Lift Test", group="Test")
public class LiftTest extends LinearOpMode {
    private Timer opmodeTimer;
    private Follower follower;
    private HardwareController hardwareController;
    private TelemetryPacket packet;
    private FtcDashboard dashboard;


    public static double clutchPower = 0.5;
    public double leftPower = 0.0;
    public double rightPower = 0.0;
    public static double powerModifier = 0.04;
    public boolean clutchEngaged = false;
    public double yaw, pitch, roll;
    public double pitchOffset = 0.0;
    public boolean liftStarted = false;



    @Override
    public void runOpMode() {

        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = ConstantsEpsilon.createFollower(hardwareMap);
        follower.setStartingPose(new Pose());
        follower.update();

        // Instanciate controllers
        hardwareController = new HardwareController(hardwareMap);
        packet = new TelemetryPacket();
        dashboard = FtcDashboard.getInstance();

        hardwareController.clutchLeft.setPosition(HardwareController.LEFT_CLUTCH_DRIVE_ANGLE);
        hardwareController.clutchRight.setPosition(HardwareController.RIGHT_CLUTCH_DRIVE_ANGLE);

        hardwareController.leftFront.setPower(0.0);
        hardwareController.rightFront.setPower(0.0);
        waitForStart();

        /* ###############################
                        START
           ############################### */

        while (opModeIsActive()) {
            /* NON-DRIVING CONTROLS */

            // Switch gate to closed only if robot is not feeding
            if (gamepad1.right_bumper && gamepad1.left_bumper) {
                hardwareController.leftFront.setPower(-leftPower);
                hardwareController.rightFront.setPower(-rightPower);
            }
            else if (gamepad1.right_bumper) {
                if (!liftStarted){
                    liftStarted = true;
                    pitchOffset = 0.0 - hardwareController.imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
                }
                hardwareController.clutchRight.setPosition(hardwareController.RIGHT_CLUTCH_LIFT_ANGLE);
                hardwareController.clutchLeft.setPosition(hardwareController.LEFT_CLUTCH_LIFT_ANGLE);
                hardwareController.leftFront.setPower(-clutchPower);
                hardwareController.rightFront.setPower(-clutchPower);

                hardwareController.leftBack.setPower(clutchPower);
                hardwareController.rightBack.setPower(clutchPower);
            }
            else {
                hardwareController.leftFront.setPower(0.0);
                hardwareController.rightFront.setPower(0.0);

                hardwareController.leftBack.setPower(0.0);
                hardwareController.rightBack.setPower(0.0);
            }



            yaw = hardwareController.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            roll = hardwareController.imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);
            pitch = hardwareController.imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);

            leftPower = Math.min(Math.max(1.0 + powerModifier * (pitch + pitchOffset), 0.0), 1.0);
            rightPower = Math.min(Math.max(1.0 - powerModifier * (pitch + pitchOffset), 0.0), 1.0);

            updateTelemetry();
        }
    }

    public void updateTelemetry() {
        packet.put("Left Motor Current", hardwareController.leftFront.getCurrent(CurrentUnit.MILLIAMPS));
        packet.put("Right Motor Current", hardwareController.rightFront.getCurrent(CurrentUnit.MILLIAMPS));
        packet.put("Left Motor Position", hardwareController.leftFront.getCurrentPosition());
        packet.put("Right Motor Position", hardwareController.rightFront.getCurrentPosition());

        packet.put("_Yaw", yaw);
        packet.put("_Roll", roll);
        packet.put("_Pitch", pitch);
        packet.put("LeftPower", leftPower);
        packet.put("RightPower", rightPower);
        packet.put("PitchOffset", pitchOffset);



        dashboard.sendTelemetryPacket(packet);

        // Controls (On driver hub telemetry)
        telemetry.addLine("A - Precision Mode");
        telemetry.addLine("B - Movement Center");
        telemetry.addLine("X - Auto Aim Turret");
        telemetry.addLine("Y - Enable Flywheel\n");

        telemetry.addLine("LT - Intake");
        telemetry.addLine("LB - Outtake\n");

        telemetry.addLine("RT - Feed");

        telemetry.update();
    }
}