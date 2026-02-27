package org.firstinspires.ftc.teamcode.teamcode.state.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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


    public double clutchPower = 0.2;
    public double liftPower = 0.8;

    public double targetSpeed = 30.0;
    public boolean clutchEngaged = false;
    public int liftPosition = 0;
    public int liftIncrement = 100;

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
            if (gamepad1.rightBumperWasPressed()) clutchEngaged = !clutchEngaged;

            hardwareController.clutchRight.setPosition((clutchEngaged) ?
                    hardwareController.RIGHT_CLUTCH_LIFT_ANGLE :
                    hardwareController.RIGHT_CLUTCH_DRIVE_ANGLE);
            hardwareController.clutchLeft.setPosition((clutchEngaged) ?
                    hardwareController.LEFT_CLUTCH_LIFT_ANGLE :
                    hardwareController.LEFT_CLUTCH_DRIVE_ANGLE);

            if (!liftStarted) {
                if (gamepad1.left_trigger > 0.05)
                {
                    hardwareController.clutchLeft.setPosition(hardwareController.LEFT_CLUTCH_LIFT_ANGLE);
                    hardwareController.clutchRight.setPosition(hardwareController.RIGHT_CLUTCH_LIFT_ANGLE);
                    hardwareController.leftFront.setPower(-clutchPower);
                    hardwareController.rightFront.setPower(-clutchPower);
                }
                else {
                    hardwareController.leftFront.setPower(0.0);
                    hardwareController.rightFront.setPower(0.0);
                }
            }
            else {
                if (gamepad1.dpadUpWasPressed()) {
                    liftPosition -= liftIncrement;
                    hardwareController.rightFront.setTargetPosition(liftPosition);
                    hardwareController.leftFront.setTargetPosition(liftPosition);
                }
            }
            if (gamepad1.xWasPressed()) {
                liftStarted = true;
                hardwareController.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardwareController.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardwareController.rightFront.setPower(liftPower);
                hardwareController.leftFront.setPower(liftPower);
            }


            updateTelemetry();
        }
    }

    public void updateTelemetry() {
        packet.put("Left Motor Current", hardwareController.leftFront.getCurrent(CurrentUnit.MILLIAMPS));
        packet.put("Right Motor Current", hardwareController.rightFront.getCurrent(CurrentUnit.MILLIAMPS));
        packet.put("Left Motor Position", hardwareController.rightFront.getCurrentPosition());
        packet.put("Right Motor Position", hardwareController.rightFront.getCurrentPosition());
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