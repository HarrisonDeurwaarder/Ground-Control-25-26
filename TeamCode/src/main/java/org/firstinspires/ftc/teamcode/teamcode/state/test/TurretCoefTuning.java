/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teamcode.state.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.epsilon.ConstantsEpsilon;
import org.firstinspires.ftc.teamcode.teamcode.state.HardwareController;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Turret Regression Tuner State", group="Test")
public class TurretCoefTuning extends LinearOpMode {
    private Timer opmodeTimer;
    private Follower follower;
    private HardwareController hardwareController;
    private TelemetryPacket packet;
    private FtcDashboard dashboard;

    private static double hoodZero = 0.19;
    private double timeState = 0.0;
    public static Pose startingPose = new Pose(0.0, 0.0, Math.toRadians(90.0));
    public static Pose goalPose = new Pose(60.0, 60.0);

    // Tuning variables
    public static double flywheelTargetSpeed = 30.0; // RPS
    public static double hoodPosition = 0.0; //

    public double targetSpeed = 30.0;

    @Override
    public void runOpMode() {

        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // Instanciate controllers
        hardwareController = new HardwareController(hardwareMap);

        // Configure follower
        follower = ConstantsEpsilon.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();

        packet = new TelemetryPacket();
        dashboard = FtcDashboard.getInstance();

        follower.startTeleOpDrive(true);

        waitForStart();

        /* ###############################
                        START
           ############################### */

        while (opModeIsActive()) {

            hardwareController.turretRotation.setTargetPosition((int) (-7.0));

            follower.update();
            follower.setTeleOpDrive(
                    gamepad1.left_stick_x,
                    -gamepad1.left_stick_y,
                    -gamepad1.right_stick_x,
                    false
            );

            /* NON-DRIVING CONTROLS */

            // Switch gate to closed only if robot is not feeding
            if (gamepad1.right_trigger < 0.05) hardwareController.gate.setPosition(HardwareController.CLOSED_ANGLE);

            // FEEDING CONDITIONAL
            // When trigger is held and flywheel velocity is acceptable, feed
            if (gamepad1.right_trigger >= 0.05) {
                // Switch gate to open
                hardwareController.gate.setPosition(HardwareController.OPEN_ANGLE);
                // Switch intake mode to [intake] if needed
                if (!hardwareController.intake.getDirection().equals(DcMotorSimple.Direction.FORWARD)) {
                    hardwareController.intake.setDirection(DcMotorSimple.Direction.REVERSE);
                    hardwareController.transfer.setDirection(DcMotorSimple.Direction.REVERSE);
                }
                // Then feed and intake
                hardwareController.intake.setPower(HardwareController.INTAKE_POWER);
                hardwareController.transfer.setPower(HardwareController.TRANSFER_POWER);
            }

            // INTAKE CONDITIONAL
            // When trigger is held, intake
            else if (gamepad1.left_trigger >= 0.05) {
                // Switch intake mode to reverse if needed
                if (hardwareController.intake.getDirection().equals(DcMotorSimple.Direction.FORWARD)) {
                    hardwareController.intake.setDirection(DcMotorSimple.Direction.REVERSE);
                    hardwareController.transfer.setDirection(DcMotorSimple.Direction.REVERSE);
                }
                // Then power intake and gate
                hardwareController.intake.setPower(HardwareController.INTAKE_POWER);
                hardwareController.transfer.setPower(HardwareController.TRANSFER_POWER);
            }

            // OUTTAKE CONDITIONAL
            // When trigger is held, intake
            else if (gamepad1.left_bumper) {
                // Switch intake mode to reverse if needed
                if (hardwareController.intake.getDirection().equals(DcMotorSimple.Direction.REVERSE)) {
                    hardwareController.intake.setDirection(DcMotorSimple.Direction.FORWARD);
                    hardwareController.transfer.setDirection(DcMotorSimple.Direction.FORWARD);
                }
                // Then power intake and gate
                hardwareController.intake.setPower(HardwareController.INTAKE_POWER);
                hardwareController.transfer.setPower(HardwareController.TRANSFER_POWER);
            }

            // Else don't power either motor
            else {
                hardwareController.intake.setPower(0.0);
                hardwareController.transfer.setPower(0.0);
            }

            /* TUNING LOGIC */

            // Flywheel
            hardwareController.targetSpeed = flywheelTargetSpeed;
            HardwareController.flywheelPID.compute(targetSpeed, opmodeTimer.getElapsedTimeSeconds() - timeState);
            timeState = opmodeTimer.getElapsedTimeSeconds();
            // Hood
            hardwareController.turretHood.setPosition(Math.max(hoodZero, hoodPosition));

            updateTelemetry();
        }
    }

    public void updateTelemetry() {
        Pose turretCenter = new Pose(follower.getPose().getX(), follower.getPose().getY() - 3.0);

        // Debug telemetry (On panels)
        packet.put("Distance (in)", turretCenter.distanceFrom(goalPose));
        packet.put("Flywheel Velocity (RPS)", hardwareController.flywheelA.getVelocity() / 28.0);

        packet.put("Target Speed", flywheelTargetSpeed);
        packet.put("Hood Position", hardwareController.turretHood.getPosition());
        packet.put("Rotation Encoder Position", hardwareController.turretRotation.getCurrentPosition());
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
