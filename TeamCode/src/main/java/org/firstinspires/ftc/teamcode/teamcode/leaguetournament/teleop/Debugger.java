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

package org.firstinspires.ftc.teamcode.teamcode.leaguetournament.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsAlpha;
import org.firstinspires.ftc.teamcode.teamcode.leaguetournament.HardwareController;

@Configurable
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp Debugger", group="League Tournament")
public class Debugger extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private HardwareController hardwareController;

    private Follower follower;
    public static Pose startingPose = new Pose(0.0, 0.0, Math.toRadians(90.0));
    private TelemetryManager telemetryM;

    private boolean isRobotCentric = false;

    private boolean slowMode = false;
    private boolean flywheelOn = false;
    private boolean autoAimTurret = true;

    private double SLOW_MODE_MULTIPLIER = 0.25;
    private double TRIGGER_THRESHOLD = 0.05;

    @Override
    public void runOpMode() {
        // Instanciate controllers
        hardwareController = new HardwareController(hardwareMap);

        // Configure follower
        follower = ConstantsAlpha.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        runtime.reset();

        follower.startTeleOpDrive(true);


        /* ###############################
                        START
           ############################### */

        while (opModeIsActive()) {

            follower.update();

            // Normal driving mode
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    isRobotCentric
            );
            // Precision driving mode
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * SLOW_MODE_MULTIPLIER,
                    -gamepad1.left_stick_x * SLOW_MODE_MULTIPLIER,
                    -gamepad1.right_stick_x * SLOW_MODE_MULTIPLIER,
                    isRobotCentric
            );

            /* NON-DRIVING CONTROLS */

            // Toggle slow mode
            if (gamepad1.aWasPressed()) slowMode = !slowMode;

            // Toggle robot-centered
            if (gamepad1.bWasPressed()) isRobotCentric = !isRobotCentric;

            // Toggle auto-aiming
            if (gamepad1.xWasPressed()) autoAimTurret = !autoAimTurret;

            // Toggle flywheel
            if (gamepad1.yWasPressed()) {
                flywheelOn = !flywheelOn;
                // If flywheel is NOT on, disable
                // HardwareController will handle it if it's enabled
                if (!flywheelOn) {
                    hardwareController.turretFlywheel.setPower(0.0);
                }
            }


            // FEEDING CONDITIONAL
            // When trigger is held and flywheel velocity is acceptable, feed
            if (gamepad1.right_trigger >= TRIGGER_THRESHOLD) {
                // Switch transfer mode to reverse if needed
                if (hardwareController.transfer.getDirection().equals(DcMotorSimple.Direction.REVERSE)) {
                    hardwareController.transfer.setDirection(DcMotorSimple.Direction.FORWARD);
                }
                // Switch intake mode to [intake] if needed
                if (!hardwareController.intake.getDirection().equals(DcMotorSimple.Direction.REVERSE)) {
                    hardwareController.intake.setDirection(DcMotorSimple.Direction.FORWARD);
                }
                // Then feed and intake
                hardwareController.transfer.setPower(HardwareController.TRANSFER_POWER);
                hardwareController.intake.setPower(HardwareController.INTAKE_POWER);
            }

            // INTAKE CONDITIONAL
            // When trigger is held, intake
            else if (gamepad1.left_trigger >= TRIGGER_THRESHOLD) {
                // Switch transfer mode to reverse if needed
                if (hardwareController.transfer.getDirection().equals(DcMotorSimple.Direction.FORWARD)) {
                    hardwareController.transfer.setDirection(DcMotorSimple.Direction.REVERSE);
                }
                // Switch intake mode to reverse if needed
                if (hardwareController.intake.getDirection().equals(DcMotorSimple.Direction.REVERSE)) {
                    hardwareController.intake.setDirection(DcMotorSimple.Direction.FORWARD);
                }
                // Then power intake and transfer
                hardwareController.intake.setPower(HardwareController.INTAKE_POWER);
                hardwareController.transfer.setPower(HardwareController.TRANSFER_POWER);
            }

            // OUTTAKE CONDITIONAL
            // When trigger is held, intake
            else if (gamepad1.left_bumper) {
                // Switch transfer mode to reverse if needed
                if (hardwareController.transfer.getDirection().equals(DcMotorSimple.Direction.FORWARD)) {
                    hardwareController.transfer.setDirection(DcMotorSimple.Direction.REVERSE);
                }
                // Switch intake mode to reverse if needed
                if (hardwareController.intake.getDirection().equals(DcMotorSimple.Direction.FORWARD)) {
                    hardwareController.intake.setDirection(DcMotorSimple.Direction.REVERSE);
                }
                // Then power intake and transfer
                hardwareController.intake.setPower(HardwareController.INTAKE_POWER);
                hardwareController.transfer.setPower(HardwareController.TRANSFER_POWER);
            }

            // Else don't power either motor
            else {
                hardwareController.intake.setPower(0.0);
                hardwareController.transfer.setPower(0.0);
            }

            hardwareController.updateTurret(
                    follower.getPose(),
                    new Pose(60.0, 60.0),
                    autoAimTurret,
                    flywheelOn
            );

            updateTelemetry();
        }
    }

    public void updateTelemetry() {
        // Debug telemetry (On panels)
        telemetryM.addData("Position (In)", follower.getPose());
        telemetryM.addData("Velocity (In/Sec)", follower.getVelocity());
        telemetryM.addData("Flywheel Velocity (Degrees/Sec)", hardwareController.turretFlywheel.getVelocity(AngleUnit.DEGREES));

        telemetryM.addData("Turret Angle", hardwareController.turretAngle);
        telemetryM.addData("Turret Ticks", hardwareController.turretTicks);
        telemetryM.addData("Tag in Frame", hardwareController.tagDetected);

        telemetryM.addData("Distance: ", hardwareController.distance);
        telemetryM.update();

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
