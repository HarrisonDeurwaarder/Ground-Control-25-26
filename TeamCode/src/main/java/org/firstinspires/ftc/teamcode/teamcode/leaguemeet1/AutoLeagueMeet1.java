/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.teamcode.leaguemeet1;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
 * This OpMode runs a simple 9 ball auto
 */

@Autonomous(name="League Meet 1 (Auto)", group="Robot")
public class AutoLeagueMeet1 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Declare OpMode members
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotor intakeMotor, beltMotor, flywheelMotor;

    private static final boolean USE_WEBCAM = true;


    @Override
    public void runOpMode() {

        initMotors();
        initAprilTag();

        // Initialize the drive system variables.
        frontLeftDrive = hardwareMap.get(DcMotor.class, "drivetrain_fl");
        frontRightDrive = hardwareMap.get(DcMotor.class, "drivetrain_fr");
        backLeftDrive = hardwareMap.get(DcMotor.class, "drivetrain_bl");
        backRightDrive = hardwareMap.get(DcMotor.class, "drivetrain_br");

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        beltMotor = hardwareMap.get(DcMotor.class, "belt");
        flywheelMotor = hardwareMap.get(DcMotor.class, "flywheel");

        // The left-side motors need to be reversed
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        beltMotor.setDirection(DcMotor.Direction.REVERSE);
        flywheelMotor.setDirection(DcMotor.Direction.FORWARD);

        // Build starter pose and odometry drive
        Pose2d start = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, start);

        // Build drive action
        TrajectoryActionBuilder tab = drive.actionBuilder(start);
        Action route = tab.build();

        // Add preliminary telemetry data
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Begin streaming
        visionPortal.resumeStreaming();

        // Stop execution if it is requested
        if (isStopRequested()) {
            visionPortal.close();
            return;
        }

        // Process apriltag and derive pose
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        // Loop apriltags
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == 0) {

            }
        }

        // Quit streaming
        visionPortal.close();
    }

    private void initMotors() {

        // Initialize motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "drivetrain_fl");
        frontRightDrive = hardwareMap.get(DcMotor.class, "drivetrain_fr");
        backLeftDrive = hardwareMap.get(DcMotor.class, "drivetrain_bl");
        backRightDrive = hardwareMap.get(DcMotor.class, "drivetrain_br");

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        beltMotor = hardwareMap.get(DcMotor.class, "belt");
        flywheelMotor = hardwareMap.get(DcMotor.class, "flywheel");

        // The left-side motors need to be reversed
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        beltMotor.setDirection(DcMotor.Direction.REVERSE);
        flywheelMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set motors to brake if under zero power
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set motor power
        frontLeftDrive.setPower(LM1DriveConfig.MAX_DRIVE_POWER);
        frontRightDrive.setPower(LM1DriveConfig.MAX_DRIVE_POWER);
        backLeftDrive.setPower(LM1DriveConfig.MAX_DRIVE_POWER);
        backRightDrive.setPower(LM1DriveConfig.MAX_DRIVE_POWER);

        intakeMotor.setPower(LM1DriveConfig.INTAKE_POWER);
        beltMotor.setPower(LM1DriveConfig.BELT_POWER);
        flywheelMotor.setPower(LM1DriveConfig.FLYWHEEL_POWER);
    }

    private void initAprilTag() {
        // Initialize the apriltag processor and vision portal
        aprilTag = new AprilTagProcessor.Builder().build();
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera to use webcam
        builder.setCamera(hardwareMap.get(WebcamName.class, "webcam"));

        // Set and enable the processor and vision portal
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }
}