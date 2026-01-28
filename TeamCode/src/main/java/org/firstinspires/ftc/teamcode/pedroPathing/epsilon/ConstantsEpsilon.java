package org.firstinspires.ftc.teamcode.pedroPathing.epsilon;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.CustomIMU;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ConstantsEpsilon {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(14.5)
            // Deceleration rates
            .forwardZeroPowerAcceleration(-70.0)
            .lateralZeroPowerAcceleration(-90.0)
            // Enable secondary PID
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false)
            // PID values
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0.0, 0.004, 0.05))
            .headingPIDFCoefficients(new PIDFCoefficients(1.5, 0.0, 0.05, 0.04))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1, 0.0, 0.0, 0.1, 0.6));

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1.0)
            // Drivetrain names
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftRearMotorName("leftBack")
            .leftFrontMotorName("leftFront")
            // Default drivetrain directions
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            // Cartesian velocity
            .xVelocity(51.0) //51
            .yVelocity(41.0); //41

    public static PinpointConstants pinpointConstants = new PinpointConstants()
            // Pinpoint offset from robot center
            .forwardPodY(-3.622)
            .strafePodX(-7.244)
            // Pinpoint configuration
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            // Encoder directions
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(pinpointConstants)
                .build();
    }
}
