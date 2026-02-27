package org.firstinspires.ftc.teamcode.pedroPathing.epsilon;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ConstantsEpsilon {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(14.06)
            // Deceleration rates
            .forwardZeroPowerAcceleration(-43.0)
            .lateralZeroPowerAcceleration(-69.0)
            // Enable secondary PID
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false)
            // PID values
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0.0, 0.002, 0.04))
            .headingPIDFCoefficients(new PIDFCoefficients(1.0, 0.0, 0.03, 0.045))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1, 0.0, 0.0, 0.1, 0.5))
            .centripetalScaling(0.0002);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1.0)
            // Drivetrain names
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftRearMotorName("leftBack")
            .leftFrontMotorName("leftFront")
            // Default drivetrain directions
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            // Cartesian velocity
            .xVelocity(72.0) //51
            .yVelocity(56.0); //41

    public static PinpointConstants pinpointConstants = new PinpointConstants()
            // Pinpoint offset from robot center
            .forwardPodY(-5.43)
            .strafePodX(1.57)
            // Pinpoint configuration
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            // Encoder directions
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
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
