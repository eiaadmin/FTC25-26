package org.firstinspires.ftc.teamcode.pedroPathing;

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

/* The final Constants File */
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9.07185) //4.08233
            .forwardZeroPowerAcceleration(((-51.5815 + -47.7870 + -49.9782 + -51.3841) / 4))
            .lateralZeroPowerAcceleration(((-77.4899 + -72.8000 + -73.0976 + -81.7169) / 4))
            //.useSecondaryTranslationalPIDF(true)
            //.useSecondaryHeadingPIDF(true)
            //.useSecondaryDrivePIDF(true)
            .centripetalScaling(0.00076)
            //.translationalPIDFCoefficients(new PIDFCoefficients(0.10, 0, 0.01, 0)) //0.17
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.03)) //0.17
            .headingPIDFCoefficients(new PIDFCoefficients(1.7, 0, 0.1, 0))
            .drivePIDFCoefficients(
                    new FilteredPIDFCoefficients(0.045, 0, 0.000009, 0.6, 0)
            );
    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("Flch3")
            .leftRearMotorName("Blch2")
            .rightFrontMotorName("FRch1")
            .rightRearMotorName("BRch0")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity((82.8770+84.9784+83.4908+82.4853) / 4)
            .yVelocity((71.8362+72.7322+73.7502+71.3661) / 4);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-2.8125) //2.75
            .strafePodX(4.75) //-5.875
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(
                    GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.95,
            100,
            1,
            1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
