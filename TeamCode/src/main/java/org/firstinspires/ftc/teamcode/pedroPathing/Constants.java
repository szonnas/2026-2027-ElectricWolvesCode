package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(5)
            .forwardZeroPowerAcceleration(-31.5569882832)
            .lateralZeroPowerAcceleration(-48.0227496651)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.001, 0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(0.85, 0, 0.03, 0.025))
            .drivePIDFCoefficients( new FilteredPIDFCoefficients(.0105, 0, 0.00001, 0.6, 0.01))
            .centripetalScaling(0.0005)
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightf")
            .rightRearMotorName("rightb")
            .leftRearMotorName("leftb")
            .leftFrontMotorName("leftf")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(68.3234224004)
            .yVelocity(56.1712632066)
            ;

    // No odometry pods / Pinpoint plugged in right now, so Pedro tracks position with the
    // encoders built into the four drive motors. Motor names match driveConstants above.
    // The ticks-to-inches values are placeholders (1) until you run the localization tuners
    // in Tuning > Localization. Driving in Drive TeleOp works fine without tuning.
    public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
            .leftFrontMotorName("leftf")
            .leftRearMotorName("leftb")
            .rightFrontMotorName("rightf")
            .rightRearMotorName("rightb")
            .leftFrontEncoderDirection(Encoder.REVERSE)
            .leftRearEncoderDirection(Encoder.REVERSE)
            .rightFrontEncoderDirection(Encoder.FORWARD)
            .rightRearEncoderDirection(Encoder.FORWARD)
            .robotWidth(1)
            .robotLength(1)
            .forwardTicksToInches(1)
            .strafeTicksToInches(1)
            .turnTicksToInches(1);

    /* Old Pinpoint setup. To go back: uncomment this, delete the DriveEncoderConstants above,
       put the Pinpoint imports back, and change .driveEncoderLocalizer to .pinpointLocalizer below.
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(1.8)
            .strafePodX(-6.75)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    */

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            0.1,
            0.1,
            0.009,
            50,
            1.25,
            10,
            1
    );





    // ---- Rotate servo (continuous rotation servo, used by HelpfulClasses/Rotate_Servo) ----
    // A CR servo can't go to an angle, it just spins. So "rotate 60 degrees" is done by spinning
    // for ROTATE_SERVO_SPIN_TIME_MS. Tune that number until it turns as far as you want.
    // Panels can still override these live while tuning.
    public static String ROTATE_SERVO_NAME = "rotateServo";   // name in the hardware config (type: Continuous Rotation Servo)
    public static boolean ROTATE_SERVO_REVERSED = false;      // flip if it spins the wrong way
    public static double ROTATE_SERVO_POWER = 1;            // spin speed, 0 to 1
    public static double ROTATE_SERVO_SPIN_TIME_MS = 2000;     // how long it spins out (sets how far it turns)
    public static double ROTATE_SERVO_HOLD_TIME_MS = 500;     // wait at the end before spinning back

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .driveEncoderLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }


}