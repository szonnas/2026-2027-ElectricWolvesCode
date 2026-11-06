//Software Written by "Electric Wolves Coding Team"
//Base Code accelerated using Claude AI (Anthropic)
//Verified, tested, and debugged by Electric Wolves

package org.firstinspires.ftc.teamcode.pedroPathing.Autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.HelpfulClasses.Flywheel;
import org.firstinspires.ftc.teamcode.pedroPathing.HelpfulClasses.Flywheelii;
import org.firstinspires.ftc.teamcode.pedroPathing.HelpfulClasses.Rotate_Servo;

/*
Simple autonomous. Timeline (all times counted from pressing Play):

   0 s     launcher (Flywheelii) turns on at medium speed, intake (Flywheel) turns on at max speed
  10 s     rotate servo runs once (turns out, holds, comes back). The launcher gets 10 s to spin up first.
  17 s     robot drives straight forward for DRIVE_TIME_MS, then stops (7 s after the servo starts)

Driving is just timed power (no Pedro, no odometry), so how far it goes depends on DRIVE_POWER, DRIVE_TIME_MS,
the battery and the floor. To tune the distance: run it, measure how far it went, then
  new DRIVE_TIME_MS = old DRIVE_TIME_MS x (distance you want / distance it went).
A few feet is about 36 to 60 inches. The 1500 ms below is only a starting guess.

By default the launcher and intake keep running while the robot drives. Set TURN_OFF_BEFORE_DRIVING to true
if you want them to shut off right before it moves. Everything is turned off when the autonomous ends.

Hardware config names needed: the four drive motors (from Constants.driveConstants), "fly1" (intake),
"flywheelII" (launcher), and "rotateServo" (Continuous Rotation Servo).

Make sure the FLIP_ settings match the ones in Main_TeleOp, so the robot drives forward and not backward.
 */
@Autonomous(name = "Auto Launch And Drive")
public class Auto_Launch_And_Drive extends LinearOpMode {
    // ---- Timing (milliseconds from pressing Play) ----
    private static final double SERVO_DELAY_MS = 10000;            // servo runs this long after the flywheels start
    private static final double DRIVE_DELAY_AFTER_SERVO_MS = 7000; // robot starts driving this long after the servo starts

    // ---- Driving ----
    private static final double DRIVE_POWER = 0.5;         // 0 to 1
    private static final double DRIVE_TIME_MS = 1500;      // how long it drives (sets the distance, tune this)
    private static final boolean TURN_OFF_BEFORE_DRIVING = false;

    // ---- Launcher speed in ticks/sec (1750 is the medium preset). The intake always runs at its max speed. ----
    private static final double LAUNCHER_SPEED = 1890;

    // ---- Drive wheel direction fixes (same as Main_TeleOp) ----
    private static final boolean FLIP_LEFT_FRONT = false;
    private static final boolean FLIP_LEFT_REAR = true;
    private static final boolean FLIP_RIGHT_FRONT = true;
    private static final boolean FLIP_RIGHT_REAR = false;

    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    private final Flywheel intake = new Flywheel();          // the "Flywheel" class is really the intake
    private final Flywheelii launcher = new Flywheelii();
    private final Rotate_Servo rotateServo = new Rotate_Servo();

    private final ElapsedTime runTime = new ElapsedTime();
    private String stage = "Waiting";

    @Override
    public void runOpMode() {
        // ---------- Hardware ----------
        leftFront = hardwareMap.get(DcMotorEx.class, Constants.driveConstants.getLeftFrontMotorName());
        leftRear = hardwareMap.get(DcMotorEx.class, Constants.driveConstants.getLeftRearMotorName());
        rightFront = hardwareMap.get(DcMotorEx.class, Constants.driveConstants.getRightFrontMotorName());
        rightRear = hardwareMap.get(DcMotorEx.class, Constants.driveConstants.getRightRearMotorName());

        leftFront.setDirection(Constants.driveConstants.getLeftFrontMotorDirection());
        leftRear.setDirection(Constants.driveConstants.getLeftRearMotorDirection());
        rightFront.setDirection(Constants.driveConstants.getRightFrontMotorDirection());
        rightRear.setDirection(Constants.driveConstants.getRightRearMotorDirection());

        if (FLIP_LEFT_FRONT) leftFront.setDirection(flip(leftFront.getDirection()));
        if (FLIP_LEFT_REAR) leftRear.setDirection(flip(leftRear.getDirection()));
        if (FLIP_RIGHT_FRONT) rightFront.setDirection(flip(rightFront.getDirection()));
        if (FLIP_RIGHT_REAR) rightRear.setDirection(flip(rightRear.getDirection()));

        for (DcMotorEx m : new DcMotorEx[]{leftFront, leftRear, rightFront, rightRear}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setPower(0);
        }

        intake.init(hardwareMap);
        launcher.init(hardwareMap);
        rotateServo.init(hardwareMap);

        telemetry.addLine("Ready. Clear the area in front of the robot, then press Play.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        try {
            runTime.reset();

            // ---------- 0 s: start the launcher and the intake ----------
            stage = "Launcher and intake spinning up";
            launcher.setSpeed(LAUNCHER_SPEED);
            launcher.on();
            intake.setSpeed(intake.maxSpeed);   // intake at max speed
            intake.on();

            // ---------- 10 s: run the servo (launcher has had time to spin up) ----------
            waitUntil(SERVO_DELAY_MS);
            if (!opModeIsActive()) return;
            stage = "Servo running";
            rotateServo.trigger();

            // ---------- wait until 7 s after the servo started, then drive ----------
            waitUntil(SERVO_DELAY_MS + DRIVE_DELAY_AFTER_SERVO_MS);
            if (!opModeIsActive()) return;

            // ---------- drive forward ----------
            if (TURN_OFF_BEFORE_DRIVING) {
                launcher.off();
                intake.off();
            }
            stage = "Driving forward";
            setDrivePower(DRIVE_POWER);
            ElapsedTime driveTimer = new ElapsedTime();
            while (opModeIsActive() && driveTimer.milliseconds() < DRIVE_TIME_MS) {
                updateAll();
            }
            setDrivePower(0);

            stage = "Done";
            updateAll();
        } finally {
            // Always shut everything down, even if Stop is pressed partway through
            setDrivePower(0);
            launcher.off();
            intake.off();
            rotateServo.stop();
        }
    }

    // Keeps everything updating until the run time reaches the given number of milliseconds
    private void waitUntil(double ms) {
        while (opModeIsActive() && runTime.milliseconds() < ms) {
            updateAll();
        }
    }

    // These must be called every loop or the flywheels and servo stop working
    private void updateAll() {
        launcher.update();
        intake.update();
        rotateServo.update();

        telemetry.addData("Stage", stage);
        telemetry.addData("Time (s)", "%.1f", runTime.seconds());
        telemetry.addData("Launcher target / actual", "%.0f / %.0f", launcher.targetSpeed, launcher.getVelocity());
        telemetry.addData("Intake target / actual", "%.0f / %.0f", intake.targetSpeed, intake.getVelocity());
        telemetry.addData("Rotate servo", rotateServo.getState());
        telemetry.update();
    }

    private void setDrivePower(double power) {
        leftFront.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(power);
        rightRear.setPower(power);
    }

    private static DcMotorSimple.Direction flip(DcMotorSimple.Direction d) {
        return d == DcMotorSimple.Direction.FORWARD
                ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD;
    }
}