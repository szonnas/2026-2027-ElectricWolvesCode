//Software Written by "Electric Wolves Coding Team"
//Base Code accelerated using Claude AI (Anthropic)
//Verified, tested, and debugged by Electric Wolves

package org.firstinspires.ftc.teamcode.pedroPathing.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.HelpfulClasses.Flywheel;
import org.firstinspires.ftc.teamcode.pedroPathing.HelpfulClasses.Flywheelii;
import org.firstinspires.ftc.teamcode.pedroPathing.HelpfulClasses.Rotate_Servo;

/*
Everything in one TeleOp: drivetrain, intake, launcher, and the rotate servo. Needs TWO gamepads.

Gamepad 1 (driver):
  Left stick = drive, right stick x = turn, right trigger = slow down, left trigger (held) = reverse
  A             = intake on/off          (the intake is the class called Flywheel)
  D-pad up/down = intake speed +/- one step

Gamepad 2 (operator):
  A             = launcher on/off        (the launcher is Flywheelii)
  D-pad up/down = launcher speed +/- one step
  X / Y / B     = launcher presets (low / mid / high), edit the PRESET constants below
  Right bumper  = launcher emergency stop
  Left bumper   = rotate servo: turn out, hold, then turn back automatically
  D-pad left    = rotate servo: cancel and turn back now
  D-pad right   = rotate servo: emergency stop (stops where it is)

Gamepad 2 rumbles once each time the launcher reaches its target speed.

Hardware config names needed: the four drive motors (from Constants.driveConstants), "fly1" (intake),
"flywheelII" (launcher), and "rotateServo" (Continuous Rotation Servo).

The drivetrain is a plain robot-centric mecanum drive (no Pedro, no odometry), same as Drive_TeleOp.
 */

@TeleOp(name = "Main TeleOp")
public class Main_TeleOp extends OpMode {
    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    private final Flywheel intake = new Flywheel();          // the "Flywheel" class is really the intake
    private final Flywheelii launcher = new Flywheelii();
    private final Rotate_Servo rotateServo = new Rotate_Servo();

    // Launcher speeds in ticks/sec. Change these to whatever your shots need.
    private static final double PRESET_LOW = 1200;
    private static final double PRESET_MID = 1750;
    private static final double PRESET_HIGH = 2100;

    // How close (ticks/sec) the launcher has to be to the target to count as ready
    private static final double READY_TOLERANCE = 50;

    private boolean wasReady = false;

    private static final boolean FLIP_LEFT_FRONT = false;
    private static final boolean FLIP_LEFT_REAR = true;
    private static final boolean FLIP_RIGHT_FRONT = true;
    private static final boolean FLIP_RIGHT_REAR = false;

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotorEx.class, Constants.driveConstants.getLeftFrontMotorName());
        leftRear = hardwareMap.get(DcMotorEx.class, Constants.driveConstants.getLeftRearMotorName());
        rightFront = hardwareMap.get(DcMotorEx.class, Constants.driveConstants.getRightFrontMotorName());
        rightRear = hardwareMap.get(DcMotorEx.class, Constants.driveConstants.getRightRearMotorName());

        leftFront.setDirection(Constants.driveConstants.getLeftFrontMotorDirection());
        leftRear.setDirection(Constants.driveConstants.getLeftRearMotorDirection());
        rightFront.setDirection(Constants.driveConstants.getRightFrontMotorDirection());
        rightRear.setDirection(Constants.driveConstants.getRightRearMotorDirection());

        for (DcMotorEx m : new DcMotorEx[]{leftFront, leftRear, rightFront, rightRear}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setPower(0);
        }

        intake.init(hardwareMap);
        launcher.init(hardwareMap);
        rotateServo.init(hardwareMap);

        telemetry.addLine("Main TeleOp ready (needs gamepad 1 and gamepad 2)");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ================= Gamepad 1: driving =================
        double speedFactor = 1;
        if (gamepad1.right_trigger > 0.05) {
            speedFactor = 1 - (0.85 * gamepad1.right_trigger);
        }
        if (gamepad1.left_trigger > 0.7) {
            speedFactor = -speedFactor;
        }

        double forward = -gamepad1.left_stick_y * speedFactor;   // push stick up = drive forward
        double strafe = gamepad1.left_stick_x * speedFactor;     // push stick right = strafe right
        double turn = gamepad1.right_stick_x;                    // push stick right = turn right

        double lf = forward + strafe + turn;
        double lr = forward - strafe + turn;
        double rf = forward - strafe - turn;
        double rr = forward + strafe - turn;

        // Scale everything down together if any wheel would go over full power
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(lf), Math.abs(lr)), Math.max(Math.abs(rf), Math.abs(rr))));
        leftFront.setPower(lf / max);
        leftRear.setPower(lr / max);
        rightFront.setPower(rf / max);
        rightRear.setPower(rr / max);

        // ================= Gamepad 1: intake =================
        if (gamepad1.aWasPressed()) intake.toggle();
        if (gamepad1.dpadUpWasPressed()) intake.adjustSpeed(intake.step);
        if (gamepad1.dpadDownWasPressed()) intake.adjustSpeed(-intake.step);
        intake.update();

        // ================= Gamepad 2: launcher =================
        if (gamepad2.aWasPressed()) launcher.toggle();
        if (gamepad2.rightBumperWasPressed()) launcher.off();

        if (gamepad2.dpadUpWasPressed()) launcher.adjustSpeed(launcher.step);
        if (gamepad2.dpadDownWasPressed()) launcher.adjustSpeed(-launcher.step);

        if (gamepad2.xWasPressed()) launcher.setSpeed(PRESET_LOW);
        if (gamepad2.yWasPressed()) launcher.setSpeed(PRESET_MID);
        if (gamepad2.bWasPressed()) launcher.setSpeed(PRESET_HIGH);

        launcher.update();

        // Rumble once when the launcher first reaches speed
        boolean ready = launcher.atSpeed(READY_TOLERANCE);
        if (ready && !wasReady) gamepad2.rumble(250);
        wasReady = ready;

        // ================= Gamepad 2: rotate servo =================
        if (gamepad2.leftBumperWasPressed()) rotateServo.trigger();
        if (gamepad2.dpadLeftWasPressed()) rotateServo.returnToRest();
        if (gamepad2.dpadRightWasPressed()) rotateServo.stop();
        rotateServo.update();

        // ================= Telemetry =================
        telemetry.addData("Intake running", intake.isRunning());
        telemetry.addData("Intake target / actual", "%.0f / %.0f", intake.targetSpeed, intake.getVelocity());
        telemetry.addData("Launcher running", launcher.isRunning());
        telemetry.addData("Launcher ready", ready);
        telemetry.addData("Launcher target / actual", "%.0f / %.0f", launcher.targetSpeed, launcher.getVelocity());
        telemetry.addData("Rotate servo", rotateServo.getState());
        telemetry.addData("Wheel power LF / LR", "%.2f / %.2f", lf / max, lr / max);
        telemetry.addData("Wheel power RF / RR", "%.2f / %.2f", rf / max, rr / max);
        telemetry.update();
    }

    @Override
    public void stop() {
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        intake.off();
        launcher.off();
        rotateServo.stop();
    }
}