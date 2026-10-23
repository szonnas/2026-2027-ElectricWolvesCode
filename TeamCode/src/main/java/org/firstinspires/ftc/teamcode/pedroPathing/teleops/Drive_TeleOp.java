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

/*
Gamepad 1: left stick = drive, right stick x = turn, right trigger = slow down, left trigger (held) = reverse
Gamepad 2: A = flywheel on/off, D-pad up/down = flywheel speed up/down by one step

The drivetrain here is a plain robot-centric mecanum drive that sends power straight to the motors.
It does NOT use Pedro or odometry, so it drives correctly even when there are no odometry pods and
the localizer isn't tuned. Motor names and directions come from Constants.driveConstants, so if a
wheel spins the wrong way, fix its direction there.
 */
@TeleOp(name = "Drive TeleOp")
public class Drive_TeleOp extends OpMode {
    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    private final Flywheel flywheel = new Flywheel();

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

        flywheel.init(hardwareMap);
    }

    @Override
    public void loop() {
        // ---- Driving ----
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

        // ---- Flywheel ----
        if (gamepad1.aWasPressed()) flywheel.toggle();
        if (gamepad1.dpadUpWasPressed()) flywheel.adjustSpeed(flywheel.step);
        if (gamepad1.dpadDownWasPressed()) flywheel.adjustSpeed(-flywheel.step);
        flywheel.update();

        telemetry.addData("Flywheel running", flywheel.isRunning());
        telemetry.addData("Flywheel target", flywheel.targetSpeed);
        telemetry.addData("Flywheel actual", flywheel.getVelocity());
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
        flywheel.off();
    }
}