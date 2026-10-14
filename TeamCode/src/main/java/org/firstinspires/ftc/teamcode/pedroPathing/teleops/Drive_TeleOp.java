package org.firstinspires.ftc.teamcode.pedroPathing.teleops;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.HelpfulClasses.Flywheel;

/*
Gamepad 1: left stick = drive, right stick x = turn, right trigger = slow down, left trigger (held) = reverse
Gamepad 2: A = flywheel on/off, D-pad up/down = flywheel speed up/down by one step
 */
@TeleOp(name = "Drive TeleOp")
public class Drive_TeleOp extends OpMode {
    private Follower follower;
    private final Flywheel flywheel = new Flywheel();

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        flywheel.init(hardwareMap);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // Driving (same as the old Turret_TeleOp)
        double speedFactor = 1;
        if (gamepad1.right_trigger > 0.05) {
            speedFactor = 1 - (0.85 * gamepad1.right_trigger);
        }
        if (gamepad1.left_trigger > 0.7) {
            speedFactor = -speedFactor;
        }

        follower.setTeleOpDrive(-gamepad1.left_stick_y * speedFactor,
                -gamepad1.left_stick_x * speedFactor,
                -gamepad1.right_stick_x, true);
        follower.update();

        // Flywheel
        if (gamepad2.aWasPressed()) flywheel.toggle();
        if (gamepad2.dpadUpWasPressed()) flywheel.adjustSpeed(flywheel.step);
        if (gamepad2.dpadDownWasPressed()) flywheel.adjustSpeed(-flywheel.step);
        flywheel.update();

        telemetry.addData("Flywheel running", flywheel.isRunning());
        telemetry.addData("Flywheel target", flywheel.targetSpeed);
        telemetry.addData("Flywheel actual", flywheel.getVelocity());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        flywheel.off();
    }
}