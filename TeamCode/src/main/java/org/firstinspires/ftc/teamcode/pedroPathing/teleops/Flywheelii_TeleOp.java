//Software Written by "Electric Wolves Coding Team"
//Base Code accelerated using Claude AI (Anthropic)
//Verified, tested, and debugged by Electric Wolves

package org.firstinspires.ftc.teamcode.pedroPathing.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.HelpfulClasses.FlywheelII;

/*
Launcher-only TeleOp (no drivetrain) for testing and tuning Flywheel II.

Gamepad 1:
  A             = launcher on/off
  D-pad up/down = target speed +/- one step (flywheelII.step, default 50 ticks/sec)
  X / Y / B     = presets (low / mid / high), edit the PRESET constants below
  Right bumper  = emergency stop

The controller rumbles once each time the wheel reaches the target speed, so you know it's
ready to launch. "Ready" means within READY_TOLERANCE ticks/sec of the target.
 */
@TeleOp(name = "Flywheel II TeleOp")
public class FlywheelII_TeleOp extends OpMode {
    private final FlywheelII flywheelII = new FlywheelII();

    // Speeds in ticks/sec. Change these to whatever your shots need.
    private static final double PRESET_LOW = 1200;
    private static final double PRESET_MID = 1750;
    private static final double PRESET_HIGH = 2100;

    // How close (ticks/sec) the wheel has to be to the target to count as ready
    private static final double READY_TOLERANCE = 50;

    private boolean wasReady = false;

    @Override
    public void init() {
        flywheelII.init(hardwareMap);
        telemetry.addLine("Flywheel II ready. A = on/off, D-pad = speed, X/Y/B = presets");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) flywheelII.toggle();
        if (gamepad1.rightBumperWasPressed()) flywheelII.off();

        if (gamepad1.dpadUpWasPressed()) flywheelII.adjustSpeed(flywheelII.step);
        if (gamepad1.dpadDownWasPressed()) flywheelII.adjustSpeed(-flywheelII.step);

        if (gamepad1.xWasPressed()) flywheelII.setSpeed(PRESET_LOW);
        if (gamepad1.yWasPressed()) flywheelII.setSpeed(PRESET_MID);
        if (gamepad1.bWasPressed()) flywheelII.setSpeed(PRESET_HIGH);

        // Must be called every loop or the wheel won't be powered
        flywheelII.update();

        boolean ready = flywheelII.atSpeed(READY_TOLERANCE);
        if (ready && !wasReady) gamepad1.rumble(250);
        wasReady = ready;

        telemetry.addData("Running", flywheelII.isRunning());
        telemetry.addData("Ready to launch", ready);
        telemetry.addData("Target (ticks/sec)", flywheelII.targetSpeed);
        telemetry.addData("Actual (ticks/sec)", flywheelII.getVelocity());
        telemetry.addData("Error", flywheelII.targetSpeed - flywheelII.getVelocity());
        telemetry.update();
    }

    @Override
    public void stop() {
        flywheelII.off();
    }
}