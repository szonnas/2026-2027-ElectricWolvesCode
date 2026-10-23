//Software Written by "Electric Wolves Coding Team"
//Base Code accelerated using Claude AI (Anthropic)

package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/*
Bare-bones check for the rotate servo. No state machine, nothing fancy.

After PLAY the servo sweeps by itself: 0.0 -> 0.5 -> 1.0, two seconds each, forever.
  - If it does NOT move during the sweep, the problem is the servo, wiring, power, or config port.
  - If it DOES move, the servo is fine and the problem is the controller/buttons.

Gamepad 1 (stops the auto sweep once you press anything):
  A = 0.0   B = 0.5   Y = 1.0

Watch the "gamepad1 A/B/Y" lines in telemetry. If they never say true when you press the buttons,
the Driver Station isn't treating your controller as gamepad 1 (hold Start + A to assign it).
 */
@TeleOp(name = "Servo Check", group = "Test")
public class Servo_Check extends OpMode {
    private Servo servo;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean autoSweep = true;
    private double position = 0.0;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, Constants.ROTATE_SERVO_NAME);
        telemetry.addData("Found servo", Constants.ROTATE_SERVO_NAME);
        telemetry.addData("Port number", servo.getPortNumber());
        telemetry.addData("Hub", servo.getConnectionInfo());
        telemetry.addLine("Press PLAY. It should start sweeping on its own.");
        telemetry.update();
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        if (gamepad1.a) { autoSweep = false; position = 0.0; }
        if (gamepad1.b) { autoSweep = false; position = 0.5; }
        if (gamepad1.y) { autoSweep = false; position = 1.0; }

        if (autoSweep) {
            int step = (int) (timer.seconds() / 2) % 3;
            position = step * 0.5;
        }

        servo.setPosition(position);

        telemetry.addData("Mode", autoSweep ? "AUTO SWEEP" : "BUTTONS");
        telemetry.addData("Commanded position", position);
        telemetry.addData("Port number", servo.getPortNumber());
        telemetry.addData("Hub", servo.getConnectionInfo());
        telemetry.addData("gamepad1 A", gamepad1.a);
        telemetry.addData("gamepad1 B", gamepad1.b);
        telemetry.addData("gamepad1 Y", gamepad1.y);
        telemetry.addData("gamepad2 A (should be false)", gamepad2.a);
        telemetry.update();
    }
}