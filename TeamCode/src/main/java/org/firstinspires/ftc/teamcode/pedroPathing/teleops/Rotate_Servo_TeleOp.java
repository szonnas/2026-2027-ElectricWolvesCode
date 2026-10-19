//Software Written by "Electric Wolves Coding Team"
//Base Code accelerated using Claude AI (Anthropic)
//Verified, tested, and debugged by Electric Wolves

package org.firstinspires.ftc.teamcode.pedroPathing.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.HelpfulClasses.RotateServo;

/*
Servo-only TeleOp for testing the rotate-and-return servo.

Gamepad 1:
  A = rotate 60 degrees, hold, then return to rest automatically
  X = cancel and return to rest immediately

Pressing A while it's already moving does nothing.
 */
@TeleOp(name = "Rotate Servo TeleOp")
public class RotateServo_TeleOp extends OpMode {
    private final RotateServo rotateServo = new RotateServo();

    @Override
    public void init() {
        rotateServo.init(hardwareMap);
        telemetry.addLine("Rotate servo ready. A = rotate and return, X = return to rest");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) rotateServo.trigger();
        if (gamepad1.xWasPressed()) rotateServo.returnToRest();

        // Must be called every loop so it knows when to return to rest
        rotateServo.update();

        telemetry.addData("State", rotateServo.getState());
        telemetry.addData("Servo position", rotateServo.getPosition());
        telemetry.addData("Rotated position", rotateServo.getTargetPosition());
        telemetry.update();
    }
}