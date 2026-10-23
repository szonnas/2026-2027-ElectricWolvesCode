//Software Written by "Electric Wolves Coding Team"
//Base Code accelerated using Claude AI (Anthropic)
//Verified, tested, and debugged by Electric Wolves

package org.firstinspires.ftc.teamcode.pedroPathing.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.HelpfulClasses.Rotate_Servo;

/*
Servo-only TeleOp for testing the rotate-and-return servo.

Gamepad 1:
  A = turn out, hold, then turn back automatically
  X = cancel and turn back right away
  B = emergency stop (stops spinning where it is)

Pressing A while it's already moving does nothing.
 */
@TeleOp(name = "Rotate Servo TeleOp")
public class Rotate_Servo_TeleOp extends OpMode {
    private final Rotate_Servo rotateServo = new Rotate_Servo();

    @Override
    public void init() {
        rotateServo.init(hardwareMap);
        telemetry.addLine("Rotate servo ready. A = turn and return, X = return, B = stop");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) rotateServo.trigger();
        if (gamepad1.xWasPressed()) rotateServo.returnToRest();
        if (gamepad1.bWasPressed()) rotateServo.stop();

        // Must be called every loop so it knows when to return to rest
        rotateServo.update();

        telemetry.addData("State", rotateServo.getState());
        telemetry.addData("Servo power", rotateServo.getPower());
        telemetry.addData("Spin time (ms)", Rotate_Servo.SPIN_TIME_MS);
        telemetry.update();
    }
}