//Software Written by "Electric Wolves Coding Team"
//Base Code accelerated using Claude AI (Anthropic)
//Verified, tested, and debugged by "Electric Wolves"

package org.firstinspires.ftc.teamcode.pedroPathing.HelpfulClasses;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
A single servo that rotates a set number of degrees from its rest position, holds there briefly,
then goes back to rest. Call trigger() to start it and update() every loop.

How the angle works: FTC servos take a position from 0.0 to 1.0, which maps onto the servo's full
range of travel. So 60 degrees = 60 / SERVO_RANGE_DEGREES. Set SERVO_RANGE_DEGREES to match your servo:
  - Most standard servos (and the REV/goBILDA 180 servos): 180
  - REV Smart Robot Servo (default mode): 270
  - goBILDA torque/speed servos: 300
If the angle comes out wrong (e.g. it moves 40 degrees instead of 60), this number is the reason.

Servos have no position sensor, so the code can't know when it has arrived. It just waits HOLD_TIME_MS
at the target before returning. If the servo isn't reaching the full angle before it comes back,
increase HOLD_TIME_MS.

Setup:
- In the hardware config, add a servo named "rotateServo" (or change SERVO_NAME below).
- If it rotates the wrong way, flip REVERSED.
- Rest position is 0 degrees. Make sure the mechanism is safe at the rest position before you
  press INIT, because the servo moves there immediately.

These values are @Configurable so they can be changed live in Panels.
 */
@Configurable
public class RotateServo {
    // ---- Hardware ----
    public static String SERVO_NAME = "rotateServo";
    public static boolean REVERSED = false;

    // ---- Motion ----
    public static double SERVO_RANGE_DEGREES = 180;  // full travel of your servo
    public static double ROTATE_DEGREES = 60;        // how far to rotate from rest
    public static double REST_POSITION = 0.0;        // 0.0 to 1.0
    public static double HOLD_TIME_MS = 500;         // time to wait at the target before returning

    public enum State { RESTING, ROTATED }

    private Servo servo;
    private final ElapsedTime timer = new ElapsedTime();
    private State state = State.RESTING;

    public void init(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, SERVO_NAME);
        servo.setDirection(REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        servo.setPosition(REST_POSITION);
        state = State.RESTING;
    }

    // Position the servo goes to when rotated, clamped to the legal 0 to 1 range
    public double getTargetPosition() {
        double target = REST_POSITION + (ROTATE_DEGREES / SERVO_RANGE_DEGREES);
        return Math.max(0.0, Math.min(1.0, target));
    }

    // Starts a rotate-and-return. Ignored if one is already in progress.
    public void trigger() {
        if (state != State.RESTING) return;
        servo.setPosition(getTargetPosition());
        timer.reset();
        state = State.ROTATED;
    }

    // Sends it straight back to rest, cancelling anything in progress
    public void returnToRest() {
        servo.setPosition(REST_POSITION);
        state = State.RESTING;
    }

    public boolean isBusy() {
        return state != State.RESTING;
    }

    public State getState() {
        return state;
    }

    public double getPosition() {
        return servo.getPosition();
    }

    // Call every loop
    public void update() {
        if (state == State.ROTATED && timer.milliseconds() >= HOLD_TIME_MS) {
            returnToRest();
        }
    }
}