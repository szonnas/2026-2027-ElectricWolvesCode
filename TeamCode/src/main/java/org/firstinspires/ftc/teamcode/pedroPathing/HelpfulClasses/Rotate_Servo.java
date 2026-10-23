//Software Written by "Electric Wolves Coding Team"
//Base Code accelerated using Claude AI (Anthropic)
//Verified, tested, and debugged by "Electric Wolves"

package org.firstinspires.ftc.teamcode.pedroPathing.HelpfulClasses;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/*
A CONTINUOUS ROTATION servo that turns out, waits, then turns back. Call trigger() to start it and
update() every loop.

A CR servo has no idea what angle it's at. It only spins at a speed (power). So "how far it turns"
is set by how long it spins: SPIN_TIME_MS at POWER. It spins back for the same amount of time to
return. Because it's timed, it can drift a little over many cycles, especially as the battery drops.

Setup:
- In the hardware config, set the port type to "Continuous Rotation Servo" and name it "rotateServo"
  (or change ROTATE_SERVO_NAME in Constants.java).
- If it spins the wrong way, flip ROTATE_SERVO_REVERSED in Constants.java.
- To make it turn farther, raise ROTATE_SERVO_SPIN_TIME_MS. Less far, lower it.

These values are @Configurable so they can be changed live in Panels. Defaults come from Constants.java.
 */
@Configurable
public class Rotate_Servo {
    // ---- Hardware ----
    public static String SERVO_NAME = Constants.ROTATE_SERVO_NAME;
    public static boolean REVERSED = Constants.ROTATE_SERVO_REVERSED;

    // ---- Motion ----
    public static double POWER = Constants.ROTATE_SERVO_POWER;               // spin speed, 0 to 1
    public static double SPIN_TIME_MS = Constants.ROTATE_SERVO_SPIN_TIME_MS; // how long it spins out
    public static double HOLD_TIME_MS = Constants.ROTATE_SERVO_HOLD_TIME_MS; // wait before spinning back

    public enum State { RESTING, TURNING_OUT, HOLDING, RETURNING }

    private CRServo servo;
    private final ElapsedTime timer = new ElapsedTime();
    private State state = State.RESTING;
    private double returnTimeMs = 0;   // how long to spin back (= how long it actually spun out)

    public void init(HardwareMap hardwareMap) {
        servo = hardwareMap.get(CRServo.class, SERVO_NAME);
        servo.setDirection(REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        servo.setPower(0);
        state = State.RESTING;
    }

    // Starts a turn-out-and-back. Ignored if one is already in progress.
    public void trigger() {
        if (state != State.RESTING) return;
        servo.setPower(POWER);
        timer.reset();
        state = State.TURNING_OUT;
    }

    // Cancels and spins back right away, only for as long as it already spun out
    public void returnToRest() {
        if (state == State.TURNING_OUT) {
            startReturn(timer.milliseconds());
        } else if (state == State.HOLDING) {
            startReturn(SPIN_TIME_MS);
        }
    }

    // Emergency stop: stops spinning wherever it is
    public void stop() {
        servo.setPower(0);
        state = State.RESTING;
    }

    private void startReturn(double ms) {
        returnTimeMs = ms;
        servo.setPower(-POWER);
        timer.reset();
        state = State.RETURNING;
    }

    public boolean isBusy() {
        return state != State.RESTING;
    }

    public State getState() {
        return state;
    }

    public double getPower() {
        return servo.getPower();
    }

    // Call every loop
    public void update() {
        switch (state) {
            case TURNING_OUT:
                if (timer.milliseconds() >= SPIN_TIME_MS) {
                    servo.setPower(0);
                    timer.reset();
                    state = State.HOLDING;
                }
                break;
            case HOLDING:
                if (timer.milliseconds() >= HOLD_TIME_MS) {
                    startReturn(SPIN_TIME_MS);
                }
                break;
            case RETURNING:
                if (timer.milliseconds() >= returnTimeMs) {
                    servo.setPower(0);
                    state = State.RESTING;
                }
                break;
            default:
                break;
        }
    }
}