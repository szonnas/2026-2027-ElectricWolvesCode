//Software Written by "Electric Wolves Coding Team"
//Base Code accelerated using Claude AI (Anthropic)
//Verified, tested, and debugged by "Electric Wolves"

package org.firstinspires.ftc.teamcode.pedroPathing.HelpfulClasses;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

/*
Flywheel II: the launcher flywheel (the one that actually shoots the ball).
NOT the same as the Flywheel class, which is really the intake.

One motor, velocity controlled. Speeds are in encoder ticks per second (what getVelocity() returns).
Uses VelocityPID_wStatic, so see the tuning steps in the comment at the top of that class.

Setup:
- In the Robot Controller hardware config, name the launcher motor "flywheelII" (or change MOTOR_NAME below).
- If it spins the wrong way, flip REVERSED.
- The PID values below are STARTING POINTS ONLY and are not tuned for your motor. They're @Configurable,
  so you can change them live in Panels. They're read when on() is called, so turn the flywheel
  off and on again after changing them. Once tuned, copy the final values into this file.
- Set maxSpeed to your motor's real max ticks/sec (full speed ticks/sec is roughly 1 / F).

If the launcher ends up with two motors, add a second DcMotorEx, set it to the same power as the
first, and only use the first one's encoder for the PID (see the note in VelocityPID_wStatic).

Call init() once in init, then update() every loop.
 */
@Configurable
public class Flywheelii {
    // ---- Hardware ----
    public static String MOTOR_NAME = "flywheelII";
    public static boolean REVERSED = false;

    // ---- PID (starting values, tune these) ----
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0.004;   // volts per tick/sec. Does most of the work.
    public static double kS = 0;       // static friction volts. Tune this first.

    private DcMotorEx motor;
    private LynxModule hub;
    private VelocityPID_wStatic pid;

    public double targetSpeed = 1750;   // ticks/sec
    public double minSpeed = 0;
    public double maxSpeed = 2400;      // set this to your motor's real max ticks/sec
    public double step = 50;            // how much adjustSpeed changes by default
    private boolean running = false;

    public void init(HardwareMap hardwareMap) {
        hub = hardwareMap.getAll(LynxModule.class).get(0);

        motor = (DcMotorEx) hardwareMap.dcMotor.get(MOTOR_NAME);
        motor.setDirection(REVERSED ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0);
        running = false;
    }

    // Starts with a fresh PID, which also picks up any values changed live in Panels
    public void on() {
        if (running) return;
        pid = new VelocityPID_wStatic(kP, kI, kD, kF, kS, false);
        running = true;
    }

    public void off() {
        running = false;
        motor.setPower(0);
    }

    public void toggle() {
        if (running) off(); else on();
    }

    public boolean isRunning() {
        return running;
    }

    public void setSpeed(double ticksPerSecond) {
        targetSpeed = Math.max(minSpeed, Math.min(maxSpeed, ticksPerSecond));
    }

    public void adjustSpeed(double delta) {
        setSpeed(targetSpeed + delta);
    }

    public double getVelocity() {
        return motor.getVelocity();
    }

    // True once the wheel is within the tolerance (ticks/sec) of the target
    public boolean atSpeed(double tolerance) {
        return running && Math.abs(motor.getVelocity() - targetSpeed) < tolerance;
    }

    // Call every loop
    public void update() {
        if (!running) return;
        double power = pid.update(targetSpeed, motor.getVelocity(), hub.getInputVoltage(VoltageUnit.VOLTS));
        motor.setPower(power);
    }
}