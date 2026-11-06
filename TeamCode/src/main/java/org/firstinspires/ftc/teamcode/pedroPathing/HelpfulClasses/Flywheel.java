//Software Written by "Electric Wolves Coding Team"
//Base Code accelerated using Claude AI (Anthropic)
//Verified, tested, and debugged by "Electric Wolves"


package org.firstinspires.ftc.teamcode.pedroPathing.HelpfulClasses;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.PID;

/*
Minimal flywheel controller: on/off plus adjustable speed. Speeds are in encoder ticks per second
(the same units getVelocity() returns and the old targetSpeed used).

Same hardware names and PID values as the old Turret_GameSpecific class (fly1, fly2, PID.fly1*).
If the wheels are driven by only one motor, delete everything about fly2.

Per the note in VelocityPID_wStatic: the two motors are mechanically linked, so fly1 is the leader.
Only fly1's encoder feeds the PID and both motors get the same power.

Reverse: setReverse(true) runs the intake backward at the same speed (targetSpeed), even if it was off.
setReverse(false) puts it back the way it was (on or off). Call it every loop with whether the reverse
button is held, and the intake runs backward only while the button is held.

Call init() once in init, then update() every loop.
 */
public class Flywheel {
    private DcMotorEx flywheel, fly2;
    private LynxModule hub;
    private VelocityPID_wStatic pid;

    public double targetSpeed = 1750;   // ticks/sec
    public double minSpeed = 0;
    public double maxSpeed = 2400;      // set this to your motor's real max ticks/sec
    public double step = 50;            // how much adjustSpeed changes by default
    private boolean running = false;
    private boolean reversed = false;
    private boolean wasRunningBeforeReverse = false;

    public void init(HardwareMap hardwareMap) {
        hub = hardwareMap.getAll(LynxModule.class).get(0);

        flywheel = (DcMotorEx) hardwareMap.dcMotor.get("fly1");


        flywheel.setDirection(DcMotorEx.Direction.FORWARD);

        flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        flywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        flywheel.setPower(0);

        running = false;
        reversed = false;
    }

    // Starts with a fresh PID. This also picks up any values you changed live in Panels,
    // because the PID class values are only read when the PID is constructed.
    public void on() {
        if (running) return;
        pid = new VelocityPID_wStatic(PID.fly1P, PID.fly1I, PID.fly1D, PID.fly1F, PID.fly1S, false);
        running = true;
    }

    public void off() {
        running = false;
        flywheel.setPower(0);

    }

    public void toggle() {
        if (running) off(); else on();
    }

    public boolean isRunning() {
        return running;
    }

    public boolean isReversed() {
        return reversed;
    }

    // Hold-to-reverse. Call every loop with true while the reverse button is held.
    public void setReverse(boolean reverse) {
        if (reverse == reversed) return;
        reversed = reverse;
        if (reverse) {
            wasRunningBeforeReverse = running;
            on();                               // runs backward even if it was off
        } else if (!wasRunningBeforeReverse) {
            off();                              // it was off before, so turn it back off
        }
    }

    // The speed the PID aims for: negative while reversed
    private double signedTarget() {
        return reversed ? -targetSpeed : targetSpeed;
    }

    public void setSpeed(double ticksPerSecond) {
        targetSpeed = Math.max(minSpeed, Math.min(maxSpeed, ticksPerSecond));
    }

    public void adjustSpeed(double delta) {
        setSpeed(targetSpeed + delta);
    }

    public double getVelocity() {
        return flywheel.getVelocity();
    }

    // True once the wheel is within the tolerance (ticks/sec) of the target
    public boolean atSpeed(double tolerance) {
        return running && Math.abs(flywheel.getVelocity() - signedTarget()) < tolerance;
    }

    // Call every loop
    public void update() {
        if (!running) return;
        double power = pid.update(signedTarget(), flywheel.getVelocity(), hub.getInputVoltage(VoltageUnit.VOLTS));
        flywheel.setPower(power);

    }
}