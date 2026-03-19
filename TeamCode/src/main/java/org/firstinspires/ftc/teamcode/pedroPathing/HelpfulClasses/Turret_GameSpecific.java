package org.firstinspires.ftc.teamcode.pedroPathing.HelpfulClasses;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.PID;
import org.firstinspires.ftc.teamcode.pedroPathing.SERVO_POSITIONS;


import java.util.List;

/*
I like having a single class that controls all of the non-drivetrain aspects of the robot. It just makes the
teleop and auton codes much easier to write and understand. Importantly, when the class is defined you define all the
motors/servos/other stuff and then you have an initialization method, that initializes everything and would be called
on init. The rest of the class is just methods that will be called throughout Opmodes. This year I tried to have a big
finite state machine control the launch sequence and a tiny one to control the feed servo. They never worked perfectly
but are something that would be really good to have for the drivers and for awards. Depends on next years game,
but implementing one might be really useful
 */




public class Turret_GameSpecific {
    private TelemetryManager telemetryM;
    public ServoImplEx hardStopL, hardStopR, feed, tAngle;
    public DcMotorEx tRotate;
    public DcMotorEx flywheel1, flywheel2;
    public DcMotorEx intake;
    private LynxModule hub;

    public VelocityPID_wStatic fly1_PID = new VelocityPID_wStatic(PID.fly1P, PID.fly1I, PID.fly1D, PID.fly1F, PID.fly1S,false);
    public VelocityPID_wStatic fly2_PID = new VelocityPID_wStatic(PID.fly2P, PID.fly2I, PID.fly2D, PID.fly2F,PID.fly1S, false);
    public PositionPID turret_PID = new PositionPID(PID.autoaimP, PID.autoaimI, PID.autoaimD);
    public VelocityPID_wStatic intake_PID = new VelocityPID_wStatic(PID.intakeP, PID.intakeI, PID.intakeD, PID.intakeF, PID.intakeS, false);


    public Limelight3A limelight;
    private ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime timeSinceLastShot = new ElapsedTime();
    private ElapsedTime pumpTimer = new ElapsedTime();

    private enum turretState {
        IDLE,
        SPIN_AIM_ANGLE,
        LAUNCH,
        RESET
    }

    public enum allianceColor {
        BLUE,
        RED,
        NONE
    }

    public enum Motif {
        GPP,
        PPG,
        PGP
    }

    private enum FEEDSTATE{
        IDLE,
        MOVING_UP,
        MOVING_DOWN
    }

    FEEDSTATE feedState = FEEDSTATE.IDLE;

    private double tx = 0;
    private double angleTarget = 0;
    public turretState turretstate = turretState.IDLE;
    public allianceColor color = allianceColor.NONE;
    private Motif motif = Motif.GPP;
    public int shotsRemaining = 0;
    private int ballsStored = 0;
    public double targetSpeed = 2000;
    private double minFlywheelSpeed = 1600;
    private final double maxSpinUpTime = 4;
    public int timesToFeed =0;
    public  boolean feedWasAtBottom = true;
    public double turretStartTicks;
    public Pose robotPose = new Pose(0, 0, 0);
    public boolean limelightTracking = false;
    public double limelightTarget = 0;
    public double flyInput = 0;
    double shotsCalled = 0;
    public double headingInput = 0;


    //call this in the init section of the OpMode
    public void init(HardwareMap hardwareMap) {
        hub = hardwareMap.getAll(LynxModule.class).get(0);


        tAngle = hardwareMap.get(ServoImplEx.class, "tAngle");
        hardStopL = hardwareMap.get(ServoImplEx.class, "gateL");
        hardStopR = hardwareMap.get(ServoImplEx.class, "gateR");
        feed = hardwareMap.get(ServoImplEx.class, "feed");


        tAngle.setDirection(ServoImplEx.Direction.FORWARD);
        hardStopL.setDirection(ServoImplEx.Direction.FORWARD);
        hardStopR.setDirection(ServoImplEx.Direction.REVERSE);
        feed.setDirection(ServoImplEx.Direction.FORWARD);

        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        flywheel1 = (DcMotorEx) hardwareMap.dcMotor.get("fly1");
        flywheel2 = (DcMotorEx) hardwareMap.dcMotor.get("fly2");
        tRotate = (DcMotorEx) hardwareMap.dcMotor.get("turret");

        flywheel1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheel2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        tRotate.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        flywheel2.setDirection(DcMotorEx.Direction.REVERSE);
        flywheel1.setDirection(DcMotorEx.Direction.FORWARD);
        flywheel2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheel1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        tRotate.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setPower(0);
        flywheel1.setPower(0);
        flywheel2.setPower(0);
        tRotate.setPower(0);


        tAngle.setPosition(angleTarget);
        hardStopL.setPosition(SERVO_POSITIONS.gateStartL);
        hardStopR.setPosition(SERVO_POSITIONS.gateStartR);
        feed.setPosition(SERVO_POSITIONS.feedStart);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(120);// This sets how often we ask Limelight for data (100 times per second)
        limelight.pipelineSwitch(2);

        limelight.start(); // This tells Limelight to start looking!

        storage.startingTicks = tRotate.getCurrentPosition();

        feedState = FEEDSTATE.IDLE;
        turretstate = turretState.IDLE;
        shotsRemaining = 0;
        stateTimer.reset();

    }

    //Call this in every loop. It won't do anything until you do fireShots()
    public void AHHH() {

        initloop();
        switch (turretstate) {
            case IDLE:
                angleTarget = 0;
                if (shotsRemaining > 0) {
                    setSpeed();

                    feed.setPosition(SERVO_POSITIONS.feedStart);
                    hardStopL.setPosition(SERVO_POSITIONS.gateStartL);
                    hardStopR.setPosition(SERVO_POSITIONS.gateStartR);

                    stateTimer.reset();
                    turretstate = turretState.SPIN_AIM_ANGLE;
                }
                break;

            case SPIN_AIM_ANGLE:
                //xaimTurretBlind();
                settAngle();
                setSpeed();

                hardStopL.setPosition(SERVO_POSITIONS.gateStartL);
                hardStopR.setPosition(SERVO_POSITIONS.gateStartR);

                if (((flywheel1.getVelocity() > minFlywheelSpeed) || (flywheel2.getVelocity() > minFlywheelSpeed)
                        || (stateTimer.seconds() > maxSpinUpTime))
                        //&& isAimed() && isAngled()
                ) {


                    hardStopR.setPosition(SERVO_POSITIONS.gateEndR);
                    hardStopL.setPosition(SERVO_POSITIONS.gateEndL);

                    setSpeed();

                    intakeOn();
                    stateTimer.reset();
                    turretstate = turretState.LAUNCH;
                    timeSinceLastShot.reset();
                }
                break;

            case LAUNCH:
                setSpeed();

                intakeOn();
                //aimTurretBlind();
                settAngle();
                feedPumpingBit();


                if (stateTimer.seconds() > 3 || shotsRemaining == 1) {
                    if (timesToFeed == 0) {
                        pumpFeed(2);
                    }
                }

                if (((flywheel1.getCurrent(CurrentUnit.AMPS) > 3.5)
                        || ((flywheel2.getCurrent(CurrentUnit.AMPS) > 3.5)))){
                    if (shotsRemaining == shotsCalled){
                        shotsRemaining--;
                        timeSinceLastShot.reset();
                    }
                    else if (timeSinceLastShot.seconds() > 0.5){
                        shotsRemaining--;
                        timeSinceLastShot.reset();
                    }


                }
                if (stateTimer.seconds() > 5){
                    feed.setPosition(SERVO_POSITIONS.feedStart);
                    hardStopL.setPosition(SERVO_POSITIONS.gateStartL);
                    hardStopR.setPosition(SERVO_POSITIONS.gateStartR);
                    shotsRemaining = 0;
                    flywheel1.setPower(0);
                    flywheel2.setPower(0);
                    stateTimer.reset();
                    turretstate = turretState.IDLE;
                }
                break;
            case RESET:
                feed.setPosition(SERVO_POSITIONS.feedStart);

                if (shotsRemaining > 0) {
                    settAngle();
                    flywheel1.setPower(fly1_PID.update(targetSpeed, flywheel1.getVelocity(), hub.getInputVoltage(VoltageUnit.VOLTS)));
                    flywheel2.setPower(fly2_PID.update(targetSpeed, flywheel2.getVelocity(), hub.getInputVoltage(VoltageUnit.VOLTS)));

                    hardStopL.setPosition(SERVO_POSITIONS.gateStartL);
                    hardStopR.setPosition(SERVO_POSITIONS.gateStartR);

                    stateTimer.reset();
                    turretstate = turretState.SPIN_AIM_ANGLE;
                }
                else {
                    flyInput = 0;
                    flywheel1.setPower(0);
                    flywheel2.setPower(0);
                    intake.setPower(0);
                    stateTimer.reset();
                    turretstate = turretState.IDLE;
                }
                break;
        }
    }


    //Call this on start
    public void Start(){
        storage.startingTicks = tRotate.getCurrentPosition();

        motifScan();
    }

    public void initloop(){
        tRotate.setPower(turret_PID.update(0,
                turretTickstoDegrees()));
    }


    //This starts the launching sequence and iterates through it until thinks its launched a certain
    //number of balls
    //Call it once because it triggers something in update
    public void shoot(int shots) {
        if (turretstate == turretState.IDLE && shotsRemaining == 0 && shots > 0) {
            shotsRemaining = shots;
            shotsCalled = shots;
            stateTimer.reset();
        }
    }

    //Returns true when the robot is in the middle of the shooting sequence and false when it isn't
    //use it to make sure the robot doesn't move while its still shooting
    public boolean isshooting(){
        return (turretstate != turretState.IDLE);
    }


    //Aims the turret. First uses the limelights angle off of the goal to aim but if the limelight can't see it,
    //then it uses its position and heading until it can
    //This is called continously while the robot is shooting. Might be worth it to start aiming before starting the shoot sequence

//    private double lastTx = 0; // store last valid target
//    private double Tx = 0;
//
//    public void aimTurretLimelight() {
//        LLResult result = limelight.getLatestResult();
//        double newTarget = 0;
//
//        // --- Vision-based aiming ---
//        if (result != null && result.isValid()) {
//            lastTx = Tx;
//            Tx = result.getTx();
//
//        }
//        else {
//            Tx = lastTx;
//        }
//        if (Math.abs(tx) < 0.5) tx = 0; // deadzone
//
//        // Convert Limelight offset to absolute turret target
//        newTarget = turretTickstoDegrees() + tx;
//
//
//        double power = turret_PID.update(newTarget, turretTickstoDegrees());
//
//        // --- Clip power to safe range ---
//        if (power > 1) power = 1;
//        if (power < -1) power = -1;
//
//        limelightTarget = newTarget;
//        // --- Set motor power ---
//        tRotate.setPower(power);
//    }

public void setSpeed(){
        minFlywheelSpeed = targetSpeed - 200;
//        Pose goalPose = new Pose(0,0,0);
//        if (color == allianceColor.RED){
//            goalPose = new Pose(144,144,0);}
//        else if (color == allianceColor.BLUE){
//            goalPose = new Pose(0, 144,0);
//            }
//
//        if ((robotPose.distanceFrom(goalPose) / 12 > 8)){
//            targetSpeed = 1900;
//            }
//        else if ((robotPose.distanceFrom(goalPose) / 12) > 4){
//            targetSpeed = 1750;
//            }
//        else {
//            targetSpeed = 1200;
//            }



        flyInput = fly1_PID.update(targetSpeed, flywheel1.getVelocity(), hub.getInputVoltage(VoltageUnit.VOLTS));
        flywheel1.setPower(flyInput);
        flywheel2.setPower(flyInput);
}

public void aimTurretBlind(){
    double goalX = 0;
    double goalY = 144;

    // --- Set goal position based on alliance ---
    if (color == allianceColor.BLUE) {
        goalX = 0;
        goalY = 144;
    }
    else if (color == allianceColor.RED) {
        goalX = 144;
        goalY = 144;
    }

    // --- Calculate angle from robot to goal (field frame) ---
    double dx = goalX - robotPose.getX();
    double dy = goalY - robotPose.getY();
    double angleToGoalField = Math.toDegrees(Math.atan2(dy, dx));

    // --- Convert robot heading from radians to degrees ---
    double robotHeadingDeg = Math.toDegrees(robotPose.getHeading());

    // --- Calculate turret angle relative to robot chassis ---
    // FLIPPED: subtract goal angle FROM robot heading instead
    double turretTargetRelativeToRobot = robotHeadingDeg - angleToGoalField;

    // --- Normalize to -180 to 180 (shortest path) ---
    while (turretTargetRelativeToRobot > 180) turretTargetRelativeToRobot -= 360;
    while (turretTargetRelativeToRobot < -180) turretTargetRelativeToRobot += 360;

    // --- Update PID ---
    double power = turret_PID.update(turretTargetRelativeToRobot, turretTickstoDegrees());

    // --- Clip power to safe range ---
    power = Math.max(-1, Math.min(1, power));

    // --- Set motor power ---
    tRotate.setPower(0);
}

        // --- Smooth the target to prevent jerky PID jumps ---
        //lastTurretTarget = lastTurretTarget * (1 - targetSmoothing) + newTarget * targetSmoothing;

        // --- Update PID with absolute target ---




//    public void aimTurret(){
//        LLResult result = limelight.getLatestResult();
//        if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {
//            tx = result.getTx();
//            if (Math.abs(tx) < 0.5){
//                tx = 0;
//            }
//            turretHeadingError = -tx;
//        }
//        else{
//            if (color == allianceColor.BLUE){
//                turretHeadingError = Math.toDegrees(Math.atan2((132-robotPose.getY()), (17-robotPose.getX())));
//                if (robotPose.getX() > 17 && robotPose.getY() < 132){
//                    turretHeadingError += 180;
//                }
//                turretHeadingError -= turretTickstoDegrees();
//
//                //turretTargetAngle = turretHeading - robotPose.getHeading();
//            }
//            else if (color == allianceColor.RED){
//                turretHeadingError = Math.toDegrees(Math.atan2((132-robotPose.getY()), (127-robotPose.getX())));
//                if (robotPose.getY() > 132){
//                    turretHeadingError += 180;
//                }
//                turretHeadingError -= turretTickstoDegrees();
//
//                //turretTargetAngle = turretHeading - robotPose.getHeading();
//            }
//        }
//        tRotate.setPower(turret_PID.update(0, turretHeadingError));
//
//    }

    //Checks if the turret is within one degree of being aimed. Might need to make this more or less precise
    public boolean isAimed(){
        return (Math.abs(turret_PID.error) < 0.5);
    }


    //sets the angle for the turret based on it's distance from the goal. Should be called continously.
    // Won't work if the limelight isn't looking at the goal
    public void settAngle(){
        double distance = 1;
        Pose goalPose = new Pose(0,0,0);
        if (color == allianceColor.RED){
            goalPose = new Pose(144,144,0);}
        else if (color == allianceColor.BLUE){
            goalPose = new Pose(0, 144,0);
        }


        double x = flywheel1.getVelocity();
        if ((robotPose.distanceFrom(goalPose) * 12 > 8)){
            angleTarget = 0.129;
        }
        else{
//            ((robotPose.distanceFrom(goalPose) * 12) > 4){
            angleTarget = 0.52;
        }




            if (color == allianceColor.BLUE){
                distance = robotPose.distanceFrom(new Pose(0,144));
            }
            else if (color == allianceColor.RED){
                distance = robotPose.distanceFrom(new Pose(144, 144));
            }

            if (distance > 3) {angleTarget = 0.3;}
            if (distance < 3) {angleTarget = 0;}
            //TODO fit a curve on a bunch of distance vs. angle points and get the right angle
            //TODO write this code to change angle target not just generate a value so that isangled() works
            tAngle.setPosition(angleTarget);

    }

    //checks if the turret's angle is within 0.01 of a rotation from the target. Might need to maake this more or less precise
    public boolean isAngled(){
        return (Math.abs((tAngle.getPosition() - angleTarget)) < 0.01);
    }

    //powers the intake
    // TODO Update this with the right value
    public void intakeOn(){
        intake.setPower(intake_PID.update(1800,intake.getVelocity(),hub.getInputVoltage(VoltageUnit.VOLTS)));
    }

    //powers off the intake
    public void intakeOff(){
        intake.setPower(0);
    }


    //Converts turret motor ticks to degrees the turret has turned. Used for robotTurretReferenceAngle
    public double turretTickstoDegrees(){
            double degrees =
                    (storage.startingTicks - tRotate.getCurrentPosition()) * (-360.0 / 1008);
            while (degrees > 180) degrees -= 360;
            while (degrees < -180) degrees += 360;
            return degrees;

    }

//This code isn't finished but can be used to determine what motif we are for autonomous
    public void motifScan(){
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()){
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            if (fiducialResults.get(0).getFiducialId() == 21){
                motif = Motif.GPP;
            }
            else if (fiducialResults.get(0).getFiducialId() == 22){
                motif = Motif.PGP;
            }
            else if (fiducialResults.get(0).getFiducialId() == 23){
                motif = Motif.PPG;
            }
        }
    }

    //Tell the code that our color is blue. Also makes the limelight only look at the apriltag on the blue goal
    //Other parts of the code care about color
    public void setBlue(){
        limelight.pipelineSwitch(0);
        color = allianceColor.BLUE;
    }


    //Tell the code that our color is blue. Also makes the limelight only look at the apriltag on the blue goal
    //Other parts of the code care about color
    public void setRed(){
        limelight.pipelineSwitch(1);
        color = allianceColor.RED;
    }


    //Uses the limelight to return the position of the robot and it's heading
    //Returns (0,0,0) if the limelight isn't looking at anything
    public Pose limelightGetPose(){
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()){
            limelight.updateRobotOrientation(turretTickstoDegrees()
                    +Math.toDegrees(robotPose.getHeading()) + 90);
            Pose3D botpose = result.getBotpose();
            if (botpose != null){
                double limelightY = -botpose.getPosition().x * 39.3700787402 + 72;
                double limelightX = botpose.getPosition().y * 39.3700787402 + 72;
//                double theta = Math.toRadians(botpose.getOrientation().getYaw(AngleUnit.DEGREES) - turretTickstoDegrees())
//                        - (Math.PI / 2.0);
                double theta = robotPose.getHeading();
                //TODO MATH THAT FINDS THE ROBOT HEADING BASED ON tx and the limelights X and Y
                return new Pose(limelightX, limelightY, theta);

            }
        }
        return robotPose;
    }



//This was supposed to be a simple finite state machine to control the kicker
//Never really ended up working well because I wrote it at ILT in between matches
    public void pumpFeed(int times){
        timesToFeed = times;
        feedState = FEEDSTATE.MOVING_UP;
        pumpTimer.reset();
        feed.setPosition(SERVO_POSITIONS.feedStart);
    }

    public void feedPumpingBit() {

        if (timesToFeed <= 0) {
            feedState = FEEDSTATE.IDLE;
            return;
        }

        switch (feedState) {

            case MOVING_UP:
                feed.setPosition(SERVO_POSITIONS.feedEnd);

                if (pumpTimer.milliseconds() > 300) { // tune this
                    feedState = feedState.MOVING_DOWN;
                    pumpTimer.reset();
                }
                break;

            case MOVING_DOWN:
                feed.setPosition(SERVO_POSITIONS.feedStart);

                if (pumpTimer.milliseconds() > 300) {
                    timesToFeed--;
                    feedState = feedState.MOVING_UP;
                    pumpTimer.reset();
                }
                break;
        }
    }


    public void end(){
        storage.teleOpStart = new Pose(robotPose.getX(), robotPose.getY(),robotPose.getHeading());
    }


}
