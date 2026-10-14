//Software Written by "Electric Wolves Coding Team"
//Base Code accelerated using Claude AI (Anthropic)
//Verified, tested, and debugged by Electric Wolves
package org.firstinspires.ftc.teamcode.pedroPathing.Autons.NoTurret;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.HelpfulClasses.VelocityPID;
import org.firstinspires.ftc.teamcode.pedroPathing.PID;
/*
@Disabled
@Autonomous(name = "Meet 2 Auto", group = "Autonomous")
public class Meet_2_Auto extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, shootTimer;
    private int pathState;
    CRServoImplEx CR1, CR2, CR0;
    DcMotorEx intake, flywheel1, flywheel2;
    VelocityPID fly1_PID = new VelocityPID(PID.fly1P, PID.fly1I, PID.fly1D, PID.fly1F, true);
    VelocityPID fly2_PID = new VelocityPID(PID.fly2P, PID.fly2I, PID.fly2D, PID.fly2F, true);
    VelocityPID autoaim = new VelocityPID(PID.autoaimP,PID.autoaimI,PID.autoaimD,PID.autoaimF, false);
    Limelight3A limelight;
    private Pose currentPose = new Pose(0, 0, 0);
    private Pose startPose = new Pose(56, 8, Math.toRadians(-90));
    private final Pose farScore = new Pose(56, 10, Math.toRadians(-90));
    private final Pose GPP_pickup = new Pose(43, 35.5, Math.toRadians(180));
    private final Pose GPP_end = new Pose(15, 35.5, Math.toRadians(180));
    private final Pose closeScore = new Pose(67, 84, Math.toRadians(135));
    private Path scorePreload;
    private PathChain GPP, GPP_finish, PGP, PGP_finish, PPG,PPG_finish, goBack, rotateToScore, backFromGPP;
    public double Tx = 0;
    double closeTarget = 1070;
    double farTarget = 1250;
    double ballsLaunched = 0;
    boolean intakeOn = false;
    boolean shooting = false;

    public void buildPaths() {


        scorePreload = new Path(new BezierLine(startPose, farScore));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), farScore.getHeading());

        rotateToScore = follower.pathBuilder()
                .addPath(new BezierLine(follower::getPose, poseToAimTo()))
                .setConstantHeadingInterpolation(poseToAimTo().getHeading())
                .build();

        GPP = follower.pathBuilder()
                .addPath(new BezierLine(follower::getPose, GPP_pickup))
                .setConstantHeadingInterpolation(GPP_pickup.getHeading())
                .build();

        GPP_finish = follower.pathBuilder()
                .addPath(new BezierLine(GPP_pickup, GPP_end))
                .setLinearHeadingInterpolation(GPP_pickup.getHeading(), GPP_end.getHeading())
                .build();

        backFromGPP = follower.pathBuilder()
                .addPath(new BezierLine(GPP_end, farScore))
                .setLinearHeadingInterpolation(GPP_end.getHeading(), farScore.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    shootAll(true);
                    follower.followPath(scorePreload, true);
                    if (!shooting){
                        setPathState(2);
                    }
                }
                break;

            case 2:
              follower.followPath(GPP);
              setPathState(3);
              break;

            case 3:
                if(!follower.isBusy()){
                    follower.followPath(GPP_finish);
                    intakeOn = true;
                    setPathState(4);
                }

            case 4:
                follower.followPath(backFromGPP,true);
                intakeOn = false;
                if (!follower.isBusy()) {
                    shootAll(true);
                    if (!shooting) {
                        setPathState(-1);
                    }
                }
                break;
        }
    }


    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }


    @Override
    public void init() {

        CR1 = hardwareMap.get(CRServoImplEx.class, "CR1");
        CR0 = hardwareMap.get(CRServoImplEx.class, "CR0");
        CR2 = hardwareMap.get(CRServoImplEx.class, "CR2");

        CR0.setDirection(CRServoImplEx.Direction.FORWARD);
        CR1.setDirection(CRServoImplEx.Direction.FORWARD);
        CR2.setDirection(CRServoImplEx.Direction.FORWARD);

        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        flywheel1 = (DcMotorEx) hardwareMap.dcMotor.get("fly1");
        flywheel2 = (DcMotorEx) hardwareMap.dcMotor.get("fly2");

        flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flywheel2.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!

        if (follower == null) {
            follower = Constants.createFollower(hardwareMap);
        }
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        shootTimer = new Timer();
        opmodeTimer.resetTimer();
        buildPaths();
        follower.setStartingPose(startPose);

    }

    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        if (intakeOn){
            activateIntake();
        }
        autonomousPathUpdate();
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (gamepad1.b)
            limelight.pipelineSwitch(0);
        if (gamepad1.x) {
            limelight.pipelineSwitch(1);
        }
        follower.update();
    }


    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }

    public void shootAll(boolean far) {
        if (pathTimer.getElapsedTimeSeconds() < 5) {
            shooting = true;
            CR0.setPower(1);
            CR1.setPower(1);
            CR2.setPower(1);
            intake.setPower(0.6);
            if (far) {
//                flywheel1.setPower(fly1_PID.update(farTarget, flywheel1.getVelocity()));
//                flywheel2.setPower(fly2_PID.update(farTarget, flywheel2.getVelocity()));
            } else {
//                flywheel1.setPower(fly1_PID.update(closeTarget, flywheel1.getVelocity()));
//                flywheel2.setPower(fly2_PID.update(closeTarget, flywheel2.getVelocity()));
            }
        }
        else{
            shooting = false;
        }



//        if ((flywheel1.getCurrent(CurrentUnit.AMPS) >= 6) || (flywheel2.getCurrent(CurrentUnit.AMPS) >= 6)){
//            flywheel1.setPower(0);
//            flywheel2.setPower(0);
//            CR0.setPower(-1);
//            CR1.setPower(-1);
//            CR2.setPower(-1);
//            ballsLaunched += 1;
//        }
    }

    private Pose limelightScan() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                double x = botpose.getPosition().x;
                double y = botpose.getPosition().y;
                currentPose = new Pose(x, y, follower.getHeading(), FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
                return currentPose;
            }
        }
        return follower.getPose();

    }

    private double updateTx() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getTx();
        }
        else{
            return 0;
        }
    }


    private void activateIntake(){
        intake.setPower(0.6);
        CR0.setPower(1);
        CR1.setPower(1);
        CR2.setPower(1);
    }

    public Pose poseToAimTo(){
        Pose here = follower.getPose();
        double tx = updateTx();
        here = here.rotate(tx, true);
        return here;
    }


}
