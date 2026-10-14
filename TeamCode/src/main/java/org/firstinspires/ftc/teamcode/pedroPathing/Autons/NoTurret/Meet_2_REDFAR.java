package org.firstinspires.ftc.teamcode.pedroPathing.Autons.NoTurret;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.HelpfulClasses.OldGameSpecific;
import org.firstinspires.ftc.teamcode.pedroPathing.HelpfulClasses.VelocityPID;
import org.firstinspires.ftc.teamcode.pedroPathing.PID;
/*
@Disabled
@Autonomous(name = "RED FAR", group = "Autonomous")
public class Meet_2_REDFAR extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private enum PathState{
        DRIVE_start_to_farshoot,
        SHOOT_farshoot_1,
        DRIVE_farshoot_to_GPPPICKUP,
        DRIVEINTAKE_GPP,
        GPPEND_to_farshoot,
        SHOOOT_farshoot_2,
        LEAVE

    }
    private PathState THESTATEOFTHEPATH = PathState.DRIVE_start_to_farshoot;

    private OldGameSpecific gamespecific = new OldGameSpecific();
    private boolean shotsTriggered = false;
    VelocityPID autoaim = new VelocityPID(PID.autoaimP,PID.autoaimI,PID.autoaimD,PID.autoaimF, false);
    Limelight3A limelight;
    private final Pose REDstartPose = new Pose(56, 8, Math.toRadians(-90));
    private final Pose startPose = REDstartPose.mirror();
    private final Pose REDfarScore = new Pose(56, 12, Math.toRadians(-68));
    private final Pose farScore = REDfarScore.mirror();
    private final Pose REDGPP_pickup = new Pose(43, 35.5, Math.toRadians(180));
    private final Pose GPP_pickup = REDGPP_pickup.mirror();
    private final Pose REDGPP_end = new Pose(8, 35.5, Math.toRadians(180));
    private final Pose GPP_end = REDGPP_end.mirror();
    private final Pose closeScore = new Pose(67, 84, Math.toRadians(135));
    private PathChain start_to_farshoot, farshoot_to_GPPPickup, GPPPickup, GPPEnd_to_Farshoot;
    double closeflyspeed = 1070;
    double farflyspeed = 1250;
    double ballsLaunched = 0;
    boolean intakeOn = false;


    public void buildPaths() {
        start_to_farshoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose,farScore))
                .setLinearHeadingInterpolation(startPose.getHeading(), farScore.getHeading())
                .build();

        farshoot_to_GPPPickup = follower.pathBuilder()
                .addPath(new BezierLine(farScore,GPP_pickup))
                .setLinearHeadingInterpolation(farScore.getHeading(),GPP_pickup.getHeading())
                .build();

        GPPPickup = follower.pathBuilder()
                .addPath(new BezierLine(GPP_pickup, GPP_end))
                .setLinearHeadingInterpolation(GPP_pickup.getHeading(),GPP_end.getHeading())
                .setBrakingStart(0.1)
                .setBrakingStrength(2.5)
                .build();

        GPPEnd_to_Farshoot = follower.pathBuilder()
                .addPath(new BezierLine(GPP_end,farScore))
                .setLinearHeadingInterpolation(GPP_end.getHeading(), farScore.getHeading())
                .build();
    }

    public void statePathUpdate(){
        if (THESTATEOFTHEPATH == null){
            telemetry.addLine("ERROR: PathState is NULL");
            return;
        }

        switch(THESTATEOFTHEPATH){
            case DRIVE_start_to_farshoot:
                follower.followPath(start_to_farshoot, true);
                setPathState(PathState.SHOOT_farshoot_1);
                break;
            case SHOOT_farshoot_1:
                if (!follower.isBusy()){
                    if (!shotsTriggered){
                        gamespecific.fireShots(3,true);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !gamespecific.isShooting()){
                        shotsTriggered = false;
                        follower.followPath(farshoot_to_GPPPickup, true);
                        setPathState(PathState.DRIVE_farshoot_to_GPPPICKUP);
                    }
                }
                break;
            case DRIVE_farshoot_to_GPPPICKUP:
                if (!follower.isBusy()){
                    gamespecific.powerIntake();
                    follower.followPath(GPPPickup, 0.4,true);
                    setPathState(PathState.DRIVEINTAKE_GPP);
                }
                break;
            case DRIVEINTAKE_GPP:
                gamespecific.powerIntake();
                if (!follower.isBusy()){
                    gamespecific.intakeOff();
                    follower.followPath(GPPEnd_to_Farshoot, true);
                    setPathState(PathState.GPPEND_to_farshoot);
                }
                break;
            case GPPEND_to_farshoot:
                if (follower.getDistanceTraveledOnPath() < 30)
                    gamespecific.powerIntake();
                else{
                    gamespecific.intakeOff();
                }

                if (!follower.isBusy()){
                    if (!shotsTriggered){
                        gamespecific.fireShots(3,true);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !gamespecific.isShooting()){
                        shotsTriggered = false;
                        follower.followPath(farshoot_to_GPPPickup);
                        setPathState(PathState.LEAVE);
                    }
                }
                break;
            case LEAVE:

            default:
                telemetry.addLine("No Pathstate selected or whatever");
        }
    }


    public void setPathState(PathState newState){
        if (newState == null) {
            telemetry.addLine("YOU DONE GOOFED AND IT DID A NULL YOU DUMB FUCK");
            return;
        }
        THESTATEOFTHEPATH = newState;
        pathTimer.resetTimer();

        shotsTriggered = false;

    }



    @Override
    public void init() {
        gamespecific.init(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        if (follower == null) {
            follower = Constants.createFollower(hardwareMap);
        }

        THESTATEOFTHEPATH = PathState.DRIVE_start_to_farshoot;
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        buildPaths();
        follower.setStartingPose(startPose);

    }

    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        statePathUpdate();
        gamespecific.update();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", THESTATEOFTHEPATH.toString());
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
        setPathState(PathState.DRIVE_start_to_farshoot);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }




//        if ((flywheel1.getCurrent(CurrentUnit.AMPS) >= 6) || (flywheel2.getCurrent(CurrentUnit.AMPS) >= 6)){
//            flywheel1.setPower(0);
//            flywheel2.setPower(0);
//            CR0.setPower(-1);
//            CR1.setPower(-1);
//            CR2.setPower(-1);
//            ballsLaunched += 1;
//        }


    private Pose limelightScan() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                double x = botpose.getPosition().x;
                double y = botpose.getPosition().y;
                return new Pose(x, y, follower.getHeading(), FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);

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


    public Pose poseToAimTo(){
        Pose here = follower.getPose();
        double tx = updateTx();
        here = here.rotate(tx, true);
        return here;
    }


}
