package org.firstinspires.ftc.teamcode.pedroPathing.Autons;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.HelpfulClasses.Turret_GameSpecific;

/*
@Autonomous(name = "BLUE CLOSE")
public class Blue_Close_T extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    public enum PathState{
        DRIVE_START_TO_FARSHOOT,
        SHOOT1,
        DRIVE_FARSHOOT_TO_GPP,
        GPP_PICKUP,
        DRIVE_GPP_END_TO_FARSHOOT,
        SHOOT2,
        PARK
    }

    public PathState THESTATEOFTHEPATH = PathState.DRIVE_START_TO_FARSHOOT;
    private Turret_GameSpecific gameSpecific = new Turret_GameSpecific();
    private final Pose startPose = new Pose(120,127,-53).mirror();
    private final Pose closeShoot = new Pose(90,90,0).mirror();
    private final Pose PPG_Pickup = new Pose(103,84,0).mirror();
    private final Pose PPG_End = new Pose(123,84,0).mirror();
    private final Pose park = new Pose(110,90,0).mirror();
    private PathChain start_farshoot, farshoot_GPP_pickup, GPP_pickup_end, GPP_end_farshoot, gotopark;
    private boolean shotsTriggered = false;


    public void buildPaths(){
        start_farshoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose,closeShoot))
                .setLinearHeadingInterpolation(startPose.getHeading(), closeShoot.getHeading())
                .build();

        farshoot_GPP_pickup = follower.pathBuilder()
                .addPath(new BezierLine(closeShoot,PPG_Pickup))
                .setLinearHeadingInterpolation(closeShoot.getHeading(),PPG_Pickup.getHeading())
                .build();

        GPP_pickup_end = follower.pathBuilder()
                .addPath(new BezierLine(PPG_Pickup,PPG_End))
                .setLinearHeadingInterpolation(PPG_Pickup.getHeading(), PPG_End.getHeading())
                .build();

        GPP_end_farshoot = follower.pathBuilder()
                .addPath(new BezierLine(PPG_End, closeShoot))
                .setLinearHeadingInterpolation(PPG_End.getHeading(), closeShoot.getHeading())
                .build();

        gotopark = follower.pathBuilder()
                .addPath(new BezierLine(closeShoot, park))
                .setLinearHeadingInterpolation(closeShoot.getHeading(), park.getHeading())
                .build();
    }

    public void statePathUpdate(){
        if (THESTATEOFTHEPATH == null){
            telemetry.addLine("ERROR: PathState is NULL");
            return;
        }

        switch(THESTATEOFTHEPATH){
            case DRIVE_START_TO_FARSHOOT:
                follower.followPath(start_farshoot, true);
                setPathState(PathState.SHOOT1);
                break;

            case SHOOT1:
                if (!follower.isBusy()){
                    if (!gameSpecific.isshooting() && !shotsTriggered) {
                        gameSpecific.shoot(3);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered & !gameSpecific.isshooting()){
                        shotsTriggered = false;
                        follower.followPath(farshoot_GPP_pickup);
                        setPathState(PathState.DRIVE_FARSHOOT_TO_GPP);
                    }
                }
                break;

            case DRIVE_FARSHOOT_TO_GPP:
                if (!follower.isBusy()){
                    follower.followPath(GPP_pickup_end, 0.75,true);
                    gameSpecific.intakeOn();
                    setPathState(PathState.GPP_PICKUP);
                }
                break;

            case GPP_PICKUP:
                gameSpecific.intakeOn();
                if (!follower.isBusy()){
                    gameSpecific.intakeOff();
                    follower.followPath(GPP_end_farshoot,true);
                    setPathState(PathState.DRIVE_GPP_END_TO_FARSHOOT);
                }
                break;

            case DRIVE_GPP_END_TO_FARSHOOT:

                if (!follower.isBusy()){
                    setPathState(PathState.SHOOT2);
                }
                break;

            case SHOOT2:
                if (!follower.isBusy()){
                    if (!gameSpecific.isshooting() && !shotsTriggered) {
                        gameSpecific.shoot(3);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered & !gameSpecific.isshooting()){
                        shotsTriggered = false;
                        follower.followPath(gotopark);
                        setPathState(PathState.PARK);
                    }
                }
                break;

            case PARK:
                gameSpecific.initloop();

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
        gameSpecific.init(hardwareMap);
        if (follower == null) {
            follower = Constants.createFollower(hardwareMap);
        }
        THESTATEOFTHEPATH = PathState.DRIVE_START_TO_FARSHOOT;
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        buildPaths();
        follower.setStartingPose(startPose);
        gameSpecific.targetSpeed = 1750;

    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();
        gameSpecific.AHHH();


        telemetry.addData("path state", THESTATEOFTHEPATH.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }



}
