package org.firstinspires.ftc.teamcode.pedroPathing.Autons;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Autons.NoTurret.MEET_2_REDCLOSE;
import org.firstinspires.ftc.teamcode.pedroPathing.Autons.NoTurret.Meet_2_BLUECLOSE;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.HelpfulClasses.Turret_GameSpecific;

/*
@Autonomous(name = "RED FAR")
public class Red_Far_T extends OpMode {
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
    private final Pose startPose = new Pose(88,8,0);
    private final Pose farShoot = new Pose(88,12,0);
    private final Pose GPP_Pickup = new Pose(103,35,0);
    private final Pose GPP_End = new Pose(123,35,0);
    private final Pose park = new Pose(95,20,0);
    private PathChain start_farshoot, farshoot_GPP_pickup, GPP_pickup_end, GPP_end_farshoot, gotopark;
    private boolean shotsTriggered = false;


    public void buildPaths(){
        start_farshoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose,farShoot))
                .setLinearHeadingInterpolation(startPose.getHeading(), farShoot.getHeading())
                .build();

        farshoot_GPP_pickup = follower.pathBuilder()
                .addPath(new BezierLine(farShoot,GPP_Pickup))
                .setLinearHeadingInterpolation(farShoot.getHeading(),GPP_Pickup.getHeading())
                .build();

        GPP_pickup_end = follower.pathBuilder()
                .addPath(new BezierLine(GPP_Pickup,GPP_End))
                .setLinearHeadingInterpolation(GPP_Pickup.getHeading(), GPP_End.getHeading())
                .build();

        GPP_end_farshoot = follower.pathBuilder()
                .addPath(new BezierLine(GPP_End, farShoot))
                .setLinearHeadingInterpolation(GPP_End.getHeading(), farShoot.getHeading())
                .build();

        gotopark = follower.pathBuilder()
                .addPath(new BezierLine(farShoot, park))
                .setLinearHeadingInterpolation(farShoot.getHeading(), park.getHeading())
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
        gameSpecific.targetSpeed = 1900;

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
