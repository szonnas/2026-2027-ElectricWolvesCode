package org.firstinspires.ftc.teamcode.pedroPathing.teleops;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.HelpfulClasses.PositionPID;
import org.firstinspires.ftc.teamcode.pedroPathing.HelpfulClasses.Turret_GameSpecific;
import org.firstinspires.ftc.teamcode.pedroPathing.HelpfulClasses.storage;
import org.firstinspires.ftc.teamcode.pedroPathing.PID;
import org.firstinspires.ftc.teamcode.pedroPathing.SERVO_POSITIONS;

import java.util.Objects;
/*
@TeleOp(name = "Turret TeleOp")
public class Turret_TeleOp extends OpMode {
    TelemetryManager telemetryM;

    public static Follower follower;
    Turret_GameSpecific game = new Turret_GameSpecific();
    private Pose quickSavePose = new Pose (0,0,0);
    private boolean poseStored = false;
    private boolean holdingPosition = false;
    private PathChain holdingPathchain;
    PositionPID autoaim = new PositionPID(PID.autoaimP, PID.autoaimI, PID.autoaimD);



    @Override
    public void init() {
        game.init(hardwareMap);

        if (follower == null) {
            follower = Constants.createFollower(hardwareMap);
        }
        follower.setStartingPose(storage.teleOpStart);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void init_loop() {
        game.initloop();
        if (gamepad1.b)
            game.setBlue();
        if (gamepad1.x) {
            game.setRed();
        }

        follower.update();
    }

    @Override
    public void start() {
        //SETS TURRET START POSITION
        game.Start();
        follower.setPose(new Pose(0,0,0));
        follower.startTeleopDrive();
        follower.update();
    }

    @Override
    public void loop() {

        game.initloop();

        telemetry.addLine("Distance from goal in feet: " +
                (game.robotPose.distanceFrom(new Pose(144,144,0))) / 12);
        telemetry.addLine("The Turet Degrees is: "+ game.turretTickstoDegrees());
        telemetry.addLine("The turret thinks its global heading is: " + (game.turretTickstoDegrees()
        +Math.toDegrees(game.robotPose.getHeading())));
        telemetry.addLine("The Turret PID Error is: "+game.turret_PID.error);


        if (gamepad1.dpad_up){
            game.setBlue();}
        if (gamepad1.dpad_left) {
            game.setRed();}

        if (gamepad1.b){
            follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), 0));
        }
        if (gamepad1.x){
            follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), Math.PI));

        }


        //Driver Control
        double speed_factor = 1;
        if (gamepad1.right_trigger > 0.05) {
            speed_factor = 1 - (0.85 * gamepad1.right_trigger);
        }
        if (gamepad1.left_trigger > 0.7) {
            speed_factor = -speed_factor;
        }

        game.robotPose = follower.getPose();

        if (gamepad2.y){
            telemetry.addLine("TRYING TO SET POSE");

            if (!Objects.equals(game.limelightGetPose(), new Pose(0, 0, 0))) {
                telemetry.addLine("PAST THE IF STATEMENT");
                telemetry.addLine("The Pose Is " + game.limelightGetPose().toString());
                Pose llPose = game.limelightGetPose();
                follower.setPose(new Pose(llPose.getX(), llPose.getY(), llPose.getHeading()));

            }
        }
        double headingInput = -gamepad1.right_stick_x;
        if (gamepad1.y){
            LLResult result = game.limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double tx = result.getTx();
                headingInput = autoaim.update(0, -tx);
            }
        }



        follower.setTeleOpDrive((-gamepad1.left_stick_y * speed_factor), (-gamepad1.left_stick_x * speed_factor),
                headingInput, true);

//        //Pressing A records the current position of the robot
//        if (gamepad1.aWasPressed()){
//            toHold = follower.getPose();
//        }


        //Pressing should create a pathchain from the robot to the recorded position
        //Holding B should make the robot bee-line to the recorded position and once its there it should hold its position
//        if (gamepad1.bWasPressed() && toHold != null){
//            holdingPathchain = follower.pathBuilder()
//                    .addPath(new BezierLine(follower::getPose, () -> toHold))
//                    .setConstantHeadingInterpolation(toHold.getHeading())
//                    .build();
//
//            holdingPosition = false;
//        }
//
//        if (gamepad1.b && holdingPathchain != null && !holdingPosition){
//                follower.followPath(holdingPathchain);
//                if (!follower.isBusy()){
//                    holdingPosition = true;
//                }
//        }


        follower.update();

//        if (holdingPosition && toHold != null && gamepad1.b){
//            follower.holdPoint(toHold);
//        }
//        else {holdingPosition = false; }


        telemetry.addLine("Color: "+ game.color);
        telemetry.addLine("x:" + follower.getPose().getX());
        telemetry.addLine("y:" + follower.getPose().getY());
        telemetry.addLine("heading:" + Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addLine("total heading:" + follower.getTotalHeading());
        telemetry.addLine(String.valueOf((game.turretstate)));
        telemetry.addLine("SHOTS LEFT: " + game.shotsRemaining);

        telemetry.update();


        //Intake Control
        if (gamepad2.right_trigger > 0.1){
            game.intake.setPower(0.75 * gamepad2.right_trigger);
        }
        else if (gamepad2.left_trigger > 0.1) {
            game.intake.setPower(-0.75 * gamepad2.left_trigger);
        }
        else {
            game.intake.setPower(0);
        }





        //launching control
        if (gamepad2.aWasPressed()){
            game.shoot(3);
            game.targetSpeed = 2150;
        }
        if (gamepad2.bWasPressed()){
            game.shoot(3);
            game.targetSpeed = 1750;
        }
        if (gamepad2.xWasPressed()){
            game.shoot(3);
        }

        if (gamepad2.dpad_up){
            game.aimTurretBlind();
        }

        else if (gamepad2.dpad_right){
            game.tRotate.setPower(0.75);
        }
        else if (gamepad2.dpad_left){
            game.tRotate.setPower(-0.75);
        }
        else {
            game.tRotate.setPower(0);
        }

        if (gamepad2.right_bumper){
            game.feed.setPosition(SERVO_POSITIONS.feedEnd);
        }
        else if (gamepad2.left_bumper){
            game.feed.setPosition(SERVO_POSITIONS.feedStart);
        }



        game.AHHH();


    }

    public void setQuickSave(){
        quickSavePose = follower.getPose();
    }

    public void runQuickSave(){
        PathChain quicksavepchain = follower.pathBuilder()
                .addPath(new BezierLine(follower::getPose, quickSavePose))
                .setLinearHeadingInterpolation(follower.getHeading(), quickSavePose.getHeading())
                .build();
    }







}
