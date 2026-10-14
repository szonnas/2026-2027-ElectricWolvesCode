package org.firstinspires.ftc.teamcode.pedroPathing.teleops;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.HelpfulClasses.Turret_GameSpecific;
import org.firstinspires.ftc.teamcode.pedroPathing.SERVO_POSITIONS;
/*
@TeleOp(name = "Manual TeleOp")
public class Manual_Turret extends OpMode {
    TelemetryManager telemetryM;

    public static Follower follower;
    private LynxModule hub;
    Turret_GameSpecific game = new Turret_GameSpecific();
    private Pose toHold = new Pose (0,0,0);
    private boolean poseStored = false;
    private boolean holdingPosition = false;
    private PathChain holdingPathchain;


    @Override
    public void init() {
        hub = hardwareMap.getAll(LynxModule.class).get(0);
        game.init(hardwareMap);

        if (follower == null) {
            follower = Constants.createFollower(hardwareMap);
        }
        follower.setStartingPose(new Pose(0,0,0));

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

        follower.startTeleopDrive();
        follower.update();

        follower.setPose(game.limelightGetPose());

    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up){
            game.setBlue();}
        if (gamepad1.dpad_left) {
            game.setRed();
        }

        if (gamepad1.dpad_up){
            follower.setPose(game.limelightGetPose());
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


        follower.setTeleOpDrive((-gamepad1.left_stick_y * speed_factor), (-gamepad1.left_stick_x * speed_factor),
                -gamepad1.right_stick_x, true);

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
        telemetry.update();


        //Intake Control
        if (gamepad2.right_trigger > 0.1){
            game.intake.setPower(1 * gamepad2.right_trigger);
        }
        else if (gamepad2.left_trigger > 0.1) {
            game.intake.setPower(-0.75 * gamepad2.left_trigger);
        }
        else {
            game.intake.setPower(0);
        }



        //launching control
        if (gamepad2.a){
            game.flywheel1.setPower(game.fly1_PID.update(game.targetSpeed, game.flywheel1.getVelocity(), hub.getInputVoltage(VoltageUnit.VOLTS)));
            game.flywheel2.setPower(game.fly2_PID.update(game.targetSpeed, game.flywheel2.getVelocity(), hub.getInputVoltage(VoltageUnit.VOLTS)));
        }
        else {
            game.flywheel1.setPower(0);
            game.flywheel2.setPower(0);

        }

        if (gamepad2.b){
            game.hardStopR.setPosition(SERVO_POSITIONS.gateStartR);
            game.hardStopL.setPosition(SERVO_POSITIONS.gateStartL);
        }
        else if (gamepad2.x){
            game.hardStopR.setPosition(SERVO_POSITIONS.gateEndR);
            game.hardStopL.setPosition(SERVO_POSITIONS.gateEndL);

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

}
