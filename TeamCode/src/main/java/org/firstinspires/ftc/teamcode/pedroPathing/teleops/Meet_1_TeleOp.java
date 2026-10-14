package org.firstinspires.ftc.teamcode.pedroPathing.teleops;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.HelpfulClasses.VelocityPID;
import org.firstinspires.ftc.teamcode.pedroPathing.PID;
/*
@Disabled
@TeleOp(name = "Meet 2 TeleOp")
public class Meet_1_TeleOp extends OpMode {
    TelemetryManager telemetryM;

    CRServoImplEx CR1, CR2, CR0;
    DcMotorEx intake, flywheel1, flywheel2;
    public static Follower follower;
    public static Pose currentPose = new Pose(0, 0, Math.toRadians(0));
    ElapsedTime timer1 = new ElapsedTime();
    VelocityPID fly1_PID = new VelocityPID(PID.fly1P, PID.fly1I, PID.fly1D, PID.fly1F, true);
    VelocityPID fly2_PID = new VelocityPID(PID.fly2P, PID.fly2I, PID.fly2D, PID.fly2F,true);
    VelocityPID autoaim = new VelocityPID(PID.autoaimP, PID.autoaimI, PID.autoaimD,PID.autoaimD,true);
    double closeTarget = 750;
    double farTarget = 960;
    Boolean intakeMovin = true;
    Limelight3A limelight;

    public enum color {BLUE, RED};

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
        limelight.setPollRateHz(100);// This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        if (follower == null) {
            follower = Constants.createFollower(hardwareMap);
        }
        follower.setStartingPose(currentPose);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();


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
        follower.startTeleopDrive();
        follower.update();

    }

    @Override
    public void loop() {
        //Driver Controller Stuff

        if (gamepad1.dpad_up)
            limelight.pipelineSwitch(0);
        if (gamepad1.dpad_left) {
            limelight.pipelineSwitch(1);
        }

        //Limelight Stuff
        //When Y is pressed, the robot will reset localization based on the limelight
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            telemetry.addLine("ITS LOOKING AT SOMETHING");
        }


        if (gamepad1.y) {
            if (result != null && result.isValid()) {
                Pose3D botpose = result.getBotpose();
                if (botpose != null) {

                    double x = botpose.getPosition().x;
                    double y = botpose.getPosition().y;
                    currentPose = new Pose(x, y, follower.getHeading(), FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
//                    follower.setPose(currentPose);
                    telemetry.addLine("The Limelight Thinks Our Position is: ");
                    telemetry.addLine("X = " + currentPose.getX());
                    telemetry.addLine("Y = " + currentPose.getY());
                    telemetry.addLine("Heading = " + currentPose.getHeading());
                }
            }
        }

        //autoaim when they press a
        double headingInput = -gamepad1.right_stick_x;
        if (gamepad1.a) {
            if (result != null && result.isValid()) {
                double tx = result.getTx();
//                headingInput = autoaim.update(2.5, -tx);
            }


//            double y_difference = (135 - currentPose.getPose().getY());
//            double x_difference = (12 - currentPose.getPose().getX());
//            double targetHeading = Math.toDegrees(Math.atan(y_difference / x_difference));
//            if (targetHeading > 180){
//                targetHeading -= 360;
//            }
//            else if (targetHeading < -180){
//                targetHeading += 360;
//            }
//            headingInput = autoaim.update(targetHeading, follower.getHeading());
        }
        if (gamepad1.b) {
            if (result != null && result.isValid()) {
                double tx = result.getTx();
//                headingInput = autoaim.update(2.5, -tx);
            }
        }



            //Slow mode and reverse mode
            double speed_factor = 1;
            if (gamepad1.right_trigger > 0.05) {
                speed_factor = 1 - (0.85 * gamepad1.right_trigger);
            }
            if (gamepad1.left_trigger > 0.7) {
                speed_factor = -speed_factor;
            }

            //actual driving
            follower.setTeleOpDrive((-gamepad1.left_stick_y * speed_factor), (-gamepad1.left_stick_x * speed_factor),
                    headingInput, true);
            follower.update();

            telemetry.addLine("x:" + follower.getPose().getX());
            telemetry.addLine("y:" + follower.getPose().getY());
            telemetry.addLine("heading:" + follower.getPose().getHeading());
            telemetry.addLine("total heading:" + follower.getTotalHeading());
            telemetry.update();


            //Game Specific Controller Stuff
            //Intake control
            if (gamepad2.right_trigger > 0.1) {
                intake.setPower(gamepad2.right_trigger * 0.75);
            } else if (gamepad2.left_trigger > 0.1) {
                intake.setPower(-gamepad2.left_trigger * 0.75);

            } else if (intake.getCurrent(CurrentUnit.AMPS) >= 8.4) {
                intakeMovin = false;
            } else {
                intake.setPower(0);
            }

            //Transfer control
            //Bottom Two Motors
            if (gamepad2.right_bumper) {
                CR0.setPower(1);
                CR1.setPower(1);
            } else if (gamepad2.left_bumper) {
                CR0.setPower(-1);
                CR1.setPower(-1);
            } else {
                CR0.setPower(0);
                CR1.setPower(0);
            }

            //Top Motor
            if (gamepad2.dpad_up) {
                CR2.setPower(1);
            } else if (gamepad2.dpad_down) {
                CR2.setPower(-1);
            } else {
                CR2.setPower(0);
            }


            //Launch control
            if (gamepad2.a) {
//                flywheel1.setPower(fly1_PID.update(closeTarget, flywheel1.getVelocity()));
//                flywheel2.setPower(fly2_PID.update(closeTarget, flywheel2.getVelocity()));
            } else if (gamepad2.b) {
//                flywheel1.setPower(fly1_PID.update(farTarget, flywheel1.getVelocity()));
//                flywheel2.setPower(fly2_PID.update(farTarget, flywheel2.getVelocity()));

            } else {
                flywheel1.setPower(0);
                flywheel2.setPower(0);
            }
    }

}