package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.HelpfulClasses.VelocityPID;
import org.firstinspires.ftc.teamcode.pedroPathing.PID;

@Disabled
@TeleOp(name = "Drivetrain Testing", group = "Test")
    public class DriveTrainTesting extends OpMode {

        TelemetryManager telemetryM;
        CRServoImplEx CR1, CR2, CR0;
        DcMotorEx intake, flywheel1, frontLeft,frontRight,backLeft,backRight;
        VelocityPID fly1_PID = new VelocityPID(PID.fly1P, PID.fly1I, PID.fly1D, PID.fly1F,true);

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

            flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);

            flywheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            frontLeft = (DcMotorEx) hardwareMap.dcMotor.get("motor_lf");
            frontRight = (DcMotorEx) hardwareMap.dcMotor.get("motor_rf");
            backLeft = (DcMotorEx) hardwareMap.dcMotor.get("motor_lb");
            backRight = (DcMotorEx) hardwareMap.dcMotor.get("motor_rb");

            telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();


        }

        @Override
        public void loop() {

            if (gamepad1.right_bumper){
                backRight.setPower(1);
            }
            else{backRight.setPower(0);}

            if(gamepad1.right_trigger>0){
                frontRight.setPower(1);
            }
            else{frontRight.setPower(0);}

            if (gamepad1.left_bumper){
                backLeft.setPower(1);
            }
            else{backLeft.setPower(0);}

            if(gamepad1.left_trigger>0){
                frontLeft.setPower(1);
            }
            else{frontLeft.setPower(0);
            }

            telemetryM.addData("Back Right", backRight.getVelocity());
            telemetryM.addData("Back Left", backLeft.getVelocity());
            telemetryM.addData("Front Right", (frontRight.getVelocity()));
            telemetryM.addData("Front Left", (frontLeft.getVelocity()));
            telemetryM.update();



        }
    }



