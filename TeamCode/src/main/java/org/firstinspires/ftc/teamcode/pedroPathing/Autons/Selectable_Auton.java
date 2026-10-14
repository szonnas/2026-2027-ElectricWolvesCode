package org.firstinspires.ftc.teamcode.pedroPathing.Autons;


import com.pedropathing.telemetry.SelectScope;
import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.function.Consumer;
import java.util.function.Supplier;
/*
Never made this work but it could be really useful for control award next year. And shouldn't be too hard to implement
The idea is to make the autonomous work sort of like tuning where theres one opmode and you select different versions with it
An advanced version could be used to make it customizable. like before the match starts the driver can select where the robot
will go and what it will do by stringing together paths and actions
IDK how hard it will be to implement though.
 */
/*
@Disabled
@Autonomous(name="Selectable Auton")
public class Selectable_Auton extends SelectableOpMode {
    public Selectable_Auton() {
        super("Select A Tuner", s -> {







        });
    }
}
