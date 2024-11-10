package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Sweeper", group="Linear opMode")
public class Sweeper extends LinearOpMode {
    //Declare sweeper
    private DcMotor Sweeper = null;

    @Override
    public void runOpMode(){

        //Connect variable to actual motor
        Sweeper = hardwareMap.get(DcMotor.class, "Sweeper");

        //Set motor direction
        Sweeper.setDirection(DcMotor.Direction.FORWARD);

        //Wait until driver presses Start
        waitForStart();

        //Continue until game stops (driver presses stop)
        while (opModeIsActive()) {

            //Read gamepad2 values
            double drive = -1 * gamepad2.left_stick_y;

            Sweeper.setPower(drive);



        }

    }
}
