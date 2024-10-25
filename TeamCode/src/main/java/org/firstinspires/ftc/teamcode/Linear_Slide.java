package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Linear_Slide extends LinearOpMode {
    //Initialise linear slide motor
    private DcMotor linear_slide = null;

    @Override
    public void runOpMode() {
        //connect linear_slide motor to robot
        linear_slide = hardwareMap.get(DcMotor.class, "Linear_slide");

        //set motor direction
        linear_slide.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {

            //use gamepad2 to set power
            double drive = 0.5 * gamepad2.right_stick_y;

            //set power to linear slide
            linear_slide.setPower(drive);


        }
    }
}
