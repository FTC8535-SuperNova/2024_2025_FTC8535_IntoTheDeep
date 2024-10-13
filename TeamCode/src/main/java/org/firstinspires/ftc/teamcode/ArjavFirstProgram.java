package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ArjavFirstProgram extends LinearOpMode {
    // Declare Servo **
    private Servo out_servo = null;

    // Declare Motor **
    private DcMotor left_linear_slide = null;

    // This is required!
    @Override
    public void runOpMode() {
        // Connect variable in code to physical motor on the bot **
        left_linear_slide = hardwareMap.get(DcMotor.class, "left_linear_slide");

        //Connect variable in code to servo on bot **
        out_servo = hardwareMap.get(Servo.class, "out_servo");

        // Set motor direction **
        left_linear_slide.setDirection(DcMotor.Direction.FORWARD);

        // Wait here until the start button is pressed  **
        waitForStart();

        while (opModeIsActive()){

            // Read values from gamepad1 (motor)
            double drive = -1 * gamepad1.left_stick_y;

            // Make position variable
            double pos;
            if (gamepad1.a) {
                pos = 1.0;
            } else {
                pos = 0.0;
            }

            // Assign power value to motor
            left_linear_slide.setPower(drive);

            // Set position to servo
            out_servo.setPosition(pos);




        }


    }
}