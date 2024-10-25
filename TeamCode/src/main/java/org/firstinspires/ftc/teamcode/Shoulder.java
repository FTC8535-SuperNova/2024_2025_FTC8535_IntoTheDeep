package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Shoulder extends LinearOpMode{

    //initialize motors
    private DcMotor shoulder_motor_1 = null;
    private DcMotor shoulder_motor_2 = null;


    @Override
    public void runOpMode() {
        //connect shoulder_motor variables to shoulder motors on robot
        shoulder_motor_1 = hardwareMap.get(DcMotor.class, "shoulder_left");
        shoulder_motor_2 = hardwareMap.get(DcMotor.class, "shoulder_right");

        //set motor directions
        shoulder_motor_1.setDirection(DcMotor.Direction.FORWARD);
        shoulder_motor_2.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            //use gamepad2 to set power
            double drive = -1 * gamepad2.left_stick_y;

            //assign power to motors
            shoulder_motor_1.setPower(drive);
            shoulder_motor_2.setPower(drive);

        }
    }


}
