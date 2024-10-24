package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Climber extends LinearOpMode {

    //initialize climber
    private DcMotor climber = null;

    @Override
    public void runOpMode() {
        //connect linear_slide variable to motor on robot
        climber = hardwareMap.get(DcMotor.class, "Climber");

        //set climber direction
        climber.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {

            //use gamepad2 to set power
            double drive = -1 * gamepad2.right_trigger;

            //assign power to motor
            climber.setPower(drive);
        }
    }
}
