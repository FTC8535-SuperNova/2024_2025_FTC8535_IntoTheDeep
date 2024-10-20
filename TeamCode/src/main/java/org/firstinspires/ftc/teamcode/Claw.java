package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Claw extends LinearOpMode {

    //Declare Servo
    private Servo Claw = null;

    @Override
    public void runOpMode() {
        Claw = hardwareMap.get(Servo.class, "Claw");

        waitForStart();

        double pos = 1.0;
        while (opModeIsActive()) {

            if (gamepad1.x) {
                pos = 1.0;
            } else if (gamepad1.y){
                pos = 0.75;
            }

            // Set position to servo
            Claw.setPosition(pos);
        }
    }


}
