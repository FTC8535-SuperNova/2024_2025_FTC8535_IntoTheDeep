package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
@Disabled
public class ConstantServo_Test extends LinearOpMode {

    private Servo outServo = null;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData( "Status", "Initialized");
        telemetry.update();

        outServo = hardwareMap.servo.get("out_servo");

        waitForStart();

        while (opModeIsActive()) {

            // check to see if we need to move the servo.
            if (gamepad1.x) {
                // move to 135 degrees (close).
                outServo.setPosition(0.75);
            } else if (gamepad1.y) {
                //move to 45 degrees.
                outServo.setPosition(0.25);
            } //else if (gamepad1.a){
                //move to 135 degrees.
                //outServo.setPosition(0.75);
            //} //else if (gamepad1.b) {
                // move to 180 degrees.
                //outServo.setPosition(1);
            //}

            telemetry.addData("Servo Position", outServo.getPosition());
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
