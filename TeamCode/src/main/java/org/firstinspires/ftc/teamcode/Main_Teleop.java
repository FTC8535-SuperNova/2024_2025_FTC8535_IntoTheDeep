package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.control.RobotController;

@TeleOp
public class Main_Teleop extends LinearOpMode {

    private final RobotController robotController = new RobotController();

    boolean clawClosed = true;

    @Override
    public void runOpMode() {

        robotController.init(hardwareMap, telemetry);

        waitForStart();


        while (opModeIsActive()) {

            //reading controller inputs
            double shoulderCommand = -1 * gamepad2.left_stick_y;
            double linearSlideCommand = -1 * gamepad2.right_stick_y;

            double climberDrive;
            if (gamepad2.right_trigger > 0) {
                climberDrive = gamepad2.right_trigger;
            } else if (gamepad2.left_trigger > 0) {
                climberDrive = -gamepad2.left_trigger;
            } else {
                climberDrive = 0;
            }

            if (gamepad2.x) {
                clawClosed = true;
            } else if (gamepad2.y){
                clawClosed = false;
            }

            boolean zeroLinearSlide = gamepad1.a;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.right_stick_x;
            double yaw     =  gamepad1.left_stick_x;

            boolean isFastMode = (gamepad1.right_trigger == 1);

            robotController.update(shoulderCommand, linearSlideCommand, climberDrive,
                    axial, lateral, yaw, isFastMode, clawClosed, zeroLinearSlide);

        }

    }

}
