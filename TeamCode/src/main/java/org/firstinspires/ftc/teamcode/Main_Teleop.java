package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.control.ArmMode;
import org.firstinspires.ftc.teamcode.control.RobotController;

@TeleOp
public class Main_Teleop extends LinearOpMode {

    private final RobotController robotController = new RobotController();

    boolean clawClosed = true;

    @Override
    public void runOpMode() {

        robotController.init(hardwareMap, telemetry, false);

        waitForStart();


        while (opModeIsActive()) {

            //reading controller inputs
            double shoulderCommand = -1 * gamepad2.left_stick_y;
            double linearSlideCommand = -1 * gamepad2.right_stick_y;

            double climberDrive;
            if (gamepad1.right_bumper) {
                climberDrive = 1.0;
            } else if (gamepad1.left_bumper) {
                climberDrive = -1.0;
            } else {
                climberDrive = 0;
            }
            robotController.setClimberOverride(gamepad1.a);

            if (gamepad2.left_bumper) {
                clawClosed = true;
            } else if (gamepad2.right_bumper){
                clawClosed = false;
            }

            boolean overrideArmLowLimits = gamepad2.right_trigger > 0.01;
            boolean zeroLinearSlide = gamepad1.x;
            boolean zeroShoulder = gamepad1.y;

            if (gamepad2.dpad_down) {
                // Go to grab specimen position
                robotController.setArmMode(ArmMode.GRAB_SPECIMEN);
            } else if (gamepad2.dpad_up) {
                // Go to deliver high specimen position
                robotController.setArmMode(ArmMode.HIGH_SPECIMEN);
            } else if (gamepad2.dpad_right) {
                robotController.setArmMode(ArmMode.HIGH_BASKET);
            }
            if (gamepad2.a) {
                robotController.setArmMode(ArmMode.DRIVER_CONTROL);
            }

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            boolean isFastMode = (gamepad1.right_trigger != 1);

            robotController.update(shoulderCommand, linearSlideCommand, climberDrive,
                    clawClosed, zeroLinearSlide, zeroShoulder, overrideArmLowLimits);
            robotController.updateDriveCommands(axial, lateral, yaw, isFastMode);
        }

    }

}
