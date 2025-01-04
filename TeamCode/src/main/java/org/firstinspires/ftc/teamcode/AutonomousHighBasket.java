package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.ArmMode;
import org.firstinspires.ftc.teamcode.control.RobotController;


@Autonomous(preselectTeleOp = "Main_Teleop")
public class AutonomousHighBasket extends LinearOpMode {
    private final RobotController robotController = new RobotController();

    private final ElapsedTime runtime = new ElapsedTime();

    double lateral = 0;
    double axial = 0;
    double shoulderCommand = 0;
    boolean clawClosed;

    @Override
    public void runOpMode() {
        robotController.init(hardwareMap, telemetry, true);

        waitForStart();

        runtime.reset();

        while (opModeIsActive()) {

            double elapsedTime = runtime.seconds();


            if(elapsedTime < 3.0) {
                //move the arm motor to the point where it can place the specimen on the bar
                robotController.setArmMode(ArmMode.HIGH_BASKET);
                robotController.update(0, 0, 0,
                        true, false,
                        false, false);
                robotController.updateDriveCommands(0, lateral, 0, false);
            } else if(elapsedTime < 4.0) {
                //move the robot forward to the observation zone
                robotController.setArmMode(ArmMode.HIGH_BASKET);
                axial = 1;
                robotController.update(0, 0, 0,
                        true, false,
                        false, false);
                robotController.updateDriveCommands(axial, 0, 0, false);
            } else if(elapsedTime < 4.5){
                //make the claw let go of the specimen
                clawClosed = false;
                robotController.update(0, 0, 0,
                        false, false,
                        false, false);
                robotController.updateDriveCommands(0, 0, 0, false);

            } else if(elapsedTime < 5.5){
                //make the robot go backwards to the wall
                axial = -1;
                robotController.update(0, 0, 0,
                        true, false,
                        false, false);
                robotController.updateDriveCommands(axial, 0, 0, false);
            } else if(elapsedTime < 8.5){
                // do what noah said and move the robot to the park zone
                robotController.setArmMode(ArmMode.GRAB_SPECIMEN);
                robotController.update(0, 0, 0,
                        true, false,
                        false, false);
                robotController.updateDriveCommands(0, 0, 0, false);
            } else {
                //stop
                robotController.update(0, 0, 0,
                        true, false,
                        false, false);
                robotController.updateDriveCommands(0, 0, 0, false);
            }





        }
    }
}

