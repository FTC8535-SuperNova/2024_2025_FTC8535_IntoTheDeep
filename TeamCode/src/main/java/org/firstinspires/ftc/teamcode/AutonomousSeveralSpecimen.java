package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.ArmMode;
import org.firstinspires.ftc.teamcode.control.RobotController;




@Autonomous(name= "AutonomousSeveralSpecimen", preselectTeleOp = "Main_Teleop")
public class AutonomousSeveralSpecimen extends LinearOpMode {
    private final RobotController robotController = new RobotController();

    private final ElapsedTime runtime = new ElapsedTime();
    double yaw = 0;
    double lateral = 0;
    double axial = 0;
    double shoulderCommand = 0;
    boolean clawClosed;
    double linearSlideCommand;

    @Override
    public void runOpMode() {
        robotController.init(hardwareMap, telemetry, true);

        waitForStart();

        runtime.reset();

        while (opModeIsActive()) {

            double elapsedTime = runtime.seconds();


            if (elapsedTime < 1.0) {
                //move the arm motor to the point where it can place the specimen on the bar
                robotController.setArmMode(ArmMode.HIGH_SPECIMEN);
                robotController.update(0, 0, 0,
                        0, lateral, 0, false, true, false,
                        false, false);
            } else if (elapsedTime < 3.5) {
                //move the robot forward to the observation zone
                robotController.setArmMode(ArmMode.HIGH_SPECIMEN);
                axial = 1;
                robotController.update(0, 0, 0, axial, lateral, 0, false, true, false,
                        false, false);
            } else if (elapsedTime < 4.3) {
                //push the arm down a bit to place the specimen on the bar
                shoulderCommand = -1;
                robotController.setArmMode(ArmMode.DRIVER_CONTROL);
                robotController.update(shoulderCommand, 0, 0,
                        0, 0, 0, false, true, false,
                        false, false);
            } else if (elapsedTime < 4.8) {
                //make the claw let go of the specimen
                clawClosed = false;
                robotController.update(0, 0, 0,
                        0, 0, 0, false, false, false,
                        false, false);
            } else if (elapsedTime < 7.3) {
                //make the robot drive backwards a small amount
                axial = 1;
                robotController.update(0, 0, 0, axial, 0, 0, false, true, false,
                        false, false);
            } else if (elapsedTime < 8.0) {
                //peer pressure the robot into going left
                lateral = 1;
                robotController.update(0, 0, 0,
                        0, lateral, 0, false, true, false,
                        false, false);
            } else if (elapsedTime < 10.0) {
                //make the robot drive forward
                axial = 1;
                robotController.update(0, 0, 0, axial, 0, 0, false, true, false,
                        false, false);
            } else if (elapsedTime < 11.0) {
                //make the robot go to the right
                lateral = 1;
                robotController.update(0, 0, 0,
                        0, lateral, 0, false, true, false,
                        false, false);
            } else if (elapsedTime < 11.5) {
                //use the force to mind trick the robot into pushing a specimen to the observation zone >:)
                axial = -1;
                robotController.update(0, 0, 0, axial, 0, 0, false, true, false,
                        false, false);
            } else if (elapsedTime < 12.0) {
                //make the robot go forwards again
                axial = 1;
                robotController.update(0, 0, 0, axial, 0, 0, false, true, false,
                        false, false);
            } else if (elapsedTime < 12.5) {
               //make the robot spin 180 degrees
                yaw = 1;
                robotController.update(0, 0, 0,
                        0, 0, yaw, false, true, false,
                        false, false);
            } else if (elapsedTime < 13.0) {
                //make the robot drive forwards
                axial = 1;
                robotController.update(0, 0, 0, axial, 0, 0, false, true, false,
                        false, false);
            } else if (elapsedTime < 13.0) {
               //move the arm up to grab the specimen
                robotController.setArmMode(ArmMode.GRAB_SPECIMEN);
                robotController.update(0, 0, 0,
                        0, 0, 0, false, true, false,
                        false, false);
            } else if (elapsedTime < 13.5) {
               // open the claw
                clawClosed = false;
                robotController.update(0, 0, 0,
                        0, 0, 0, false, false, false,
                        false, false);
            } else if (elapsedTime < 14.2) {
                //extend the linear slide a TINY amount
                linearSlideCommand = 1;
                robotController.update(0, 0, 0,
                        0, 0, 0, false, true, false,
                        false, false);
            } else if (elapsedTime < 14.5) {
                //close the claw
                clawClosed = true;
                robotController.update(0, 0, 0,
                        0, 0, 0, false, true, false,
                        false, false);
            } else if (elapsedTime < 16.0) {
                //move the arm up to the high specimen
                robotController.setArmMode(ArmMode.HIGH_SPECIMEN);
                robotController.update(0, 0, 0,
                        0, 0, 0, false, true, false,
                        false, false);
            } else if (elapsedTime < 16.5) {
                // spin 180 degrees
                yaw = 1;
                robotController.update(0, 0, 0,
                        0, 0, yaw, false, true, false,
                        false, false);
            } else if (elapsedTime < 17.5) {
                //move to the left
                lateral = -1;
                robotController.update(0, 0, 0,
                        0, 0, 0, false, true, false,
                        false, false);
            } else if (elapsedTime < 18.5) {
                //drive forward a bit
                axial = 1;
                robotController.update(0, 0, 0, axial, 0, 0, false, true, false,
                        false, false);
            } else if(elapsedTime < 18.7) {
                //place specimen on bar
                shoulderCommand = -1;
                robotController.update(shoulderCommand, 0, 0,
                        0, 0, 0, false, true, false,
                        false, false);
            } else if (elapsedTime < 19.0) {
                //make the robot go backwards
                axial = -1;
                robotController.update(0, 0, 0, axial, 0, 0, false, true, false,
                        false, false);
            } else if (elapsedTime < 20.0) {
                //persuade the robot with the Federalist Papers no. 10 to drive to the right
                lateral = 1;
                robotController.update(0, 0, 0,
                        0, lateral, 0, false, true, false,
                        false, false);

            } else {
                //STOP THE ROBOT
                //FOR THE LOVE OF GOD
                //STOP THE ROBOT
                //STOP IT
            }

        }


    }}
