package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.ArmMode;
import org.firstinspires.ftc.teamcode.control.RobotController;




@Autonomous(name= "AutonomousSpecimenObservation", preselectTeleOp = "Main_Teleop")
public class AutonomousSpecimenObservation extends LinearOpMode {
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


            if(elapsedTime < 1.0) {
                //move the arm motor to the point where it can place the specimen on the bar
                robotController.setArmMode(ArmMode.HIGH_SPECIMEN);
                robotController.update(0, 0, 0,
                        0, lateral, 0, false, true, false,
                        false, false);
            } else if(elapsedTime < 3.5) {
                //move the robot forward to the observation zone
                robotController.setArmMode(ArmMode.HIGH_SPECIMEN);
                axial = 1;
                robotController.update(0, 0, 0, axial, lateral, 0, false, true, false,
                        false, false);
            } else if(elapsedTime < 4.3){
                //push the arm down a bit to place the specimen on the bar
                shoulderCommand = -1;
                robotController.setArmMode(ArmMode.DRIVER_CONTROL);
                robotController.update(shoulderCommand, 0, 0,
                        0, 0, 0, false, true, false,
                        false, false);
            } else if(elapsedTime < 4.8){
                //make the claw let go of the specimen
                clawClosed = false;
                robotController.update(0, 0, 0,
                        0, 0, 0, false, false, false,
                        false, false);
            } else if(elapsedTime < 7.3){
                //make the robot go backwards to the wall
                axial = -1;
                robotController.update(0, 0, 0, axial, 0, 0, false, true, false,
                        false, false);
            } else if(elapsedTime < 12.3){
                // do what noah said and move the robot to the park zone
                robotController.setArmMode(ArmMode.GRAB_SPECIMEN);
                lateral = 1;
                robotController.update(0, 0, 0,
                        0, lateral, 0, false, true, false,
                        false, false);
            } else {
                //stop
                robotController.update(0, 0, 0,
                        0, 0, 0, false, true, false,
                        false, false);
            }





        }
    }
}

