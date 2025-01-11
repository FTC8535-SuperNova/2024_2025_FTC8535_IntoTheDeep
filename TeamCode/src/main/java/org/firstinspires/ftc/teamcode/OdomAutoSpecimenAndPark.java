package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.control.ArmMode;
import org.firstinspires.ftc.teamcode.control.RobotController;

@Autonomous(name="OdomAutoSpecimenAndPark", group="OdomAutonomous", preselectTeleOp = "Main_Teleop")
//@Disabled

public class OdomAutoSpecimenAndPark extends LinearOpMode {
    RobotController robotController = new RobotController();
    private final ElapsedTime runtime = new ElapsedTime();

    DriveToPoint nav = new DriveToPoint(this); //OpMode member for the point-to-point navigation class

    enum StateMachine {
        WAITING_FOR_START,
        AT_TARGET,
        RAISE_ARM_HIGH_SPECIMEN,
        DRIVE_TO_HIGH_SPECIMEN,
        DELIVER_SPECIMEN,
        OPEN_CLAW,
        DRIVE_TO_OBSERVATION_ZONE
    }

    static final Pose2D TARGET_HIGH_SPECIMEN = new Pose2D(DistanceUnit.MM,750,0,AngleUnit.DEGREES,0);
    static final Pose2D TARGET_OBSERVATION = new Pose2D(DistanceUnit.MM,0, -1485, AngleUnit.DEGREES, 0);


    @Override
    public void runOpMode() {
        robotController.init(hardwareMap, telemetry, true);
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        //nav.setXYCoefficients(0.02,0.002,0.0,DistanceUnit.MM,12);
        //nav.setYawCoefficients(1,0,0.0, AngleUnit.DEGREES,2);
        nav.setDriveType(DriveToPoint.DriveType.MECANUM);

        StateMachine stateMachine;
        stateMachine = StateMachine.WAITING_FOR_START;

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            robotController.updateOdometry();


            switch (stateMachine){
                case WAITING_FOR_START:
                    //the first step in the autonomous
                    stateMachine = StateMachine.RAISE_ARM_HIGH_SPECIMEN;
                    runtime.reset();
                    break;
                case RAISE_ARM_HIGH_SPECIMEN:
                    robotController.setArmMode(ArmMode.HIGH_SPECIMEN);
                    robotController.update(0, 0, 0,
                            true, false,
                            false, false);
                    if (runtime.seconds() >= 1.0){
                        stateMachine = StateMachine.DRIVE_TO_HIGH_SPECIMEN;
                    }
                    break;
                case DRIVE_TO_HIGH_SPECIMEN:
                    /*
                    drive the robot to the first target, the nav.driveTo function will return true once
                    the robot has reached the target, and has been there for (holdTime) seconds.
                    Once driveTo returns true, it prints a telemetry line and moves the state machine forward.
                     */
                    if (nav.driveTo(robotController.getOdometryPosition(), TARGET_HIGH_SPECIMEN, 0.7, 0)){
                        //drive to the submersible
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DELIVER_SPECIMEN;
                        runtime.reset();
                    }
                    break;
                case DELIVER_SPECIMEN:
                    robotController.setArmMode(ArmMode.DRIVER_CONTROL);
                    robotController.update(-1, 0, 0,
                            true, false,
                            false, false);
                    if (runtime.seconds() >= 0.8) {
                        stateMachine = StateMachine.OPEN_CLAW;
                        runtime.reset();
                    }
                    break;
                case OPEN_CLAW:
                    //make the claw let go of the specimen
                    robotController.update(0, 0, 0,
                            false, false,
                            false, false);
                    robotController.updateDriveCommands(0, 0, 0, false);
                    if (runtime.seconds() >= 0.5) {
                        stateMachine = StateMachine.DRIVE_TO_OBSERVATION_ZONE;
                        runtime.reset();
                    }
                    break;
                case DRIVE_TO_OBSERVATION_ZONE:
                    //raise the arm
                    if (nav.driveTo(robotController.getOdometryPosition(), TARGET_OBSERVATION, 0.7, 1)){
                        telemetry.addLine("at position #2!");
                        stateMachine = StateMachine.AT_TARGET;
                    }
                    break;

            }

            if (stateMachine == StateMachine.AT_TARGET){
                RobotController.WheelPower wheelPower = new RobotController.WheelPower(0,0,0,0);
                robotController.assignWheelPowers(wheelPower);
            } else {
                //nav calculates the power to set to each motor in a mecanum or tank drive. Use nav.getMotorPower to find that value.
                RobotController.WheelPower wheelPower = new RobotController.WheelPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT),
                        nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT),
                        nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK),
                        nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));
                robotController.assignWheelPowers(wheelPower);
            }

            telemetry.addData("current state:",stateMachine);

            telemetry.update();

        }
    }
}