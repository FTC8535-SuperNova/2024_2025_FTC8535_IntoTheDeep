package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.control.ArmMode;
import org.firstinspires.ftc.teamcode.control.RobotController;

@Autonomous(name="OdomAutoSampleHighBasket", group="OdomAutonomous", preselectTeleOp = "Main_Teleop")
//@Disabled

public class OdomAutoSampleHighBasket extends LinearOpMode {
    private static final Pose2D TARGET_RAISE_ARM_HIGH_BASKET = ;
    RobotController robotController = new RobotController();
    private final ElapsedTime runtime = new ElapsedTime();

    DriveToPoint nav = new DriveToPoint(this); //OpMode member for the point-to-point navigation class

    enum StateMachine {
        WAITING_FOR_START,
        AT_TARGET,
        RAISE_ARM_HIGH_BASKET,
        DRIVE_TO_HIGH_BASKET_STEP1,
        DRIVE_TO_HIGH_BASKET_STEP2,
        DELIVER_SAMPLE,
        OPEN_CLAW,
        DRIVE_TO_BAR_STEP1,
        DRIVE_TO_BAR_STEP2,
        TOUCH_BAR_STEP1,
        TOUCH_BAR_STEP2,
//        DRIVE_TO_SPEC1_STEP1,
//        DRIVE_TO_SPEC1_STEP2,
//        DRIVE_TO_SPEC1_STEP3,
//        DRIVE_TO_SPEC1_STEP4,
//        DRIVE_TO_SPEC2_STEP1,
//        DRIVE_TO_SPEC2_STEP2,
//        DRIVE_TO_SPEC2_STEP3,
//        DRIVE_TO_OBSERVATION_ZONE,
//        DRIVE_TO_SPEC3_STEP1,
//        DRIVE_TO_SPEC3_STEP2,
//        DRIVE_TO_SPEC3_STEP3,
    }

    static final Pose2D TARGET_HIGH_BASKET_STEP1 = new Pose2D(DistanceUnit.MM,0,75,AngleUnit.DEGREES,0);
    static final Pose2D TARGET_HIGH_BASKET_STEP2 = new Pose2D(DistanceUnit.MM,-1400, 100, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_HIGH_BASKET_STEP3 = new Pose2D(DistanceUnit.MM,-1400, 100, AngleUnit.DEGREES, 135);
    static final Pose2D TARGET_BAR_STEP1 = new Pose2D(DistanceUnit.MM,-1400, 100, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_BAR_STEP2 = new Pose2D(DistanceUnit.MM,-1200, 100, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_BAR_STEP3 = new Pose2D(DistanceUnit.MM,-1200, -1500, AngleUnit.DEGREES, 0);
//    static final Pose2D TARGET_SPEC1_STEP3 = new Pose2D(DistanceUnit.MM,0, 0, AngleUnit.DEGREES, 0);
//    static final Pose2D TARGET_SPEC1_STEP4 = new Pose2D(DistanceUnit.MM,0, 0, AngleUnit.DEGREES, 0);
//    static final Pose2D TARGET_SPEC2_STEP1 = new Pose2D(DistanceUnit.MM,0, 0, AngleUnit.DEGREES, 0);
//    static final Pose2D TARGET_SPEC2_STEP2 = new Pose2D(DistanceUnit.MM,0, 0, AngleUnit.DEGREES, 0);
//    static final Pose2D TARGET_SPEC2_STEP3 = new Pose2D(DistanceUnit.MM,0, 0, AngleUnit.DEGREES, 0);
//    static final Pose2D TARGET_SPEC3_STEP1 = new Pose2D(DistanceUnit.MM,0, 0, AngleUnit.DEGREES, 0);
//    static final Pose2D TARGET_SPEC3_STEP2 = new Pose2D(DistanceUnit.MM,0, 0, AngleUnit.DEGREES, 0);
//    static final Pose2D TARGET_SPEC3_STEP3 = new Pose2D(DistanceUnit.MM,0, 0, AngleUnit.DEGREES, 0);

    private double shoulderCommand = 0;
    private boolean clawClosed = true;

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
                    stateMachine = StateMachine.WAITING_FOR_START;
                    runtime.reset();
                    break;
                case DRIVE_TO_HIGH_BASKET_STEP1:
                    shoulderCommand = 0;
                    clawClosed = true;
                    if (runtime.seconds() >= 1.0){
                        stateMachine = StateMachine.DRIVE_TO_HIGH_BASKET_STEP1;
                    }
                    break;
                case RAISE_ARM_HIGH_BASKET:
                    /*
                    drive the robot to the first target, the nav.driveTo function will return true once
                    the robot has reached the target, and has been there for (holdTime) seconds.
                    Once driveTo returns true, it prints a telemetry line and moves the state machine forward.
                     */
                    robotController.setArmMode(ArmMode.HIGH_BASKET);
                    shoulderCommand = 0;
                    clawClosed = true;
                    if (nav.driveTo(robotController.getOdometryPosition(), TARGET_RAISE_ARM_HIGH_BASKET, 0.45, 0)){
                        //drive to the submersible
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DELIVER_SAMPLE;
                        runtime.reset();
                    }
                    break;
                case DELIVER_SAMPLE:
                    robotController.setArmMode(ArmMode.DRIVER_CONTROL);
                    shoulderCommand = -1;
                    clawClosed = true;
                    if (runtime.seconds() >= 0.5) {
                        stateMachine = StateMachine.OPEN_CLAW;
                        runtime.reset();
                    }
                    break;
                case OPEN_CLAW:
                    //make the claw let go of the sample
                    shoulderCommand = 0;
                    clawClosed = false;
                    robotController.updateDriveCommands(0, 0, 0, false);
                    if (runtime.seconds() >= 0.5) {
                        stateMachine = StateMachine.DRIVE_TO_BAR_STEP1;
                        runtime.reset();
                    }
                    break;
                case DRIVE_TO_BAR_STEP1:
                    //raise the arm
                    shoulderCommand = 0;
                    clawClosed = false;
                    if (nav.driveTo(robotController.getOdometryPosition(), TARGET_BAR_STEP1, 0.7, 0)){
                        telemetry.addLine("at position #2!");
                        stateMachine = StateMachine.DRIVE_TO_BAR_STEP2;
                    }
                    break;
                case DRIVE_TO_BAR_STEP2:
                    //raise the arm
                    shoulderCommand = 0;
                    clawClosed = false;
                    if (nav.driveTo(robotController.getOdometryPosition(), TARGET_BAR_STEP2, 0.7, 0)){
                        telemetry.addLine("at position #2!");
                        stateMachine = StateMachine.TOUCH_BAR_STEP1;
                    }
                    break;
                case TOUCH_BAR_STEP1:
                    //raise the arm
                    shoulderCommand = 0;
                    clawClosed = false;
                    if (nav.driveTo(robotController.getOdometryPosition(), TARGET_SPEC1_STEP3, 0.7, 0)){
                        telemetry.addLine("at position #2!");
                        stateMachine = StateMachine.DRIVE_TO_SPEC1_STEP4;
                    }
//                    break;
//                case DRIVE_TO_SPEC1_STEP4:
//                    //raise the arm
//                    shoulderCommand = 0;
//                    clawClosed = false;
//                    if (nav.driveTo(robotController.getOdometryPosition(), TARGET_SPEC1_STEP4, 0.7, 0)){
//                        telemetry.addLine("at position #2!");
//                        stateMachine = StateMachine.DRIVE_TO_SPEC2_STEP1;
//                    }
//                    break;
//                case DRIVE_TO_SPEC2_STEP1:
//                    //raise the arm
//                    shoulderCommand = 0;
//                    clawClosed = false;
//                    if (nav.driveTo(robotController.getOdometryPosition(), TARGET_SPEC2_STEP1, 0.7, 0)){
//                        telemetry.addLine("at position #2!");
//                        stateMachine = StateMachine.DRIVE_TO_SPEC2_STEP2;
//                    }
//                    break;
//                case DRIVE_TO_SPEC2_STEP2:
//                    //raise the arm
//                    shoulderCommand = 0;
//                    clawClosed = false;
//                    if (nav.driveTo(robotController.getOdometryPosition(), TARGET_SPEC2_STEP2, 0.7, 0)){
//                        telemetry.addLine("at position #2!");
//                        stateMachine = StateMachine.DRIVE_TO_SPEC2_STEP3;
//                    }
//                    break;
//                case DRIVE_TO_SPEC2_STEP3:
//                    //raise the arm
//                    shoulderCommand = 0;
//                    clawClosed = false;
//                    if (nav.driveTo(robotController.getOdometryPosition(), TARGET_SPEC2_STEP3, 0.7, 0)){
//                        telemetry.addLine("at position #2!");
//                        stateMachine = StateMachine.DRIVE_TO_SPEC3_STEP1;
//                    }
//                    break;
//                case DRIVE_TO_SPEC3_STEP1:
//                    //raise the arm
//                    shoulderCommand = 0;
//                    clawClosed = false;
//                    if (nav.driveTo(robotController.getOdometryPosition(), TARGET_SPEC3_STEP1, 0.7, 0)){
//                        telemetry.addLine("at position #2!");
//                        stateMachine = StateMachine.DRIVE_TO_SPEC3_STEP2;
//                    }
//                    break;
//                case DRIVE_TO_SPEC3_STEP2:
//                    //raise the arm
//                    shoulderCommand = 0;
//                    clawClosed = false;
//                    if (nav.driveTo(robotController.getOdometryPosition(), TARGET_SPEC3_STEP2, 0.7, 0)){
//                        telemetry.addLine("at position #2!");
//                        stateMachine = StateMachine.DRIVE_TO_SPEC3_STEP3;
//                    }
//                    break;
//                case DRIVE_TO_SPEC3_STEP3:
//                    //raise the arm
//                    shoulderCommand = 0;
//                    clawClosed = false;
//                    if (nav.driveTo(robotController.getOdometryPosition(), TARGET_SPEC3_STEP3, 0.7, 0)){
//                        telemetry.addLine("at position #2!");
//                        stateMachine = StateMachine.AT_TARGET;
//                    }
            }

            robotController.update(shoulderCommand, 0, 0,
                    clawClosed, false,
                    false, false);
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