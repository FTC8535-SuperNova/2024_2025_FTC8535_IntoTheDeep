package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.RobotController;


@Autonomous(name= "Observation Park")

public class ObservationPark extends LinearOpMode {
    private final RobotController robotController = new RobotController();

    private final ElapsedTime runtime = new ElapsedTime();

    double lateral = 0;


    @Override
    public void runOpMode() {
        robotController.init(hardwareMap, telemetry);

        waitForStart();

        runtime.reset();

        while (opModeIsActive()) {


            if(runtime.seconds() < 5) {
                lateral = 1;
            } else {
                lateral = 0;
            }


            robotController.update(0, 0, 0,
                    0, lateral, 0, false, true, false);

        }
    }
}

