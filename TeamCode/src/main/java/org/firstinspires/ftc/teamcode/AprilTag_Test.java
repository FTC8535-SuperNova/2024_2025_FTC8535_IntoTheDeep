package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp (name = "AprilTag Detection", group = "Test")
@Disabled
public class AprilTag_Test extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //initialize our telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        int myAprilTagIdCode = 0;                           // ID code of current detection

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "back_webcam"))
                .setCameraResolution(new Size(640, 480))
                .build();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {

            if (tagProcessor.getDetections().size()>0){
                AprilTagDetection tag = tagProcessor.getDetections().get(0);

                if (tag.metadata != null) {  // This check for non-null Metadata is not needed for reading only ID code.
                    myAprilTagIdCode = tag.id;

                    // Now take action based on this tag's ID code, or store info for later action.

                }

                //telemetry

                telemetry.addData("x", tag.ftcPose.x);
                telemetry.addData("y", tag.ftcPose.y);
                telemetry.addData("z", tag.ftcPose.z);
                telemetry.addData("roll", tag.ftcPose.roll);
                telemetry.addData("pitch", tag.ftcPose.pitch);
                telemetry.addData("yaw", tag.ftcPose.yaw);
                telemetry.addData("range", tag.ftcPose.range);
                telemetry.addData("bearing", tag.ftcPose.bearing);
                telemetry.addData("elevation", tag.ftcPose.elevation);
                telemetry.addData("id", myAprilTagIdCode);
                telemetry.update();

            }
        }

    }
}   // end class
