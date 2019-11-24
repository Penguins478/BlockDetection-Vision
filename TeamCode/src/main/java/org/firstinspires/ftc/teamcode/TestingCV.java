package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "TestingCV", group = "Autonomous")
//@Disabled
public class TestingCV extends LinearOpMode {

    private SkystoneDetector2 skystoneDetector = new SkystoneDetector2();
    private StoneDetector detector = new StoneDetector();
    private OpenCvCamera phoneCam;

    String pattern = "N/A";


    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        phoneCam.setPipeline(skystoneDetector);

        detector.useDefaults();


        waitForStart();

        while (opModeIsActive()) {


            if (skystoneDetector.isFound()) {
                if (skystoneDetector.getScreenPosition().x < 80) {
                    pattern = "left";
                } else if (80 <= skystoneDetector.getScreenPosition().x && skystoneDetector.getScreenPosition().x < 160) {
                    pattern = "middle";
                } else {
                    pattern = "right";
                }
            }

            telemetry.addData("X-Value: ", skystoneDetector.getScreenPosition().x);
            telemetry.addData("Pattern: ", pattern);
            telemetry.update();


        }
    }
}