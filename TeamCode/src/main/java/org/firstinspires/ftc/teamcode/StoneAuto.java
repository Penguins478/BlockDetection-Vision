package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


@Autonomous(name = "StoneCameraTesting", group = "Autonomous")
//@Disabled
public class StoneAuto extends LinearOpMode {

    private StoneDetector detector = new StoneDetector();
    private OpenCvCamera phoneCam;

    @Override
    public void runOpMode(){

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);


        phoneCam.openCameraDevice();
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        phoneCam.setPipeline(detector);

        detector.useDefaults();

        waitForStart();

        while(opModeIsActive()){

            telemetry.addData("Stone? ", detector.isFound());
            telemetry.update();

        }
    }
}
