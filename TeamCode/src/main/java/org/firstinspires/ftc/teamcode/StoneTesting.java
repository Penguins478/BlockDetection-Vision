package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


@Autonomous(name = "StoneCameraTesting", group = "Autonomous")
//@Disabled
public class StoneTesting extends LinearOpMode {

    private DcMotor tl_motor;
    private DcMotor tr_motor;
    private DcMotor bl_motor;
    private DcMotor br_motor;

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

        tl_motor = hardwareMap.dcMotor.get("tl_motor");
        tr_motor = hardwareMap.dcMotor.get("tr_motor");
        bl_motor = hardwareMap.dcMotor.get("bl_motor");
        br_motor = hardwareMap.dcMotor.get("br_motor");

        tr_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        br_motor.setDirection(DcMotorSimple.Direction.REVERSE);

        tr_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tl_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        tr_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tl_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        waitForStart();

        while(opModeIsActive()){

            runtime.reset();

            while(!detector.isFound() || runtime.milliseconds() <= 10000){ // 10 seconds
                tl_motor.setPower(-0.5);
                tr_motor.setPower(0.5);
                bl_motor.setPower(0.5);
                br_motor.setPower(-0.5);
                telemetry.addData("Time Elapsed (Milliseconds): ", runtime.milliseconds());
                telemetry.addData("Stone? ", detector.isFound());
                telemetry.update();
            }




            telemetry.addData("Stone? ", detector.isFound());
            telemetry.update();

        }
    }
}
