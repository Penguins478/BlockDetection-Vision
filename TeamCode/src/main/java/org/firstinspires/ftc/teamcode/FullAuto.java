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

@Autonomous(name = "Full Autonomous", group = "Autonomous")
//@Disabled
public class FullAuto extends LinearOpMode {

    private SkystoneDetector2 skystoneDetector = new SkystoneDetector2();
    private StoneDetector detector = new StoneDetector();
    private OpenCvCamera phoneCam;

    String pattern = "N/A";

    private DcMotor tl_motor;
    private DcMotor tr_motor;
    private DcMotor bl_motor;
    private DcMotor br_motor;

    private static final double COUNTS_PER_MOTOR_REV = 1120;
    private static final double GEAR_RATIO = 2;
    private static final double WHEEL_DIAMETER_INCHES = 2.9527559055;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * Math.PI * GEAR_RATIO);

    ElapsedTime runtime = new ElapsedTime();

    public void runOpMode(){

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        phoneCam.setPipeline(skystoneDetector);

        detector.useDefaults();

        tl_motor = hardwareMap.dcMotor.get("tl_motor");
        tr_motor = hardwareMap.dcMotor.get("tr_motor");
        bl_motor = hardwareMap.dcMotor.get("bl_motor");
        br_motor = hardwareMap.dcMotor.get("br_motor");

        tl_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        tr_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        bl_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        br_motor.setDirection(DcMotorSimple.Direction.FORWARD);

        tl_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tr_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        tl_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tr_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        runtime.reset();

        waitForStart();

        while(opModeIsActive()){

            runtime.reset();

            while(runtime.milliseconds() <= 2000) {         // 2 seconds
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

            runtime.reset();

            sleep(100);

            encoderDrive(24 * COUNTS_PER_INCH, 24 * COUNTS_PER_INCH, 24 * COUNTS_PER_INCH, 24 * COUNTS_PER_INCH, 0.2, 2, 250);


            if(pattern == "left"){
                encoderDrive(-8 * COUNTS_PER_INCH, 8 * COUNTS_PER_INCH, 8 * COUNTS_PER_INCH, -8 * COUNTS_PER_INCH, 0.1, 2, 250);
                // use mech
                encoderDrive(56 * COUNTS_PER_INCH, -56 * COUNTS_PER_INCH, -56 * COUNTS_PER_INCH, 56 * COUNTS_PER_INCH, 0.75, 10, 250);
            }else if (pattern == "middle"){
                // just use mech
                encoderDrive(48 * COUNTS_PER_INCH, -48 * COUNTS_PER_INCH, -48 * COUNTS_PER_INCH, 48 * COUNTS_PER_INCH, 0.75, 10, 250);
            }else{  // right but need for N/A
                encoderDrive(8 * COUNTS_PER_INCH, -8 * COUNTS_PER_INCH, -8 * COUNTS_PER_INCH, 8 * COUNTS_PER_INCH, 0.1, 2, 250);
                // use mech
                encoderDrive(40 * COUNTS_PER_INCH, -40 * COUNTS_PER_INCH, -40 * COUNTS_PER_INCH, 40 * COUNTS_PER_INCH, 0.75, 10, 250);
            }

            encoderDrive(-24 * COUNTS_PER_INCH, 24 * COUNTS_PER_INCH, 24 * COUNTS_PER_INCH, -24 * COUNTS_PER_INCH, 0.75, 10, 250);

            encoderDrive(-24 * COUNTS_PER_INCH, -24 * COUNTS_PER_INCH, -24 * COUNTS_PER_INCH, -24 * COUNTS_PER_INCH, 0.75, 10, 250);

            encoderDrive(-48 * COUNTS_PER_INCH, 48 * COUNTS_PER_INCH, 48 * COUNTS_PER_INCH, -48 * COUNTS_PER_INCH, 0.75, 10, 250);

            //encoderDrive(4 * COUNTS_PER_INCH, -4 * COUNTS_PER_INCH, -4 * COUNTS_PER_INCH, 4 * COUNTS_PER_INCH, 0.75, 2, 250);





            encoderDrive(24 * COUNTS_PER_INCH, 24 * COUNTS_PER_INCH, 24 * COUNTS_PER_INCH, 24 * COUNTS_PER_INCH, 0.2, 3, 250);


            if (pattern == "left" || pattern == "middle"){
                // just use mech
                encoderDrive(72 * COUNTS_PER_INCH, -72 * COUNTS_PER_INCH, -72 * COUNTS_PER_INCH, 72 * COUNTS_PER_INCH, 0.75, 10, 250);
            }else{  // right but need for N/A
                encoderDrive(8 * COUNTS_PER_INCH, -8 * COUNTS_PER_INCH, -8 * COUNTS_PER_INCH, 8 * COUNTS_PER_INCH, 0.1, 2, 250);
                // use mech
                encoderDrive(64 * COUNTS_PER_INCH, -64 * COUNTS_PER_INCH, -64 * COUNTS_PER_INCH, 64 * COUNTS_PER_INCH, 0.75, 10, 250);
            }

            encoderDrive(-12 * COUNTS_PER_INCH, 12 * COUNTS_PER_INCH, 12 * COUNTS_PER_INCH, -12 * COUNTS_PER_INCH, 0.2, 2, 250);

            // yellow stones now if there is time
        }
    }

    public void encoderDrive(double distance1, double distance2, double distance3, double distance4, double power, double error, long timeout){

        double start_tl = tl_motor.getCurrentPosition();
        double start_tr = tr_motor.getCurrentPosition();
        double start_bl = bl_motor.getCurrentPosition();
        double start_br = br_motor.getCurrentPosition();

        boolean there_tl = false;
        boolean there_tr = false;
        boolean there_bl = false;
        boolean there_br = false;

        while(!(there_tl && there_tr && there_bl && there_br)) {
            if(!there_tl) {
                if (tl_motor.getCurrentPosition() < (start_tl + distance1 - error) / 2) {
                    tl_motor.setPower(power);
                } else if (tl_motor.getCurrentPosition() >= (start_tl + distance1 - error) / 2 && tl_motor.getCurrentPosition() < start_tl + distance1 - error) {
                    tl_motor.setPower(power * 0.65);
                } else if (tl_motor.getCurrentPosition() > start_tl + distance1 + error) {
                    //tl_motor.setPower(power * -0.5);
                    tl_motor.setPower(-power);
                } else {
                    there_tl = true;
                    tl_motor.setPower(0);
                }
            }
            if(!there_tr) {
                if (tr_motor.getCurrentPosition() < (start_tr + distance2 - error) / 2) {
                    tr_motor.setPower(power);
                } else if (tr_motor.getCurrentPosition() >= (start_tr + distance2 - error) / 2 && tr_motor.getCurrentPosition() < start_tr + distance2 - error) {
                    tr_motor.setPower(power * 0.65);
                } else if (tr_motor.getCurrentPosition() > start_tr + distance2 + error) {
                    //tr_motor.setPower(power * -0.5);
                    tr_motor.setPower(-power);
                } else {
                    there_tr = true;
                    tr_motor.setPower(0);
                }
            }
            if(!there_bl) {
                if (bl_motor.getCurrentPosition() < (start_bl + distance3 - error) / 2) {
                    bl_motor.setPower(power);
                } else if (bl_motor.getCurrentPosition() >= (start_bl + distance3 - error) / 2 && bl_motor.getCurrentPosition() < start_bl + distance3 - error) {
                    bl_motor.setPower(power * 0.65);
                } else if (bl_motor.getCurrentPosition() > start_bl + distance3 + error) {
                    //bl_motor.setPower(power * -0.5);
                    bl_motor.setPower(-power);
                } else {
                    there_bl = true;
                    bl_motor.setPower(0);
                }
            }
            if(!there_br) {
                if (br_motor.getCurrentPosition() < (start_br + distance4 - error) / 2) {
                    br_motor.setPower(power);
                } else if (br_motor.getCurrentPosition() >= (start_br + distance4 - error) / 2 && br_motor.getCurrentPosition() < start_br + distance4 - error) {
                    br_motor.setPower(power * 0.65);
                } else if (br_motor.getCurrentPosition() > start_br + distance4 + error) {
                    //br_motor.setPower(power * -0.5);
                    br_motor.setPower(-power);
                } else {
                    there_br = true;
                    br_motor.setPower(0);
                }
            }

            telemetry.addData("tl power", tl_motor.getPower());
            telemetry.addData("tr power", tr_motor.getPower());
            telemetry.addData("bl power", bl_motor.getPower());
            telemetry.addData("br power", br_motor.getPower());
            telemetry.addData("Encoder tl", tl_motor.getCurrentPosition());
            telemetry.addData("Encoder tr", tr_motor.getCurrentPosition());
            telemetry.addData("Encoder bl", bl_motor.getCurrentPosition());
            telemetry.addData("Encoder br", br_motor.getCurrentPosition());
            telemetry.addData("Encoder tl goal", start_tl + distance1);
            telemetry.addData("Encoder tr goal", start_tr + distance2);
            telemetry.addData("Encoder bl goal", start_bl + distance3);
            telemetry.addData("Encoder br goal", start_br + distance4);
            telemetry.update();
        }

        sleep(timeout);

    }
}
