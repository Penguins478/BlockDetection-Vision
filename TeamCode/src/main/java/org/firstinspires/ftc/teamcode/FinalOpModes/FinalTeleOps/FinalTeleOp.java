package org.firstinspires.ftc.teamcode.FinalOpModes.FinalTeleOps;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name = "FinalTeleOp2", group = "TeleOp") // go organize it
//@Disabled
public class FinalTeleOp extends LinearOpMode {

    private DcMotor tilt_motor;
    private DcMotor slide_motor;

    private static final int MAX = 1225;
    private static final int MIN = 25;

    private Servo servo;
    private Servo servo_f;

    BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;

    private DcMotor tl_motor;
    private DcMotor tr_motor;
    private DcMotor bl_motor;
    private DcMotor br_motor;

    private double mult = 1;

    public void runOpMode() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        tl_motor = hardwareMap.dcMotor.get("tl_motor");
        tr_motor = hardwareMap.dcMotor.get("tr_motor");
        bl_motor = hardwareMap.dcMotor.get("bl_motor");
        br_motor = hardwareMap.dcMotor.get("br_motor");

        servo = hardwareMap.servo.get("stone_servo");

        tilt_motor = hardwareMap.dcMotor.get("tilt_motor");
        slide_motor = hardwareMap.dcMotor.get("slide_motor");

        servo_f = hardwareMap.servo.get("servo_f");

        tilt_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        tilt_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        slide_motor.setDirection(DcMotorSimple.Direction.REVERSE);

        tilt_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        boolean robotPerspective = true;
        boolean enableDpad = false;


        waitForStart();

        while(opModeIsActive()) {
//            while(gamepad1.a) {                       //test for straight drive
//                tl_motor.setPower(1);
//                tr_motor.setPower(0.8 + 0.2);
//                bl_motor.setPower(1);
//                br_motor.setPower(0.8 + 0.2);
//            }

            if (gamepad1.y) robotPerspective = !robotPerspective;

            if(gamepad1.left_bumper){
                mult = 0.5;
            }
            if(gamepad1.right_bumper){
                mult = 1;
            }


            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double xPos = gamepad1.right_stick_x;
            double yPos = gamepad1.right_stick_y;
            double rot = -gamepad1.left_stick_x;
            double gyroAngle = angles.firstAngle;
            double adjustedAngle = 0;
            double scalar = Math.hypot(yPos, xPos);
            double largestPower = 1;

            if (robotPerspective) {

                largestPower = findLargest(
                        scalar * (yPos - xPos + rot),
                        scalar * (yPos + xPos - rot),
                        scalar * (yPos + xPos + rot),
                        scalar * (yPos - xPos - rot)
                );

                if(largestPower > 1 || largestPower < -1) {
                    scalar = Math.hypot(yPos, xPos) / Math.abs(largestPower);
                }else {
                    scalar = 1;
                }


                tl_motor.setPower(mult * scalar * (yPos - xPos + rot));
                tr_motor.setPower(mult * scalar * (yPos + xPos - rot));
                bl_motor.setPower(mult * scalar * (yPos + xPos + rot));
                br_motor.setPower(mult * scalar * (yPos - xPos - rot));



            }
            else {
                if((xPos != 0) || (yPos != 0) || (gamepad1.left_stick_x != 0) || (gamepad1.left_stick_y != 0)) {

                    enableDpad = false;

                    adjustedAngle = gyroAngle + Math.toDegrees(Math.atan2(yPos, xPos));
                    while (adjustedAngle >= 360) {
                        adjustedAngle -= 360;
                    }
                    while (adjustedAngle < 0) {
                        adjustedAngle += 360;
                    }
                    adjustedAngle *= (2 * Math.PI / 360);

                    largestPower = findLargest(
                            scalar * (Math.sin(adjustedAngle) - Math.cos(adjustedAngle) + rot),
                            scalar * (Math.sin(adjustedAngle) + Math.cos(adjustedAngle) - rot),
                            scalar * (Math.sin(adjustedAngle) + Math.cos(adjustedAngle) + rot),
                            scalar * (Math.sin(adjustedAngle) - Math.cos(adjustedAngle) - rot)
                    );

                    if (largestPower > 1 || largestPower < -1) {
                        scalar = Math.hypot(yPos, xPos) / Math.abs(largestPower);
                    } else {
                        scalar = 1;
                    }

                    tl_motor.setPower(mult * scalar * (Math.sin(adjustedAngle) - Math.cos(adjustedAngle) + rot));
                    tr_motor.setPower(mult * scalar * (Math.sin(adjustedAngle) + Math.cos(adjustedAngle) - rot));
                    bl_motor.setPower(mult * scalar * (Math.sin(adjustedAngle) + Math.cos(adjustedAngle) + rot));
                    br_motor.setPower(mult * scalar * (Math.sin(adjustedAngle) - Math.cos(adjustedAngle) - rot));

                }else {
                    tl_motor.setPower(0);
                    tr_motor.setPower(0);
                    bl_motor.setPower(0);
                    br_motor.setPower(0);
                }
            }

            if(gamepad2.dpad_up){
                tilt_motor.setPower(0.75);
            }else if(gamepad2.dpad_down){
                tilt_motor.setPower(-0.75);
            }else{
                tilt_motor.setPower(0);
            }

            if(slide_motor.getCurrentPosition() < MAX && gamepad2.y){
                slide_motor.setPower(0.75);
            }else if(slide_motor.getCurrentPosition() > MIN && gamepad2.a){
                slide_motor.setPower(-0.75);
            }else{
                slide_motor.setPower(0);
            }

            if(gamepad2.right_bumper){
                servo.setPosition(1);
            }else if(gamepad2.left_bumper){
                servo.setPosition(0);
            }

            if(gamepad2.right_trigger != 0){
                servo_f.setPosition(1);
            }else{
                servo_f.setPosition(0);
            }

            telemetry.addData("Servo Position: ", servo.getPosition());
            telemetry.addData("Encoder Position (tilt_motor) : ", tilt_motor.getCurrentPosition());
            telemetry.addData("Encoder Position (slide_motor) : ", slide_motor.getCurrentPosition());
            telemetry.update();

            if(gamepad1.x) enableDpad = !enableDpad;

            if(enableDpad) {

                double dpadAngle = angles.firstAngle;
                double error = 0;

                while(gamepad1.dpad_up || gamepad1.dpad_left ||
                        gamepad1.dpad_right || gamepad1.dpad_down) {

                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                    double currentAngle = angles.firstAngle;
                    error = ((currentAngle - dpadAngle) / 45) + 0.2;
                    telemetry.addData("Current Angle:", currentAngle);

                    idle();

                    if (gamepad1.dpad_up) {

                        tr_motor.setPower(Range.clip(1 - error, -1, 1));
                        tl_motor.setPower(Range.clip(1 + error, -1, 1));
                        bl_motor.setPower(Range.clip(1 + error, -1, 1));
                        br_motor.setPower(Range.clip(1 - error, -1, 1));

                    }

                    if (gamepad1.dpad_down) {

                        tr_motor.setPower(Range.clip(-1 - error, -1, 1));
                        tl_motor.setPower(Range.clip(-1 + error, -1, 1));
                        bl_motor.setPower(Range.clip(-1 + error, -1, 1));
                        br_motor.setPower(Range.clip(-1 - error, -1, 1));

                    }
                    if (gamepad1.dpad_right) {

                        tr_motor.setPower(Range.clip(1 + error, -1, 1));
                        tl_motor.setPower(Range.clip(-1 - error, -1, 1));
                        bl_motor.setPower(Range.clip(1 - error, -1, 1));
                        br_motor.setPower(Range.clip(-1 + error, -1, 1));
                    }
                    if (gamepad1.dpad_left) {
                        tr_motor.setPower(Range.clip(-1 + error, -1, 1));
                        tl_motor.setPower(Range.clip(1 - error, -1, 1));
                        bl_motor.setPower(Range.clip(-1 - error, -1, 1));
                        br_motor.setPower(Range.clip(1 + error, -1, 1));
                    }
                    telemetry.addData("Speed tl", tl_motor.getPower());
                    telemetry.addData("Speed tr", tr_motor.getPower());
                    telemetry.addData("Speed bl", bl_motor.getPower());
                    telemetry.addData("Speed br", br_motor.getPower());
                    telemetry.addData("Gyro angle:", gyroAngle);
                    telemetry.addData("Adjusted Angle:",adjustedAngle);
                    telemetry.addData("Dpad (gamepad1.x)", enableDpad);
                    telemetry.addData("Robot Perspective (gamepad1.y)", robotPerspective);
                    telemetry.addData("Error: ", error);
                    telemetry.addData("Dpad Angle: ", dpadAngle);
                    telemetry.update();
                }
                tr_motor.setPower(0);
                tl_motor.setPower(0);
                bl_motor.setPower(0);
                br_motor.setPower(0);

            }

            telemetry.addData("Speed Multiplier: ", mult);
            telemetry.addData("Speed tl", tl_motor.getPower());
            telemetry.addData("Speed tr", tr_motor.getPower());
            telemetry.addData("Speed bl", bl_motor.getPower());
            telemetry.addData("Speed br", br_motor.getPower());
            telemetry.addData("Gyro angle:", gyroAngle);
            telemetry.addData("Adjusted Angle:",adjustedAngle);
            telemetry.addData("Dpad (gamepad1.x)", enableDpad);
            telemetry.addData("Robot Perspective (gamepad1.y)", robotPerspective);
            telemetry.addData("Encoder tl", tl_motor.getCurrentPosition());
            telemetry.addData("Encoder tr", tr_motor.getCurrentPosition());
            telemetry.addData("Encoder bl", bl_motor.getCurrentPosition());
            telemetry.addData("Encoder br", br_motor.getCurrentPosition());
            telemetry.update();
        }

    }
    public double findLargest(double p1, double p2, double p3, double p4) {
        double largest = p1;
        if(p2 > largest) {
            largest = p2;
        }
        if(p3 > largest) {
            largest = p3;
        }
        if(p4 > largest) {
            largest = p4;
        }
        return largest;
    }

}