package org.firstinspires.ftc.teamcode.FinalOpModes.FinalTeleOps;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Final TeleOp", group = "TeleOp")
public class OfficialTeleOp extends OpMode {
    private DcMotor tl_motor;
    private DcMotor tr_motor;
    private DcMotor bl_motor;
    private DcMotor br_motor;
    private DcMotor tilt_motor;
    private DcMotor slide_motor;
    private Servo servo;
    private BNO055IMU imu;
    private BNO055IMU.Parameters parameters;
    private Orientation angles;
    private double tl_power;
    private double tr_power;
    private double bl_power;
    private double br_power;
    private double tl_prev_power;
    private double tr_prev_power;
    private double bl_prev_power;
    private double br_prev_power;
    private double x;
    private double y;
    private double r;
    private double angle;
    private boolean accelerate;
    private boolean change_acceleration_mode;

    @Override
    public void init() {
        tl_motor = hardwareMap.dcMotor.get("tl_motor");
        tr_motor = hardwareMap.dcMotor.get("tr_motor");
        bl_motor = hardwareMap.dcMotor.get("bl_motor");
        br_motor = hardwareMap.dcMotor.get("br_motor");
        tilt_motor = hardwareMap.dcMotor.get("tilt_motor");
        slide_motor = hardwareMap.dcMotor.get("slide_motor");
        servo = hardwareMap.servo.get("stone_servo");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        tr_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        br_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        tilt_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        slide_motor.setDirection(DcMotorSimple.Direction.REVERSE);

        tl_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tr_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tilt_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        tl_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tr_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tilt_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        tl_prev_power = 0;
        tr_prev_power = 0;
        bl_prev_power = 0;
        br_prev_power = 0;

        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        accelerate = true;
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void loop() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        angle = angles.firstAngle;
        change_acceleration_mode = gamepad1.left_bumper;

        if (change_acceleration_mode) {
            accelerate = !accelerate;
        }

        x = -gamepad1.left_stick_x;
        y = gamepad1.left_stick_y;
        r = -gamepad1.right_stick_x;

        double scalar = Math.hypot(x, y);
        double theta;

        if (angle != 0.0) {
            theta = Math.atan2(y, x);
            theta -= angle;
            x = scalar * Math.cos(theta);
            y = scalar * Math.sin(theta);
        }

        tl_power = y + x + r;
        tr_power = y - x - r;
        bl_power = y - x + r;
        br_power = y + x - r;

        tl_power = Range.clip(tl_power, -1, 1);
        tr_power = Range.clip(tr_power, -1, 1);
        bl_power = Range.clip(bl_power, -1, 1);
        br_power = Range.clip(br_power, -1, 1);

        if (accelerate) {
            tl_power = slow_accelerate(tl_power, tl_prev_power);
            tr_power = slow_accelerate(tr_power, tr_prev_power);
            bl_power = slow_accelerate(bl_power, bl_prev_power);
            br_power = slow_accelerate(br_power, br_prev_power);
        }

        tl_motor.setPower(tl_power);
        tr_motor.setPower(tr_power);
        bl_motor.setPower(bl_power);
        br_motor.setPower(br_power);

        if(gamepad2.y){
            tilt_motor.setPower(0.75);
        }else if(gamepad2.x){
            tilt_motor.setPower(-0.75);
        }else{
            tilt_motor.setPower(0);
        }

        if(gamepad2.dpad_up && slide_motor.getCurrentPosition() <= 1225){
            slide_motor.setPower(0.75);
        }else if(gamepad2.dpad_down && slide_motor.getCurrentPosition() >= 25){
            slide_motor.setPower(-0.75);
        }else{
            slide_motor.setPower(0);
        }

        if(gamepad2.right_bumper){
            servo.setPosition(1);
        }else if(gamepad2.left_bumper){
            servo.setPosition(0);
        }

        tl_prev_power = tl_power;
        tr_prev_power = tr_power;
        bl_prev_power = bl_power;
        br_prev_power = br_power;

        telemetry.addData("tl motor speed", tl_power);
        telemetry.addData("tr motor speed", tr_power);
        telemetry.addData("bl motor speed", bl_power);
        telemetry.addData("br motor speed", br_power);
        telemetry.addData("Servo Position: ", servo.getPosition());
        telemetry.addData("Encoder Position (tilt_motor) : ", tilt_motor.getCurrentPosition());
        telemetry.addData("Encoder Position (slide_motor) : ", slide_motor.getCurrentPosition());

        telemetry.addData("angle", angle);

        telemetry.addData("acceleration", accelerate);

        telemetry.update();
    }

    private double slow_accelerate(double power, double prev_power) {
        if (power > prev_power + 0.01 || power < prev_power - 0.01) {
            power = prev_power + (power - prev_power) / 5;
        }
        return power;
    }

    @Override
    public void stop() {

    }
}
