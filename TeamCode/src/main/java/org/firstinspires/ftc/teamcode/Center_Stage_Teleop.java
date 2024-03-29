package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name = "Center_Stage_Teleop", group = "14174")
public class Center_Stage_Teleop extends LinearOpMode {
    public DcMotor front_left;
    public DcMotor front_right;
    public DcMotor back_left;
    public DcMotor back_right;
    public DcMotor lift;
    public DcMotor liftrot;
    public DcMotor slidesrot;
    public DcMotor slides;
    public Servo drone;
    public Servo collection;
    public Servo collectionDrop;
    double botHeading = 0;
    double offset = 0;
    int slidesdown = 0;
    int slidesmed = 260;
    int slidesmedhigh = 350;
    int slideshigh = 430;
    int count = 0;


    double headingResetValue;
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        lift = hardwareMap.get(DcMotor.class, "lift");
        liftrot = hardwareMap.get(DcMotor.class, "liftrot");
        slides = hardwareMap.get(DcMotor.class, "slides");
        slidesrot = hardwareMap.get(DcMotor.class, "slidesrot");
        drone = hardwareMap.get(Servo.class, "drone");
        collection = hardwareMap.get(Servo.class, "collection");
        collectionDrop = hardwareMap.get(Servo.class, "collectionDrop");
        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        liftrot.setDirection(DcMotorSimple.Direction.FORWARD);
        slides.setDirection(DcMotorSimple.Direction.FORWARD);
        slidesrot.setDirection(DcMotorSimple.Direction.FORWARD);


        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftrot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slidesrot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slidesrot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesrot.setTargetPosition(slidesdown);
        slidesrot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        collection.setPosition(0);
        BNO055IMU imu;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        // Retrieve the IMU from the hardware map
        BNO055IMU.Parameters parameters2 = new BNO055IMU.Parameters();
        parameters2.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters2.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters2.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters2.loggingEnabled = true;
        parameters2.loggingTag = "IMU";
        parameters2.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Adjust the orientation parameters to match your robot
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters2);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            if(gamepad1.right_bumper) {
                offset = -botHeading;
            }
            botHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle + offset;
            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            double slidespower = -gamepad2.left_stick_y;
            slides.setPower(slidespower);

            if(gamepad2.a) {
                slidesrot.setTargetPosition(slidesdown);
                slidesrot.setPower(0);
            } else if (gamepad2.x) {
                slidesrot.setTargetPosition(slidesmed);
                slidesrot.setPower(0.7);
            } else if(gamepad2.y) {
                slidesrot.setTargetPosition(slideshigh);
                slidesrot.setPower(0.7);
            } else if(gamepad2.b) {
                slidesrot.setTargetPosition(slidesmedhigh);
                slidesrot.setPower(0.7);
            }
            if (gamepad2.right_trigger > 0.1 && count == 0) {
                slidesrot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                count = 1;
            } if(gamepad2.right_trigger == 0 && count == 1) {
                slidesrot.setTargetPosition(0);
                slidesrot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                count = 0;
            }

        if(gamepad1.dpad_up) {
            lift.setPower(0.95);
        } else if(gamepad1.dpad_down) {
            lift.setPower(-0.95);
        } else {
            lift.setPower(0);
        }
        if(gamepad1.x) {
            liftrot.setPower(-1);
        } else {
            liftrot.setPower(0);
        }
        if(gamepad2.right_bumper) {
            collection.setPosition(0.26);
        }
        if(gamepad2.left_bumper) {
            collection.setPosition(0.65);
        }
        if(gamepad2.dpad_down) {
            collection.setPosition(0.05);
        }
        if(gamepad2.dpad_up) {
            collection.setPosition(0.95);
        }
        if(gamepad1.y) {
            collectionDrop.setPosition(0.2);
        } else {
            collectionDrop.setPosition(0.83);
        }

        if(gamepad1.a) {
            drone.setPosition(0.275);
        }
        if (gamepad1.b) {
            drone.setPosition(0.625);
        }
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]


            if(gamepad1.left_trigger > 0.1) {
                front_left.setPower(0.5 * frontLeftPower);
                back_left.setPower(0.5 * backLeftPower);
                front_right.setPower(0.5 * frontRightPower);
                back_right.setPower(0.5 * backRightPower);
            } else {
                front_left.setPower(frontLeftPower);
                back_left.setPower(backLeftPower);
                front_right.setPower(frontRightPower);
                back_right.setPower(backRightPower);
            }






            telemetry.addData("Status", "Running");
            telemetry.addData("heading", (Math.toDegrees(botHeading)));
            telemetry.addData("slidesrot", slidesrot.getCurrentPosition());
            telemetry.addData("collectionpos", collection.getPosition());
            telemetry.update();
        }
    }
}
