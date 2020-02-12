package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import android.graphics.Color;
import android.app.Activity;
import android.view.View;
import java.lang.annotation.Target;
import java.util.Timer;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Locale;
import android.app.Activity;

public class AutonMethods {
    
    //Constructor
    public AutonMethods() {
        
    }
    
    //Declare and initial variables
    private static DcMotor motorFL, motorBL, motorBR, motorFR, intakeFL, intakeFR, intakeM, intakeB;
    private static Servo servoLS, servoRS, clamp;
    private static ColorSensor sensorColor1, sensorColor2;
    HardwareMap map;
    Telemetry tele;
    
    private double speed;
    private boolean clampDown = false;
    public int counter = 0, ColorEncoder = 0, blockCounter = 0;
    
    float hsvValues1[] = {0F, 0F, 0F};
    float hsvValues2[] = {0F, 0F, 0F};
    final double SCALE_FACTOR = 255;
    int block = 1;
    
    public static BNO055IMU imu;
    BNO055IMU.Parameters parameters;
    Orientation angles;
    
    //Function to move the robot in any direction
    public void motors (String direction, int distance) {
        if (direction.equals("front")) {
            if ((Math.abs(motorFL.getCurrentPosition()) < distance)) {
                motorFL.setTargetPosition(-distance - 50);
                motorBL.setTargetPosition(-distance - 50);
                motorFR.setTargetPosition(distance + 50);
                motorBR.setTargetPosition(distance + 50);
                speed(speed);
                
                speedIncrement(distance - (Math.abs(motorFL.getCurrentPosition())));
                
                /*Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
                
                telemetry.addData("Hue", hsvValues[0]);
                telemetry.update();*/
            }
        }
        
        else if (direction.equals("back")) {
            if ((Math.abs(motorFL.getCurrentPosition()) < distance)) {
                motorFL.setTargetPosition(distance + 50);
                motorBL.setTargetPosition(distance + 50);
                motorFR.setTargetPosition(-distance - 50);
                motorBR.setTargetPosition(-distance - 50);
                speed(speed);
                
                speedIncrement(distance - (Math.abs(motorFL.getCurrentPosition())));
                
                /*Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
                
                telemetry.addData("Hue", hsvValues[0]);
                telemetry.update();*/
            }
        }
        
        else if (direction.equals("right")) {
            if ((Math.abs(motorFL.getCurrentPosition()) < distance)) {
                motorFL.setTargetPosition(distance + 50);
                motorBL.setTargetPosition(-distance - 50);
                motorFR.setTargetPosition(distance + 50);
                motorBR.setTargetPosition(-distance - 50);
                speed(speed * 1.2);
                
                speedIncrement(distance - (Math.abs(motorFL.getCurrentPosition())));
                
                /*Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
                
                telemetry.addData("Hue", hsvValues[0]);
                telemetry.update();*/
            }
        }
        
        else if (direction.equals("left")) {
            if ((Math.abs(motorFL.getCurrentPosition()) < distance)) {
                motorFL.setTargetPosition(-distance - 50);
                motorBL.setTargetPosition(distance + 50);
                motorFR.setTargetPosition(-distance - 50);
                motorBR.setTargetPosition(distance + 50);
                speed(speed * 1.2);
                
                speedIncrement(distance - (Math.abs(motorFL.getCurrentPosition())));

                /*Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
                
                telemetry.addData("Hue", hsvValues[0]);
                telemetry.update();*/
            }
        }
        
        else if (direction.equals("turn_left")) {
            if ((Math.abs(motorFL.getCurrentPosition()) < distance)) {
                motorFL.setTargetPosition(-distance - 50);
                motorBL.setTargetPosition(-distance - 50);
                motorFR.setTargetPosition(-distance - 50);
                motorBR.setTargetPosition(-distance - 50);
                speed(0.6);
                
                //speedIncrement(distance - (Math.abs(motorFL.getCurrentPosition())));

                /*Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
                
                telemetry.addData("Hue", hsvValues[0]);
                telemetry.update();*/
            }
        }
        
        else if (direction.equals("turn_right")) {
            if ((Math.abs(motorFL.getCurrentPosition()) < distance)) {
                motorFL.setTargetPosition(distance + 50);
                motorBL.setTargetPosition(distance + 50);
                motorFR.setTargetPosition(distance + 50);
                motorBR.setTargetPosition(distance + 50);
                speed(0.6);
                
                //speedIncrement(distance - (Math.abs(motorFL.getCurrentPosition())));

                /*Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
                
                telemetry.addData("Hue", hsvValues[0]);
                telemetry.update();*/
            }
        }
        
        else if (direction.equals("stop")) {
            motorFL.setTargetPosition(0);
            motorBL.setTargetPosition(0);
            motorFR.setTargetPosition(0);
            motorBR.setTargetPosition(0);
            speed(0);
        }
        
        changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        if (!(Math.abs(motorFL.getCurrentPosition()) < distance)) {
            counter++;
            changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    
    //Function to slow down the robot as it approaches its destination
    public void speedIncrement(int distance) {
        if (distance < 100) speed = .2;
        else if (distance < 200) speed = .25;
        else if (distance < 300) speed = .3;
        else if (distance < 400) speed = .35;
        else if (distance < 500) speed = .4;
        else speed = .5;
    }
    
    //Initialization
    public void init(HardwareMap map, Telemetry tele, boolean auton) {
        motorFL = map.get(DcMotor.class, "motorFL");
        motorBL = map.get(DcMotor.class, "motorBL");
        motorBR = map.get(DcMotor.class, "motorBR");
        motorFR = map.get(DcMotor.class, "motorFR");
        intakeFL = map.get(DcMotor.class, "intakeFL");
        intakeFR = map.get(DcMotor.class, "intakeFR");
        intakeM = map.get(DcMotor.class, "intakeM");
        intakeB = map.get(DcMotor.class, "intakeB");
        servoLS = map.get(Servo.class, "servoLS");
        servoRS = map.get(Servo.class, "servoRS");
        clamp = map.get(Servo.class, "clamp");
        sensorColor1 = map.get(ColorSensor.class, "sensorColor1");
        sensorColor2 = map.get(ColorSensor.class, "sensorColor2");
        
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //servoRS.setDirection(FORWARD);
        //servoLS.setDirection(FORWARD);
        
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeFL.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeFR.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeM.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeB.setDirection(DcMotorSimple.Direction.FORWARD);
        
        motorFL.setTargetPosition(0);
        motorBL.setTargetPosition(0);
        motorFR.setTargetPosition(0);
        motorBR.setTargetPosition(0);
        
        int relativeLayoutId = map.appContext.getResources().getIdentifier("RelativeLayout", "id", map.appContext.getPackageName());
        
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        
        imu = map.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        
        tele.addData(">", "Gyro Calibrating. Do Not Move!");
        tele.update();
    }
    
    //Function to change the run mode of all the motors
    public static void changeRunMode(DcMotor.RunMode runMode) {
        motorFL.setMode(runMode);
        motorBL.setMode(runMode);
        motorFR.setMode(runMode);
        motorBR.setMode(runMode);
    }
    
    //Function to set the speed of all the motors
    public void speed(double in) {
        motorFL.setPower(in);
        motorBL.setPower(in);
        motorFR.setPower(in);
        motorBR.setPower(in);
    }
    
    //Function to Open or close the clamp
    public void servoClamp() {
        if (clampDown) {
            servoLS.setPosition(.6);
            servoRS.setPosition(.4);
        }
        
        else {
            servoLS.setPosition(0);
            servoRS.setPosition(1);
        }
        
        clampDown = !clampDown;
        counter++;
    }
    
    //Function to have the robot sleep
    public void sleep(long sleep) {
        try {
            Thread.sleep(sleep);
        }
        catch (InterruptedException e) {
            tele.addLine("Failed Sleep");
            tele.update();
        }
        
        counter++;
    }
    
    //Function to turn on the intake
    public void intake(String direction) {
        if (direction.equals("in")) {
            intakeFL.setPower(-.8);
            intakeFR.setPower(-.8);
            intakeM.setPower(-.5);
            intakeB.setPower(-.5);
        }
        
        else {
            intakeFL.setPower(.8);
            intakeFR.setPower(.8);
            intakeM.setPower(.5);
            intakeB.setPower(.5);
        }
        
        counter++;
    }
    
    //Function to turn off the intake
    public void intakeOff() {
        intakeFL.setPower(0);
        intakeFR.setPower(0);
        intakeM.setPower(0);
        intakeB.setPower(0);
        
        counter++;
    }
    
    //Function to turn with the imu
    public void runWithImu(int angle, String direction) {
        if (angle < Math.abs(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)) {
            if (direction.equals("turn_right")) {
                motorFL.setTargetPosition(motorFL.getCurrentPosition() + 400);
                motorBL.setTargetPosition(motorBL.getCurrentPosition() + 400);
                motorFR.setTargetPosition(motorFR.getCurrentPosition() + 400);
                motorBR.setTargetPosition(motorBR.getCurrentPosition() + 400);
                speed(.3);
            }
            
            else if (direction.equals("turn_left")) {
                motorFL.setTargetPosition(motorFL.getCurrentPosition() - 400);
                motorBL.setTargetPosition(motorBL.getCurrentPosition() - 400);
                motorFR.setTargetPosition(motorFR.getCurrentPosition() - 400);
                motorBR.setTargetPosition(motorBR.getCurrentPosition() - 400);
                speed(.3);
            }
        }
        
        else {
            motorFL.setTargetPosition(motorFL.getCurrentPosition());
            motorBL.setTargetPosition(motorBL.getCurrentPosition());
            motorFR.setTargetPosition(motorFR.getCurrentPosition());
            motorBR.setTargetPosition(motorBR.getCurrentPosition());
            speed(0);
            
            counter++;
            changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    
    //Function to run until the robot detects a black block
    public void runTillColor(String direction) {
        if (direction.equals("right")) {
            Color.RGBToHSV((int) (sensorColor1.red() * 255),
                (int) (sensorColor1.green() * 255),
                (int) (sensorColor1.blue() * 255),
                hsvValues1);
            
            if (hsvValues1[0] < 100) {
                motors("right", 200 + Math.abs(motorFL.getCurrentPosition()));
                ColorEncoder++;
            }
            
            else {
                motors("stop", 50);
                changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                counter++;
            }
        }
        
        else if (direction.equals("left")) {
            Color.RGBToHSV((int) (sensorColor1.red() * 255),
                (int) (sensorColor1.green() * 255),
                (int) (sensorColor1.blue() * 255),
                hsvValues1);
            
            if (hsvValues1[0] < 100) {
                motors("left", 200 + Math.abs(motorFL.getCurrentPosition()));
                ColorEncoder++;
            }
            
            else {
                motors("stop", 50);
                changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }
    }
    
    //Function to move the robot in any direction (fast)
    public void motorsFast (String direction, int distance) {
        if (direction.equals("front")) {
            if ((Math.abs(motorFL.getCurrentPosition()) < distance)) {
                motorFL.setTargetPosition(-distance - 50);
                motorBL.setTargetPosition(-distance - 50);
                motorFR.setTargetPosition(distance + 50);
                motorBR.setTargetPosition(distance + 50);
                speed(speed);
                
                speedIncrementFast(distance - (Math.abs(motorFL.getCurrentPosition())));
                
                /*Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
                
                telemetry.addData("Hue", hsvValues[0]);
                telemetry.update();*/
            }
        }
        
        else if (direction.equals("back")) {
            if ((Math.abs(motorFL.getCurrentPosition()) < distance)) {
                motorFL.setTargetPosition(distance + 50);
                motorBL.setTargetPosition(distance + 50);
                motorFR.setTargetPosition(-distance - 50);
                motorBR.setTargetPosition(-distance - 50);
                speed(speed);
                
                speedIncrementFast(distance - (Math.abs(motorFL.getCurrentPosition())));
                
                /*Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
                
                telemetry.addData("Hue", hsvValues[0]);
                telemetry.update();*/
            }
        }
        
        else if (direction.equals("right")) {
            if ((Math.abs(motorFL.getCurrentPosition()) < distance)) {
                motorFL.setTargetPosition(distance + 50);
                motorBL.setTargetPosition(-distance - 50);
                motorFR.setTargetPosition(distance + 50);
                motorBR.setTargetPosition(-distance - 50);
                speed(speed * 1.2);
                
                speedIncrementFast(distance - (Math.abs(motorFL.getCurrentPosition())));
                
                /*Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
                
                telemetry.addData("Hue", hsvValues[0]);
                telemetry.update();*/
            }
        }
        
        else if (direction.equals("left")) {
            if ((Math.abs(motorFL.getCurrentPosition()) < distance)) {
                motorFL.setTargetPosition(-distance - 50);
                motorBL.setTargetPosition(distance + 50);
                motorFR.setTargetPosition(-distance - 50);
                motorBR.setTargetPosition(distance + 50);
                speed(speed * 1.2);
                
                speedIncrementFast(distance - (Math.abs(motorFL.getCurrentPosition())));

                /*Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
                
                telemetry.addData("Hue", hsvValues[0]);
                telemetry.update();*/
            }
        }
        
        else if (direction.equals("turn_left")) {
            if ((Math.abs(motorFL.getCurrentPosition()) < distance)) {
                motorFL.setTargetPosition(-distance - 50);
                motorBL.setTargetPosition(-distance - 50);
                motorFR.setTargetPosition(-distance - 50);
                motorBR.setTargetPosition(-distance - 50);
                speed(0.8);
                
                //speedIncrementFast(distance - (Math.abs(motorFL.getCurrentPosition())));

                /*Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
                
                telemetry.addData("Hue", hsvValues[0]);
                telemetry.update();*/
            }
        }
        
        else if (direction.equals("turn_right")) {
            if ((Math.abs(motorFL.getCurrentPosition()) < distance)) {
                motorFL.setTargetPosition(distance + 50);
                motorBL.setTargetPosition(distance + 50);
                motorFR.setTargetPosition(distance + 50);
                motorBR.setTargetPosition(distance + 50);
                speed(0.8);
                
                //speedIncrementFast(distance - (Math.abs(motorFL.getCurrentPosition())));

                /*Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
                
                telemetry.addData("Hue", hsvValues[0]);
                telemetry.update();*/
            }
        }
        
        else if (direction.equals("stop")) {
            motorFL.setTargetPosition(motorFL.getCurrentPosition());
            motorBL.setTargetPosition(motorBL.getCurrentPosition());
            motorFR.setTargetPosition(motorFR.getCurrentPosition());
            motorBR.setTargetPosition(motorBR.getCurrentPosition());
            speed(0);
        }
        
        changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        if (!(Math.abs(motorFL.getCurrentPosition()) < distance)) {
            counter++;
            changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    
    //Function to slow down the robot as it approaches its destination
    public void speedIncrementFast(int distance) {
        if (distance < 100) speed = .2;
        else if (distance < 200) speed = .25;
        else if (distance < 300) speed = .3;
        else if (distance < 400) speed = .35;
        else if (distance < 500) speed = .4;
        else if (distance < 600) speed = .5;
        else if (distance < 800) speed = .7;
        else if (distance < 1000) speed = .9;
        else speed = 1;
    }
    
    //Function to raise or lower the clamp for the block
    public void skyClamp(String direction) {
        if (direction.equals("up")) {
            clamp.setPosition(0);
        }
        
        else {
            clamp.setPosition(1);
        }
        
        counter++;
    }
    
    //Function to acess the motorFL encoder value
    public int FL() {
        return motorFL.getCurrentPosition();
    }
    
    //Function to acess the first color sensor's value
    public float color1() {
        Color.RGBToHSV((int) (sensorColor1.red() * SCALE_FACTOR),
            (int) (sensorColor1.green() * SCALE_FACTOR),
            (int) (sensorColor1.blue() * SCALE_FACTOR),
            hsvValues1);
        return hsvValues1[0];
    }
    
    //Function to acess the first color sensor's value
    public float color2() {
        Color.RGBToHSV((int) (sensorColor2.red() * SCALE_FACTOR),
            (int) (sensorColor2.green() * SCALE_FACTOR),
            (int) (sensorColor2.blue() * SCALE_FACTOR),
            hsvValues2);
        return hsvValues2[0];
    }
    
    //Function to move the robot in any direction (large buffer)
    public void motorsBuff (String direction, int distance) {
        if (direction.equals("front")) {
            if ((Math.abs(motorFL.getCurrentPosition()) < distance)) {
                motorFL.setTargetPosition(-distance - 100);
                motorBL.setTargetPosition(-distance - 100);
                motorFR.setTargetPosition(distance + 100);
                motorBR.setTargetPosition(distance + 100);
                speed(speed);
                
                speedIncrement(distance - (Math.abs(motorFL.getCurrentPosition())));
                
                /*Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
                
                telemetry.addData("Hue", hsvValues[0]);
                telemetry.update();*/
            }
        }
        
        else if (direction.equals("back")) {
            if ((Math.abs(motorFL.getCurrentPosition()) < distance)) {
                motorFL.setTargetPosition(distance + 100);
                motorBL.setTargetPosition(distance + 100);
                motorFR.setTargetPosition(-distance - 100);
                motorBR.setTargetPosition(-distance - 100);
                speed(speed);
                
                speedIncrement(distance - (Math.abs(motorFL.getCurrentPosition())));
                
                /*Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
                
                telemetry.addData("Hue", hsvValues[0]);
                telemetry.update();*/
            }
        }
        
        else if (direction.equals("right")) {
            if ((Math.abs(motorFL.getCurrentPosition()) < distance)) {
                motorFL.setTargetPosition(distance + 100);
                motorBL.setTargetPosition(-distance - 100);
                motorFR.setTargetPosition(distance + 100);
                motorBR.setTargetPosition(-distance - 100);
                speed(speed * 1.2);
                
                speedIncrement(distance - (Math.abs(motorFL.getCurrentPosition())));
                
                /*Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
                
                telemetry.addData("Hue", hsvValues[0]);
                telemetry.update();*/
            }
        }
        
        else if (direction.equals("left")) {
            if ((Math.abs(motorFL.getCurrentPosition()) < distance)) {
                motorFL.setTargetPosition(-distance - 100);
                motorBL.setTargetPosition(distance + 100);
                motorFR.setTargetPosition(-distance - 100);
                motorBR.setTargetPosition(distance + 100);
                speed(speed * 1.2);
                
                speedIncrement(distance - (Math.abs(motorFL.getCurrentPosition())));

                /*Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
                
                telemetry.addData("Hue", hsvValues[0]);
                telemetry.update();*/
            }
        }
        
        else if (direction.equals("turn_left")) {
            if ((Math.abs(motorFL.getCurrentPosition()) < distance)) {
                motorFL.setTargetPosition(-distance - 100);
                motorBL.setTargetPosition(-distance - 100);
                motorFR.setTargetPosition(-distance - 100);
                motorBR.setTargetPosition(-distance - 100);
                speed(1);
                
                //speedIncrement(distance - (Math.abs(motorFL.getCurrentPosition())));

                /*Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
                
                telemetry.addData("Hue", hsvValues[0]);
                telemetry.update();*/
            }
        }
        
        else if (direction.equals("turn_right")) {
            if ((Math.abs(motorFL.getCurrentPosition()) < distance)) {
                motorFL.setTargetPosition(distance + 100);
                motorBL.setTargetPosition(distance + 100);
                motorFR.setTargetPosition(distance + 100);
                motorBR.setTargetPosition(distance + 100);
                speed(1);
                
                //speedIncrement(distance - (Math.abs(motorFL.getCurrentPosition())));

                /*Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
                
                telemetry.addData("Hue", hsvValues[0]);
                telemetry.update();*/
            }
        }
        
        else if (direction.equals("stop")) {
            motorFL.setTargetPosition(0);
            motorBL.setTargetPosition(0);
            motorFR.setTargetPosition(0);
            motorBR.setTargetPosition(0);
            speed(0);
        }
        
        changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        if (!(Math.abs(motorFL.getCurrentPosition()) < distance)) {
            counter++;
            changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    
    //Function to find which block is black
    public void checkBlocks(String side) {
        Color.RGBToHSV((int) (sensorColor1.red() * SCALE_FACTOR),
            (int) (sensorColor1.green() * SCALE_FACTOR),
            (int) (sensorColor1.blue() * SCALE_FACTOR),
            hsvValues1);
        Color.RGBToHSV((int) (sensorColor2.red() * SCALE_FACTOR),
            (int) (sensorColor2.green() * SCALE_FACTOR),
            (int) (sensorColor2.blue() * SCALE_FACTOR),
            hsvValues2);
        
        if (side.equals("red")) {
            if (hsvValues1[0] > 90) block = 1;
            else if (hsvValues2[0] > 90) block = 2;
            else block = 3;
        }
        
        else {
            if (hsvValues1[0] > 90) block = 2;
            else if (hsvValues2[0] > 90) block = 1;
            else block = 3;
        }
        
        counter++;
    }
    
    //Function to acess the block read by the color sensors
    public int block() {
        return block;
    }
    
    //Function to retrive the block and the foundation
    public void getBlockAndFoundation(String side) {
        int step = counter;
        if (side.equals("red")) {
            switch (block) {
                case 1:
                    switch (blockCounter) {
                        case 0:
                            intake("in");
                            break;
                        case 1:
                            motorsBuff("turn_right", 200);
                            break;
                        case 2:
                            sleep(100);
                            break;
                        case 3:
                            motorsFast("front", 600);
                            break;
                        case 4:
                            sleep(250);
                            break;
                        case 5:
                            intakeOff();
                            break;
                        case 6:
                            motorsFast("turn_left", 200);
                            break;
                        case 7:
                            sleep(100);
                            break;
                        case 8:
                            motorsFast("back", 800);
                            break;
                        case 9:
                            sleep(100);
                            break;
                        case 10:
                            motorsFast("turn_left", 850);
                            break;
                        case 11:
                            sleep(100);
                            break;
                        case 12:
                            motorsFast("back", 2000);
                            break;
                        case 13:
                            sleep(100);
                            break;
                        case 14:
                            motorsFast("turn_left", 850);
                            break;
                        case 15:
                            sleep(100);
                            break;
                        case 16:
                            motorsFast("back", 900);
                            break;
                        case 17:
                            servoClamp();
                            break;
                        case 18:
                            sleep(100);
                            break;
                        case 19:
                            motorsFast("turn_right", 400);
                            break;
                        case 20:
                            sleep(100);
                            break;
                        case 21:
                            motorsFast("front", 2400);
                            break;
                        case 22:
                            sleep(100);
                            break;
                        case 23:
                            motors("turn_right", 2500);
                            break;
                        case 24:
                            servoClamp();
                            break;
                        case 25:
                            sleep(100);
                            break;
                        case 26:
                            intake("in");
                            break;
                        case 27:
                            motorsFast("back", 1700);
                            break;
                        case 28:
                            sleep(100);
                            break;
                        case 29:
                            intakeOff();
                            break;
                        case 30:
                            motorsFast("front", 2500);
                            break;
                        case 31:
                            motors("stop", 1000);
                            break;
                    }
                    break;
                case 2:
                    switch (blockCounter) {
                        case 0:
                            intake("in");
                            break;
                        case 1:
                            motors("turn_left", 200);
                            break;
                        case 2:
                            sleep(100);
                            break;
                        case 3:
                            motors("front", 600);
                            break;
                        case 4:
                            sleep(250);
                            break;
                        case 5:
                            intakeOff();
                            break;
                        case 6:
                            motorsFast("turn_right", 200);
                            break;
                        case 7:
                            sleep(100);
                            break;
                        case 8:
                            motorsFast("back", 800);
                            break;
                        case 9:
                            sleep(100);
                            break;
                        case 10:
                            motorsFast("turn_left", 850);
                            break;
                        case 11:
                            sleep(100);
                            break;
                        case 12:
                            motorsFast("back", 2000);
                            break;
                        case 13:
                            sleep(100);
                            break;
                        case 14:
                            motorsFast("turn_left", 850);
                            break;
                        case 15:
                            sleep(100);
                            break;
                        case 16:
                            motorsFast("back", 900);
                            break;
                        case 17:
                            servoClamp();
                            break;
                        case 18:
                            sleep(100);
                            break;
                        case 19:
                            motorsFast("turn_right", 400);
                            break;
                        case 20:
                            sleep(100);
                            break;
                        case 21:
                            motorsFast("front", 2400);
                            break;
                        case 22:
                            sleep(100);
                            break;
                        case 23:
                            motors("turn_right", 2500);
                            break;
                        case 24:
                            servoClamp();
                            break;
                        case 25:
                            sleep(100);
                            break;
                        case 26:
                            intake("in");
                            break;
                        case 27:
                            motorsFast("back", 1700);
                            break;
                        case 28:
                            sleep(100);
                            break;
                        case 29:
                            intakeOff();
                            break;
                        case 30:
                            motorsFast("front", 2500);
                            break;
                        case 31:
                            motors("stop", 1000);
                            break;
                    }
                    break;
                case 3:
                    switch (blockCounter) {
                        case 0:
                            motorsFast("front", 500);
                            break;
                        case 1:
                            sleep(100);
                            break;
                        case 2:
                            motorsFast("turn_left", 850);
                            break;
                        case 3:
                            intake("in");
                            break;
                        case 4:
                            sleep(100);
                            break;
                        case 5:
                            motorsFast("front", 500);
                            break;
                        case 6:
                            sleep(250);
                            break;
                        case 7:
                            motorsFast("turn_right", 850);
                            break;
                        case 8:
                            sleep(100);
                            break;
                        case 9:
                            motorsFast("back", 600);
                            break;
                        case 10:
                            sleep(100);
                            break;
                        case 11:
                            sleep(100);
                            break;
                        case 12:
                            motorsFast("turn_left", 850);
                            break;
                        case 13:
                            sleep(100);
                            break;
                        case 14:
                            motorsFast("back", 2000);
                            break;
                        case 15:
                            sleep(100);
                            break;
                        case 16:
                            motorsFast("turn_left", 850);
                            break;
                        case 17:
                            sleep(100);
                            break;
                        case 18:
                            motorsFast("back", 900);
                            break;
                        case 19:
                            servoClamp();
                            break;
                        case 20:
                            sleep(100);
                            break;
                        case 21:
                            motorsFast("turn_right", 400);
                            break;
                        case 22:
                            sleep(100);
                            break;
                        case 23:
                            motorsFast("front", 2400);
                            break;
                        case 24:
                            sleep(100);
                            break;
                        case 25:
                            motors("turn_right", 2500);
                            break;
                        case 26:
                            servoClamp();
                            break;
                        case 27:
                            sleep(100);
                            break;
                        case 28:
                            intake("in");
                            break;
                        case 29:
                            motorsFast("back", 1700);
                            break;
                        case 30:
                            sleep(100);
                            break;
                        case 31:
                            intakeOff();
                            break;
                        case 32:
                            motorsFast("front", 2500);
                            break;
                        case 33:
                            motors("stop", 1000);
                            break;
                    }
                    break;
            }
        }
        
        else {
            switch (block) {
                case 1:
                    switch (blockCounter) {
                        case 0:
                            break;
                        case 1:
                            counter+=2;
                            break;
                    }
                    break;
                case 2:
                    switch (blockCounter) {
                        case 0:
                            break;
                        case 1:
                            counter+=2;
                            break;
                    }
                    break;
                case 3:
                    switch (blockCounter) {
                        case 0:
                            break;
                        case 1:
                            counter+=2;
                            break;
                    }
                    break;
            }
        }
        
        if (step < counter) {
            counter--;
            blockCounter++;
        }
    }
}
