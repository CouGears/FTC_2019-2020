package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous

public class Forward_Auto extends LinearOpMode {
    
    private DcMotor motorFL;
    private DcMotor motorBL;
    private DcMotor motorBR;
    private DcMotor motorFR;
    private DcMotor armAngle;
    private DcMotor armSlide;
    private Servo servoLS;
    private Servo servoRS;
    private Servo servoLR;
    private Servo servoRR;
    private CRServo intakeL;
    private CRServo intakeR;
    private Servo angleL;
    private Servo angleR;
    private double speed;
    
    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;
    
    public void motors (String direction, int distance) {
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        if (direction.equals("front")) {
            while ((Math.abs(motorFL.getCurrentPosition()) < distance) && opModeIsActive()) {
                motorFL.setTargetPosition(-distance - 25);
                motorFL.setPower(speed);
                motorBL.setTargetPosition(-distance - 25);
                motorBL.setPower(speed);
                motorFR.setTargetPosition(distance + 25);
                motorFR.setPower(speed);
                motorBR.setTargetPosition(distance + 25);
                motorBR.setPower(speed);
                
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
            while ((Math.abs(motorFL.getCurrentPosition()) < distance) && opModeIsActive()) {
                motorFL.setTargetPosition(distance + 25);
                motorFL.setPower(speed);
                motorBL.setTargetPosition(distance + 25);
                motorBL.setPower(speed);
                motorFR.setTargetPosition(-distance - 25);
                motorFR.setPower(speed);
                motorBR.setTargetPosition(-distance - 25);
                motorBR.setPower(speed);
                
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
            while ((Math.abs(motorFL.getCurrentPosition()) < distance) && opModeIsActive()) {
                motorFL.setTargetPosition(distance + 25);
                motorFL.setPower(speed);
                motorBL.setTargetPosition(-distance - 25);
                motorBL.setPower(speed);
                motorFR.setTargetPosition(distance + 25);
                motorFR.setPower(speed);
                motorBR.setTargetPosition(-distance - 25);
                motorBR.setPower(speed);
                
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
            while ((Math.abs(motorFL.getCurrentPosition()) < distance) && opModeIsActive()) {
                motorFL.setTargetPosition(-distance - 25);
                motorFL.setPower(speed);
                motorBL.setTargetPosition(distance + 25);
                motorBL.setPower(speed);
                motorFR.setTargetPosition(-distance - 25);
                motorFR.setPower(speed);
                motorBR.setTargetPosition(distance + 25);
                motorBR.setPower(speed);
                
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
            while ((Math.abs(motorFL.getCurrentPosition()) < distance) && opModeIsActive()) {
                motorFL.setTargetPosition(-distance - 25);
                motorFL.setPower(speed);
                motorBL.setTargetPosition(-distance - 25);
                motorBL.setPower(speed);
                motorFR.setTargetPosition(-distance - 25);
                motorFR.setPower(speed);
                motorBR.setTargetPosition(-distance - 25);
                motorBR.setPower(speed);
                
                speedIncrement(distance - (Math.abs(motorFL.getCurrentPosition())));
                
                /*Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                        (int) (sensorColor.green() * SCALE_FACTOR),
                        (int) (sensorColor.blue() * SCALE_FACTOR),
                        hsvValues);
                telemetry.addData("Hue", hsvValues[0]);
                telemetry.update();*/
            }
        }
        
        else if (direction.equals("turn_right")) {
            while ((Math.abs(motorFL.getCurrentPosition()) < distance) && opModeIsActive()) {
                motorFL.setTargetPosition(distance + 25);
                motorFL.setPower(speed);
                motorBL.setTargetPosition(distance + 25);
                motorBL.setPower(speed);
                motorFR.setTargetPosition(distance + 25);
                motorFR.setPower(speed);
                motorBR.setTargetPosition(distance + 25);
                motorBR.setPower(speed);
                
                speedIncrement(distance - (Math.abs(motorFL.getCurrentPosition())));
                
                /*Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                        (int) (sensorColor.green() * SCALE_FACTOR),
                        (int) (sensorColor.blue() * SCALE_FACTOR),
                        hsvValues);
                telemetry.addData("Hue", hsvValues[0]);
                telemetry.update();*/
            }
        }
        
        else if (direction.equals("stop")) {
            while (opModeIsActive()) {
                motorFL.setPower(0);
                motorBL.setPower(0);
                motorFR.setPower(0);
                motorBR.setPower(0);
            }
        }
        
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    
    public void speedIncrement(int distance) {
        if (distance < 100) speed = .3;
        else if (distance < 200) speed = .35;
        else if (distance < 300) speed = .4;
        else if (distance < 400) speed = .45;
        else speed = .5;
    }
    
    @Override
    public void runOpMode() {
        
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        armAngle = hardwareMap.get(DcMotor.class, "armAngle");
        armSlide = hardwareMap.get(DcMotor.class, "armSlide");
        //sensorColor = hardwareMap.get(ColorSensor.class, "sensorColorRange");
        servoLS = hardwareMap.get(Servo.class, "servoLS");
        servoRS = hardwareMap.get(Servo.class, "servoRS");
        servoLR = hardwareMap.get(Servo.class, "servoLR");
        servoRR = hardwareMap.get(Servo.class, "servoRR");
        intakeL = hardwareMap.get(CRServo.class, "intakeL");
        intakeR = hardwareMap.get(CRServo.class, "intakeR");
        angleL = hardwareMap.get(Servo.class, "angleL");
        angleR = hardwareMap.get(Servo.class, "angleR");
        
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armAngle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        motorFL.setTargetPosition(0);
        motorBL.setTargetPosition(0);
        motorFR.setTargetPosition(0);
        motorBR.setTargetPosition(0);
        armAngle.setTargetPosition(0);
        armSlide.setTargetPosition(0);
        
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armAngle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armAngle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        armAngle.setDirection(DcMotorSimple.Direction.FORWARD);
        armSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        
        waitForStart();
        
        while (opModeIsActive()) {
            servoLS.setPosition(.6);
            servoRS.setPosition(.4);
            angleL.setPosition(.5);
            angleR.setPosition(.5);
            
            //Grab the foundation
            
            motors("front", 2000);
            motors("stop", 0);
        }
    }
}
