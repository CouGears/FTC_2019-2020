package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous

public class ColorTestingRed extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DigitalChannel DigChannel;
    AutonMethods robot = new AutonMethods();
    
    public void init() {
        robot.init(hardwareMap, telemetry, false);
        telemetry.update();
        
        robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    
    @Override
    public void start() {
        runtime.reset();
    }
    
    @Override
    public void loop() {
        switch (robot.counter) {
            case 0:
                robot.motors("front", 1200);
                break;
            case 1:
                robot.sleep(250);
                break;
            case 2:
                robot.runTillColor("left");
                break;
            case 3:
                robot.sleep(250);
                break;
            case 4:
                robot.motors("right", 400);
                break;
            case 5:
                robot.sleep(250);
                break;
            case 6:
                robot.intake("in");
                break;
            case 7:
                robot.motors("front", 600);
                break;
            case 8:
                robot.sleep(250);
                break;
            case 9:
                robot.intakeOff();
                break;
            case 10:
                robot.motors("back", 600);
                break;
            case 11:
                robot.sleep(250);
                break;
            case 12:
                robot.motors("right", 3500 + robot.ColorEncoder);
                break;
            case 13:
                robot.intake("out");
                break;
            case 14:
                robot.sleep(1000);
                break;
            case 15:
                robot.intakeOff();
                break;
            case 16:
                robot.motors("front", 500);
                break;
            case 17:
                robot.sleep(250);
                break;
            case 18:
                robot.motors("left", 1200);
                break;
            case 19:
                robot.sleep(250);
                break;
            case 20:
                robot.motors("front", 300);
                break;
            case 21:
                robot.motors("stop", 1000);
                break;
        }
        
        telemetry.addData("Case:", robot.counter);
        telemetry.update();
    }
}
