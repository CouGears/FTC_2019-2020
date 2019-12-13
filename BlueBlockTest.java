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

public class BlueBlockTest extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DigitalChannel DigChannel;
    AutonMethods robot = new AutonMethods();
    
    public void init() {
        robot.init(hardwareMap, telemetry, false);
        telemetry.addData("Status", "Initialized");
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
                robot.intake("in");
                break;
            case 1:
                robot.motors("front", 2100);
                break;
            case 2:
                robot.sleep(1000);
                break;
            case 3:
                robot.intakeOff();
                break;
            case 4:
                robot.motors("back", 500);
                break;
            case 5:
                robot.sleep(250);
                break;
            case 6:
                robot.motors("left", 2200);
                break;
            case 7:
                robot.intake("out");
                break;
            case 8:
                robot.sleep(500);
                break;
            case 9:
                robot.motors("right", 2500);
                break;
            case 10:
                robot.sleep(250);
                break;
            case 11:
                robot.motors("stop", 1300);
                break;
            case 12:
                robot.sleep(250);
                break;
            case 13:
                robot.motors("back", 500);
                break;
            case 14:
                robot.sleep(250);
                break;
            case 15:
                robot.motors("right", 2000);
                break;
            case 16:
                robot.sleep(250);
                break;
            case 17:
                robot.motors("front", 2500);
                break;
            case 18:
                robot.motors("stop", 1000);
                break;
        }
        
        telemetry.addData("Case:", robot.counter);
        telemetry.update();
    }
}
