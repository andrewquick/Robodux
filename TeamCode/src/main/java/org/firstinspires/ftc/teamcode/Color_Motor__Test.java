package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Quick's on 9/21/2017.
 */
@TeleOp
@Disabled
public class Color_Motor__Test extends OpMode {

    public LynxI2cColorRangeSensor color1 = null;
    Hardware_robodux robot = new Hardware_robodux();

    double red;
    double blue;
    int get = 7;
    @Override
    public void init(){
        robot.init(hardwareMap);
        color1 = hardwareMap.get(LynxI2cColorRangeSensor.class,"color1");
    }
    @Override
    public void loop(){

        if(color1.blue() > 90 && color1.blue() > color1.red()){
            robot.left_front_drive.setPower(.3);
        }else if (color1.blue() < 90 && color1.red() > 90 && color1.red() > color1.blue()){
            robot.right_front_drive.setPower(.3);
        }else if (color1.blue() < 90 && color1.red() < 90){
            robot.left_front_drive.setPower(0);
            robot.right_front_drive.setPower(0);
        }

        red = color1.red();
        blue = color1.blue();

        telemetry.addData("Red Color = ", red);
        telemetry.addData("Blue Color = ", blue);
        telemetry.update();
    }
    @Override
    public void stop(){
        robot.left_front_drive.setPower(0);
        robot.right_front_drive.setPower(0);
    }

}
