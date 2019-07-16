package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static com.qualcomm.hardware.bosch.BNO055IMU.AngleUnit.DEGREES;
import static java.lang.Math.abs;

/**
 * Created by Quick's on 10/26/2017.
 */
@Autonomous(name = "Red_Center_Iowa", group = "Red")
@Disabled
public class Red_Center_Iowa extends LinearOpMode {

    OpenGLMatrix lastLocation = null;
    public LynxI2cColorRangeSensor color1 = null;

    Hardware_robodux robot = new Hardware_robodux();
    BNO055IMU imu;


    double arm_down = 0.098;
    double arm_up = .69;


    double red = 0;
    double blue = 0;

    double total_drive = 1918;

    int zone = 0;

    boolean spike = false;

    double initialheading = 0;

    double time = 0;

    double rjs_down = .375;
    double rjs_up = 1;
    double ljs_down = .47;
    double ljs_up = 0;

    boolean once = false;
    int i = 0;

    VuforiaLocalizer vuforia;
    public LynxI2cColorRangeSensor color2 = null;

    @Override
    public void runOpMode() throws InterruptedException {
        color1 = hardwareMap.get(LynxI2cColorRangeSensor.class, "color1");
        color2 = hardwareMap.get(LynxI2cColorRangeSensor.class, "color2");
        robot.init(hardwareMap);

        robot.left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.liftl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.jewel_arm.setPosition(arm_up);

        robot.intake_sevr.setPosition(0.63);//one is closed
        robot.intake_sevl.setPosition(0.34);//zero is closed
        sleep(10);
        robot.clampl.setPosition(0.92);
        sleep(10);
        robot.clampr.setPosition(0.1);
        sleep(10);
        robot.right_jewel_stop.setPosition(rjs_up);
        robot.left_jewel_stop.setPosition(ljs_up);


        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = DEGREES;

        imu.initialize(parameters);

        VuforiaLocalizer.Parameters vuparameters = new VuforiaLocalizer.Parameters();
        vuparameters.vuforiaLicenseKey = "AQ4c5An/////AAAAGXHPS/VK/kM9p1fd0MoGsN5AKZHfSVWfqv0W1ccbbHtwCY0bnCb6rbBGQSzIqLtSUIzim4PQLMdvUS+6Q5g873CX538/L/4FY+HrtluUIIKTQefegsH27VRIlHgue83sg6mFSPUlbvXXND52Axl8eYEV6LHwru/dCqaAkHdT3dp18+l2nOS11sw5P8NeTiO06D5zg5NIB7L+qfGAYpnWrq8YBCJw2xcZFyZKcj+sqQzYJvGBbokn/dKmxwO5xteK3uvW908EcJ/1jGtY73MwnbkXO2QTRnVqN924N509GxecFAG15XY4UDZtNVS5LLB8Ik3u85o7K7nlQ47MUObGCicSIa0I2+tszy6VPsZjIikw\n";

        vuparameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(vuparameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        waitForStart();

        robot.clampr.setPosition(0.095);
        sleep(10);
        robot.clampl.setPosition(0.905);
        sleep(10);
        robot.intake_sevl.setPosition(.57);
        sleep(10);
        robot.intake_sevr.setPosition(.43);
        sleep(10);
        robot.jewel_arm.setPosition(arm_down);
        robot.right_jewel_stop.setPosition(rjs_down);

        sleep(50);
        relicTrackables.activate();

        time = getRuntime();

        while (zone == 0 && getRuntime() < (time + 2)) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark == RelicRecoveryVuMark.RIGHT) {
                total_drive = 1918;
                zone = 3;
            }
            if (vuMark == RelicRecoveryVuMark.CENTER) {
                total_drive = 2533;
                zone = 2;
            }
            if (vuMark == RelicRecoveryVuMark.LEFT) {
                total_drive = 3103;
                zone = 1;
            }
        }
        sleep(650);

        robot.liftl.setTargetPosition(125);
        robot.liftr.setTargetPosition(125);
        robot.liftl.setPower(.5);
        robot.liftr.setPower(.5);

        sleep(25);

        red = color1.red() / 15;
        blue = color1.blue() / 11;

        //If BLUE
        if (red > blue) {
            robot.right_jewel_stop.setPosition(rjs_up);
            sleep(250);
            DriveStraight(190, -0.25);//back up to knock jewel off
            sleep(30);
            robot.jewel_arm.setPosition(arm_up);
            //put jewel stopper servo up
            sleep(300);
            DriveStraight((1600 + 190), 0.25);//Drive Off far enough to line up with cryptobox
            sleep(50);
            DriveStraight(total_drive - 1600, .55);
        }
        //If RED
        if (blue > red) {
            DriveStraight(1100, .25);//Knock off Jewel and drive off stone
            sleep(100);
            robot.jewel_arm.setPosition(arm_up);
            sleep(10);
            robot.right_jewel_stop.setPosition(rjs_up);
            sleep(900);
            DriveStraight((total_drive - 1100), 0.55);//drive to line up with cryptobox

        }
        //If NONE
        if (blue == red) {
            robot.jewel_arm.setPosition(arm_up);
            sleep(10);
            robot.right_jewel_stop.setPosition(rjs_up);
            sleep(9000);
            DriveStraight(1100, .25);//Drive Off
            sleep(100);
            DriveStraight(total_drive - 1100, .55);
        }
        sleep(50);
        robot.liftl.setTargetPosition(45);
        robot.liftr.setTargetPosition(45);
        robot.liftl.setPower(.5);
        robot.liftr.setPower(.5);
        GyroTurn(88.9, .45);//Turn 90 DEGREES
        sleep(75);
        DriveStraight(435, .7);
        sleep(75);
        robot.liftl.setTargetPosition(-15);
        robot.liftr.setTargetPosition(-15);
        robot.liftl.setPower(.5);
        robot.liftr.setPower(.5);
        sleep(150);
        robot.clampr.setPosition(.27);
        robot.clampl.setPosition(.73);
        robot.intake_sevl.setPosition(.05);
        robot.intake_sevr.setPosition(.95);
        sleep(400);
        robot.intakel.setPower(-.75);
        robot.intaker.setPower(-.75);
        sleep(300);
        DriveStraight(1040, -.7);
        sleep(75);
        robot.intakel.setPower(0);
        robot.intaker.setPower(0);
        GyroTurn(-88.9, .36);
        sleep(75);
        robot.intakel.setPower(1);
        robot.intaker.setPower(1);
        if(zone == 2 || zone == 3 || zone == 0) {
            DriveStraight(1425, .75);
            sleep(75);
            DriveStraight(1425, -.8);
            sleep(75);
        }
        if(zone == 1) {
            DriveStraight(1300, .75);
            sleep(75);
            DriveStraight(1300, -.8);
            sleep(75);
        }
        if(color2.getDistance(DistanceUnit.MM) < 130){
            robot.clampr.setPosition(0.065);
            robot.clampl.setPosition(0.935);
            sleep(75);
            robot.intakel.setPower(0);
            robot.intaker.setPower(0);
            sleep(75);
            if(zone == 3 || zone == 0){
                GyroTurn(61, .4);
                sleep(50);
                DriveStraight(1350, .75);
            }
            if(zone == 1){
                GyroTurn(126, .4);
                sleep(50);
                DriveStraight(1345, .75);
            }
            if(zone == 2){
                GyroTurn(70.5, .4);
                sleep(50);
                DriveStraight(1170, .75);
            }
            sleep(25);
            robot.clampr.setPosition(.27);
            robot.clampl.setPosition(.73);
            sleep(100);
            robot.intakel.setPower(-1);
            robot.intaker.setPower(-1);
            sleep(25);
            DriveStraight(150, -.8);
        }
        else{
            DriveStraight(600, -.4);
            sleep(75);
            robot.intakel.setPower(0);
            robot.intaker.setPower(0);
            sleep(100);
        }

    }


    public void GyroTurn(double target, double inputspeed) {
        robot.left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.left_rear_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right_rear_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Orientation angle;
        double heading;

        double taccdist = 0;
        double tdeccdist = 0;
        double startangle = 0;
        boolean right = false;
        double startspeed = .12;
        double endspeed = .05;
        double speed = 0;
        double rightspeed = 0;
        double leftspeed = 0;
        double ediff = 0;
        double scalefactor = 525;
        double rightpower = 0;
        double leftpower = 0;

        angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = -angle.firstAngle;

        startangle = heading;

        if(target > startangle) {
            right = true;
            taccdist = startangle + 20;
            tdeccdist = target - 50;
        }
        if(target < startangle) {
            right = false;
            taccdist = startangle - 20;
            tdeccdist = target + 50;
        }
        while((heading < (target-1)) || (heading  > (target + 1))){
            angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = -angle.firstAngle;
            ediff = abs((robot.right_front_drive.getCurrentPosition() + robot.right_rear_drive.getCurrentPosition())/2)-abs((robot.left_front_drive.getCurrentPosition() + robot.left_rear_drive.getCurrentPosition())/2);

            if(right) {
                if (heading < taccdist) {
                    speed = startspeed + (heading / taccdist) * (inputspeed - startspeed);
                }
                if (heading >= taccdist && heading <= tdeccdist) {
                    speed = inputspeed;
                }
                if (heading > tdeccdist) {
                    speed = endspeed + ((target - heading) / (target - tdeccdist)) * (inputspeed - endspeed);
                }
                rightspeed = -speed;
                leftspeed = speed;
            }
            if(!right){
                if (heading > taccdist) {
                    speed = startspeed + (heading / taccdist) * (inputspeed - startspeed);
                }
                if (heading <= taccdist && heading >= tdeccdist) {
                    speed = inputspeed;
                }
                if (heading < tdeccdist) {
                    speed = endspeed + ((target - heading) / (target - tdeccdist)) * (inputspeed - endspeed);
                }
                rightspeed = speed;
                leftspeed = -speed;
            }
            rightpower = rightspeed * (1 - (ediff/scalefactor));
            leftpower = leftspeed * (1 + (ediff/scalefactor));

            robot.right_front_drive.setPower(rightpower);
            robot.right_rear_drive.setPower(rightpower);
            robot.left_front_drive.setPower(leftpower);
            robot.left_rear_drive.setPower(leftpower);
        }
        robot.right_front_drive.setPower(0);
        robot.right_rear_drive.setPower(0);
        robot.left_front_drive.setPower(0);
        robot.left_rear_drive.setPower(0);

    }
    public void DriveStraight(double distance, double inputpower) {

        robot.left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.left_rear_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right_rear_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Orientation angle;
        double heading;

        double startingspeed = .09;
        double accdist = 250;
        double decdist = 350;
        double rightspeed = 0;
        double leftspeed = 0;
        double currentleft = 0;
        double currentright = 0;
        double switchpoint = 0;
        double originalheading = 0;

        angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = -angle.firstAngle;
        originalheading = heading;

        while ((currentright < distance) || (currentleft < distance)) {
            angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = -angle.firstAngle;
            currentright = abs(robot.right_front_drive.getCurrentPosition());
            currentleft = abs(robot.left_front_drive.getCurrentPosition());

            if (distance >= (accdist + decdist)) {
                if (((currentright + currentleft) / 2) <= accdist) {
                    if (inputpower > 0) {
                        rightspeed = (startingspeed + (currentright / accdist) * (inputpower - startingspeed)) * (1 + ((heading - originalheading) / 40));
                        leftspeed = (startingspeed + (currentleft / accdist) * (inputpower - startingspeed)) * (1 - ((heading - originalheading) / 40));
                    }
                    if (inputpower < 0) {
                        rightspeed = -(startingspeed + (currentright / accdist) * (-inputpower - startingspeed)) * (1 + ((heading - originalheading) / 40));
                        leftspeed = -(startingspeed + (currentleft / accdist) * (-inputpower - startingspeed)) * (1 - ((heading - originalheading) / 40));
                    }
                }
                if ((((currentright + currentleft) / 2) > accdist) && ((currentright + currentleft) / 2) < (distance - decdist)) {
                    rightspeed = inputpower * (1 + ((heading - originalheading) / 40));
                    leftspeed = inputpower * (1 - ((heading - originalheading) / 40));
                }
                if (((currentright + currentleft) / 2) >= (distance - decdist)) {
                    if (inputpower > 0) {
                        rightspeed = (startingspeed + ((distance - currentright) / decdist) * (inputpower - startingspeed)) * (1 + ((heading - originalheading) / 40));
                        leftspeed = (startingspeed + ((distance - currentleft) / decdist) * (inputpower - startingspeed)) * (1 - ((heading - originalheading) / 40));
                    }
                    if (inputpower < 0) {
                        rightspeed = -(startingspeed + ((distance - currentright) / decdist) * (-inputpower - startingspeed)) * (1 + ((heading - originalheading) / 40));
                        leftspeed = -(startingspeed + ((distance - currentleft) / decdist) * (-inputpower - startingspeed)) * (1 - ((heading - originalheading) / 40));
                    }
                }
            }
            if (distance < (accdist + decdist)) {
                switchpoint = (accdist + decdist) / 2;

                if (((currentright + currentleft) / 2) <= switchpoint) {
                    if (inputpower > 0) {
                        rightspeed = (startingspeed + (currentright / switchpoint) * (inputpower - startingspeed)* (1 + ((heading - originalheading) / 40)));
                        leftspeed = (startingspeed + (currentleft / switchpoint) * (inputpower - startingspeed)* (1 - ((heading - originalheading) / 40)));
                    }
                    if (inputpower < 0) {
                        rightspeed = -(startingspeed + (currentright / switchpoint) * (-inputpower - startingspeed))* (1 + ((heading - originalheading) / 40));
                        leftspeed = -(startingspeed + (currentleft / switchpoint) * (-inputpower - startingspeed))* (1 - ((heading - originalheading) / 40));
                    }
                }
                if (((currentright + currentleft) / 2) > switchpoint) {
                    if (inputpower > 0) {
                        rightspeed = (startingspeed + ((distance - currentright) / switchpoint) * (inputpower - startingspeed))* (1 + ((heading - originalheading) / 40));
                        leftspeed = (startingspeed + ((distance - currentleft) / switchpoint) * (inputpower - startingspeed))* (1 - ((heading - originalheading) / 40));
                    }
                    if (inputpower < 0) {
                        rightspeed = -(startingspeed + ((distance - currentright) / switchpoint) * (-inputpower - startingspeed))* (1 + ((heading - originalheading) / 40));
                        leftspeed = -(startingspeed + ((distance - currentleft) / switchpoint) * (-inputpower - startingspeed))* (1 - ((heading - originalheading) / 40));
                    }
                }
            }
            robot.left_front_drive.setPower(leftspeed);
            robot.left_rear_drive.setPower(leftspeed);
            robot.right_front_drive.setPower(rightspeed);
            robot.right_rear_drive.setPower(rightspeed);
        }
        robot.left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
}

