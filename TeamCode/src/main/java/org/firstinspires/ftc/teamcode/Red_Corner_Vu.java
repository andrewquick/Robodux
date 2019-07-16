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
@Autonomous(name = "Red_Corner_Vu", group = "Red")
@Disabled
public class Red_Corner_Vu extends LinearOpMode {

    OpenGLMatrix lastLocation = null;
    public LynxI2cColorRangeSensor color1 = null;

    Hardware_robodux robot = new Hardware_robodux();
    BNO055IMU imu;


    double arm_down = 0.154;
    double arm_up = .56;


    double red = 0;
    double blue = 0;

    double total_drive = 1000;

    int zone = 0;

    boolean spike = false;

    double initialheading = 0;

    boolean once = false;
    int i = 0;

    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() throws InterruptedException {
        color1 = hardwareMap.get(LynxI2cColorRangeSensor.class, "color1");
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
        robot.clampl.setPosition(0.905);
        robot.clampr.setPosition(0.095);


        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = DEGREES;

        imu.initialize(parameters);

        VuforiaLocalizer.Parameters vuparameters = new VuforiaLocalizer.Parameters();
        vuparameters.vuforiaLicenseKey = "AQ4c5An/////AAAAGXHPS/VK/kM9p1fd0MoGsN5AKZHfSVWfqv0W1ccbbHtwCY0bnCb6rbBGQSzIqLtSUIzim4PQLMdvUS+6Q5g873CX538/L/4FY+HrtluUIIKTQefegsH27VRIlHgue83sg6mFSPUlbvXXND52Axl8eYEV6LHwru/dCqaAkHdT3dp18+l2nOS11sw5P8NeTiO06D5zg5NIB7L+qfGAYpnWrq8YBCJw2xcZFyZKcj+sqQzYJvGBbokn/dKmxwO5xteK3uvW908EcJ/1jGtY73MwnbkXO2QTRnVqN924N509GxecFAG15XY4UDZtNVS5LLB8Ik3u85o7K7nlQ47MUObGCicSIa0I2+tszy6VPsZjIikw\n";

        vuparameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(vuparameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        waitForStart();

        relicTrackables.activate();


        while(zone == 0 && getRuntime()< 13){
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark == RelicRecoveryVuMark.RIGHT){
                total_drive = 375;
                telemetry.addData("Right", "");
                zone = 3;
            }
            if (vuMark == RelicRecoveryVuMark.CENTER){
                total_drive = 1000;
                telemetry.addData("Center","");
                zone = 2;
            }
            if (vuMark == RelicRecoveryVuMark.LEFT){
                total_drive = 1520;
                telemetry.addData("Left","");
                zone = 1;
            }
        }

        sleep(30);

        robot.clampr.setPosition(0.095);
        robot.clampl.setPosition(0.905);

        robot.intake_sevl.setPosition(.57);
        robot.intake_sevr.setPosition(.43);

        robot.jewel_arm.setPosition(arm_down);


        sleep(850);

        robot.liftl.setTargetPosition(125);
        robot.liftr.setTargetPosition(125);
        robot.liftl.setPower(.5);
        robot.liftr.setPower(.5);

        sleep(100);

        red = color1.red()/15;
        blue = color1.blue()/11;

        telemetry.addData("Blue ", blue);
        telemetry.addData("Red ", red);
        telemetry.update();

        //If BLUE
        if(red > blue){


            DriveStraight(-160, 0.2);//back up to knock jewel off
            sleep(200);
            robot.jewel_arm.setPosition(arm_up);
            sleep(200);
            DriveStraight(1960, 0.25);//Drive Off far enough to line up with cryptobox

        }
        //If RED
        if (blue > red){
            DriveStraight(1800, 0.25);//Knock off Jewel and drive off stone
            sleep(100);
            robot.jewel_arm.setPosition(arm_up);
        }
        //If NONE
        if (blue == red){
            robot.jewel_arm.setPosition(arm_up);
            sleep(100);
            DriveStraight(1800, .25);//Drive Off
        }
        sleep(100);
        robot.liftl.setTargetPosition(25);
        robot.liftr.setTargetPosition(25);
        robot.liftl.setPower(.5);
        robot.liftr.setPower(.5);
        GyroTurn(-88.7,.25);//Turn 90 DEGREES
        sleep(100);
        DriveStraight(total_drive,.3);
        sleep(100);
        GyroTurn(-1,.25);
        sleep(100);
        DriveStraightColl(800,.4);
        if(spike == true) {
            DriveStraight(300, .4);
            sleep(100);
            robot.clampr.setPosition(.19);
            robot.clampl.setPosition(.81);
            sleep(100);
            DriveStraight(-300,.3);
            robot.liftl.setTargetPosition(0);
            robot.liftr.setTargetPosition(0);
            robot.liftl.setPower(.5);
            robot.liftr.setPower(.5);
            sleep(4000);
        }
        if(spike == false){
            DriveStraight(200, .3);
            sleep(200);
            robot.clampr.setPosition(.19);
            robot.clampl.setPosition(.81);
            sleep(100);
            DriveStraight(-700, .5);
            robot.liftl.setTargetPosition(0);
            robot.liftr.setTargetPosition(0);
            robot.liftl.setPower(.5);
            robot.liftr.setPower(.5);
            sleep(100);
            GyroTurn(175, .35);
            sleep(100);
            robot.intake_sevl.setPosition(0);
            robot.intake_sevr.setPosition(1);
            DriveStraight(-100,.2);
        }
    }

    public void GyroTurn (double target_angle, double speed) {



        robot.left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.left_rear_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right_rear_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double ADiff;
        double MagDiff = 0;
        double EDiff;
        double Scaling_Factor = 500;
        double right_motor_power = 0;
        double left_motor_power = 0;
        Orientation angle;
        double heading;

        angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = -angle.firstAngle;

        ADiff = target_angle - heading;
        MagDiff = abs(ADiff);

        while(MagDiff > 2){


            angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = -angle.firstAngle;

            ADiff = target_angle - heading;
            MagDiff = abs(ADiff);
            EDiff = abs(robot.right_front_drive.getCurrentPosition()) - abs(robot.left_front_drive.getCurrentPosition());


            if (ADiff > 0 && MagDiff >= 12){
                right_motor_power = -speed;
                left_motor_power = speed;
            }

            if (ADiff > 0 && MagDiff < 12){
                right_motor_power = -speed/2.5;
                left_motor_power = speed/2.5;
            }

            if (ADiff < 0 && MagDiff >= 12){
                right_motor_power = speed;
                left_motor_power = -speed;
            }

            if (ADiff < 0 && MagDiff < 12){
                right_motor_power = speed/2.5;
                left_motor_power = -speed/2.5;
            }

            if (ADiff == 0){
                right_motor_power = 0;
                left_motor_power = 0;
            }

            if(EDiff > 0 && right_motor_power > 0) {right_motor_power = right_motor_power - (abs(EDiff) / Scaling_Factor);}
            if(EDiff > 0 && right_motor_power < 0) {right_motor_power = right_motor_power + (abs(EDiff) / Scaling_Factor);}

            if (EDiff < 0 && left_motor_power > 0){left_motor_power = left_motor_power -(abs(EDiff) / Scaling_Factor);}
            if (EDiff < 0 && left_motor_power < 0){left_motor_power = left_motor_power +(abs(EDiff) / Scaling_Factor);}


            robot.right_front_drive.setPower(right_motor_power);
            robot.right_rear_drive.setPower(right_motor_power);
            robot.left_front_drive.setPower(left_motor_power);
            robot.left_rear_drive.setPower(left_motor_power);
        }

        robot.left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        initialheading = -angle.firstAngle;

    }


    public void DriveStraight(double distance, double inputpower) {

        double rightpower = 0;
        double leftpower = 0;
        double leftposition = 0;
        double rightposition = 0;
        double accmult;
        double deccmult;
        double diff;
        double scalingfactor = 600;
        double accdist = 132;
        double oneshot = 0;

        robot.left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.left_rear_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right_rear_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (((((abs(leftposition + rightposition)) / 2) + 25) < abs(distance)) || ((((abs(leftposition + rightposition)) / 2) - 25) > abs(distance))) {

            leftposition = robot.left_rear_drive.getCurrentPosition();
            rightposition = robot.right_front_drive.getCurrentPosition();

            if (distance < 0 && oneshot == 0) {
                inputpower = inputpower * (-1);
                oneshot = 1;
            }

            diff = rightposition - leftposition;

            if (diff > 0 && inputpower > 0) {
                rightpower = inputpower - (diff / scalingfactor);
                leftpower = inputpower;
            }

            if (diff < 0 && inputpower > 0) {
                rightpower = inputpower;
                leftpower = inputpower + (diff / scalingfactor);

            }

            if (diff > 0 && inputpower < 0) {
                rightpower = inputpower;
                leftpower = inputpower + (diff / scalingfactor);
            }

            if (diff < 0 && inputpower < 0) {
                rightpower = inputpower - (diff / scalingfactor);
                leftpower = inputpower;
            }

            if (diff == 0.0) {
                rightpower = inputpower;
                leftpower = inputpower;
            }

            if (abs(distance) < (2 * accdist)) {
                accmult = 1;
                deccmult = 1;
            } else {
                accmult = (0.05 / (abs(inputpower))) + ((abs((rightposition + leftposition) / 2)) / accdist);
                if (accmult > 1)
                    accmult = 1;

                if ((abs(distance - ((rightposition + leftposition) / 2))) < accdist) {
                    deccmult = ((0.05 / abs(inputpower)) + ((abs(distance - ((rightposition + leftposition) / 2))) / accdist));
                    if (deccmult > 1)
                        deccmult = 1;
                } else
                    deccmult = 1;
            }
            rightpower = rightpower * accmult * deccmult;
            leftpower = leftpower * accmult * deccmult;

            //set motors to motor powers
            robot.right_front_drive.setPower(rightpower);
            robot.right_rear_drive.setPower(rightpower);
            robot.left_front_drive.setPower(leftpower);
            robot.left_rear_drive.setPower(leftpower);



        }
        robot.left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void DriveStraightColl(double distance, double inputpower) {

        double rightpower = 0;
        double leftpower = 0;
        double leftposition = 0;
        double rightposition = 0;
        double accmult;
        double deccmult;
        double diff;
        double scalingfactor = 600;
        double accdist = 132;
        double oneshot = 0;
        double acceleration = 0;

        double first = 0;
        double second = 0;
        double third = 0;
        double fourth = 0;
        double fifth = 0;
        double average = 0;
        int count = 0;


        robot.left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.left_rear_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right_rear_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while ((((((abs(leftposition + rightposition)) / 2) + 25) < abs(distance)) || ((((abs(leftposition + rightposition)) / 2) - 25) > abs(distance))) && (acceleration < .75 || robot.left_front_drive.getCurrentPosition()< 400)){

            leftposition = robot.left_rear_drive.getCurrentPosition();
            rightposition = robot.right_front_drive.getCurrentPosition();

            acceleration = imu.getLinearAcceleration().yAccel;


            /*while(count <= 1){
                if(count == 0)
                    first = acceleration;
                if(count == 1)
                    second = acceleration;
                if(count == 2)
                    third = acceleration;
                if(count == 3)
                    fourth = acceleration;
                if(count == 4)
                    fifth = acceleration;
                count++;
            }
            if(count == 2)
                count = 0;


            average = (first + second + third + fourth + fifth)/2;
*/
            if (distance < 0 && oneshot == 0) {
                inputpower = inputpower * (-1);
                oneshot = 1;
            }

            diff = rightposition - leftposition;

            if (diff > 0 && inputpower > 0) {
                rightpower = inputpower - (diff / scalingfactor);
                leftpower = inputpower;
            }

            if (diff < 0 && inputpower > 0) {
                rightpower = inputpower;
                leftpower = inputpower + (diff / scalingfactor);

            }

            if (diff > 0 && inputpower < 0) {
                rightpower = inputpower;
                leftpower = inputpower + (diff / scalingfactor);
            }

            if (diff < 0 && inputpower < 0) {
                rightpower = inputpower - (diff / scalingfactor);
                leftpower = inputpower;
            }

            if (diff == 0.0) {
                rightpower = inputpower;
                leftpower = inputpower;
            }

            if (abs(distance) < (2 * accdist)) {
                accmult = 1;
                deccmult = 1;
            } else {
                accmult = (0.05 / (abs(inputpower))) + ((abs((rightposition + leftposition) / 2)) / accdist);
                if (accmult > 1)
                    accmult = 1;

                if ((abs(distance - ((rightposition + leftposition) / 2))) < accdist) {
                    deccmult = ((0.05 / abs(inputpower)) + ((abs(distance - ((rightposition + leftposition) / 2))) / accdist));
                    if (deccmult > 1)
                        deccmult = 1;
                } else
                    deccmult = 1;
            }
            rightpower = rightpower * accmult * deccmult;
            leftpower = leftpower * accmult * deccmult;

            //set motors to motor powers
            robot.right_front_drive.setPower(rightpower);
            robot.right_rear_drive.setPower(rightpower);
            robot.left_front_drive.setPower(leftpower);
            robot.left_rear_drive.setPower(leftpower);
        }

        if(robot.right_front_drive.getCurrentPosition()< 770) {
            spike = true;
        }

        robot.left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}

