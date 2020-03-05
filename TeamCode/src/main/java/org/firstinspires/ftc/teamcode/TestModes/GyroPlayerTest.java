package org.firstinspires.ftc.teamcode.TestModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;
import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareGyro;
import org.firstinspires.ftc.teamcode.Library.ColorTools;
import org.firstinspires.ftc.teamcode.Library.GeneralTools;
import org.firstinspires.ftc.teamcode.Library.OmniWheel;
import org.firstinspires.ftc.teamcode.Library.OrientationTools;

//by Lena and Simeon
@Disabled
@TeleOp(name = "GyroPlayerTest")

public class GyroPlayerTest extends OpMode {
    HardwareGyro gyro;
    OmniWheel oWheel;
    HardwareChassis robot;
    ColorTools colorTools;
    OrientationTools oTool;

    double smoothness;
    double offset;
    double posTodrive;

    double power_X;
    double power_Y;


    boolean adjust;

    double timestamp1;
    double timestamp2;
    double timestamp3;


    boolean turning

    //-------------------------------

    double smootingValue;
    double[] liftZeros = new double[2];



    @Override
    public void init() {
        robot = new HardwareChassis(hardwareMap);
        colorTools = new ColorTools();
        oWheel = new OmniWheel(robot);
        oTool = new OrientationTools(robot,hardwareMap,null);
        gyro = new HardwareGyro(hardwareMap);
        gyro.init(hardwareMap);

        smoothness = 100;
        posTodrive = oTool.getDegree360(gyro.imu);
        offset = posTodrive -oTool.getDegree360(gyro.imu);

        adjust = true;
        turning = false;

        super.msStuckDetectLoop = 10000000;
    }

    @Override
    public void loop() {

        if(Math.abs(offset)>=180){
            gyro.init(hardwareMap);
            posTodrive = oTool.getDegree360(gyro.imu);
            offset = posTodrive -oTool.getDegree360(gyro.imu);
        }

        if((System.currentTimeMillis() -timestamp1 > 500) && adjust){
            posTodrive = oTool.getDegree360(gyro.imu);
            adjust = false;
        }

        if((System.currentTimeMillis() -timestamp1 > 500) && gamepad1.right_trigger == 0){
            offset = oTool.getDegree360(gyro.imu) - posTodrive;}
        else{
            offset = 0;}




        if(gamepad1.right_trigger != 0) {
            offset = -gamepad1.right_trigger * smoothness;
            adjust = true;
            timestamp1 = System.currentTimeMillis();
            //posTodrive = oTool.getDegree360(gyro.imu);
        }
        if(gamepad1.left_trigger !=0) {
            offset = gamepad1.right_trigger * smoothness;
            adjust = true;
            timestamp1 = System.currentTimeMillis();
            //posTodrive = oTool.getDegree360(gyro.imu);
        }


        oWheel.setMotors(power_Y*0.5,power_X*0.5,offset/smoothness);


        //-----------------------------------------------------------------------------------




        //-----------------------------------------------------------------------------------
        //power_X = (double)(gamepad1.right_trigger*(gamepad1.left_stick_x/Math.max(Math.abs(gamepad1.left_stick_x),Math.abs(gamepad1.left_stick_y)+0.001)));
        //power_Y = -(double)(gamepad1.right_trigger*(gamepad1.left_stick_y/Math.max(Math.abs(gamepad1.left_stick_x),Math.abs(gamepad1.left_stick_y)+0.001)));
        power_X = gamepad1.left_stick_x;
        power_Y = -gamepad1.left_stick_y;


        //------------------------------------------------------------------------------------------------------------

        //extender
        if (gamepad2.right_stick_y > 0 || gamepad2.right_stick_y < 0) {
            robot.motor_extender.setPower(-gamepad2.right_stick_y);
        } else {
            robot.motor_extender.setPower(0);
        }


        //* lift *//
        // upper limit
        if (gamepad2.left_stick_y < 0 && (
                (robot.motor_lift_left.getCurrentPosition() - liftZeros[0]) >= -7126 &&
                        (robot.motor_lift_right.getCurrentPosition() - liftZeros[1]) <= 7126
        )) {
            robot.motor_lift_left.setPower(gamepad2.left_stick_y);
            robot.motor_lift_right.setPower(-gamepad2.left_stick_y);
        }
        // override/ignore upper limit
        else if (gamepad2.left_stick_y < 0 && gamepad2.b) {
            robot.motor_lift_left.setPower(gamepad2.left_stick_y);
            robot.motor_lift_right.setPower(-gamepad2.left_stick_y);
        }

        // lower limit
        else if (gamepad2.left_stick_y > 0 && (
                robot.motor_lift_left.getCurrentPosition() <= liftZeros[0] &&
                        robot.motor_lift_right.getCurrentPosition() >= liftZeros[1]
        )) {
            robot.motor_lift_left.setPower(gamepad2.left_stick_y);
            robot.motor_lift_right.setPower(-gamepad2.left_stick_y);
        }
        // override/ignore lower limit
        else if (gamepad2.left_stick_y > 0 && gamepad2.b) {
            robot.motor_lift_left.setPower(gamepad2.left_stick_y);
            robot.motor_lift_right.setPower(-gamepad2.left_stick_y);
        }

        // set all motors 0 if nothing is pressed
        else {
            robot.motor_lift_left.setPower(0);
            robot.motor_lift_right.setPower(0);
        }
        // set new zero-point
        if (gamepad2.a) {
            setZeros();
        }

        if (gamepad2.dpad_up) {
            robot.servo_capstone.setPosition(0.4);
        }

        //servo clamp
        if(gamepad2.y) {
            GeneralTools.closeClamp(robot);
        } else if (gamepad2.x) {
            GeneralTools.openClamp(robot);
        }

        //servos foundation
        if (gamepad2.right_bumper) { //grab
            GeneralTools.grabFoundation(robot);
        } else if (gamepad2.left_bumper) { //release
            GeneralTools.releaseFoundation(robot);
        }


        //telemetry.addData("Smoothing Value: ", smootingValue);
        //telemetry.addLine();
        //telemetry.addData("Ärmchen R:", robot.servo_claw_right.getPosition());
        //telemetry.addData("Ärmchen L:", robot.servo_claw_left.getPosition());
        //telemetry.addLine();
        telemetry.addData("Touch Left: ", robot.touch_left.getState());
        telemetry.addData("Touch Right: ", robot.touch_right.getState());
        //telemetry.addLine();
        //telemetry.addData("H: ", colorTools.showHSV(robot.color_back)[0]);
        //telemetry.addData("S: ", colorTools.showHSV(robot.color_back)[1]);
        telemetry.addLine();
        telemetry.addData("LiftLpos: ", (robot.motor_lift_left.getCurrentPosition() - liftZeros[0]) / 712.6);
        telemetry.addData("LiftRpos: ", (robot.motor_lift_right.getCurrentPosition() - liftZeros[1]) / 712.6);
        telemetry.addData("ExtenderPos", robot.motor_extender.getCurrentPosition()/195.4);


        telemetry.addLine();


        telemetry.addData("offset",offset);
        telemetry.addData("offset/smoothness",offset/smoothness);
        telemetry.addData("posTodrive",posTodrive);
        telemetry.addData("r_trigger",gamepad1.right_trigger);
        telemetry.addData("power_X",power_X);
        telemetry.addData("power_Y",power_Y);

        telemetry.addData("x",gamepad1.left_stick_x);
        telemetry.addData("y",gamepad1.left_stick_y);
        telemetry.addData("isNaN_x",gamepad1.left_stick_x != 0.0/0.0);
        telemetry.addData("isNaN_y",Float.isNaN(gamepad1.left_stick_y));
        telemetry.update();
    }

    public void setZeros() {
        liftZeros[0] = robot.motor_lift_left.getCurrentPosition();
        liftZeros[1] = robot.motor_lift_right.getCurrentPosition();
    }


    private void placeholer(){
        if(gamepad1.right_trigger != 0) {
            while(gamepad1.right_trigger != 0){
                oWheel.setMotors(0,0,-gamepad1.right_trigger);
            }
            adjust = true;
            timestamp1 = System.currentTimeMillis();
            //posTodrive = oTool.getDegree360(gyro.imu);
        }
        if(gamepad1.left_trigger !=0) {
            while(gamepad1.left_trigger !=0){
                oWheel.setMotors(0,0,gamepad1.left_trigger);
            }
            adjust = true;
            timestamp1 = System.currentTimeMillis();
            //posTodrive = oTool.getDegree360(gyro.imu);
        }




    }
}


