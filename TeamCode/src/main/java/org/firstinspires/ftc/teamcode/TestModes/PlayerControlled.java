package org.firstinspires.ftc.teamcode.TestModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;
import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareGyro;
import org.firstinspires.ftc.teamcode.Library.ColorTools;
import org.firstinspires.ftc.teamcode.Library.GeneralTools;
import org.firstinspires.ftc.teamcode.Library.OmniWheel;
import org.firstinspires.ftc.teamcode.Library.OrientationTools;

//by Lena and Simeon

@TeleOp(name = "Player_Controlled")

public class PlayerControlled extends OpMode {
    HardwareChassis robot;
    ColorTools colorTools;
    double smootingValue;
    double[] liftZeros = new double[2];

    HardwareGyro gyro;
    OmniWheel oWheel;
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

    //-------------------------------

    @Override
    public void init() {
        robot = new HardwareChassis(hardwareMap);
        colorTools = new ColorTools();
        smootingValue = -0.5;
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

        super.msStuckDetectLoop = 10000000;

        setZeros();
    }

    @Override
    public void loop() {

        if(Math.abs(offset)>=180){
            gyro.init(hardwareMap);
            posTodrive = oTool.getDegree360(gyro.imu);
            offset = posTodrive -oTool.getDegree360(gyro.imu);
        }

        oWheel.setMotors(power_Y*0.5,power_X*0.5,offset/smoothness);
        //-----------------------------------------------------------------------------------
        if((System.currentTimeMillis() -timestamp1 > 500) && adjust){
            posTodrive = oTool.getDegree360(gyro.imu);
            adjust = false;
        }

        if(System.currentTimeMillis() -timestamp1 > 500){
            offset = oTool.getDegree360(gyro.imu) - posTodrive;}
        else{
            offset = 0;}


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
        else if (gamepad2.left_stick_y < 0 && gamepad2.y) {
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
        else if (gamepad2.left_stick_y > 0 && gamepad2.y) {
            robot.motor_lift_left.setPower(gamepad2.left_stick_y);
            robot.motor_lift_right.setPower(-gamepad2.left_stick_y);
        }

        // set all motors 0 if nothing is pressed
        else {
            robot.motor_lift_left.setPower(0);
            robot.motor_lift_right.setPower(0);
        }
        // set new zero-point
        if (gamepad2.x) {
            setZeros();
        }

        if (gamepad2.right_bumper) {
            robot.servo_capstone.setPosition(0.4);
        } else if (gamepad2.left_bumper) {
            robot.servo_capstone.setPosition(robot.servo_capstone.getPosition()+0.1);
        }

        //servo clamp
        if(gamepad2.a) {
            GeneralTools.closeClamp(robot);
        } else if (gamepad2.b) {
            GeneralTools.openClamp(robot);
        }

        //servos foundation
        if (gamepad1.right_bumper) { //grab
            GeneralTools.grabFoundation(robot);
        } else if (gamepad1.left_bumper) { //release
            GeneralTools.releaseFoundation(robot);
        }


        //telemetry.addData("Smoothing Value: ", smootingValue);
        //telemetry.addLine();
        //telemetry.addData("Ärmchen R:", robot.servo_claw_right.getPosition());
        //telemetry.addData("Ärmchen L:", robot.servo_claw_left.getPosition());
        //telemetry.addLine();
        //telemetry.addData("Touch Left: ", robot.touch_left.getState());
        //telemetry.addData("Touch Right: ", robot.touch_right.getState());
        //telemetry.addLine();
        //telemetry.addData("H: ", colorTools.showHSV(robot.color_back)[0]);
        //telemetry.addData("S: ", colorTools.showHSV(robot.color_back)[1]);
        //telemetry.addLine();
        //telemetry.addData("LiftLpos: ", (robot.motor_lift_left.getCurrentPosition() - liftZeros[0]) / 712.6);
        //telemetry.addData("LiftRpos: ", (robot.motor_lift_right.getCurrentPosition() - liftZeros[1]) / 712.6);
        //telemetry.addData("ExtenderPos", robot.motor_extender.getCurrentPosition()/195.4);
        //telemetry.update();
    }

    public void setZeros() {
        liftZeros[0] = robot.motor_lift_left.getCurrentPosition();
        liftZeros[1] = robot.motor_lift_right.getCurrentPosition();
    }
}


