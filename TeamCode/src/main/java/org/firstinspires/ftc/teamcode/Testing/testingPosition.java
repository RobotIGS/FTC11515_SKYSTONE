package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;
import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareGyro;
import org.firstinspires.ftc.teamcode.Library.OmniWheel;
import org.firstinspires.ftc.teamcode.Library.OrientationTools;

@TeleOp(name = "posTesting")
public class testingPosition extends OpMode{
    HardwareChassis robot;
    private HardwareGyro gyro;
    private OmniWheel wheel;
    private double[] pos = new double[]{0,0};
    private double[] powers = new double[]{0,0,0,0};
    double offset = 0;
    private double posTodrive;

    static final double     COUNTS_PER_MOTOR_REV    = 753.2 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_CMS      = 10.0 ;     // For figuring circumference
    static final double     COUNTS_PER_CM           = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CMS * Math.PI);


    @Override
    public void init() {
        robot = new HardwareChassis(hardwareMap);
        gyro = new HardwareGyro(hardwareMap);
        gyro.init(hardwareMap);
        wheel = new OmniWheel(robot);
        posTodrive = getDegree360(gyro.imu);
    }

    @Override
    public void loop() {
        offset = getDegree360(gyro.imu) - posTodrive;
        powers = wheel.calculate(WHEEL_DIAMETER_CMS/2,0,0,gamepad1.left_stick_y,gamepad1.left_stick_x,0);

        robot.motor_front_left.setPower(powers[0]);
        robot.motor_front_right.setPower(powers[1]);
        robot.motor_rear_left.setPower(powers[2]);
        robot.motor_rear_right.setPower(powers[3]);


        pos[0] +=getVector(robot.motor_front_left.getPower(),robot.motor_front_right.getPower(),robot.motor_rear_left.getPower(),robot.motor_rear_right.getPower(),1,1)[0];
        pos[1] +=getVector(robot.motor_front_left.getPower(),robot.motor_front_right.getPower(),robot.motor_rear_left.getPower(),robot.motor_rear_right.getPower(),1,1)[1];




        telemetry.addData("FL",robot.motor_front_left.getPower());
        telemetry.addData("pos",getVector(robot.motor_front_left.getPower(),robot.motor_front_right.getPower(),robot.motor_rear_left.getPower(),robot.motor_rear_right.getPower(),1,1)[1]);
        telemetry.addData("x",pos[0]);
        telemetry.addData("y",pos[1]);
        telemetry.update();

    }
    public double[] getVector(double p1,double p2, double p3, double p4, double width, double length){
        return new double[]{0.25*(p1+p2+p3+p4),-0.25*(p1+p4)+0.25*(p2+p3),-1};
    }

    public double getDegree360(BNO055IMU imu){
        return 180+imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
    }

}
