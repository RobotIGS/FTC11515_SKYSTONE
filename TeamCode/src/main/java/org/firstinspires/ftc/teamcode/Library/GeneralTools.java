package org.firstinspires.ftc.teamcode.Library;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;
import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareGyro;

public class GeneralTools {
    HardwareMap hardwareMap;
    HardwareChassis robot;

    HardwareGyro gyro;

    private LinearOpMode opMode;
    public GeneralTools(LinearOpMode opMode, HardwareChassis robot) {
        this.opMode = opMode;
        this.robot = robot;

        this.gyro = new HardwareGyro(this.opMode.hardwareMap);
        this.gyro.init(this.opMode.hardwareMap);
    }


    /**
     * pauses the program for additional seconds
     * @param timeStop double, in Milliseconds
     */
    public void stopForMilliSeconds(double timeStop) {
        double time = System.currentTimeMillis();

        while ((System.currentTimeMillis() < time + timeStop) && !opMode.isStopRequested()) {}
    }

    /**
     * set claw to close
     */
    public void grabSkysstone () {
        robot.servo_grab.setPosition(1);
    }

    public void openClamp () {
        robot.servo_2.setPosition(0.6);
    }

    //get current degrees of robot
    public double getDegree(){
        return 180 + gyro.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle; //hier first vllt zu second/third
    }
}

