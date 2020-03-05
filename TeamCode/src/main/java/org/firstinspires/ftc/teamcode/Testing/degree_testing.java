package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;
import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareGyro;
import org.firstinspires.ftc.teamcode.Library.GeneralTools;
import org.firstinspires.ftc.teamcode.Library.OmniWheel;
import org.firstinspires.ftc.teamcode.Library.OrientationTools;


@TeleOp(name = "testing color sideways")
public class degree_testing extends LinearOpMode {
    HardwareChassis hwmp;
    HardwareGyro gyro;
    OmniWheel wheel;
    OrientationTools tool;
    GeneralTools t;

    double startPos;


    @Override
    public void runOpMode() {
        hwmp = new HardwareChassis(hardwareMap);
        wheel = new OmniWheel(hwmp);
        gyro = new HardwareGyro(hardwareMap);
        tool = new OrientationTools(hwmp, hardwareMap, this);
        gyro.init(hardwareMap);
        startPos = tool.getDegree360(gyro.imu);
        t = new GeneralTools(this,hwmp);

        waitForStart();

        if (opModeIsActive()) {
            telemetry.addData("before","");
            telemetry.update();
            tool.driveEncoder(this, 20, 20, 0.3, wheel, startPos, gyro.imu, 200, 125);
            telemetry.addData("after","");
            telemetry.update();
        }

    }
}

