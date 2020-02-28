package org.firstinspires.ftc.teamcode.TestModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;
import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareGyro;
import org.firstinspires.ftc.teamcode.Library.ColorTools;

@TeleOp(name="tests")
public class GeneralTests extends OpMode {
    HardwareGyro gyro;
    HardwareChassis robot;
    ColorTools ctools;
    @Override
    public void init() {
        robot = new HardwareChassis(hardwareMap);
        ctools = new ColorTools();
        gyro = new HardwareGyro(hardwareMap);
        gyro.init(hardwareMap);
    }

    @Override
    public void loop() {

        telemetry.addData("alpha",robot.color_clamp.alpha());
        telemetry.addData("found",ctools.isSkystoneFound(robot.color_clamp));
        telemetry.update();
    }
}
