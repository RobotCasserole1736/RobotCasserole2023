package frc.robot.Arm;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.Constants;
import frc.robot.ArmTelemetry;

import frc.robot.Arm.ArmEndEffectorState;

public class ArmSoftLimits {

    double minX = Units.inchesToMeters(5);
    double maxX = Units.inchesToMeters(62.75);
    double minY = Units.inchesToMeters(5);
    double maxY = Units.inchesToMeters(72);

    double[] restrictionXPoints = { minX, minX, maxX, maxX };
    double[] restrictionYPoints = { minY, maxY, maxY, minY };

    public ArmSoftLimits() {

        updateTelemetry();
    }

    public void updateTelemetry() {
        // for now - send over the restriction points one-time to telemetry
        var softLimitPoly = new ArrayList<Translation2d>();
        for (int idx = 0; idx < restrictionXPoints.length; idx++) {
            // Pack x/y coordinates into Translation2d's
            var x = restrictionXPoints[idx] + Constants.WHEEL_BASE_HALF_LENGTH_M;
            var y = restrictionYPoints[idx];
            softLimitPoly.add(new Translation2d(x, y));
        }
        // Close back to first point
        var x = restrictionXPoints[0];
        var y = restrictionYPoints[0];
        softLimitPoly.add(new Translation2d(x, y));

        ArmTelemetry.getInstance().setSoftLimits(softLimitPoly);
    }

    public ArmEndEffectorState applyLimit(ArmEndEffectorState in) {

        var out = new ArmEndEffectorState();

        if (in.x > maxX) {
            out.x = maxX;
            out.xvel = 0;
        } else if (in.x < minX) {
            out.x = minX;
            out.xvel = 0;
        } else {
            out.x = in.x;
            out.yvel = in.yvel;
        }

        if (in.y > maxY) {
            out.y = maxY;
            out.yvel = 0;
        } else if (in.y < minY) {
            out.y = minY;
            out.yvel = 0;
        } else {
            out.y = in.y;
            out.yvel = in.yvel;
        }

        out.reflexFrac = in.reflexFrac;

        return out;

    }

}