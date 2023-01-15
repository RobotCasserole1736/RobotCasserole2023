package frc.robot.Arm;

public class ArmPosition {
    double x;
    double y;

    ArmPosition(double x, double y){
        this.x = x;
        this.y = y;
    }

    double distTo(ArmPosition other){
        double x2 = Math.pow(this.x - other.x, 2);
        double y2 = Math.pow(this.y - other.y, 2);
        return Math.sqrt(x2 + y2);
    }

    ArmPosition interpolateTo(ArmPosition other, double frac){
        double fracInv = (1.0 - frac);
        double x = other.x * frac + this.x * fracInv;
        double y = other.y * frac + this.y * fracInv;
        return new ArmPosition(x, y);
    }
}
