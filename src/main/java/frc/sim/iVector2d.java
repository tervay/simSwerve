package frc.sim;

import edu.wpi.first.wpilibj.drive.Vector2d;

public class iVector2d extends Vector2d {
    public iVector2d(double x, double y) {
        super(x, y);
    }

    /**
     * Returns the cross product of this vector with another.
     *
     * @param other Other vector to cross with
     * @return this X other
     */
    public double cross(Vector2d other) {
        return this.x * other.y - this.y * other.x;
    }
}
