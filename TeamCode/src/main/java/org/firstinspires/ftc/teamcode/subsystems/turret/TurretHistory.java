package org.firstinspires.ftc.teamcode.subsystems.turret;

public class TurretHistory {
    public static final int SIZE = 64;

    private final long[] times = new long[SIZE];
    private final double[] turretAngles = new double[SIZE];
    private final double[] xs = new double[SIZE];
    private final double[] ys = new double[SIZE];
    private final double[] headings = new double[SIZE];
    private int count = 0;
    private int next = 0;

    public static class Sample {
        public final long nanos;
        public final double turretAngle;
        public final double x;
        public final double y;
        public final double heading;

        public Sample(long nanos, double turretAngle, double x, double y, double heading) {
            this.nanos = nanos;
            this.turretAngle = turretAngle;
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }

    public void add(long nanos, double turretAngle, double x, double y, double heading) {
        times[next] = nanos;
        turretAngles[next] = turretAngle;
        xs[next] = x;
        ys[next] = y;
        headings[next] = heading;
        next = (next + 1) % SIZE;
        count = Math.min(count + 1, SIZE);
    }

    public Sample at(long nanos) {
        if (count == 0) return null;
        int newest = (next - 1 + SIZE) % SIZE;
        if (nanos >= times[newest]) return sample(newest);

        int later = newest;
        for (int i = 1; i < count; i++) {
            int earlier = (newest - i + SIZE) % SIZE;
            if (times[earlier] <= nanos) {
                double t = (double) (nanos - times[earlier]) / (times[later] - times[earlier]);
                return new Sample(
                        nanos,
                        lerp(turretAngles[earlier], turretAngles[later], t),
                        lerp(xs[earlier], xs[later], t),
                        lerp(ys[earlier], ys[later], t),
                        headings[earlier] + t * Turret.wrap(headings[later] - headings[earlier])
                );
            }
            later = earlier;
        }
        return sample(later);
    }

    private Sample sample(int i) {
        return new Sample(times[i], turretAngles[i], xs[i], ys[i], headings[i]);
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }
}
