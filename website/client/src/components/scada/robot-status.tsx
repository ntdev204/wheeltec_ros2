'use client';
import { useRobotState } from '@/hooks/use-robot-state';
import { Card, CardContent, CardHeader, CardTitle, CardDescription } from '@/components/ui/card';

export function RobotStatus() {
  const { telemetry } = useRobotState();

  const mockTelemetry = {
    odom: { x: 0.0, y: 0.0, z: 0.0, v_x: 0.0, v_y: 0.0, v_z: 0.0 },
    imu: { ax: 0.0, ay: 0.0, az: 0.0 },
    voltage: 12.4,
    charging: false
  };

  const data = telemetry || mockTelemetry;

  return (
    <Card>
      <CardHeader>
        <CardTitle>Odometry / IMU</CardTitle>
        <CardDescription>Real-time pose and acceleration data.</CardDescription>
      </CardHeader>
      <CardContent className="flex-1 flex flex-col justify-center">
        <div className="grid grid-cols-2 gap-8">
          <div className="flex flex-col gap-3 font-mono text-[13px] leading-relaxed">
            <div className="flex justify-between border-b border-border/50 pb-2"><span className="text-muted-foreground font-semibold uppercase tracking-wider">Pose X</span> <span className="font-bold text-foreground">{data.odom.x.toFixed(3)}</span></div>
            <div className="flex justify-between border-b border-border/50 pb-2"><span className="text-muted-foreground font-semibold uppercase tracking-wider">Pose Y</span> <span className="font-bold text-foreground">{data.odom.y.toFixed(3)}</span></div>
            <div className="flex justify-between border-b border-border/50 pb-2"><span className="text-muted-foreground font-semibold uppercase tracking-wider">Vel X</span> <span className="font-bold text-primary">{data.odom.v_x.toFixed(3)}</span></div>
            <div className="flex justify-between"><span className="text-muted-foreground font-semibold uppercase tracking-wider">Vel Z</span> <span className="font-bold text-primary">{data.odom.v_z.toFixed(3)}</span></div>
          </div>

          <div className="flex flex-col gap-3 font-mono text-[13px] leading-relaxed">
            <div className="flex justify-between border-b border-border/50 pb-2"><span className="text-muted-foreground font-semibold uppercase tracking-wider">Accel X</span> <span className="font-bold text-foreground">{data.imu.ax.toFixed(3)}</span></div>
            <div className="flex justify-between border-b border-border/50 pb-2"><span className="text-muted-foreground font-semibold uppercase tracking-wider">Accel Y</span> <span className="font-bold text-foreground">{data.imu.ay.toFixed(3)}</span></div>
            <div className="flex justify-between"><span className="text-muted-foreground font-semibold uppercase tracking-wider">Accel Z</span> <span className="font-bold text-foreground">{data.imu.az.toFixed(3)}</span></div>
          </div>
        </div>
      </CardContent>
    </Card>
  );
}
