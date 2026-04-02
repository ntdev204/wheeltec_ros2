'use client';
import { useRobotState } from '@/hooks/use-robot-state';
import { Card, CardContent, CardHeader, CardTitle, CardDescription } from '@/components/ui/card';

export function RobotStatus() {
  const { telemetry } = useRobotState();

  if (!telemetry) {
    return (
      <Card>
        <CardHeader>
          <CardTitle>Pose & Kinematics</CardTitle>
          <CardDescription>Waiting for telemetry data...</CardDescription>
        </CardHeader>
      </Card>
    );
  }

  const mapPose = telemetry.map_pose || { x: 0, y: 0, yaw: 0 };
  const v_mag = Math.sqrt(telemetry.odom.v_x**2 + telemetry.odom.v_y**2);

  return (
    <Card>
      <CardHeader className="pb-4">
        <CardTitle className="text-lg">Pose & Kinematics</CardTitle>
        <CardDescription>Real-time translation and rotation.</CardDescription>
      </CardHeader>
      <CardContent className="flex-1 flex flex-col justify-center">
        <div className="grid grid-cols-2 gap-8">
          <div className="flex flex-col gap-3 font-mono text-[13px] leading-relaxed">
            <div className="flex justify-between border-b border-border/50 pb-2"><span className="text-muted-foreground font-semibold uppercase tracking-wider">Map X</span> <span className="font-bold text-foreground">{mapPose.x.toFixed(2)} m</span></div>
            <div className="flex justify-between border-b border-border/50 pb-2"><span className="text-muted-foreground font-semibold uppercase tracking-wider">Map Y</span> <span className="font-bold text-foreground">{mapPose.y.toFixed(2)} m</span></div>
            <div className="flex justify-between border-b border-border/50 pb-2"><span className="text-muted-foreground font-semibold uppercase tracking-wider">Heading</span> <span className="font-bold text-status-blue">{(mapPose.yaw * 180 / Math.PI).toFixed(0)}°</span></div>
            <div className="flex justify-between"><span className="text-muted-foreground font-semibold uppercase tracking-wider">Vel Mag</span> <span className="font-bold text-primary">{v_mag.toFixed(3)} m/s</span></div>
          </div>

          <div className="flex flex-col gap-3 font-mono text-[13px] leading-relaxed">
            <div className="flex justify-between border-b border-border/50 pb-2"><span className="text-muted-foreground font-semibold uppercase tracking-wider">Accel X</span> <span className="font-bold text-foreground">{telemetry.imu.ax.toFixed(2)}</span></div>
            <div className="flex justify-between border-b border-border/50 pb-2"><span className="text-muted-foreground font-semibold uppercase tracking-wider">Accel Y</span> <span className="font-bold text-foreground">{telemetry.imu.ay.toFixed(2)}</span></div>
            <div className="flex justify-between"><span className="text-muted-foreground font-semibold uppercase tracking-wider">Rot Z</span> <span className="font-bold text-foreground">{telemetry.odom.v_z.toFixed(2)} r/s</span></div>
          </div>
        </div>
      </CardContent>
    </Card>
  );
}
