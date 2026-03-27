import { OdomChart } from '@/components/scada/odom-chart';
import { RobotStatus } from '@/components/scada/robot-status';
import { SystemHealth } from '@/components/scada/system-health';
import { Card, CardHeader, CardTitle, CardDescription, CardContent } from '@/components/ui/card';
import { Badge } from '@/components/ui/badge';
import { Database } from 'lucide-react';

export default function DashboardPage() {
  return (
    <div className="flex flex-col gap-10 min-h-full">

      {/* Page Header */}
      <div className="flex flex-col md:flex-row md:items-center justify-between pb-6 border-b border-border gap-4">
        <div className="flex flex-col">
          <h1 className="text-3xl font-bold tracking-tight text-foreground mb-1">Data Analytics</h1>
          <p className="text-sm text-muted-foreground">Telemetry logs and system health diagnostics.</p>
        </div>
        <Badge variant="outline" className="w-fit gap-2 text-[11px] font-bold font-mono tracking-widest text-status-green uppercase px-4 py-2 bg-status-green-bg border-status-green/20">
          <span className="w-1.5 h-1.5 bg-status-green rounded-full animate-pulse" />
          Live DB Sync
        </Badge>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-12 gap-10">
        <div className="lg:col-span-8 flex flex-col gap-8">
          <OdomChart />

          <div className="grid grid-cols-1 md:grid-cols-2 gap-8">
            <RobotStatus />

            {/* Activity Logs Placeholder */}
            <Card className="border-dashed opacity-80">
              <CardHeader>
                <CardTitle>Activity Logs</CardTitle>
                <CardDescription>SQLite persistence layer.</CardDescription>
              </CardHeader>
              <CardContent className="flex flex-col items-center justify-center gap-3 py-8">
                <div className="w-12 h-12 bg-muted rounded-md flex items-center justify-center text-muted-foreground">
                  <Database size={24} />
                </div>
                <span className="text-sm font-bold text-foreground">SQLite Database</span>
                <span className="text-xs text-muted-foreground">Pending implementation.</span>
              </CardContent>
            </Card>
          </div>
        </div>

        <div className="lg:col-span-4 flex flex-col gap-8">
          <SystemHealth />

          {/* ROS2 Node Metadata */}
          <Card>
            <CardHeader>
              <CardTitle>Node Details</CardTitle>
              <CardDescription>ROS2 middleware configuration.</CardDescription>
            </CardHeader>
            <CardContent className="flex flex-col gap-3">
              {[
                { label: 'Domain ID', value: '0' },
                { label: 'Middleware', value: 'FastRTPS' },
                { label: 'Frequency', value: '50 Hz' },
              ].map(({ label, value }) => (
                <div key={label} className="flex justify-between items-center pb-3 border-b border-border/50 last:border-0 last:pb-0">
                  <span className="text-sm text-muted-foreground">{label}</span>
                  <span className="font-mono font-bold text-foreground">{value}</span>
                </div>
              ))}
            </CardContent>
          </Card>
        </div>
      </div>
    </div>
  );
}
