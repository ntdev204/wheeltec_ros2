import { JoystickPad } from '@/components/control/joystick-pad';
import { SpeedControl } from '@/components/control/speed-control';
import { EmergencyStop } from '@/components/control/emergency-stop';
import { CameraFeed } from '@/components/camera/camera-feed';
import { AIStreamFeed } from '@/components/ai/ai-stream-feed';
import { Card, CardContent, CardHeader, CardTitle, CardDescription } from '@/components/ui/card';
import { Badge } from '@/components/ui/badge';

export default function HomePage() {
  return (
    <div className="flex flex-col gap-10 min-h-full">

      {/* Page Header */}
      <div className="flex flex-col md:flex-row md:items-center justify-between pb-6 border-b border-border gap-4">
        <div className="flex flex-col">
          <h1 className="text-3xl font-bold tracking-tight text-foreground mb-1">Operation Center</h1>
          <p className="text-sm text-muted-foreground">Active Teleoperation Link in Session.</p>
        </div>
        <Badge variant="outline" className="w-fit gap-2 text-[11px] font-bold font-mono tracking-widest text-primary uppercase px-4 py-2 bg-primary/10 border-primary/20">
          <span className="w-1.5 h-1.5 bg-primary rounded-full animate-pulse" />
          Hotkeys Active
        </Badge>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-12 gap-10">
        {/* Left Column: Visuals */}
        <div className="lg:col-span-8 flex flex-col gap-10">
          {/* Dual Video Streams */}
          <div className="grid grid-cols-1 xl:grid-cols-2 gap-6">
            <CameraFeed />
            <AIStreamFeed
              port={5558}
              title="AI Detection"
              description="YOLOv8s Real-time Detection"
              streamType="detection"
            />
          </div>

          {/* Driver Control hint */}
          <Card>
            <CardHeader>
              <CardTitle>Driver Control</CardTitle>
              <CardDescription>Focus within the app to use WASD keys for teleoperation.</CardDescription>
            </CardHeader>
          </Card>
        </div>

        {/* Right Column: Controls Sidebar */}
        <div className="lg:col-span-4 flex justify-center lg:justify-end">
          <div className="w-full max-w-[340px] flex flex-col gap-8">
            <EmergencyStop />
            <JoystickPad />
            <SpeedControl />
          </div>
        </div>
      </div>
    </div>
  );
}
