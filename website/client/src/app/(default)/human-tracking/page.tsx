import { AIStreamFeed } from '@/components/ai/ai-stream-feed';
import { Card, CardHeader, CardTitle, CardContent } from '@/components/ui/card';

export default function HumanTrackingPage() {
  return (
    <div className="container mx-auto p-6 space-y-6">
      <div>
        <h1 className="text-3xl font-bold">Human Tracking</h1>
        <p className="text-muted-foreground mt-2">
          Real-time human tracking using ByteTrack with persistent track IDs
        </p>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
        <div className="lg:col-span-2">
          <AIStreamFeed
            port={5559}
            title="Human Tracking Stream"
            description="ByteTrack - Humans Only"
            streamType="tracking"
          />
        </div>

        <div className="space-y-4">
          <Card>
            <CardHeader>
              <CardTitle>Tracking Stats</CardTitle>
            </CardHeader>
            <CardContent>
              <div className="space-y-3">
                <div className="flex justify-between">
                  <span className="text-sm text-muted-foreground">Active Tracks</span>
                  <span className="text-sm font-mono font-bold">--</span>
                </div>
                <div className="flex justify-between">
                  <span className="text-sm text-muted-foreground">Total Tracked</span>
                  <span className="text-sm font-mono font-bold">--</span>
                </div>
                <div className="flex justify-between">
                  <span className="text-sm text-muted-foreground">Avg Duration</span>
                  <span className="text-sm font-mono font-bold">-- s</span>
                </div>
              </div>
            </CardContent>
          </Card>

          <Card>
            <CardHeader>
              <CardTitle>Active Tracks</CardTitle>
            </CardHeader>
            <CardContent>
              <div className="text-sm text-muted-foreground">
                No active tracks
              </div>
            </CardContent>
          </Card>
        </div>
      </div>
    </div>
  );
}
