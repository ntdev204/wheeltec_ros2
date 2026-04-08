import { AIStreamFeed } from '@/components/ai/ai-stream-feed';
import { Card, CardHeader, CardTitle, CardContent } from '@/components/ui/card';

export default function AIDetectionPage() {
  return (
    <div className="container mx-auto p-6 space-y-6">
      <div>
        <h1 className="text-3xl font-bold">AI Object Detection</h1>
        <p className="text-muted-foreground mt-2">
          Real-time object detection using YOLOv8m with all detected objects annotated
        </p>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
        <div className="lg:col-span-2">
          <AIStreamFeed
            port={5558}
            title="AI Detection Stream"
            description="YOLOv8m - All Objects"
            streamType="detection"
          />
        </div>

        <div className="space-y-4">
          <Card>
            <CardHeader>
              <CardTitle>Detection Stats</CardTitle>
            </CardHeader>
            <CardContent>
              <div className="space-y-3">
                <div className="flex justify-between">
                  <span className="text-sm text-muted-foreground">FPS</span>
                  <span className="text-sm font-mono font-bold">--</span>
                </div>
                <div className="flex justify-between">
                  <span className="text-sm text-muted-foreground">Latency</span>
                  <span className="text-sm font-mono font-bold">-- ms</span>
                </div>
                <div className="flex justify-between">
                  <span className="text-sm text-muted-foreground">Objects</span>
                  <span className="text-sm font-mono font-bold">--</span>
                </div>
              </div>
            </CardContent>
          </Card>

          <Card>
            <CardHeader>
              <CardTitle>Detected Classes</CardTitle>
            </CardHeader>
            <CardContent>
              <div className="text-sm text-muted-foreground">
                No detections yet
              </div>
            </CardContent>
          </Card>
        </div>
      </div>
    </div>
  );
}
