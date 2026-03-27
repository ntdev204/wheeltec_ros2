'use client';
import { useEffect, useState, useRef } from 'react';
import { rosClient } from '@/lib/ros-client';
import { Card, CardHeader, CardTitle, CardDescription, CardAction, CardContent, CardFooter } from '@/components/ui/card';
import { Badge } from '@/components/ui/badge';

export function CameraFeed() {
  const [blobUrl, setBlobUrl] = useState<string | null>(null);
  const [isReceiving, setIsReceiving] = useState(false);
  const timeoutRef = useRef<NodeJS.Timeout | null>(null);

  useEffect(() => {
    const handleFrame = (blobData: Blob) => {
      setBlobUrl((prev) => {
        if (prev) URL.revokeObjectURL(prev);
        return URL.createObjectURL(blobData);
      });
      
      setIsReceiving(true);
      if (timeoutRef.current) clearTimeout(timeoutRef.current);
      timeoutRef.current = setTimeout(() => setIsReceiving(false), 2000);
    };

    rosClient.onBinary(handleFrame);

    return () => {
      rosClient.offBinary(handleFrame);
      if (timeoutRef.current) clearTimeout(timeoutRef.current);
    };
  }, []);

  return (
    <Card className="w-full overflow-hidden">
      <CardHeader>
        <CardTitle>Video Stream</CardTitle>
        <CardDescription>/camera/raw — ZMQ+JPEG</CardDescription>
        <CardAction>
          <Badge variant={isReceiving ? 'default' : 'secondary'} className="font-mono text-[10px] uppercase">
            {isReceiving ? 'Live' : 'Pending'}
          </Badge>
        </CardAction>
      </CardHeader>

      <CardContent className="p-0 border-b border-border aspect-video bg-muted/20 relative flex items-center justify-center">
        {blobUrl ? (
          // eslint-disable-next-line @next/next/no-img-element
          <img 
            src={blobUrl} 
            alt="Robot Camera Stream" 
            width={640}
            height={480}
            className="w-full h-full object-contain pointer-events-none bg-black/5"
          />
        ) : (
          <div className="flex flex-col items-center justify-center gap-2 text-center">
            <span className="text-sm font-bold text-muted-foreground">Video Stream Offline</span>
            <span className="text-[10px] uppercase text-muted-foreground/60 tracking-widest font-mono">WebSocket Link Pending…</span>
          </div>
        )}
      </CardContent>

      <CardFooter className="justify-between">
        <div className="flex flex-col">
          <span className="text-[10px] font-bold tracking-widest uppercase text-muted-foreground">Resolution</span>
          <span className="text-xs font-mono font-bold text-foreground mt-0.5 tabular-nums">640×480</span>
        </div>
        <div className="flex flex-col text-right">
          <span className="text-[10px] font-bold tracking-widest uppercase text-muted-foreground">Transport</span>
          <span className="text-xs font-mono font-bold text-foreground mt-0.5">ZMQ Binary</span>
        </div>
      </CardFooter>
    </Card>
  );
}
