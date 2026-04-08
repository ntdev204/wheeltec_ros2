'use client';

import { useEffect, useState, useRef } from 'react';
import { Card, CardHeader, CardTitle, CardDescription, CardContent } from '@/components/ui/card';
import { Badge } from '@/components/ui/badge';

interface AIStreamFeedProps {
  port: number;
  title: string;
  description: string;
  streamType: 'detection' | 'tracking';
}

export function AIStreamFeed({ port, title, description, streamType }: AIStreamFeedProps) {
  const [blobUrl, setBlobUrl] = useState<string | null>(null);
  const [isReceiving, setIsReceiving] = useState(false);
  const [fps, setFps] = useState(0);
  const timeoutRef = useRef<NodeJS.Timeout | null>(null);
  const wsRef = useRef<WebSocket | null>(null);
  const frameCountRef = useRef(0);
  const lastFpsTimeRef = useRef(Date.now());

  useEffect(() => {
    // Connect to WebSocket that forwards ZMQ stream
    const ws = new WebSocket(`ws://localhost:8000/ws/ai/${streamType}`);
    wsRef.current = ws;

    ws.onopen = () => {
      console.log(`Connected to ${streamType} stream`);
    };

    ws.onmessage = (event) => {
      if (event.data instanceof Blob) {
        setBlobUrl((prev) => {
          if (prev) URL.revokeObjectURL(prev);
          return URL.createObjectURL(event.data);
        });

        setIsReceiving(true);
        if (timeoutRef.current) clearTimeout(timeoutRef.current);
        timeoutRef.current = setTimeout(() => setIsReceiving(false), 2000);

        // Calculate FPS
        frameCountRef.current++;
        const now = Date.now();
        const elapsed = (now - lastFpsTimeRef.current) / 1000;
        if (elapsed >= 1.0) {
          setFps(Math.round(frameCountRef.current / elapsed));
          frameCountRef.current = 0;
          lastFpsTimeRef.current = now;
        }
      }
    };

    ws.onerror = (error) => {
      console.error(`${streamType} stream error:`, error);
    };

    ws.onclose = () => {
      console.log(`${streamType} stream disconnected`);
    };

    return () => {
      if (wsRef.current) {
        wsRef.current.close();
      }
      if (timeoutRef.current) {
        clearTimeout(timeoutRef.current);
      }
      if (blobUrl) {
        URL.revokeObjectURL(blobUrl);
      }
    };
  }, [port, streamType]);

  return (
    <Card className="w-full overflow-hidden">
      <CardHeader>
        <div className="flex items-center justify-between">
          <div>
            <CardTitle>{title}</CardTitle>
            <CardDescription>{description}</CardDescription>
          </div>
          <div className="flex gap-2">
            <Badge variant={isReceiving ? 'default' : 'secondary'} className="font-mono text-[10px] uppercase">
              {isReceiving ? 'Live' : 'Offline'}
            </Badge>
            {fps > 0 && (
              <Badge variant="outline" className="font-mono text-[10px]">
                {fps} FPS
              </Badge>
            )}
          </div>
        </div>
      </CardHeader>

      <CardContent className="p-0 border-b border-border aspect-video bg-muted/20 relative flex items-center justify-center">
        {blobUrl ? (
          <img
            src={blobUrl}
            alt={`${title} Stream`}
            className="w-full h-full object-contain pointer-events-none bg-black/5"
          />
        ) : (
          <div className="flex flex-col items-center justify-center gap-2 text-center">
            <span className="text-sm font-bold text-muted-foreground">Stream Offline</span>
            <span className="text-[10px] uppercase text-muted-foreground/60 tracking-widest font-mono">
              Waiting for connection...
            </span>
          </div>
        )}
      </CardContent>
    </Card>
  );
}
