'use client';
import { useEffect } from 'react';
import { rosClient } from '@/lib/ros-client';
import { useRobotState } from './use-robot-state';

export function useROSConnection() {
  const { setIsConnected, setTelemetry } = useRobotState();

  useEffect(() => {
    rosClient.onConnectionChange = setIsConnected;
    rosClient.connect();

    const telemetryHandler = (data: any) => {
      // Merge: preserve map field from separate map channel
      setTelemetry((prev) => prev ? { ...prev, ...data } : data);
    };

    const mapHandler = (data: any) => {
      setTelemetry((prev) => prev ? { ...prev, map: data } : null);
    };

    rosClient.on('telemetry', telemetryHandler);
    rosClient.on('map', mapHandler);

    return () => {
      rosClient.off('telemetry', telemetryHandler);
      rosClient.off('map', mapHandler);
    };
  }, [setIsConnected, setTelemetry]);
}
