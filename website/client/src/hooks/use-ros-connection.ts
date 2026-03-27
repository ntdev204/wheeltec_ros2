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
      setTelemetry(data);
    };

    rosClient.on('telemetry', telemetryHandler);

    return () => {
      rosClient.off('telemetry', telemetryHandler);
    };
  }, [setIsConnected, setTelemetry]);
}
