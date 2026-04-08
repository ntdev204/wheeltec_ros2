'use client';
import { useEffect } from 'react';
import { rosClient } from '@/lib/ros-client';
import { useRobotState, PatrolStatusPayload, TelemetryData } from './use-robot-state';

export function useROSConnection() {
  const { setIsConnected, setTelemetry, setPatrolStatus } = useRobotState();

  useEffect(() => {
    rosClient.onConnectionChange = setIsConnected;
    rosClient.connect();

    const API_URL = process.env.NEXT_PUBLIC_API_URL || 'http://localhost:8000';
    fetch(`${API_URL}/api/robot/patrol/status`)
      .then((response) => response.json())
      .then((data) => setPatrolStatus(data ?? null))
      .catch(() => {});

    const telemetryHandler = (data: unknown) => {
      if (!data || typeof data !== 'object') {
        return;
      }
      const telemetryPayload = data as Partial<TelemetryData>;
      setTelemetry((prev) => prev ? { ...prev, ...telemetryPayload } : (telemetryPayload as TelemetryData));
    };

    const mapHandler = (data: unknown) => {
      if (!data) {
        return;
      }
      setTelemetry((prev) => prev ? { ...prev, map: data as TelemetryData['map'] } : null);
    };

    // When home is set, immediately inject it into telemetry state
    const homeSetHandler = (data: unknown) => {
      if (!data || typeof data !== 'object') {
        return;
      }
      const home = data as NonNullable<TelemetryData['home_position']>;
      setTelemetry((prev) => prev
        ? { ...prev, home_position: home }
        : ({ home_position: home } as TelemetryData)
      );
    };

    const patrolStatusHandler = (data: unknown) => {
      if (!data || typeof data !== 'object') {
        setPatrolStatus(null);
        return;
      }
      setPatrolStatus(data as PatrolStatusPayload);
    };

    rosClient.on('telemetry', telemetryHandler);
    rosClient.on('map', mapHandler);
    rosClient.on('home_set', homeSetHandler);
    rosClient.on('patrol_status', patrolStatusHandler);

    return () => {
      rosClient.off('telemetry', telemetryHandler);
      rosClient.off('map', mapHandler);
      rosClient.off('home_set', homeSetHandler);
      rosClient.off('patrol_status', patrolStatusHandler);
    };
  }, [setIsConnected, setTelemetry, setPatrolStatus]);
}
