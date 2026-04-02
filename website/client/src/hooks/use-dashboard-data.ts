'use client';

import { useState, useEffect } from 'react';

export function useDashboardData() {
  const [data, setData] = useState<any>(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    const fetchSummary = async () => {
      try {
        const res = await fetch('/api/analytics/summary');
        if (res.ok) {
          const json = await res.json();
          setData(json);
        }
      } catch (e) {
        console.error('Failed to fetch dashboard summary', e);
      } finally {
        setLoading(false);
      }
    };

    fetchSummary();
    const interval = setInterval(fetchSummary, 10000); // 10s poll

    return () => clearInterval(interval);
  }, []);

  return { data, loading };
}

export function useVoltageHistory() {
  const [history, setHistory] = useState<any[]>([]);

  useEffect(() => {
    const fetchHistory = async () => {
      try {
        const res = await fetch('/api/analytics/voltage-history?hours=24');
        if (res.ok) {
          const json = await res.json();
          setHistory(json.history || []);
        }
      } catch (e) {
        console.error('Failed to fetch voltage history', e);
      }
    };

    fetchHistory();
    const interval = setInterval(fetchHistory, 30000); // 30s poll

    return () => clearInterval(interval);
  }, []);

  return history;
}

export function useEventLogs(category?: string, severity?: string) {
  const [logs, setLogs] = useState<any[]>([]);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    const fetchLogs = async () => {
      try {
        let url = '/api/logs/latest?n=50';
        if (category || severity) {
            url = `/api/logs?limit=50&category=${category || ''}&severity=${severity || ''}`;
        }
        
        const res = await fetch(url);
        if (res.ok) {
          const json = await res.json();
          setLogs(json.logs || []);
        }
      } catch (e) {
        console.error('Failed to fetch logs', e);
      } finally {
        setLoading(false);
      }
    };

    fetchLogs();
    const interval = setInterval(fetchLogs, 3000); // 3s poll

    return () => clearInterval(interval);
  }, [category, severity]);

  return { logs, loading };
}
