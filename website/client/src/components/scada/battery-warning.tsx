'use client';
import { useEffect, useState, useCallback } from 'react';
import { rosClient } from '@/lib/ros-client';
import { Zap, X, Home, AlertTriangle } from 'lucide-react';

interface WarningState {
  type: 'warning' | 'critical' | null;
  percent: number;
  voltage?: number;
}

export function BatteryWarning() {
  const [warning, setWarning] = useState<WarningState>({ type: null, percent: 0 });
  const [visible, setVisible] = useState(false);

  const dismiss = useCallback(() => {
    setVisible(false);
    setTimeout(() => setWarning({ type: null, percent: 0 }), 300);
  }, []);

  useEffect(() => {
    const handleWarning = (payload: any) => {
      setWarning({ type: 'warning', percent: payload.percent, voltage: payload.voltage });
      setVisible(true);
      // Auto-dismiss after 10s
      setTimeout(() => {
        setWarning(prev => prev.type === 'warning' ? { type: null, percent: 0 } : prev);
        setVisible(prev => prev);
      }, 10000);
    };

    const handleAutoReturn = (payload: any) => {
      setWarning({ type: 'critical', percent: payload.percent });
      setVisible(true);
      // No auto-dismiss for critical
    };

    rosClient.on('battery_warning', handleWarning);
    rosClient.on('auto_return', handleAutoReturn);

    return () => {
      rosClient.off('battery_warning', handleWarning);
      rosClient.off('auto_return', handleAutoReturn);
    };
  }, []);

  // Auto-dismiss warning after timeout
  useEffect(() => {
    if (warning.type === 'warning') {
      const timer = setTimeout(() => {
        setVisible(false);
        setTimeout(() => setWarning({ type: null, percent: 0 }), 300);
      }, 10000);
      return () => clearTimeout(timer);
    }
  }, [warning.type]);

  if (!warning.type) return null;

  const isCritical = warning.type === 'critical';

  return (
    <div
      className={`
        fixed top-6 left-1/2 -translate-x-1/2 z-[100]
        transition-all duration-300 ease-out
        ${visible ? 'opacity-100 translate-y-0' : 'opacity-0 -translate-y-4'}
      `}
    >
      <div
        className={`
          relative flex items-center gap-4 px-6 py-4 rounded-2xl
          shadow-2xl border backdrop-blur-xl
          min-w-[380px] max-w-[520px]
          ${isCritical
            ? 'bg-red-950/80 border-red-500/40 shadow-red-500/20'
            : 'bg-amber-950/80 border-amber-500/40 shadow-amber-500/20'
          }
        `}
      >
        {/* Animated glow */}
        <div
          className={`
            absolute inset-0 rounded-2xl opacity-20 animate-pulse
            ${isCritical ? 'bg-red-500' : 'bg-amber-500'}
          `}
        />

        {/* Icon */}
        <div
          className={`
            relative z-10 flex items-center justify-center w-12 h-12 rounded-xl
            ${isCritical
              ? 'bg-red-500/20 text-red-400'
              : 'bg-amber-500/20 text-amber-400'
            }
          `}
        >
          {isCritical ? (
            <Home size={24} strokeWidth={2.5} className="animate-pulse" />
          ) : (
            <AlertTriangle size={24} strokeWidth={2.5} />
          )}
        </div>

        {/* Content */}
        <div className="relative z-10 flex-1">
          <div className={`text-sm font-bold tracking-tight ${isCritical ? 'text-red-200' : 'text-amber-200'}`}>
            {isCritical ? 'Pin cực thấp — Tự động về Home' : 'Cảnh báo pin yếu'}
          </div>
          <div className={`text-xs mt-0.5 ${isCritical ? 'text-red-300/80' : 'text-amber-300/80'}`}>
            {isCritical
              ? `Battery ${warning.percent}% — Robot đang tự động điều hướng về trạm sạc`
              : `Battery ${warning.percent}% (${warning.voltage?.toFixed(1)}V) — Vui lòng sạc robot`
            }
          </div>
        </div>

        {/* Battery indicator */}
        <div className="relative z-10 flex items-center gap-2">
          <Zap
            size={16}
            className={`${isCritical ? 'text-red-400 animate-pulse' : 'text-amber-400'}`}
          />
          <span
            className={`text-lg font-mono font-black ${isCritical ? 'text-red-300' : 'text-amber-300'}`}
          >
            {Math.round(warning.percent)}%
          </span>
        </div>

        {/* Close button */}
        {!isCritical && (
          <button
            onClick={dismiss}
            className="relative z-10 p-1.5 rounded-lg hover:bg-white/10 transition-colors text-amber-400/60 hover:text-amber-300"
            aria-label="Dismiss warning"
          >
            <X size={16} />
          </button>
        )}
      </div>
    </div>
  );
}
