'use client';
import { OdomChart } from '@/components/scada/odom-chart';
import { RobotStatus } from '@/components/scada/robot-status';
import { SystemHealth } from '@/components/scada/system-health';
import { KpiBar } from '@/components/scada/kpi-bar';
import { VoltageChart } from '@/components/scada/voltage-chart';
import { EventLogTable } from '@/components/scada/event-log-table';
import { SessionInfo } from '@/components/scada/session-info';
import { Badge } from '@/components/ui/badge';
import { useRobotState } from '@/hooks/use-robot-state';
import { useDashboardData } from '@/hooks/use-dashboard-data';

export default function DashboardPage() {
  const { telemetry } = useRobotState();
  const { data, loading } = useDashboardData();
  
  const sessionData = data?.active_session;
  const logStats = data?.log_stats;

  return (
    <div className="flex flex-col gap-8 min-h-full">
      {/* Page Header */}
      <div className="flex flex-col md:flex-row md:items-center justify-between pb-4 border-b border-border gap-4">
        <div className="flex flex-col">
          <h1 className="text-3xl font-bold tracking-tight text-foreground mb-1 uppercase">SCADA Monitor</h1>
          <p className="text-sm text-muted-foreground">Industrial Telemetry & Event Logging</p>
        </div>
        <Badge variant="outline" className={`w-fit gap-2 text-[11px] font-bold font-mono tracking-widest uppercase px-4 py-2 ${
          loading ? 'text-muted-foreground border-border bg-muted' : 'text-status-green bg-status-green-bg border-status-green/20'
        }`}>
          <span className={`w-1.5 h-1.5 rounded-full ${loading ? 'bg-muted-foreground' : 'bg-status-green animate-pulse'}`} />
          {loading ? 'SYNCING DB...' : 'DB SYNC: OK'}
        </Badge>
      </div>

      <div className="flex flex-col gap-6">
        {/* Row 1: KPI Bar */}
        <KpiBar telemetry={telemetry} sessionData={sessionData} />
        
        {/* Main Content Area */}
        <div className="grid grid-cols-1 lg:grid-cols-3 gap-6 items-stretch">
          {/* Left Column: Charts and Logs */}
          <div className="lg:col-span-2 flex flex-col gap-6 h-full">
            <div className="grid grid-cols-1 xl:grid-cols-2 gap-6">
               <OdomChart />
               <VoltageChart />
            </div>
            
            <EventLogTable />
          </div>

          {/* Right Column: Status Cards */}
          <div className="lg:col-span-1 flex flex-col gap-6 h-full">
            <SessionInfo sessionData={sessionData} logStats={logStats} />
            <RobotStatus />
            <SystemHealth />
          </div>
        </div>
      </div>
    </div>
  );
}
