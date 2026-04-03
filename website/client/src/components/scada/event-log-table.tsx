'use client';
import { useState, useRef, useEffect } from 'react';
import { useEventLogs } from '@/hooks/use-dashboard-data';
import { Card, CardHeader, CardTitle, CardContent } from '@/components/ui/card';
import { Badge } from '@/components/ui/badge';
import { format } from 'date-fns';

export function EventLogTable() {
  const [filterCat, setFilterCat] = useState<string>('');
  const [filterSev, setFilterSev] = useState<string>('');
  
  const { logs, loading } = useEventLogs(filterCat, filterSev);
  
  // Live tail scroll
  const scrollRef = useRef<HTMLDivElement>(null);
  
  const categories = ['NAVIGATION', 'POWER', 'COMMAND', 'SYSTEM'];
  
  const getCatColor = (cat: string) => {
    switch(cat) {
      case 'NAVIGATION': return 'text-status-blue border-status-blue/20 bg-status-blue/10';
      case 'POWER': return 'text-status-orange border-status-orange/20 bg-status-orange/10';
      case 'COMMAND': return 'text-status-green border-status-green/20 bg-status-green/10';
      default: return 'text-muted-foreground border-border bg-muted';
    }
  };

  const getSevColor = (sev: string) => {
    switch(sev) {
      case 'INFO': return 'bg-status-blue text-white';
      case 'WARNING': return 'bg-status-orange text-white';
      case 'ERROR': 
      case 'CRITICAL': return 'bg-destructive text-destructive-foreground';
      default: return 'bg-muted-foreground text-white';
    }
  };

  return (
    <Card className="flex flex-col flex-1 h-full min-h-[500px]">
      <CardHeader className="py-4 border-b border-border flex flex-row items-center justify-between shrink-0">
        <CardTitle className="text-lg">Event Logs</CardTitle>
        <div className="flex items-center gap-2">
          {categories.map(c => (
            <Badge 
              key={c}
              variant={filterCat === c ? 'default' : 'outline'}
              className="cursor-pointer text-[10px] uppercase font-mono shadow-none"
              onClick={() => setFilterCat(filterCat === c ? '' : c)}
            >
              {c}
            </Badge>
          ))}
        </div>
      </CardHeader>
      
      <CardContent className="p-0 overflow-y-auto flex-1 font-mono text-[12px] bg-muted/10 leading-relaxed" ref={scrollRef}>
        <table className="w-full text-left">
          <thead className="sticky top-0 bg-card border-b border-border shadow-sm z-10 text-[10px] uppercase tracking-widest text-muted-foreground">
            <tr>
              <th className="font-bold py-3 px-4 font-sans">Time</th>
              <th className="font-bold py-3 px-4 font-sans">Category</th>
              <th className="font-bold py-3 px-4 font-sans">Event</th>
              <th className="font-bold py-3 px-4 font-sans w-full">Message</th>
            </tr>
          </thead>
          <tbody className="divide-y divide-border/50">
            {logs.length === 0 && !loading && (
              <tr><td colSpan={4} className="text-center py-8 text-muted-foreground font-sans">No events recorded.</td></tr>
            )}
            
            {logs.map((log) => {
              const timeStr = format(new Date(log.timestamp), 'HH:mm:ss');
              return (
                <tr key={log.id} className="hover:bg-muted/50 transition-colors">
                  <td className="py-2.5 px-4 whitespace-nowrap text-muted-foreground">
                    <div className="flex items-center gap-2">
                      <span className={`w-2 h-2 rounded-full ${getSevColor(log.severity)}`} title={log.severity} />
                      {timeStr}
                    </div>
                  </td>
                  <td className="py-2.5 px-4">
                    <span className={`px-2 py-0.5 rounded-sm border text-[10px] uppercase tracking-wider ${getCatColor(log.category)}`}>
                      {log.category}
                    </span>
                  </td>
                  <td className="py-2.5 px-4 font-bold text-foreground">
                    {log.event_type}
                  </td>
                  <td className="py-2.5 px-4 text-muted-foreground max-w-[200px] truncate" title={log.message}>
                    {log.message}
                  </td>
                </tr>
              );
            })}
          </tbody>
        </table>
      </CardContent>
    </Card>
  );
}
