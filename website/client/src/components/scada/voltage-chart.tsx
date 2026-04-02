'use client';
import { useVoltageHistory } from '@/hooks/use-dashboard-data';
import { AreaChart, Area, XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer } from 'recharts';
import { Card, CardHeader, CardTitle, CardDescription, CardContent } from '@/components/ui/card';

export function VoltageChart() {
  const history = useVoltageHistory();

  return (
    <Card className="w-full h-[400px] flex flex-col">
      <CardHeader className="pb-2">
        <CardTitle>Voltage Timeline</CardTitle>
        <CardDescription>24-hour power supply history</CardDescription>
      </CardHeader>
      <CardContent className="flex-1 w-full relative pl-0">
        <ResponsiveContainer width="100%" height="100%">
          <AreaChart data={history} margin={{ top: 5, right: 10, left: 0, bottom: 5 }}>
            <defs>
              <linearGradient id="colorVoltage" x1="0" y1="0" x2="0" y2="1">
                <stop offset="5%" stopColor="var(--status-orange)" stopOpacity={0.3}/>
                <stop offset="95%" stopColor="var(--status-orange)" stopOpacity={0}/>
              </linearGradient>
            </defs>
            <CartesianGrid strokeDasharray="4 4" stroke="hsl(var(--border))" vertical={false} />
            <XAxis 
              dataKey="time_label" 
              stroke="hsl(var(--muted-foreground))" 
              fontSize={10} 
              tickMargin={12} 
              tickLine={false} 
              axisLine={false} 
              fontFamily="monospace"
              minTickGap={30}
            />
            <YAxis 
              stroke="hsl(var(--muted-foreground))" 
              fontSize={10} 
              domain={['dataMin - 0.5', 'dataMax + 0.5']} 
              tickLine={false} 
              axisLine={false} 
              tickFormatter={(v) => v.toFixed(1)} 
              fontFamily="monospace" 
            />
            <Tooltip 
              contentStyle={{ backgroundColor: 'hsl(var(--card))', borderColor: 'hsl(var(--border))', borderRadius: '6px', fontSize: '12px' }}
              itemStyle={{ fontFamily: 'monospace', fontWeight: 'bold' }}
              labelStyle={{ color: 'hsl(var(--muted-foreground))', fontWeight: 'bold', fontSize: '10px', textTransform: 'uppercase', letterSpacing: '0.1em' }}
            />
            <Area 
              type="monotone" 
              dataKey="voltage" 
              stroke="var(--status-orange)" 
              fillOpacity={1} 
              fill="url(#colorVoltage)" 
              isAnimationActive={false}
            />
          </AreaChart>
        </ResponsiveContainer>
      </CardContent>
    </Card>
  );
}
