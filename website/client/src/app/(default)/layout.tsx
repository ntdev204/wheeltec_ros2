import { Sidebar } from '@/components/layout/sidebar';
import { Header } from '@/components/layout/header';
import { BatteryWarning } from '@/components/scada/battery-warning';

export default function DefaultLayout({ children }: { children: React.ReactNode }) {
  return (
    <div className="flex h-screen w-full bg-background text-foreground overflow-hidden font-sans">
      <Sidebar />
      <div className="flex-1 flex flex-col h-full relative overflow-hidden bg-muted/20">
        <Header />
        <main className="flex-1 overflow-auto relative">
          <BatteryWarning />
          <div className="px-8 py-4 lg:px-10 lg:py-6 w-full max-w-[1400px] mx-auto min-h-full pb-20">
            {children}
          </div>
        </main>
      </div>
    </div>
  );
}
