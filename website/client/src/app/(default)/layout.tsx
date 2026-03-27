import { Sidebar } from '@/components/layout/sidebar';
import { Header } from '@/components/layout/header';

export default function DefaultLayout({ children }: { children: React.ReactNode }) {
  return (
    <div className="flex h-screen w-full bg-background text-foreground overflow-hidden font-sans">
      <Sidebar />
      <div className="flex-1 flex flex-col h-full relative overflow-hidden bg-muted/20">
        <Header />
        <main className="flex-1 overflow-auto relative">
          <div className="p-8 lg:p-10 w-full max-w-[1400px] mx-auto min-h-full pb-20">
            {children}
          </div>
        </main>
      </div>
    </div>
  );
}
