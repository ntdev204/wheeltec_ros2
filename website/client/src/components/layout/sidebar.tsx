'use client';
import Link from 'next/link';
import { usePathname } from 'next/navigation';
import { Layers } from 'lucide-react';

export function Sidebar() {
  const pathname = usePathname();

  const links = [
    { href: '/', label: 'Omni Control' },
    { href: '/dashboard', label: 'Analytics' },
    { href: '/dashboard/slam', label: 'Slam Engine' },
  ];

  return (
    <div className="w-64 h-full bg-card text-foreground flex flex-col shrink-0 border-r border-border z-20">
      <div className="h-[72px] flex items-center justify-center border-b border-border gap-2">
        <Layers className="text-primary w-5 h-5" aria-hidden="true" />
        <span className="text-sm font-black tracking-[0.2em] uppercase text-foreground">Scada UI</span>
      </div>
      <nav className="flex-1 px-4 py-8 flex flex-col gap-1" aria-label="Main navigation">
        {links.map(({ href, label }) => {
          const isActive = pathname === href;
          return (
            <Link
              key={href}
              href={href}
              className={`px-5 py-2.5 rounded-md transition-all duration-200 font-semibold text-[13px] tracking-wide ${
                isActive
                  ? 'bg-primary text-primary-foreground shadow-sm'
                  : 'text-muted-foreground hover:bg-muted/50 hover:text-foreground active:bg-accent'
              }`}
              aria-current={isActive ? 'page' : undefined}
            >
              {label}
            </Link>
          );
        })}
      </nav>
      <div className="p-8 text-[10px] text-muted-foreground font-bold tracking-widest uppercase text-center border-t border-border">
        Wheeltec ROS2
      </div>
    </div>
  );
}
