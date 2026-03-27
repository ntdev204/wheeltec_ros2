import "@/styles/globals.css";

import type { Metadata } from "next";
import { Fira_Code, Fira_Sans } from "next/font/google";

const firaCode = Fira_Code({
  variable: "--font-fira-code",
  subsets: ["latin"],
});

const firaSans = Fira_Sans({
  weight: ["300", "400", "500", "600", "700"],
  variable: "--font-fira-sans",
  subsets: ["latin"],
});

export const metadata: Metadata = {
  title: "SCADA Robot Control — Wheeltec ROS2",
  description: "Industrial-grade teleoperation dashboard for Wheeltec Mecanum robot. Real-time camera, SLAM mapping, and telemetry.",
  other: { "theme-color": "#f8f9fc" },
};

import { AppProviders } from '@/components/providers/robot-provider';

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <html
      lang="en"
      className={`${firaSans.variable} ${firaCode.variable} font-sans h-full antialiased`}
    >
      <body className="min-h-full flex flex-col">
        <AppProviders>
          {children}
        </AppProviders>
      </body>
    </html>
  );
}
